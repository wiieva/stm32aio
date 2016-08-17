#include "stm32x.h"

#include "espeva_stmboard.h"
#include "circular_buffer.h"
#include "aioiface.h"

#include "ir_send_int.h"

#define ir_raw_buf_size (128*2)
#define ir_sym_buf_size (sizeof(AIO_IR_Payload)*8)

circular_buffer ir_sym_buf,ir_raw_buf;
int ir_tx_state = 0;
static int ir_khz = 0;

void InitTMR2 (int khz) {
    ir_khz = khz;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_TimeBaseInitTypeDef tim;
    tim.TIM_Period = (CPUFREQ_KHZ / khz) - 1;
    tim.TIM_Prescaler = 0;
    tim.TIM_ClockDivision = 0;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &tim);
    TIM_Cmd(TIM2, ENABLE);
}

void InitTMR6 () {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
    TIM_TimeBaseInitTypeDef tim;
    tim.TIM_Period = 100;
    tim.TIM_Prescaler = CPUFREQ_MHZ-1;
    tim.TIM_ClockDivision = 0;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM6, &tim);
    TIM_Cmd(TIM6, DISABLE);
    TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
}

void InitTMR2_PWM() {
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Mode    = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed   = GPIO_Speed_50MHz;
    gpio.GPIO_Pin     = ESP_IR_TX_PIN;
    GPIO_Init (ESP_IR_TX_PORT,&gpio);
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM2,ENABLE);

    TIM_OCInitTypeDef timerPWM;
    timerPWM.TIM_Pulse = (CPUFREQ_KHZ / ir_khz) * 40 / 100;
    timerPWM.TIM_OCMode = TIM_OCMode_PWM1;
    timerPWM.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OC4Init(TIM2, &timerPWM);
}

void TMR6_Interrupts_Config(void) {
    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel = TIM6_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

static void enableIROut(int khz) {
    cbuf_init (&ir_raw_buf,ir_raw_buf_size);
    InitTMR2(khz);
    InitTMR2_PWM ();
    ir_tx_state = 1;
}

static void disableIROut () {
    TIM_Cmd(TIM2, DISABLE);
    TIM_Cmd(TIM6, DISABLE);
    ir_tx_state = 0;
    cbuf_destroy (&ir_raw_buf);
}

void TIM6_IRQHandler () {
    volatile int16_t c;

    if (TIM_GetITStatus(TIM6, TIM_IT_Update) == RESET)
        return;
    TIM_ClearITPendingBit(TIM6, TIM_IT_Update);

    if (!ir_tx_state)
        return;

    if (!cbuf_read(&ir_raw_buf,(uint8_t*)&c,sizeof(c))) {
        disableIROut();
        return ;
    }

    if (c < 0) {
        c = -c;
        TIM_SelectOCxM(TIM2,TIM_Channel_4,TIM_ForcedAction_InActive);
    }
    else {
        TIM_SelectOCxM(TIM2,TIM_Channel_4,TIM_OCMode_PWM1);
        TIM_CCxCmd(TIM2, TIM_Channel_4, TIM_CCx_Enable);
    }
    TIM_SetAutoreload(TIM6,c);
    if (c == 0)
        disableIROut();
}


/* Leave pin off for time (given in microseconds) */
static void space(int16_t time) {
    time = -time;
    cbuf_write (&ir_raw_buf, (uint8_t*)&time,sizeof (time));
    if (!time) {
        TIM_SetAutoreload(TIM6,100);
        TIM_Cmd(TIM6, ENABLE);
    }
}

static void mark(int16_t time) {
    cbuf_write (&ir_raw_buf, (uint8_t*)&time,sizeof (time));
}

void ir_sendNEC(unsigned long data, int nbits) {
    int i;
    enableIROut(38);
    mark(NEC_HDR_MARK);
    space(NEC_HDR_SPACE);
    for (i = 0; i < nbits; i++) {
        if (data & TOPBIT) {
            mark(NEC_BIT_MARK);
            space(NEC_ONE_SPACE);
        }
        else {
            mark(NEC_BIT_MARK);
            space(NEC_ZERO_SPACE);
        }
        data <<= 1;
    }
    mark(NEC_BIT_MARK);
    space(0);
}

void ir_sendWhynter(unsigned long data, int nbits) {
    int i;
    enableIROut(38);
    mark(WHYNTER_ZERO_MARK);
    space(WHYNTER_ZERO_SPACE);
    mark(WHYNTER_HDR_MARK);
    space(WHYNTER_HDR_SPACE);
    for (i = 0; i < nbits; i++) {
        if (data & TOPBIT) {
            mark(WHYNTER_ONE_MARK);
            space(WHYNTER_ONE_SPACE);
        }
        else {
            mark(WHYNTER_ZERO_MARK);
            space(WHYNTER_ZERO_SPACE);
        }
        data <<= 1;
    }
    mark(WHYNTER_ZERO_MARK);
    space(WHYNTER_ZERO_SPACE);
    space(0);
}

void ir_sendSony(unsigned long data, int nbits) {
    int i;
    enableIROut(38);
    mark(SONY_HDR_MARK);
    space(SONY_HDR_SPACE);
    data = data << (32 - nbits);
    for (i = 0; i < nbits; i++) {
        if (data & TOPBIT) {
            mark(SONY_ONE_MARK);
            space(SONY_HDR_SPACE);
        }
        else {
            mark(SONY_ZERO_MARK);
            space(SONY_HDR_SPACE);
        }
        data <<= 1;
    }
    space(0);
}

void ir_sendRaw(unsigned int buf[], int len, int hz) {
    int i;
    enableIROut(hz);
    for (i = 0; i < len; i++) {
        if (i & 1) {
            space(buf[i]);
        }
        else {
            mark(buf[i]);
        }
    }
    space(0); // Just to be sure
}
// Note: first bit must be a one (start bit)
void ir_sendRC5(unsigned long data, int nbits) {
    int i;
    enableIROut(36);
    data = data << (32 - nbits);
    mark(RC5_T1); // First start bit
    space(RC5_T1); // Second start bit
    mark(RC5_T1); // Second start bit
    for (i = 0; i < nbits; i++) {
        if (data & TOPBIT) {
            space(RC5_T1); // 1 is space, then mark
            mark(RC5_T1);
        }
        else {
            mark(RC5_T1);
            space(RC5_T1);
        }
        data <<= 1;
    }
    space(0); // Turn off at end
}

// Caller needs to take care of flipping the toggle bit
void ir_sendRC6(unsigned long data, int nbits) {
    int i;
    enableIROut(36);
    data = data << (32 - nbits);
    mark(RC6_HDR_MARK);
    space(RC6_HDR_SPACE);
    mark(RC6_T1); // start bit
    space(RC6_T1);
    int t;
    for (i = 0; i < nbits; i++) {
        if (i == 3) {
            // double-wide trailer bit
            t = 2 * RC6_T1;
        }
        else {
            t = RC6_T1;
        }
        if (data & TOPBIT) {
            mark(t);
            space(t);
        }
        else {
            space(t);
            mark(t);
        }

        data <<= 1;
    }
    space(0); // Turn off at end
}

void ir_sendPanasonic(unsigned int address, unsigned long data)
{
    int i;
    enableIROut(35);
    mark(PANASONIC_HDR_MARK);
    space(PANASONIC_HDR_SPACE);
    for(i=0;i<16;i++)
    {
        mark(PANASONIC_BIT_MARK);
        if (address & 0x8000) {
            space(PANASONIC_ONE_SPACE);
        } else {
            space(PANASONIC_ZERO_SPACE);
        }
        address <<= 1;
    }
    for (i=0; i < 32; i++) {
        mark(PANASONIC_BIT_MARK);
        if (data & TOPBIT) {
            space(PANASONIC_ONE_SPACE);
        } else {
            space(PANASONIC_ZERO_SPACE);
        }
        data <<= 1;
    }
    mark(PANASONIC_BIT_MARK);
    space(0);
}

void ir_sendJVC(unsigned long data, int nbits, int repeat)
{
    int i;
    enableIROut(38);
    data = data << (32 - nbits);
    if (!repeat){
        mark(JVC_HDR_MARK);
        space(JVC_HDR_SPACE);
    }
    for (i = 0; i < nbits; i++) {
        if (data & TOPBIT) {
            mark(JVC_BIT_MARK);
            space(JVC_ONE_SPACE);
        }
        else {
            mark(JVC_BIT_MARK);
            space(JVC_ZERO_SPACE);
        }
        data <<= 1;
    }
    mark(JVC_BIT_MARK);
    space(0);
}

void ir_sendSAMSUNG(unsigned long data, int nbits)
{
    int i;
    enableIROut(38);
    mark(SAMSUNG_HDR_MARK);
    space(SAMSUNG_HDR_SPACE);
    for (i = 0; i < nbits; i++) {
        if (data & TOPBIT) {
            mark(SAMSUNG_BIT_MARK);
            space(SAMSUNG_ONE_SPACE);
        }
        else {
            mark(SAMSUNG_BIT_MARK);
            space(SAMSUNG_ZERO_SPACE);
        }
        data <<= 1;
    }
    mark(SAMSUNG_BIT_MARK);
    space(0);
}

void ESP_IRSend_Init ()
{
    cbuf_init (&ir_sym_buf,ir_sym_buf_size);
    InitTMR6 ();
    TMR6_Interrupts_Config ();
    ir_tx_state = 0;
}

void ESP_IRSend_Run ()
{
    AIO_IR_Payload *p;
    if (ir_tx_state)
        return;

    if (cbuf_read_ptr (&ir_sym_buf,(uint8_t**)&p,sizeof (*p))) {
        cbuf_read_commit(&ir_sym_buf);
        switch (p->mode) {
        case AIO_SEND_IR_SONY:      ir_sendSony(p->data,p->n_bits); break;
        case AIO_SEND_IR_NEC:       ir_sendNEC (p->data,p->n_bits); break;
        case AIO_SEND_IR_WHYNTER:   ir_sendWhynter(p->data,p->n_bits); break;
        case AIO_SEND_IR_RC5:       ir_sendRC5(p->data,p->n_bits); break;
        case AIO_SEND_IR_RC6:       ir_sendRC6(p->data,p->n_bits); break;
        case AIO_SEND_IR_PANASONIC: ir_sendPanasonic(p->addr,p->data); break;
        case AIO_SEND_IR_JVC:       ir_sendJVC(p->data,p->n_bits,p->repeat); break;
        case AIO_SEND_IR_SAMSUNG:   ir_sendSAMSUNG(p->data,p->n_bits); break;
        }
    }
}
