
#include "stm32x.h"
#include "aioiface.h"
#include "aio_server.h"
#include "espeva_stmboard.h"
#include "audio_in.h"
#include "audio_out.h"
#include "arduino_wiring.h"
#include "ir_send.h"
#include "ui_input.h"
#include "bme280.h"

//#define _SPI_PROFILE_

#ifdef STM32F10X_CL
#define SPI_FLAG_RXNE SPI_SR_RXNE
#endif

volatile uint16_t *scheduled_buf = 0;
volatile uint16_t scheduled_recv_buf_size = 0;
volatile uint16_t scheduled_cmd_rx_d = 0;
void (*scheduled_buf_handler) () = 0;

void SPI1_RxSkip_IRQHandler ();
void SPI1_AnalogWrite_IRQHandler ();

inline void ESP_SPI1_Set_IRQ_Handler (void (*pfn)())
{
    extern volatile uint32_t _svector[];
    _svector[51] = (uint32_t)pfn;
}

inline void ESP_SPI_Send (uint16_t val)
{
//    while (!(ESP_SPI->SR & SPI_I2S_FLAG_TXE));
    ESP_SPI->DR = val;
    ESP_SPI1_Set_IRQ_Handler (SPI1_RxSkip_IRQHandler);
}

inline void ESP_SPI_ScheduleSendBuffer (volatile void *buf,uint16_t size)
{
    ESP_SPI_Send (size);
    if (size){
        ESP_SPI->CR2 &= ~((uint16_t)1 << (SPI_I2S_IT_RXNE >>4));
        DMA1_Channel3->CCR &= (uint16_t)(~DMA_CCR1_EN);
        DMA1_Channel3->CMAR =(uint32_t) buf;
        DMA1_Channel3->CNDTR = (size+1)/2;
        DMA1_Channel3->CCR |= DMA_CCR1_EN;
    }
}

inline void ESP_SPI_ScheduleRecvBuffer (volatile void *buf,uint16_t size,void (*handler)())
{
    ESP_SPI_Send (size);
    scheduled_recv_buf_size = (size+1)/2;
    scheduled_buf = (uint16_t *)buf;
    scheduled_buf_handler = handler;
}

typedef void (*ESP_SPI_Cmd_Handler) (uint16_t);
#define ARG1 ((rx_d >>8)  & 0xFF)
#define ARG2 ((rx_d >> 5) & 0x7)

void ESP_SPI_Cmd_Nop (uint16_t rx_d)
{
	(void)rx_d;
}

void ESP_SPI_Cmd_RdStream(uint16_t rx_d)
{
	uint8_t *ptr = 0;
    uint16_t cnt = cbuf_read_ptr (ESP_Mic_Buffer(),&ptr,ARG1);
	ESP_SPI_ScheduleSendBuffer (ptr,cnt);
    cbuf_read_commit (ESP_Mic_Buffer());
}

void ESP_SPI_Cmd_WrStream_Buf_Handler ()
{
    circular_buffer *abuf = ESP_Speaker_Buffer();
    // Buffer received. Commit data in circular buffer
    cbuf_write_commit(abuf);
    if (abuf->full)
        DMA_Cmd(DMA2_Channel3,ENABLE);
}

void ESP_SPI_Cmd_WrStream(uint16_t rx_d)
{
    uint8_t *ptr = 0;
    uint16_t cnt = cbuf_write_ptr (ESP_Speaker_Buffer(),&ptr,ARG1);
    ESP_SPI_ScheduleRecvBuffer (ptr,cnt,ESP_SPI_Cmd_WrStream_Buf_Handler);
}

void ESP_SPI_Cmd_AnalogRead(uint16_t rx_d)
{
    ESP_SPI_Send (ESP_Wiring_AnalogRead (ARG1));
}

void ESP_SPI_Cmd_DigitalRead(uint16_t rx_d)
{
    ESP_SPI_Send (ESP_Wiring_DigitalRead (ARG1));
}

void ESP_SPI_Cmd_DigitalWrite(uint16_t rx_d)
{
    ESP_Wiring_DigitalWrite (ARG1,ARG2);
}

void ESP_SPI_Cmd_AnalogWrite(uint16_t rx_d)
{
    scheduled_cmd_rx_d = rx_d;
    ESP_SPI1_Set_IRQ_Handler (SPI1_AnalogWrite_IRQHandler);
}

void ESP_SPI_Cmd_PinMode(uint16_t rx_d)
{
    ESP_Wiring_PinMode (ARG1,ARG2);
}

static AIO_PwmParms pwm_parms;

void ESP_SPI_Cmd_SetPwmParms_Handler ()
{
    ESP_Wiring_SetPwmParms(scheduled_cmd_rx_d>>8,pwm_parms.freq,pwm_parms.limit);
}

void ESP_SPI_Cmd_SetPwmParms (uint16_t rx_d)
{
    scheduled_cmd_rx_d = rx_d;
    uint16_t cnt =  ARG1;

    ESP_SPI_ScheduleRecvBuffer (&pwm_parms,sizeof (pwm_parms),ESP_SPI_Cmd_SetPwmParms_Handler);
}

void ESP_SPI_Cmd_AudioInMode (uint16_t rx_d)
{

    switch (ARG1) {
    case AIO_AUDIO_OFF:   ESP_Wiring_ADC_Start();    break;
    case AIO_AUDIO_IN_SPEEX: ESP_Mic_Start (8000,1); break;
    case AIO_AUDIO_RAW_8KHZ:  ESP_Mic_Start (8000,0); break;
    case AIO_AUDIO_RAW_16KHZ: ESP_Mic_Start (16000,0); break;
    case AIO_AUDIO_RAW_22KHZ: ESP_Mic_Start (22050,0); break;
    case AIO_AUDIO_RAW_44KHZ: ESP_Mic_Start (44100,0); break;
    }
}

void ESP_SPI_Cmd_AudioOutMode (uint16_t rx_d)
{
    switch (ARG1) {
    case AIO_AUDIO_OFF:   ESP_Speaker_Stop();    break;
    case AIO_AUDIO_OUT_MP3: ESP_Speaker_Start (8000,1); break;
    case AIO_AUDIO_RAW_8KHZ:  ESP_Speaker_Start (8000,0); break;
    case AIO_AUDIO_RAW_16KHZ: ESP_Speaker_Start (16000,0); break;
    case AIO_AUDIO_RAW_22KHZ: ESP_Speaker_Start (22050,0); break;
    case AIO_AUDIO_RAW_44KHZ: ESP_Speaker_Start (44100,0); break;
    }
}

void ESP_SPI_Cmd_GetVadState (uint16_t rx_d)
{
    (void)rx_d;
    extern int vad_state;
    ESP_SPI_Send (vad_state);
}

void ESP_SPI_Cmd_SendIR_Buf_Handler ()
{
    cbuf_write_commit(&ir_sym_buf);
}

void ESP_SPI_Cmd_SendIR (uint16_t rx_d)
{
    uint8_t *ptr = 0;
    uint16_t cnt =  ARG1;
    //if (cnt != sizeof (AIO_IR_Payload))
    //    cnt = 0;

    cnt = cbuf_write_ptr(&ir_sym_buf,&ptr,cnt);
    ESP_SPI_ScheduleRecvBuffer (ptr,cnt,ESP_SPI_Cmd_SendIR_Buf_Handler);
}

void ESP_SPI_Cmd_SendUART (uint16_t rx_d)
{
    (void)rx_d;
}

void ESP_SPI_Cmd_RecvUART (uint16_t rx_d)
{
    (void)rx_d;
}

void ESP_SPI_Cmd_SendI2C (uint16_t rx_d)
{
    (void)rx_d;
}

void ESP_SPI_Cmd_RecvI2C (uint16_t rx_d)
{
    (void)rx_d;
}

void ESP_SPI_Cmd_GetSysInfo (uint16_t rx_d)
{
    (void)rx_d;
    ESP_SPI_ScheduleSendBuffer (&sys_info,sizeof (sys_info));
}

void ESP_SPI_Cmd_GetInputState (uint16_t rx_d)
{
    (void)rx_d;
    ESP_SPI_ScheduleSendBuffer (&input_state,sizeof (input_state));
}

void ESP_SPI_Cmd_SetUartMode  (uint16_t rx_d)
{
    (void)rx_d;
}


void ESP_SPI_Cmd_GetBmeData (uint16_t rx_d)
{
    (void)rx_d;
    static AIO_BME_Data data;
    data = *ESP_BME280_GetData();
    ESP_SPI_ScheduleSendBuffer (&data,sizeof (data));
    ESP_BME280_RequestData ();
}

void ESP_SPI_Cmd_Test(uint16_t rx_d)
{
    (void)rx_d;
    static uint16_t test;
    ESP_SPI_Send (++test);
}

const ESP_SPI_Cmd_Handler cmd_handlers[] =
{
    ESP_SPI_Cmd_Nop,
    ESP_SPI_Cmd_RdStream,    // 67
    ESP_SPI_Cmd_WrStream,    // 91

    ESP_SPI_Cmd_AnalogRead,  // 52
    ESP_SPI_Cmd_DigitalRead, // 65
    ESP_SPI_Cmd_AnalogWrite, // 34
    ESP_SPI_Cmd_DigitalWrite,// 42
    ESP_SPI_Cmd_PinMode,     // 67
    ESP_SPI_Cmd_SetPwmParms,

    ESP_SPI_Cmd_AudioInMode,
    ESP_SPI_Cmd_AudioOutMode,
    ESP_SPI_Cmd_GetVadState,

    ESP_SPI_Cmd_SendIR,
    ESP_SPI_Cmd_SendUART,
    ESP_SPI_Cmd_RecvUART,
    ESP_SPI_Cmd_SendI2C,
    ESP_SPI_Cmd_RecvI2C,
    ESP_SPI_Cmd_GetSysInfo,
    ESP_SPI_Cmd_GetInputState,
    ESP_SPI_Cmd_SetUartMode,
    ESP_SPI_Cmd_GetBmeData,

    ESP_SPI_Cmd_Test,        // 47

    ESP_SPI_Cmd_Nop, ESP_SPI_Cmd_Nop,
    ESP_SPI_Cmd_Nop, ESP_SPI_Cmd_Nop, ESP_SPI_Cmd_Nop, ESP_SPI_Cmd_Nop,
    ESP_SPI_Cmd_Nop, ESP_SPI_Cmd_Nop, ESP_SPI_Cmd_Nop, ESP_SPI_Cmd_Nop,
    ESP_SPI_Cmd_Nop,
};

void SPI1_IRQHandler(void)
{

#ifdef _SPI_PROFILE_
    uint32_t start_tick = SysTick->VAL;
    static int32_t cmd_profile_times[32] = {0};
#endif
    if (!(SPI1->SR & SPI_FLAG_RXNE))
        return;

     uint16_t rx_d = SPI1->DR;
     uint8_t cmd = rx_d & 0x1F;
     cmd_handlers[cmd] (rx_d);

#ifdef _SPI_PROFILE_
    uint32_t end_tick = SysTick->VAL;
    if (end_tick < start_tick && cmd != 0)
    {
        uint16_t time = start_tick - end_tick;
        if (cmd_profile_times[cmd] < time)
            cmd_profile_times[cmd] = time;
    }
#endif
}

void SPI1_AnalogWrite_IRQHandler ()
{
    if (!(SPI1->SR & SPI_FLAG_RXNE))
        return;

    ESP_Wiring_AnalogWrite(scheduled_cmd_rx_d>>8,SPI1->DR);
    ESP_SPI1_Set_IRQ_Handler (SPI1_IRQHandler);
}

void SPI1_RecvBuf_IRQHandler ()
{
    if (!(SPI1->SR & SPI_FLAG_RXNE))
        return;

    *scheduled_buf++ = SPI1->DR;
    if (!--scheduled_recv_buf_size) {
        ESP_SPI1_Set_IRQ_Handler (SPI1_IRQHandler);
        if (scheduled_buf_handler)
            scheduled_buf_handler ();
    }
}

void SPI1_RxSkip_IRQHandler ()
{
    if (!(SPI1->SR & SPI_FLAG_RXNE))
        return;

    SPI1->DR;
    if (!scheduled_recv_buf_size)
        ESP_SPI1_Set_IRQ_Handler (SPI1_IRQHandler);
    else
        ESP_SPI1_Set_IRQ_Handler (SPI1_RecvBuf_IRQHandler);
}

void DMA1_Channel3_IRQHandler(void)
{
   if(DMA1->ISR & DMA1_IT_TC3) {
       DMA1->IFCR = DMA1_IT_TC3;
       ESP_SPI->CR2 |= (uint16_t) (1 << (SPI_I2S_IT_RXNE >>4));
    }
}

void ESP_SPI_Init ()
{
    RCC_AHBPeriphClockCmd (RCC_AHBPeriph_DMA1,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

    DMA_InitTypeDef dma;
    dma.DMA_PeripheralBaseAddr = (uint32_t)&(SPI1->DR);
    dma.DMA_MemoryBaseAddr = 0;
    dma.DMA_DIR = DMA_DIR_PeripheralDST;
    dma.DMA_BufferSize = 0;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    dma.DMA_Mode = DMA_Mode_Normal;
    dma.DMA_Priority = DMA_Priority_Medium;
    dma.DMA_M2M = DMA_M2M_Disable;
    DMA_Init (DMA1_Channel3, &dma);

    GPIO_InitTypeDef gpio;
    gpio.GPIO_Speed   = GPIO_Speed_50MHz;
    /* Connect PXx to SPI */
    GPIO_PinRemapConfig (GPIO_Remap_SPI1,ENABLE);
    /* Configure SPI pins alternate function push-pull */
    gpio.GPIO_Pin = ESP_SPI_MISO_PIN;
    gpio.GPIO_Mode    = GPIO_Mode_AF_PP;
    GPIO_Init(ESP_SPI_MISO_PORT, &gpio);
    gpio.GPIO_Mode    = GPIO_Mode_IN_FLOATING;
    gpio.GPIO_Pin = ESP_SPI_MOSI_PIN | ESP_SPI_SCK_PIN;
    GPIO_Init(ESP_SPI_SCK_PORT, &gpio);
    gpio.GPIO_Pin = ESP_SPI_NSS_PIN;
    GPIO_Init(ESP_SPI_NSS_PORT, &gpio);

    SPI_InitTypeDef spi;
    spi.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_Mode              = SPI_Mode_Slave;
    spi.SPI_DataSize          = SPI_DataSize_16b;
    spi.SPI_CPOL              = SPI_CPOL_Low;
    spi.SPI_CPHA              = SPI_CPHA_1Edge;
    spi.SPI_NSS               = SPI_NSS_Hard;
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    spi.SPI_FirstBit          = SPI_FirstBit_MSB;
    spi.SPI_CRCPolynomial     = 0;
    /* Init & start SPI */
    SPI_Init(ESP_SPI, &spi);
    SPI_Cmd(ESP_SPI, ENABLE);

    SPI_I2S_ITConfig (ESP_SPI,SPI_I2S_IT_RXNE,ENABLE);

    DMA_ITConfig(DMA1_Channel3,DMA_IT_TC, ENABLE);
    SPI_I2S_DMACmd(SPI1,SPI_I2S_DMAReq_Tx,ENABLE);
}

void ESP_SPI_Interrupts_Init () {
    NVIC_InitTypeDef nvic;

    nvic.NVIC_IRQChannelCmd = ENABLE;
    nvic.NVIC_IRQChannel = ESP_SPI_IRQ;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&nvic);

    nvic.NVIC_IRQChannel =   DMA1_Channel3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&nvic);
}

void AIO_Server_Init () {
    ESP_SPI_Init ();
    ESP_SPI_Interrupts_Init ();
}
