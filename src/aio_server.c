
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
#include "debug.h"

volatile uint16_t *scheduled_buf = 0;
volatile uint16_t scheduled_recv_buf_size = 0;
volatile uint16_t scheduled_cmd_rx_d = 0;
void (*scheduled_buf_handler) () = 0;

static uint16_t analog_write_value;
static AIO_PwmParms pwm_parms;
static int test=0;

inline void ESP_SPI_Send (uint16_t val) {
    ESP_SPI->DR = val;
}

inline void ESP_SPI_SendBuffer (volatile void *buf,uint16_t size) {
    ESP_SPI_Send (size);
    if (size){
        ESP_SPI->CR2 &= ~((uint16_t)1 << (SPI_I2S_IT_RXNE >>4));
        DMA1_Channel3->CCR &= (uint16_t)(~DMA_CCR1_EN);
        DMA1_Channel3->CMAR =(uint32_t) buf;
        DMA1_Channel3->CNDTR = (size+1)/2;
        DMA1_Channel3->CCR |= DMA_CCR1_EN;
    }
}

inline void ESP_SPI_ScheduleRecvBuffer (volatile void *buf,uint16_t size,void (*handler)()) {
    ESP_SPI_Send (size);
    scheduled_recv_buf_size = size;
    scheduled_buf_handler = handler;
    scheduled_buf = buf;
}

void ESP_SPI_RecvBuffer (volatile void *buf,uint16_t size) {
    ESP_SPI->CR2 &= ~((uint16_t)1 << (SPI_I2S_IT_RXNE >>4));
    DMA1_Channel2->CCR &= (uint16_t)(~DMA_CCR1_EN);
    DMA1_Channel2->CMAR =(uint32_t) buf;
    DMA1_Channel2->CNDTR = (size+1)/2;
    DMA1_Channel2->CCR |= DMA_CCR1_EN;
    ESP_SPI->CR2 |= SPI_I2S_DMAReq_Rx;
}

#define ARG1 ((rx_d >>8)  & 0xFF)
#define ARG2 ((rx_d >> 5) & 0x7)

void ESP_SPI_Cmd_RdStream(uint16_t rx_d) {
	uint8_t *ptr = 0;
    uint16_t cnt = cbuf_read_ptr (ESP_Mic_Buffer(),&ptr,ARG1);
	ESP_SPI_SendBuffer (ptr,cnt);
    cbuf_read_commit (ESP_Mic_Buffer());
}

void ESP_SPI_Cmd_WrStream_Buf_Handler () {
    circular_buffer *abuf = ESP_Speaker_Buffer();
    // Buffer received. Commit data in circular buffer
    cbuf_write_commit(abuf);
}

void ESP_SPI_Cmd_WrStream(uint16_t rx_d) {
    circular_buffer *abuf = ESP_Speaker_Buffer();
    uint8_t *ptr = 0;
    uint16_t cnt = cbuf_write_ptr (abuf,&ptr,ARG1);
    // Align size. For DMA operaion size and address must be aligned to 16 bit
    if (cnt & 1)
        cnt = cbuf_write_ptr (abuf,&ptr,cnt & ~1);
    ESP_SPI_ScheduleRecvBuffer (ptr,cnt,ESP_SPI_Cmd_WrStream_Buf_Handler);
}

void ESP_SPI_Cmd_AudioInMode (uint16_t rx_d) {
    switch (ARG1) {
    case AIO_AUDIO_OFF:   ESP_Mic_Stop();    break;
    case AIO_AUDIO_IN_SPEEX: ESP_Mic_Start (8000,1); break;
    case AIO_AUDIO_RAW_8KHZ:  ESP_Mic_Start (8000,0); break;
    case AIO_AUDIO_RAW_16KHZ: ESP_Mic_Start (16000,0); break;
    case AIO_AUDIO_RAW_22KHZ: ESP_Mic_Start (22050,0); break;
    case AIO_AUDIO_RAW_44KHZ: ESP_Mic_Start (44100,0); break;
    }
}

void ESP_SPI_Cmd_AudioOutMode (uint16_t rx_d) {
    switch (ARG1) {
    case AIO_AUDIO_OFF:   ESP_Speaker_Stop();    break;
    case AIO_AUDIO_OUT_MP3: ESP_Speaker_Start (8000,1); break;
    case AIO_AUDIO_RAW_8KHZ:  ESP_Speaker_Start (8000,0); break;
    case AIO_AUDIO_RAW_16KHZ: ESP_Speaker_Start (16000,0); break;
    case AIO_AUDIO_RAW_22KHZ: ESP_Speaker_Start (22050,0); break;
    case AIO_AUDIO_RAW_44KHZ: ESP_Speaker_Start (44100,0); break;
    }
}

void ESP_SPI_Cmd_AnalogWrite_Handler () {
    ESP_Wiring_AnalogWrite(scheduled_cmd_rx_d>>8,analog_write_value);
}
void ESP_SPI_Cmd_SetPwmParms_Handler () {
    ESP_Wiring_SetPwmParms(scheduled_cmd_rx_d>>8,pwm_parms.freq,pwm_parms.limit);
}

void ESP_SPI_Cmd_SendIR_Buf_Handler () {
    cbuf_write_commit(&ir_sym_buf);
}

void ESP_SPI_Cmd_SendIR (uint16_t rx_d) {
    uint8_t *ptr = 0;
    uint16_t cnt = cbuf_write_ptr(&ir_sym_buf,&ptr,sizeof (AIO_IR_Payload));
    ESP_SPI_ScheduleRecvBuffer (ptr,cnt,ESP_SPI_Cmd_SendIR_Buf_Handler);
}

void ESP_SPI_Cmd_SendUART (uint16_t rx_d) {
    (void)rx_d;
}

void ESP_SPI_Cmd_RecvUART (uint16_t rx_d) {
    (void)rx_d;
}

void ESP_SPI_Cmd_SendI2C (uint16_t rx_d) {
    (void)rx_d;
}

void ESP_SPI_Cmd_RecvI2C (uint16_t rx_d) {
    (void)rx_d;
}

void ESP_SPI_Cmd_SetUartMode  (uint16_t rx_d) {
    (void)rx_d; 
}

void ESP_SPI_Cmd_GetBmeData (uint16_t rx_d) {
    (void)rx_d;
    static AIO_BME_Data data;
    data = *ESP_BME280_GetData();
    ESP_SPI_SendBuffer (&data,sizeof (data));
    ESP_BME280_RequestData ();
}

void SPI1_IRQHandler(void) {

    if (!(ESP_SPI->SR & SPI_SR_RXNE))
        return;

    // Set busy flag
    ESP_GPIO0_PORT->BRR = ESP_GPIO0_PIN;

    uint16_t rx_d = SPI1->DR;
    if (scheduled_recv_buf_size) {
        ESP_SPI_RecvBuffer (scheduled_buf,scheduled_recv_buf_size);
        scheduled_recv_buf_size = 0;
        return;
    }

    switch (rx_d & 0x1F) {
    // Standart wiring commands: control GPIO's
    // ****************************************
    case AIO_CMD_ANALOG_READ: 
        ESP_SPI_Send (ESP_Wiring_AnalogRead (ARG1)); 
        break;
    case AIO_CMD_DIGITAL_READ: 
        ESP_SPI_Send (ESP_Wiring_DigitalRead (ARG1));
        break;
    case AIO_CMD_ANALOG_WRITE: 
        scheduled_cmd_rx_d = rx_d;
        ESP_SPI_ScheduleRecvBuffer (&analog_write_value,sizeof (analog_write_value),ESP_SPI_Cmd_AnalogWrite_Handler);
        break;
    case AIO_CMD_DIGITAL_WRITE:     
        ESP_Wiring_DigitalWrite (ARG1,ARG2);
        break;
    case AIO_CMD_PIN_MODE:
        ESP_Wiring_PinMode (ARG1,ARG2); 
        break;
    case AIO_CMD_SET_PWM_PARMS: 
        scheduled_cmd_rx_d = rx_d;
        ESP_SPI_ScheduleRecvBuffer (&pwm_parms,sizeof (pwm_parms),ESP_SPI_Cmd_SetPwmParms_Handler);
        break;
    // Audio IN/OUT commands. Play and record
    // **************************************
    case AIO_CMD_RD_STREAM: 
        ESP_SPI_Cmd_RdStream (rx_d); 
        break;
    case AIO_CMD_WR_STREAM:
        ESP_SPI_Cmd_WrStream (rx_d); 
        break;
    case AIO_CMD_AUDIO_IN_MODE: 
        ESP_SPI_Cmd_AudioInMode (rx_d); 
        break;
    case AIO_CMD_AUDIO_OUT_MODE: 
        ESP_SPI_Cmd_AudioOutMode (rx_d); 
        break;
    case AIO_CMD_GET_VAD_STATE: 
        ESP_SPI_Send (vad_state);
        break;
    case AIO_CMD_AUDIO_IN_VOLUME: 
        audio_in_volume = ARG1;
        break;
    case AIO_CMD_AUDIO_OUT_VOLUME: 
        audio_out_volume = ARG1;
        break;
    // Sensor's commmands (touch,bme, IR)
    // **********************************
    case AIO_CMD_SEND_IR_CMD: 
        ESP_SPI_Cmd_SendIR (rx_d); 
        break;
    case AIO_CMD_GET_INPUT_STATE:     
        ESP_SPI_SendBuffer (&input_state,sizeof (input_state));
        break;
    case AIO_CMD_GET_BME_DATA: 
        ESP_SPI_Cmd_GetBmeData (rx_d); 
        break;

    // Extra communication's commands: UART & I2C
    // ******************************************
    case AIO_CMD_UART_MODE: 
        ESP_SPI_Cmd_SetUartMode (rx_d); 
        break;
    case AIO_CMD_SEND_UART: ESP_SPI_Cmd_SendUART (rx_d); break;
    case AIO_CMD_RECV_UART: ESP_SPI_Cmd_RecvUART (rx_d); break;
    case AIO_CMD_SEND_I2C: ESP_SPI_Cmd_SendI2C (rx_d); break; 
    case AIO_CMD_RECV_I2C: ESP_SPI_Cmd_RecvI2C (rx_d); break;

    // System commands
    // ***************
    case AIO_CMD_GET_SYSINFO:     
        ESP_SPI_SendBuffer (&sys_info,sizeof (sys_info));
        break;
    case AIO_CMD_TEST: 
        ESP_SPI_Send (++test); 
        break;
    default: 
        break;
    }

    // Reset busy flag
    ESP_GPIO0_PORT->BSRR = ESP_GPIO0_PIN;
}

void DMA1_Channel3_IRQHandler(void) {
    if(DMA1->ISR & DMA1_IT_TC3) {
        DMA1->IFCR = DMA1_IT_TC3;
        ESP_SPI->CR2 |= (uint16_t) (1 << (SPI_I2S_IT_RXNE >>4));
    }
}

void DMA1_Channel2_IRQHandler(void) {
    if(DMA1->ISR & DMA1_IT_TC2) {
        DMA1->IFCR = DMA1_IT_TC2;
        ESP_SPI->CR2 |= (uint16_t) (1 << (SPI_I2S_IT_RXNE >>4));
        ESP_SPI->CR2 &= ~SPI_I2S_DMAReq_Rx;
        if (scheduled_buf_handler)
            scheduled_buf_handler ();
        // Reset busy flag
        ESP_GPIO0_PORT->BSRR = ESP_GPIO0_PIN;
    }
}

void ESP_SPI_Init () {
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

    dma.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_Init (DMA1_Channel2, &dma);

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
    SPI_I2S_DMACmd(ESP_SPI,SPI_I2S_DMAReq_Tx,ENABLE);

    DMA_ITConfig(DMA1_Channel2,DMA_IT_TC, ENABLE);

}

void ESP_SPI_Interrupts_Init () {
    NVIC_InitTypeDef nvic;

    nvic.NVIC_IRQChannelCmd = ENABLE;
    nvic.NVIC_IRQChannel = ESP_SPI_IRQ;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&nvic);

    nvic.NVIC_IRQChannel = DMA1_Channel3_IRQn;
    NVIC_Init(&nvic);

    nvic.NVIC_IRQChannel = DMA1_Channel2_IRQn;
    NVIC_Init(&nvic);
}

void AIO_Server_Init () {
    ESP_SPI_Init ();
    ESP_SPI_Interrupts_Init ();
}
