#include <stm32f10x.h>



uint16_t ESP_USART_Init(int baudrate)
{
    USART_InitTypeDef usart;
    NVIC_InitTypeDef NVIC_InitStructure;

    usart.USART_BaudRate = baudrate;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1_5;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_PartialRemap_USART3,ENABLE);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    GPIO_InitTypeDef gpio;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;

    gpio.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOC, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &gpio);

    USART_Init(USART3, &usart);
    USART_Cmd(USART3, ENABLE);

    /* Enable the USART Receive interrupt */
    //USART_ITConfig(USART3, USART_IT_RXNE|USART_IT_RXNE, ENABLE);

    /* Enable USART Interrupt */
    //NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //NVIC_Init(&NVIC_InitStructure);

    return 0;
}

void ESP_USART_Send_Char (uint8_t chr)
{
    USART_SendData(USART3, chr );
    while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
}

uint16_t ESP_USART_Send_Buf (uint8_t* Buf, uint32_t Len)
{
    uint32_t i;
    for (i = 0; i < Len; i++)
        ESP_USART_Send_Char (Buf[i]);
    return 0;
}



#include <stdarg.h>

int ESP_USART_Debug_Printf (char *fmt, ...)
{
    char tmpbuf[1024];

    int ret;
    va_list va;
    va_start(va, fmt);
    ret = mini_vsnprintf(tmpbuf, 1024, fmt, va);
    va_end(va);
    ESP_USART_Send_Buf (tmpbuf,ret);

    return ret;
}
