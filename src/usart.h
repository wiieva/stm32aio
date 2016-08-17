
#pragma once

uint16_t ESP_USART_Init(int baudrate);
void ESP_USART_Send_Char (uint8_t chr);
uint16_t ESP_USART_Send_Buf (uint8_t* Buf, uint32_t Len);
int ESP_USART_Debug_Printf (char *fmt, ...);



