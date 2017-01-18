#pragma once

#define _VCP_DEBUG_LOG_

#ifdef _USART_DEBUG_LOG_
#include "usart.h"
#define DEBUG_PRINT_INIT() ESP_USART_Init(115200)
#define DEBUG_PRINT(x,...) ESP_USART_Debug_Printf(x, ## __VA_ARGS__)
#elif defined _VCP_DEBUG_LOG_
#include "usbd_cdc_vcp.h"
#define DEBUG_PRINT_INIT() 
#define DEBUG_PRINT(x,...) VCP_Debug_Printf(x, ## __VA_ARGS__)
#else
#define DEBUG_PRINT_INIT() 
#define DEBUG_PRINT(x,...) 
#endif
