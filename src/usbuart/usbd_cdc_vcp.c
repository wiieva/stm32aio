#include "usbd_cdc_vcp.h"
#include "espeva_stmboard.h"
#include "esp8266_ctl.h"


LINE_CODING linecoding =
  {
    115200, /* baud rate*/
    0x00,   /* stop bits-1*/
    0x00,   /* parity - none*/
    0x08    /* nb. of bits 8*/
  };


USART_InitTypeDef usart;

/* These are external variables imported from CDC core to be used for IN 
   transfer management. */
extern uint8_t  APP_Rx_Buffer []; /* Write CDC received data in this buffer.
                                     These data will be sent over USB IN endpoint
                                     in the CDC core functions. */
extern uint32_t APP_Rx_ptr_in;    /* Increment this pointer or roll it back to
                                     start address when writing received data
                                     in the buffer APP_Rx_Buffer. */

static uint16_t VCP_Init     (void);
static uint16_t VCP_DeInit   (void);
static uint16_t VCP_Ctrl     (uint32_t Cmd, uint8_t* Buf, uint32_t Len);
static uint16_t VCP_DataTx   (void);
static uint16_t VCP_DataRx   (uint8_t* Buf, uint32_t Len);

static uint16_t VCP_COMConfig(uint8_t Conf);

CDC_IF_Prop_TypeDef VCP_fops = 
{
  VCP_Init,
  VCP_DeInit,
  VCP_Ctrl,
  VCP_DataTx,
  VCP_DataRx
};

static uint16_t VCP_Init(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  usart.USART_BaudRate = 115200;
  usart.USART_WordLength = USART_WordLength_8b;
  usart.USART_StopBits = USART_StopBits_1_5;
  usart.USART_Parity = USART_Parity_No;
  usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

  GPIO_PinRemapConfig(GPIO_Remap_USART1,ENABLE);

  GPIO_InitTypeDef gpio;
  gpio.GPIO_Mode = GPIO_Mode_AF_PP;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;

  gpio.GPIO_Pin = ESP_USART_TX_PIN;
  GPIO_Init(ESP_USART_TX_PORT, &gpio);


  gpio.GPIO_Pin = ESP_USART_RX_PIN;
  gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(ESP_USART_RX_PORT, &gpio);


  USART_Init(ESP_USART, &usart);
  USART_Cmd(ESP_USART, ENABLE);

  /* Enable the USART Receive interrupt */
  USART_ITConfig(ESP_USART, USART_IT_RXNE, ENABLE);

  /* Enable USART Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = ESP_USART_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  return USBD_OK;
}

static uint16_t VCP_DeInit(void)
{

  return USBD_OK;
}

static uint16_t VCP_Ctrl (uint32_t Cmd, uint8_t* Buf, uint32_t Len)
{ 
  switch (Cmd)
  {
  case SEND_ENCAPSULATED_COMMAND:
    /* Not  needed for this driver */
    break;

  case GET_ENCAPSULATED_RESPONSE:
    /* Not  needed for this driver */
    break;

  case SET_COMM_FEATURE:
    /* Not  needed for this driver */
    break;

  case GET_COMM_FEATURE:
    /* Not  needed for this driver */
    break;

  case CLEAR_COMM_FEATURE:
    /* Not  needed for this driver */
    break;

  case SET_LINE_CODING:
    linecoding.bitrate = (uint32_t)(Buf[0] | (Buf[1] << 8) | (Buf[2] << 16) | (Buf[3] << 24));
    linecoding.format = Buf[4];
    linecoding.paritytype = Buf[5];
    linecoding.datatype = Buf[6];
    /* Set the new configuration */
    VCP_COMConfig(OTHER_CONFIG);
    break;

  case GET_LINE_CODING:
    Buf[0] = (uint8_t)(linecoding.bitrate);
    Buf[1] = (uint8_t)(linecoding.bitrate >> 8);
    Buf[2] = (uint8_t)(linecoding.bitrate >> 16);
    Buf[3] = (uint8_t)(linecoding.bitrate >> 24);
    Buf[4] = linecoding.format;
    Buf[5] = linecoding.paritytype;
    Buf[6] = linecoding.datatype; 
    break;

  case SET_CONTROL_LINE_STATE:
      ESP_CTL_Modem_SetLineState (*(uint16_t*)Buf);
    break;

  case SEND_BREAK:
    /* Not  needed for this driver */
    break;    
    
  default:
    break;
  }

  return USBD_OK;
}

static uint16_t VCP_DataTx (void)
{
  if (linecoding.datatype == 7)
  {
    APP_Rx_Buffer[APP_Rx_ptr_in] = USART_ReceiveData(ESP_USART) & 0x7F;
  }
  else if (linecoding.datatype == 8)
  {
    APP_Rx_Buffer[APP_Rx_ptr_in] = USART_ReceiveData(ESP_USART);
  }
  
  APP_Rx_ptr_in++;
  
  /* To avoid buffer overflow */
  if(APP_Rx_ptr_in == APP_RX_DATA_SIZE)
  {
    APP_Rx_ptr_in = 0;
  }  
  
  return USBD_OK;
}

static uint16_t VCP_DataRx (uint8_t* Buf, uint32_t Len)
{
  uint32_t i;
  
  for (i = 0; i < Len; i++)
  {
    USART_SendData(ESP_USART, *(Buf + i) );
    while(USART_GetFlagStatus(ESP_USART, USART_FLAG_TXE) == RESET);
  } 
 
  return USBD_OK;
}

static uint16_t VCP_COMConfig(uint8_t Conf)
{
  if (Conf == DEFAULT_CONFIG)  
  {
    usart.USART_BaudRate = 115200;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_Odd;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
    /* Configure and enable the USART */
//    STM_EVAL_COMInit(COM1, &usart);
    
    /* Enable the USART Receive interrupt */
    USART_ITConfig(ESP_USART, USART_IT_RXNE, ENABLE);
  }
  else
  {
    /* set the Stop bit*/
    switch (linecoding.format)
    {
    case 0:
      usart.USART_StopBits = USART_StopBits_1;
      break;
    case 1:
      usart.USART_StopBits = USART_StopBits_1_5;
      break;
    case 2:
      usart.USART_StopBits = USART_StopBits_2;
      break;
    default :
      VCP_COMConfig(DEFAULT_CONFIG);
      return (USBD_FAIL);
    }
    
    /* set the parity bit*/
    switch (linecoding.paritytype)
    {
    case 0:
      usart.USART_Parity = USART_Parity_No;
      break;
    case 1:
      usart.USART_Parity = USART_Parity_Even;
      break;
    case 2:
      usart.USART_Parity = USART_Parity_Odd;
      break;
    default :
      VCP_COMConfig(DEFAULT_CONFIG);
      return (USBD_FAIL);
    }
    
    /*set the data type : only 8bits and 9bits is supported */
    switch (linecoding.datatype)
    {
    case 0x07:
      /* With this configuration a parity (Even or Odd) should be set */
      usart.USART_WordLength = USART_WordLength_8b;
      break;
    case 0x08:
      if (usart.USART_Parity == USART_Parity_No)
      {
        usart.USART_WordLength = USART_WordLength_8b;
      }
      else 
      {
        usart.USART_WordLength = USART_WordLength_9b;
      }
      
      break;
    default :
      VCP_COMConfig(DEFAULT_CONFIG);
      return (USBD_FAIL);
    }
    
    usart.USART_BaudRate = linecoding.bitrate;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
    USART_Init(ESP_USART, &usart);
    USART_Cmd(ESP_USART, ENABLE);
  }
  return USBD_OK;
}

/**
  * @brief  EVAL_COM_IRQHandler
  *         
  * @param  None.
  * @retval None.
  */
void USART1_IRQHandler(void)
{
  if (USART_GetITStatus(ESP_USART, USART_IT_RXNE) != RESET)
  {
    /* Send the received data to the PC Host*/
    VCP_DataTx ();
  }

  /* If overrun condition occurs, clear the ORE flag and recover communication */
  if (USART_GetFlagStatus(ESP_USART, USART_FLAG_ORE) != RESET)
  {
    (void)USART_ReceiveData(ESP_USART);
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
