#pragma once

#include "stm32f10x.h"


#ifdef USE_USB_OTG_FS 
#define USB_OTG_FS_CORE
#endif

#ifndef USE_ULPI_PHY
/* #define USE_ULPI_PHY */
#endif /* USE_ULPI_PHY */

#ifndef USE_EMBEDDED_PHY
/* #define USE_EMBEDDED_PHY */
#endif /* USE_EMBEDDED_PHY */

#ifdef USE_USB_OTG_HS 
 #define USB_OTG_HS_CORE
#endif

/*******************************************************************************
*                      FIFO Size Configuration in Device mode
*  
*  (i) Receive data FIFO size = RAM for setup packets + 
*                   OUT endpoint control information +
*                   data OUT packets + miscellaneous
*      Space = ONE 32-bits words
*     --> RAM for setup packets = 10 spaces
*        (n is the nbr of CTRL EPs the device core supports) 
*     --> OUT EP CTRL info      = 1 space
*        (one space for status information written to the FIFO along with each 
*        received packet)
*     --> data OUT packets      = (Largest Packet Size / 4) + 1 spaces 
*        (MINIMUM to receive packets)
*     --> OR data OUT packets  = at least 2*(Largest Packet Size / 4) + 1 spaces 
*        (if high-bandwidth EP is enabled or multiple isochronous EPs)
*     --> miscellaneous = 1 space per OUT EP
*        (one space for transfer complete status information also pushed to the 
*        FIFO with each endpoint's last packet)
*
*  (ii)MINIMUM RAM space required for each IN EP Tx FIFO = MAX packet size for 
*       that particular IN EP. More space allocated in the IN EP Tx FIFO results
*       in a better performance on the USB and can hide latencies on the AHB.
*
*  (iii) TXn min size = 16 words. (n  : Transmit FIFO index)
*   (iv) When a TxFIFO is not used, the Configuration should be as follows: 
*       case 1 :  n > m    and Txn is not used    (n,m  : Transmit FIFO indexes)
*       --> Txm can use the space allocated for Txn.
*       case2  :  n < m    and Txn is not used    (n,m  : Transmit FIFO indexes)
*       --> Txn should be configured with the minimum space of 16 words
*  (v) The FIFO is used optimally when used TxFIFOs are allocated in the top 
*       of the FIFO.Ex: use EP1 and EP2 as IN instead of EP1 and EP3 as IN ones.
*   (vi) In HS case 12 FIFO locations should be reserved for internal DMA registers
*        so total FIFO size should be 1012 Only instead of 1024       
*******************************************************************************/
 
/****************** USB OTG HS CONFIGURATION **********************************/
#ifdef USB_OTG_HS_CORE
 #define RX_FIFO_HS_SIZE                          512
 #define TX0_FIFO_HS_SIZE                         128
 #define TX1_FIFO_HS_SIZE                         240
 #define TX2_FIFO_HS_SIZE                         128
 #define TX3_FIFO_HS_SIZE                           0
 #define TX4_FIFO_HS_SIZE                           0
 #define TX5_FIFO_HS_SIZE                           0

/* #define USB_OTG_HS_SOF_OUTPUT_ENABLED */

 #ifdef USE_ULPI_PHY
  #define USB_OTG_ULPI_PHY_ENABLED
 #endif
 #ifdef USE_EMBEDDED_PHY 
   #define USB_OTG_EMBEDDED_PHY_ENABLED
   /* wakeup is working only when HS core is configured in FS mode */
   /* #define USB_OTG_HS_LOW_PWR_MGMT_SUPPORT */
 #endif
 /* #define USB_OTG_HS_INTERNAL_DMA_ENABLED */ /* Be aware that enabling DMA mode will result in data being sent only by
                                                  multiple of 4 packet sizes. This is due to the fact that USB DMA does
                                                  not allow sending data from non word-aligned addresses.
                                                  For this specific application, it is advised to not enable this option
                                                  unless required. */
 #define USB_OTG_HS_DEDICATED_EP1_ENABLED
#endif

/****************** USB OTG FS CONFIGURATION **********************************/
#ifdef USB_OTG_FS_CORE
 #define RX_FIFO_FS_SIZE                          128
 #define TX0_FIFO_FS_SIZE                          32
 #define TX1_FIFO_FS_SIZE                         128
 #define TX2_FIFO_FS_SIZE                          32 
 #define TX3_FIFO_FS_SIZE                           0

//#define USB_OTG_FS_LOW_PWR_MGMT_SUPPORT
/* #define USB_OTG_FS_SOF_OUTPUT_ENABLED */
#endif

/****************** USB OTG MISC CONFIGURATION ********************************/
//#if defined(USE_STM324x9I_EVAL)
/* #define VBUS_SENSING_ENABLED */
   /* VBUS sensing is disabled because in the USART RX/TX pins are shared with USB Pins
      PA9/PA10 in the STM324x9I_EVAL */
//#else
//#define VBUS_SENSING_ENABLED
//#endif

#define USE_DEVICE_MODE


#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
#define __ALIGN_END    __attribute__ ((aligned (4)))
#define __ALIGN_BEGIN
#define __packed    __attribute__ ((__packed__))
#else
#define __ALIGN_END
#define __ALIGN_BEGIN
#define __packed    __attribute__ ((__packed__))
#endif
