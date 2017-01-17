#pragma once
#include "usb_conf.h"

#define USBD_CFG_MAX_NUM                1
#define USBD_ITF_MAX_NUM                1

#define USBD_SELF_POWERED               

#define USB_MAX_STR_DESC_SIZ            255 

#define CDC_IN_EP                       0x81  /* EP1 for data IN */
#define CDC_OUT_EP                      0x01  /* EP1 for data OUT */
#define CDC_CMD_EP                      0x82  /* EP2 for CDC commands */

/* CDC Endpoints parameters: you can fine tune these values depending on the needed baudrates and performance. */
#ifdef USE_USB_OTG_HS
 #define CDC_DATA_MAX_PACKET_SIZE       512  /* Endpoint IN & OUT Packet size */
 #define CDC_CMD_PACKET_SZE             8    /* Control Endpoint Packet size */

 #define CDC_IN_FRAME_INTERVAL          40   /* Number of micro-frames between IN transfers */
 #define APP_RX_DATA_SIZE               2048 /* Total size of IN buffer: 
                                                APP_RX_DATA_SIZE*8/MAX_BAUDARATE*1000 should be > CDC_IN_FRAME_INTERVAL*8 */
#else
 #define CDC_DATA_MAX_PACKET_SIZE       32   /* Endpoint IN & OUT Packet size */
 #define CDC_CMD_PACKET_SZE             8    /* Control Endpoint Packet size */

 #define CDC_IN_FRAME_INTERVAL          5    /* Number of frames between IN transfers */
 #define APP_RX_DATA_SIZE               512  /* Total size of IN buffer: 
                                                APP_RX_DATA_SIZE*8/MAX_BAUDARATE*1000 should be > CDC_IN_FRAME_INTERVAL */
#endif /* USE_USB_OTG_HS */

#define APP_FOPS                        VCP_fops

#define MSC_IN_EP                    0x81
#define MSC_OUT_EP                   0x01
#ifdef USE_USB_OTG_HS
#ifdef USE_ULPI_PHY
#define MSC_MAX_PACKET               512
#else
#define MSC_MAX_PACKET               64
#endif
#else  /*USE_USB_OTG_FS*/
#define MSC_MAX_PACKET                64
#endif

#define MSC_MEDIA_PACKET             4096


