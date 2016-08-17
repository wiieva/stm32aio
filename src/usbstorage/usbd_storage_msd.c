#include "usbd_msc_mem.h"
//#include "spi_sd.h"
#include "diskio.h"

#define STORAGE_LUN_NBR                  1

/* USB Mass storage Standard Inquiry Data */
const int8_t  STORAGE_Inquirydata[] = {//36
  
  /* LUN 0 */
  0x00,		
  0x80,		
  0x02,		
  0x02,
  (USBD_STD_INQUIRY_LENGTH - 5),
  0x00,
  0x00,	
  0x00,
  'S', 'T', 'M', ' ', ' ', ' ', ' ', ' ', /* Manufacturer : 8 bytes */
  'm', 'i', 'c', 'r', 'o', 'S', 'D', ' ', /* Product      : 16 Bytes */
  'F', 'l', 'a', 's', 'h', ' ', ' ', ' ',
  '1', '.', '0' ,'0',                     /* Version      : 4 Bytes */
}; 


static int8_t STORAGE_Init (uint8_t lun)
{
    return (disk_initialize(lun)!=0)?-1:0;
}

static int8_t STORAGE_GetCapacity (uint8_t lun, uint32_t *block_num, uint32_t *block_size)
{
    *block_size = 512;
    return (disk_ioctl(lun,GET_SECTOR_COUNT,block_num)!=RES_OK)?-1:0;
}

static int8_t STORAGE_IsReady (uint8_t lun)
{
    return (disk_status(lun) != 0)?-1:0;
}

static int8_t STORAGE_IsWriteProtected (uint8_t lun)
{
    return  0;
}

static int8_t STORAGE_Read (uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
    return ( disk_read(0,buf,blk_addr,blk_len) != RES_OK)?-1:0;
}

static int8_t STORAGE_Write (uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
    return ( disk_write (0,buf, blk_addr,  blk_len) != RES_OK)?-1:0;
}

static int8_t STORAGE_GetMaxLun (void)
{
  return (STORAGE_LUN_NBR - 1);
}

static USBD_STORAGE_cb_TypeDef USBD_MICRO_SDIO_fops =
{
    STORAGE_Init,
    STORAGE_GetCapacity,
    STORAGE_IsReady,
    STORAGE_IsWriteProtected,
    STORAGE_Read,
    STORAGE_Write,
    STORAGE_GetMaxLun,
    (int8_t *)STORAGE_Inquirydata,
};

USBD_STORAGE_cb_TypeDef  *USBD_STORAGE_fops = &USBD_MICRO_SDIO_fops;

