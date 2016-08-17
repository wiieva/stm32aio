/*-----------------------------------------------------------------------*/
/* MMC/SDSC/SDHC (in SPI mode) control module for STM32 Version 1.1.6    */
/* (C) Martin Thomas, 2010 - based on the AVR MMC module (C)ChaN, 2007   */
/*-----------------------------------------------------------------------*/

/* Copyright (c) 2010, Martin Thomas, ChaN
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. */


#include "stm32f10x.h"
#include "diskio.h"
#include "stdbool.h"


#define STM32_SD_USE_DMA


 #define SOCKET_CP_CONNECTED      0 /* card-present socket-switch */
 #define GPIO_CP                  GPIOB
 #define RCC_APBxPeriph_GPIO_CP   RCC_APB2Periph_GPIOB
 #define GPIO_Pin_CP              GPIO_Pin_7
 #define GPIO_Mode_CP             GPIO_Mode_IN_FLOATING /* external resistor */
 #define SPI_SD                   SPI1
 #define GPIO_CS                  GPIOC
 #define RCC_APB2Periph_GPIO_CS   RCC_APB2Periph_GPIOC
 #define GPIO_Pin_CS              GPIO_Pin_15
 #define DMA_Channel_SPI_SD_RX    DMA1_Channel2
 #define DMA_Channel_SPI_SD_TX    DMA1_Channel3
 #define DMA_FLAG_SPI_SD_TC_RX    DMA1_FLAG_TC2
 #define DMA_FLAG_SPI_SD_TC_TX    DMA1_FLAG_TC3
 #define GPIO_SPI_SD              GPIOB
 #define GPIO_Pin_SPI_SD_SCK      GPIO_Pin_3
 #define GPIO_Pin_SPI_SD_MISO     GPIO_Pin_4
 #define GPIO_Pin_SPI_SD_MOSI     GPIO_Pin_5
 #define RCC_APBPeriphClockCmd_SPI_SD  RCC_APB1PeriphClockCmd
 #define RCC_APB2Periph_SPI_SD     RCC_APB2Periph_SPI1
 #define SPI_BaudRatePrescaler_SPI_SD  SPI_BaudRatePrescaler_4


/* Definitions for MMC/SDC command */
#define CMD0	(0x40+0)	/* GO_IDLE_STATE */
#define CMD1	(0x40+1)	/* SEND_OP_COND (MMC) */
#define ACMD41	(0xC0+41)	/* SEND_OP_COND (SDC) */
#define CMD8	(0x40+8)	/* SEND_IF_COND */
#define CMD9	(0x40+9)	/* SEND_CSD */
#define CMD10	(0x40+10)	/* SEND_CID */
#define CMD12	(0x40+12)	/* STOP_TRANSMISSION */
#define CMD13	(0xC0+13)	/* SD_STATUS (SDC) */
#define ACMD13	(0xC0+13)	/* SD_STATUS (SDC) */
#define CMD16	(0x40+16)	/* SET_BLOCKLEN */
#define CMD17	(0x40+17)	/* READ_SINGLE_BLOCK */
#define CMD18	(0x40+18)	/* READ_MULTIPLE_BLOCK */
#define CMD23	(0x40+23)	/* SET_BLOCK_COUNT (MMC) */
#define ACMD23	(0xC0+23)	/* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24	(0x40+24)	/* WRITE_BLOCK */
#define CMD25	(0x40+25)	/* WRITE_MULTIPLE_BLOCK */
#define CMD55	(0x40+55)	/* APP_CMD */
#define CMD58	(0x40+58)	/* READ_OCR */

/* Card-Select Controls  (Platform dependent) */
#define SELECT()        GPIO_ResetBits(GPIO_CS, GPIO_Pin_CS)    /* MMC CS = L */
#define DESELECT()      GPIO_SetBits(GPIO_CS, GPIO_Pin_CS)      /* MMC CS = H */

/* SPI cycles in 1 ms. Must be corrected, if SPI clock will be changed */
#define MS_SPI_CYCLES 1000

#if 0
#include "usart.h"
#define DEBUG ESP_USART_Debug_Printf
void disk_test ()
{
    uint8_t buf[4096];
    int res;
    uint32_t sectors;
    ESP_USART_Init(115200);

    res = disk_initialize(0);
    DEBUG ("disk initalize ret=%d\r\n",res);

    res = disk_ioctl(0,GET_SECTOR_COUNT,&sectors);
    DEBUG ("ioctl disk sectors=%d ret=%d\r\n",sectors,res);

    int sect;
    int rd_err = 0;
    sectors  = 1000000;

    for (sect = 8; sect < sectors; sect += 8) {
        res = disk_write(0,buf,sect,8);
        if (res) {
            DEBUG ("write error!!!\r\n");
            rd_err++;
        }
        if (!(sect % 128))
            DEBUG ("Write %d sectors,errors=%d\r\n",sect,rd_err);
    }
    for (sect = 8; sect < sectors; sect += 8) {
        res = disk_read(0,buf,sect,8);
        if (res) {
            DEBUG ("read error!!!\r\n");
            rd_err++;
        }
        if (!(sect % 128))
            DEBUG ("Read %d sectors,errors=%d\r\n",sect,rd_err);
    }
}
#endif



/*--------------------------------------------------------------------------
   Module Private Functions and Variables

---------------------------------------------------------------------------*/

static const uint32_t socket_state_mask_cp = (1 << 1);

static volatile
DSTATUS Stat = STA_NOINIT;	/* Disk status */

static
uint8_t CardType;			/* Card type flags */

enum speed_setting { INTERFACE_SLOW, INTERFACE_FAST };

static void interface_speed( enum speed_setting speed )
{
	uint32_t tmp;

	tmp = SPI_SD->CR1;
	if ( speed == INTERFACE_SLOW ) {
		/* Set slow clock (100k-400k) */
		tmp = ( tmp | SPI_BaudRatePrescaler_256 );
	} else {
		/* Set fast clock (depends on the CSD) */
		tmp = ( tmp & ~SPI_BaudRatePrescaler_256 ) | SPI_BaudRatePrescaler_SPI_SD;
	}
	SPI_SD->CR1 = tmp;
}


#if SOCKET_CP_CONNECTED
/* Socket's Card-Present Pin: high = socket empty, low = card inserted */

static
void socket_cp_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure I/O for card-present */
	RCC_APB2PeriphClockCmd(RCC_APBxPeriph_GPIO_CP, ENABLE);
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_CP;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_CP;
	GPIO_Init(GPIO_CP, &GPIO_InitStructure);
}

static inline uint32_t socket_is_empty(void)
{
	return ( GPIO_ReadInputData(GPIO_CP) & GPIO_Pin_CP ) ? false: socket_state_mask_cp;
}

#else

static void socket_cp_init(void)
{
	return;
}

static inline uint32_t socket_is_empty(void)
{
	return 1; /* fake inserted */
}

#endif /* SOCKET_CP_CONNECTED */



/*-----------------------------------------------------------------------*/
/* Transmit/Receive a byte to MMC via SPI  (Platform dependent)          */
/*-----------------------------------------------------------------------*/
static uint8_t stm32_spi_rw( uint8_t out )
{
	/* Loop while DR register in not empty */
    /// not needed: while (SPI_I2S_GetFlagStatus(SPI_SD, SPI_I2S_FLAG_TXE) == RESET) { ; }

	/* Send byte through the SPI peripheral */
	SPI_I2S_SendData(SPI_SD, out);

	/* Wait to receive a byte */
	while (SPI_I2S_GetFlagStatus(SPI_SD, SPI_I2S_FLAG_RXNE) == RESET) { ; }

	/* Return the byte read from the SPI bus */
	return SPI_I2S_ReceiveData(SPI_SD);
}



/*-----------------------------------------------------------------------*/
/* Transmit a byte to MMC via SPI  (Platform dependent)                  */
/*-----------------------------------------------------------------------*/

#define xmit_spi(dat)  stm32_spi_rw(dat)

/*-----------------------------------------------------------------------*/
/* Receive a byte from MMC via SPI  (Platform dependent)                 */
/*-----------------------------------------------------------------------*/

static inline
uint8_t rcvr_spi (void)
{
	return stm32_spi_rw(0xff);
}

/* Alternative macro to receive data fast */
#define rcvr_spi_m(dst)  *(dst)=stm32_spi_rw(0xff)



/*-----------------------------------------------------------------------*/
/* Wait for card ready                                                   */
/*-----------------------------------------------------------------------*/

static
uint8_t wait_ready (void)
{
	uint8_t res;

    int32_t timeout = 500 * MS_SPI_CYCLES;	/* Wait for ready in timeout of 500ms */
	rcvr_spi();
	do
		res = rcvr_spi();
    while ((res != 0xFF) && --timeout);

	return res;
}



/*-----------------------------------------------------------------------*/
/* Deselect the card and release SPI bus                                 */
/*-----------------------------------------------------------------------*/

static
void release_spi (void)
{
	DESELECT();
    rcvr_spi();
}

#ifdef STM32_SD_USE_DMA
/*-----------------------------------------------------------------------*/
/* Transmit/Receive Block using DMA (Platform dependent. STM32 here)     */
/*-----------------------------------------------------------------------*/
static
void stm32_dma_transfer(
	bool receive,		/* false for buff->SPI, true for SPI->buff               */
	const uint8_t *buff,	/* receive true  : 512 byte data block to be transmitted
						   receive false : Data buffer to store received data    */
    uint32_t btr 			/* receive true  : Byte count (must be multiple of 2)
						   receive false : Byte count (must be 512)              */
)
{
	DMA_InitTypeDef DMA_InitStructure;
	uint16_t rw_workbyte[] = { 0xffff };

	/* shared DMA configuration values */
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(SPI_SD->DR));
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_BufferSize = btr;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

	DMA_DeInit(DMA_Channel_SPI_SD_RX);
	DMA_DeInit(DMA_Channel_SPI_SD_TX);

	if ( receive ) {

		/* DMA1 channel2 configuration SPI1 RX ---------------------------------------------*/
		/* DMA1 channel4 configuration SPI2 RX ---------------------------------------------*/
		DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)buff;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_Init(DMA_Channel_SPI_SD_RX, &DMA_InitStructure);

		/* DMA1 channel3 configuration SPI1 TX ---------------------------------------------*/
		/* DMA1 channel5 configuration SPI2 TX ---------------------------------------------*/
		DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)rw_workbyte;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
		DMA_Init(DMA_Channel_SPI_SD_TX, &DMA_InitStructure);

	} else {

		/* DMA1 channel2 configuration SPI1 RX ---------------------------------------------*/
		/* DMA1 channel4 configuration SPI2 RX ---------------------------------------------*/
		DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)rw_workbyte;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
		DMA_Init(DMA_Channel_SPI_SD_RX, &DMA_InitStructure);

		/* DMA1 channel3 configuration SPI1 TX ---------------------------------------------*/
		/* DMA1 channel5 configuration SPI2 TX ---------------------------------------------*/
		DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)buff;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_Init(DMA_Channel_SPI_SD_TX, &DMA_InitStructure);

	}

	/* Enable DMA RX Channel */
	DMA_Cmd(DMA_Channel_SPI_SD_RX, ENABLE);
	/* Enable DMA TX Channel */
	DMA_Cmd(DMA_Channel_SPI_SD_TX, ENABLE);

	/* Enable SPI TX/RX request */
	SPI_I2S_DMACmd(SPI_SD, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, ENABLE);

	/* Wait until DMA1_Channel 3 Transfer Complete */
	/// not needed: while (DMA_GetFlagStatus(DMA_FLAG_SPI_SD_TC_TX) == RESET) { ; }
	/* Wait until DMA1_Channel 2 Receive Complete */
	while (DMA_GetFlagStatus(DMA_FLAG_SPI_SD_TC_RX) == RESET) { ; }
	// same w/o function-call:
	// while ( ( ( DMA1->ISR ) & DMA_FLAG_SPI_SD_TC_RX ) == RESET ) { ; }

	/* Disable DMA RX Channel */
	DMA_Cmd(DMA_Channel_SPI_SD_RX, DISABLE);
	/* Disable DMA TX Channel */
	DMA_Cmd(DMA_Channel_SPI_SD_TX, DISABLE);

	/* Disable SPI RX/TX request */
	SPI_I2S_DMACmd(SPI_SD, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, DISABLE);
}
#endif /* STM32_SD_USE_DMA */


/*-----------------------------------------------------------------------*/
/* Power Control and interface-initialization (Platform dependent)       */
/*-----------------------------------------------------------------------*/

static
void power_on (void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	volatile uint8_t dummyread;

	/* Enable SPI clock, SPI1: APB2, SPI2: APB1 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI_SD, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SPI1,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);

	socket_cp_init();

	/* Configure I/O for Flash Chip select */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_CS;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIO_CS, &GPIO_InitStructure);

	/* De-select the Card: Chip Select high */
	DESELECT();

	/* Configure SPI pins: SCK and MOSI with default alternate function (not re-mapped) push-pull */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_SPI_SD_SCK | GPIO_Pin_SPI_SD_MOSI;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_Init(GPIO_SPI_SD, &GPIO_InitStructure);
	/* Configure MISO as Input with internal pull-up */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_SPI_SD_MISO;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
	GPIO_Init(GPIO_SPI_SD, &GPIO_InitStructure);

	/* SPI configuration */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; // 72000kHz/256=281kHz < 400kHz
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	SPI_Init(SPI_SD, &SPI_InitStructure);
	SPI_CalculateCRC(SPI_SD, DISABLE);
	SPI_Cmd(SPI_SD, ENABLE);

	/* drain SPI */
	while (SPI_I2S_GetFlagStatus(SPI_SD, SPI_I2S_FLAG_TXE) == RESET) { ; }
	dummyread = SPI_I2S_ReceiveData(SPI_SD);

#ifdef STM32_SD_USE_DMA
	/* enable DMA clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
#endif
}

static
void power_off (void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	if (!(Stat & STA_NOINIT)) {
		SELECT();
		wait_ready();
		release_spi();
	}

	SPI_I2S_DeInit(SPI_SD);
	SPI_Cmd(SPI_SD, DISABLE);
	RCC_APB2PeriphClockCmd (RCC_APB2Periph_SPI_SD, DISABLE);

	/* All SPI-Pins to input with weak internal pull-downs */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_SPI_SD_SCK | GPIO_Pin_SPI_SD_MISO | GPIO_Pin_SPI_SD_MOSI;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPD;
	GPIO_Init(GPIO_SPI_SD, &GPIO_InitStructure);

	Stat |= STA_NOINIT;		/* Set STA_NOINIT */
}


/*-----------------------------------------------------------------------*/
/* Receive a data packet from MMC                                        */
/*-----------------------------------------------------------------------*/

static
uint8_t rcvr_datablock (
	uint8_t *buff,			/* Data buffer to store received data */
	unsigned btr			/* Byte count (must be multiple of 4) */
)
{
	uint8_t token;

    int timeout = 100 * MS_SPI_CYCLES;
	do {							/* Wait for data packet in timeout of 100ms */
		token = rcvr_spi();
    } while ((token == 0xFF) && --timeout);
    if(token != 0xFE) return 1;	/* If not valid data token, return with error */

#ifdef STM32_SD_USE_DMA
	stm32_dma_transfer( true, buff, btr );
#else
	do {							/* Receive the data block into buffer */
		rcvr_spi_m(buff++);
		rcvr_spi_m(buff++);
		rcvr_spi_m(buff++);
		rcvr_spi_m(buff++);
	} while (btr -= 4);
#endif /* STM32_SD_USE_DMA */

	rcvr_spi();						/* Discard CRC */
	rcvr_spi();

    return 0;					/* Return with success */
}



/*-----------------------------------------------------------------------*/
/* Send a data packet to MMC                                             */
/*-----------------------------------------------------------------------*/
static
uint8_t xmit_datablock (
	const uint8_t *buff,	/* 512 byte data block to be transmitted */
	uint8_t token			/* Data/Stop token */
)
{
	uint8_t resp;
#ifndef STM32_SD_USE_DMA
	uint8_t wc;
#endif

	if (wait_ready() != 0xFF) return false;

	xmit_spi(token);					/* transmit data token */
	if (token != 0xFD) {	/* Is data token */

#ifdef STM32_SD_USE_DMA
		stm32_dma_transfer( false, buff, 512 );
#else
		wc = 0;
		do {							/* transmit the 512 byte data block to MMC */
			xmit_spi(*buff++);
			xmit_spi(*buff++);
		} while (--wc);
#endif /* STM32_SD_USE_DMA */

		xmit_spi(0xFF);					/* CRC (Dummy) */
		xmit_spi(0xFF);
		resp = rcvr_spi();				/* Receive data response */
		if ((resp & 0x1F) != 0x05)		/* If not accepted, return with error */
            return 1;
	}

    return 0;
}



/*-----------------------------------------------------------------------*/
/* Send a command packet to MMC                                          */
/*-----------------------------------------------------------------------*/
static
uint8_t send_cmd (
	uint8_t cmd,		/* Command byte */
	uint32_t arg		/* Argument */
)
{
	uint8_t n, res;

	if (cmd & 0x80) {	/* ACMD<n> is the command sequence of CMD55-CMD<n> */
		cmd &= 0x7F;
        res = send_cmd(CMD55, 0);
        if (res > 1) return res;
	}

	/* Select the card and wait for ready */
	SELECT();
	if (wait_ready() != 0xFF) {
        return 0xFF;
	}

	/* Send command packet */
	xmit_spi(cmd);						/* Start + Command index */
	xmit_spi((uint8_t)(arg >> 24));		/* Argument[31..24] */
	xmit_spi((uint8_t)(arg >> 16));		/* Argument[23..16] */
	xmit_spi((uint8_t)(arg >> 8));			/* Argument[15..8] */
	xmit_spi((uint8_t)arg);				/* Argument[7..0] */
    n = 0xff;							/* Dummy CRC + Stop */
	if (cmd == CMD0) n = 0x95;			/* Valid CRC for CMD0(0) */
	if (cmd == CMD8) n = 0x87;			/* Valid CRC for CMD8(0x1AA) */
	xmit_spi(n);

	/* Receive command response */
    if (cmd == CMD12) rcvr_spi();		/* Skip a stuff byte when stop reading */

    n = 10;							/* Wait for a valid response in timeout of 10 attempts */
	do
		res = rcvr_spi();
    while ((res & 0x80) && --n);

	return res;			/* Return with the response value */
}



/*--------------------------------------------------------------------------

   Public Functions

---------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
/* Initialize Disk Drive                                                 */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	uint8_t drv		/* Physical drive number (0) */
)
{
	uint8_t n, cmd, ty, ocr[4];

	if (drv) return STA_NOINIT;			/* Supports only single drive */
	if (Stat & STA_NODISK) return Stat;	/* No card in the socket */

	power_on();							/* Force socket power on and initialize interface */
	interface_speed(INTERFACE_SLOW);

    volatile int i;
    for (i = 0; i < 3000000; ++i);

    for (n = 10; n; n--) rcvr_spi();	/* 80 dummy clocks */

	ty = 0;
    int count = 10000;
	if (send_cmd(CMD0, 0) == 1) {			/* Enter Idle state */
		if (send_cmd(CMD8, 0x1AA) == 1) {	/* SDHC */
			for (n = 0; n < 4; n++) ocr[n] = rcvr_spi();		/* Get trailing return value of R7 response */
			if (ocr[2] == 0x01 && ocr[3] == 0xAA) {				/* The card can work at VDD range of 2.7-3.6V */
                while (--count && send_cmd(ACMD41, 1UL << 30));	/* Wait for leaving idle state (ACMD41 with HCS bit) */
                if (count && send_cmd(CMD58, 0) == 0) {		/* Check CCS bit in the OCR */
					for (n = 0; n < 4; n++) ocr[n] = rcvr_spi();
					ty = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;
				}
			}
		} else {							/* SDSC or MMC */
			if (send_cmd(ACMD41, 0) <= 1) 	{
				ty = CT_SD1; cmd = ACMD41;	/* SDSC */
			} else {
				ty = CT_MMC; cmd = CMD1;	/* MMC */
			}
            while (--count && send_cmd(cmd, 0));			/* Wait for leaving idle state */
            if (!count || send_cmd(CMD16, 512) != 0)	/* Set R/W block length to 512 */
				ty = 0;
		}
	}
	CardType = ty;
	release_spi();

	if (ty) {			/* Initialization succeeded */
		Stat &= ~STA_NOINIT;		/* Clear STA_NOINIT */
		interface_speed(INTERFACE_FAST);
	} else {			/* Initialization failed */
		power_off();
	}

	return Stat;
}



/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	uint8_t drv		/* Physical drive number (0) */
)
{
	if (drv) return STA_NOINIT;		/* Supports only single drive */
	return Stat;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	uint8_t drv,			/* Physical drive number (0) */
	uint8_t *buff,			/* Pointer to the data buffer to store read data */
	uint32_t sector,		/* Start sector number (LBA) */
	uint8_t count			/* Sector count (1..255) */
)
{
    int err = 1,_count = count;

    if (drv) return RES_PARERR;
	if (Stat & STA_NOINIT) return RES_NOTRDY;
    if (!count) return RES_OK;

	if (!(CardType & CT_BLOCK)) sector *= 512;	/* Convert to byte address if needed */

    for (;;) {
        if (send_cmd(CMD18, sector))
            break;
        do {
            if (rcvr_datablock(buff, 512))
                break;
            buff += 512;
        } while (--count);
        if (count)
            break;
        send_cmd(CMD12, 0);
        if (wait_ready() == 0xFF)
            err = 0;
        break;
    }

    release_spi();

//    static int rd_errcount;
//    if (err)
//        rd_errcount++;
//    DEBUG ("SD: Read sector %d,count %d,res %d, (total errors=%d)\r\n",sector,_count,err,rd_errcount);

    return err ? RES_ERROR : RES_OK;
}


/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

DRESULT disk_write (
	uint8_t drv,			/* Physical drive number (0) */
	const uint8_t *buff,	/* Pointer to the data to be written */
	uint32_t sector,		/* Start sector number (LBA) */
	uint8_t count			/* Sector count (1..255) */
)
{
    int err = 1,_count = count;
    if (drv) return RES_PARERR;
	if (Stat & STA_NOINIT) return RES_NOTRDY;
    if (!count) return RES_OK;

	if (!(CardType & CT_BLOCK)) sector *= 512;	/* Convert to byte address if needed */

    for (;;) {

        if (send_cmd(ACMD23, count))
            break;
        if (send_cmd(CMD25, sector))
            break;
        do {
            if (xmit_datablock(buff, 0xFC))
                break;
            buff += 512;
        } while (--count);
        if (count)
            break;
        if (wait_ready() != 0xFF)
            break;
        xmit_spi(0xFD);
        if (wait_ready() == 0xFF)
            err =0;
        break;
    }
	release_spi();

//    static in wr_errcount;
//    if (err)
//        wr_errcount++;
//    DEBUG ("SD: Write sector %d,count %d,res %d, (total errors=%d)\r\n",sector,_count,err,wr_errcount);

    return err ? RES_ERROR : RES_OK;
}


DRESULT disk_ioctl (
	uint8_t drv,		/* Physical drive number (0) */
	uint8_t ctrl,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT res;
	uint8_t n, csd[16], *ptr = buff;
	uint16_t csize;

	if (drv) return RES_PARERR;

	res = RES_ERROR;

	if (ctrl == CTRL_POWER) {
			res = RES_PARERR;
	}
	else {
		if (Stat & STA_NOINIT) return RES_NOTRDY;

		switch (ctrl) {
		case CTRL_SYNC :		/* Make sure that no pending write process */
			SELECT();
			if (wait_ready() == 0xFF)
				res = RES_OK;
			break;

		case GET_SECTOR_COUNT :	/* Get number of sectors on the disk (uint32_t) */
            if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16) ==0) {
				if ((csd[0] >> 6) == 1) {	/* SDC version 2.00 */
					csize = csd[9] + ((uint16_t)csd[8] << 8) + 1;
					*(uint32_t*)buff = (uint32_t)csize << 10;
				} else {					/* SDC version 1.XX or MMC*/
					n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
                    csize = (csd[8] >> 6) + ((uint16_t)csd[7] << 2) + ((uint16_t)(csd[6] & 3) << 10) + 1;
					*(uint32_t*)buff = (uint32_t)csize << (n - 9);
				}
				res = RES_OK;
			}
			break;

		case GET_SECTOR_SIZE :	/* Get R/W sector size (uint16_t) */
			*(uint16_t*)buff = 512;
			res = RES_OK;
			break;

		case GET_BLOCK_SIZE :	/* Get erase block size in unit of sector (uint32_t) */
			if (CardType & CT_SD2) {	/* SDC version 2.00 */
				if (send_cmd(ACMD13, 0) == 0) {	/* Read SD status */
					rcvr_spi();
                    if (rcvr_datablock(csd, 16) ==0) {				/* Read partial block */
						for (n = 64 - 16; n; n--) rcvr_spi();	/* Purge trailing data */
						*(uint32_t*)buff = 16UL << (csd[10] >> 4);
						res = RES_OK;
					}
				}
			} else {					/* SDC version 1.XX or MMC */
                if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16) ==0) {	/* Read CSD */
					if (CardType & CT_SD1) {	/* SDC version 1.XX */
						*(uint32_t*)buff = (((csd[10] & 63) << 1) + ((uint16_t)(csd[11] & 128) >> 7) + 1) << ((csd[13] >> 6) - 1);
					} else {					/* MMC */
						*(uint32_t*)buff = ((uint16_t)((csd[10] & 124) >> 2) + 1) * (((csd[11] & 3) << 3) + ((csd[11] & 224) >> 5) + 1);
					}
					res = RES_OK;
				}
			}
			break;

		case MMC_GET_TYPE :		/* Get card type flags (1 byte) */
			*ptr = CardType;
			res = RES_OK;
			break;

		case MMC_GET_CSD :		/* Receive CSD as a data block (16 bytes) */
			if (send_cmd(CMD9, 0) == 0		/* READ_CSD */
                && rcvr_datablock(ptr, 16) ==0)
				res = RES_OK;
			break;

		case MMC_GET_CID :		/* Receive CID as a data block (16 bytes) */
			if (send_cmd(CMD10, 0) == 0		/* READ_CID */
                && rcvr_datablock(ptr, 16)==0)
				res = RES_OK;
			break;

		case MMC_GET_OCR :		/* Receive OCR as an R3 resp (4 bytes) */
			if (send_cmd(CMD58, 0) == 0) {	/* READ_OCR */
				for (n = 4; n; n--) *ptr++ = rcvr_spi();
				res = RES_OK;
			}
			break;

		case MMC_GET_SDSTAT :	/* Receive SD status as a data block (64 bytes) */
			if (send_cmd(ACMD13, 0) == 0) {	/* SD_STATUS */
				rcvr_spi();
				if (rcvr_datablock(ptr, 64))
					res = RES_OK;
			}
			break;

		default:
			res = RES_PARERR;
		}

		release_spi();
	}

	return res;
}
