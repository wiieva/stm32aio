#include "stm32f10x.h"
#include "usbd_cdc_core.h"
#include "usbd_msc_core.h"
#include "usbd_usr.h"
#include "usb_conf.h"
#include "usbd_desc.h"
#include "espeva_stmboard.h"
#include "ir_send.h"
#include "aio_server.h"
#include "aioiface.h"
#include "ui_input.h"
#include "arduino_wiring.h"
#include "audio_in.h"
#include "audio_out.h"
#include "esp8266_ctl.h"
#include "diskio.h"
#include "usart.h"
#include "bme280.h"
#include "i2c.h"

#define _USART_DEBUG_

__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END;
int usb_init_done;

void Set_System(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD|RCC_APB2Periph_AFIO, ENABLE);
    RCC_AHBPeriphClockCmd (RCC_AHBPeriph_DMA1,ENABLE);
    RCC_ADCCLKConfig (RCC_PCLK2_Div8);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
}

typedef void (*bootjmpfn)(void);
void Enter_DFU () {
    bootjmpfn bootjump = (bootjmpfn) (*((uint32_t*)0x1fffb004)) ;
    RCC_DeInit ();
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
    __set_MSP (0x20001000);
    bootjump ();
}

int main(void)
{
    // DFU stuff
    RCC_AHBPeriphResetCmd (RCC_AHBPeriph_OTG_FS,ENABLE);
    RCC_AHBPeriphResetCmd (RCC_AHBPeriph_OTG_FS,DISABLE);
    usb_init_done = 1;

    if (espDfuModeMagic == ESP_DFU_MODE_MAGIC) {
        espDfuModeMagic = 0;
        Enter_DFU ();
    }
    Set_System();
    // Configure systick for 1 ms tick
    SysTick_Config(CPUFREQ_KHZ);
    ESP_CTL_DoResetESP (ESP8266_ResetStop);
    DelayMs (5);

    BME280_Init ();
    // Enable touch input for check boot mode
    ESP_UI_Input_Init();
    ESP_UI_Input_GetSysInfo ();

    // If key pressed, then enable USB mass storage mode
    if (sys_info.boot_mode) {
        USBD_Init(&USB_OTG_dev, USB_OTG_FS_CORE_ID, &USR_msc_desc, &USBD_MSC_cb, &USR_cb);
        USB_OTG_SetCurrentMode(&USB_OTG_dev, DEVICE_MODE);
        for(;;);
    }

    USBD_Init(&USB_OTG_dev, USB_OTG_FS_CORE_ID, &USR_cdc_desc, &USBD_CDC_cb, &USR_cb);
    USB_OTG_SetCurrentMode(&USB_OTG_dev, DEVICE_MODE);

#ifdef _USART_DEBUG_
    ESP_USART_Init(115200);
    ESP_USART_Debug_Printf("---------------------------------\r\n");
    ESP_USART_Debug_Printf("stm32aio started\r\n");
#endif

    ESP_Mic_Init ();
    ESP_Speaker_Init ();
    ESP_IRSend_Init ();
    ESP_Wiring_Init ();
    AIO_Server_Init ();

    ESP_CTL_DoResetESP (ESP8266_ResetRun);

    while (1) {
        ESP_CTL_Run ();
        ESP_Mic_Encode_Run ();
        ESP_IRSend_Run();
        ESP_Speaker_Decode_Run ();
        ESP_UI_Input_Run ();
        ESP_BME280_Run ();
    }
}
