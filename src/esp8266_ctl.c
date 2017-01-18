

#include <stm32f10x.h>

#include "esp8266_ctl.h"
#include "arduino_wiring.h"
#include "espeva_stmboard.h"
#include "tools.h"
#include "usart.h"

volatile int espAutoResetDetected = 0, espDfuModeDetected = 0;
static unsigned long pwr_pressed_systick_ms = 0;
static int esp8266_reset_state = ESP8266_ResetStop;

void SYSCLKConfig_STOP(void) {
    RCC_HSEConfig(RCC_HSE_ON);

    if(RCC_WaitForHSEStartUp() == SUCCESS)
    {
         /* Enable PLL2 */
         RCC_PLL2Cmd(ENABLE);

         /* Wait till PLL2 is ready */
         while(RCC_GetFlagStatus(RCC_FLAG_PLL2RDY) == RESET) {}

         /* Enable PLL */
         RCC_PLLCmd(ENABLE);

         /* Wait till PLL is ready */
         while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){}

         /* Select PLL as system clock source */
         RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

         /* Wait till PLL is used as system clock source */
         while(RCC_GetSYSCLKSource() != 0x08){}
    }
}

void ESP_CTL_EnterStopMode () {
    EXTI_InitTypeDef exti;

    // PWR button
    exti.EXTI_Line = EXTI_Line0|EXTI_Line9;
    exti.EXTI_LineCmd = ENABLE;
    exti.EXTI_Mode = EXTI_Mode_Event;
    exti.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_Init (&exti);
    EXTI_ClearFlag(EXTI_Line0|EXTI_Line9);
    PWR_EnterSTOPMode (PWR_Regulator_LowPower,PWR_STOPEntry_WFE);
    SYSCLKConfig_STOP ();
    EXTI_DeInit ();
}


void ESP_CTL_DoResetESP (int mode)
{
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;

    /* Configure ESP.Reset pin */
    gpio.GPIO_Pin = ESP_RESET_PIN;
    GPIO_Init(ESP_RESET_PORT, &gpio);

    GPIO_WriteBit(ESP_RESET_PORT,ESP_RESET_PIN,1); // Reset
    esp8266_reset_state = mode;
    if (mode == ESP8266_ResetStop)
        return;

    DelayMs (5);
    ESP_Wiring_OffAll ();
    /* Configure ESP.GPIO0 pin */
    gpio.GPIO_Pin = ESP_GPIO0_PIN;
    GPIO_Init(ESP_GPIO0_PORT, &gpio);

    GPIO_WriteBit(ESP_GPIO0_PORT,ESP_GPIO0_PIN,(mode==ESP8266_ResetFlash)?0:1); // GPIO0 Low - Flash mode

    /* Configure ESP.GPIO2 pin */
    gpio.GPIO_Pin = ESP_GPIO2_PIN;
    GPIO_Init(ESP_GPIO2_PORT, &gpio);
    GPIO_WriteBit(ESP_GPIO2_PORT,ESP_GPIO2_PIN,1); // GPIO2 High

    /* Configure ESP.GPIO15 pin */
    gpio.GPIO_Pin = ESP_GPIO15_PIN;
    GPIO_Init(ESP_GPIO15_PORT, &gpio);
    GPIO_WriteBit(ESP_GPIO15_PORT,ESP_GPIO15_PIN,0); // GPIO15 Low

    // Wait for signal settles, and reset done
    DelayMs (30);

    GPIO_WriteBit(ESP_RESET_PORT,ESP_RESET_PIN,0); // RUN
    // Wait for ESP start and read boot config
    DelayMs (50);

    // Release ESP boot configuration pins
    gpio.GPIO_Pin = ESP_GPIO0_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_WriteBit(ESP_GPIO0_PORT,ESP_GPIO0_PIN,1); // GPIO0 
//    GPIO_Init(ESP_GPIO0_PORT, &gpio);

    gpio.GPIO_Pin = ESP_GPIO2_PIN;
    GPIO_Init(ESP_GPIO2_PORT, &gpio);

    gpio.GPIO_Pin = ESP_GPIO15_PIN;
    GPIO_Init(ESP_GPIO15_PORT, &gpio);
}

#define B(x) bklog[((bklog_ptr+10-x)%10)]
void ESP_CTL_Modem_SetLineState (uint16_t state) {
    /* RTS & DTR config */
    /* We are using node MCU schematic emulation */
    static uint16_t bklog[10],bklog_ptr=0;
    // ESP_USART_Debug_Printf("->%d\r\n",state);

    if (bklog_ptr >= 10)
        bklog_ptr = 0;

    bklog[bklog_ptr++] = state;

    if ((B(3) == 2 && B(2) == 3 && B(1) == 2) ||
        (B(4) == 2 && /*bklog[3] == 768 &&*/ B(2) == 1 && B(1) == 0) ||
        (B(4) == 0 &&  B(3) == 2 && B(2) == 1 && B(1) == 0)
        ) {
        espAutoResetDetected = 1;
        // ESP_USART_Debug_Printf("reset detected\r\n",state);
    }
}
void ESP_CTL_Modem_TxBuf (uint8_t *Buf,uint32_t Len) {

    static uint8_t bklog[10],bklog_ptr=0;
    while (Len--) {
        if (bklog_ptr >= 10)
            bklog_ptr = 0;

        bklog[bklog_ptr++] = *Buf++;
        if (B(5) == '!' && B(4) == 'D' && B(3) == 'f' && B(2) == 'U' && B(1) == '!') {
             espDfuModeDetected = 1;
        }
    }
}

void ESP_CTL_CheckPowerKB () {
    int pwr_pressed = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0);
    if (!pwr_pressed) {
        pwr_pressed_systick_ms = 0;
        return;
    }
    if (!pwr_pressed_systick_ms) {
        pwr_pressed_systick_ms = GetSysTickMS ();
        return;
    }

    unsigned long tout = (esp8266_reset_state == ESP8266_ResetStop)?200:3000;

    if (pwr_pressed_systick_ms == UINT32_MAX || GetSysTickMS() - pwr_pressed_systick_ms < tout)
        return;

    pwr_pressed_systick_ms = UINT32_MAX;

    if (esp8266_reset_state != ESP8266_ResetStop) {
        ESP_CTL_DoResetESP (ESP8266_ResetStop);
        DelayMs (5);
        ESP_Wiring_OffAll();
        ESP_CTL_EnterStopMode();
        ESP_CTL_DoResetESP (ESP8266_ResetRun);
    }
}

void ESP_CTL_Run () {
    if (espAutoResetDetected) {
        espAutoResetDetected = 0;
        ESP_CTL_DoResetESP (ESP8266_ResetFlash);
    } 
    if (espDfuModeDetected) {
        espDfuModeDetected = 0;
        VCP_Send_Buf ("DFU OK\n",7);
        DelayMs (500);
        espDfuModeMagic = ESP_DFU_MODE_MAGIC;
        NVIC_SystemReset(); 
    }

    ESP_CTL_CheckPowerKB ();
}
