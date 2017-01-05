

#include <stm32f10x.h>

#include "esp8266_ctl.h"
#include "arduino_wiring.h"
#include "espeva_stmboard.h"
#include "tools.h"

volatile int espAutoResetDetected = 0;
static unsigned long pwr_pressed_systick_ms = 0;
static int esp8266_reset_state = ESP8266_ResetStop;

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
    GPIO_Init(ESP_GPIO0_PORT, &gpio);

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

    if (bklog_ptr >= 10)
        bklog_ptr = 0;

    bklog[bklog_ptr++] = state;

    if ((B(3) == 2 && B(2) == 3 && B(1) == 2) ||
        (B(4) == 2 && /*bklog[3] == 768 &&*/ B(2) == 1 && B(1) == 0))
        espAutoResetDetected = 1;
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

    if (esp8266_reset_state == ESP8266_ResetStop) {
        ESP_CTL_DoResetESP (ESP8266_ResetRun);
    } else {
        ESP_CTL_DoResetESP (ESP8266_ResetStop);
        DelayMs (5);
        ESP_Wiring_OffAll();
    }
}

void ESP_CTL_Run () {
    if (espAutoResetDetected) {
         espAutoResetDetected =0;
        ESP_CTL_DoResetESP (ESP8266_ResetFlash);
    }
    ESP_CTL_CheckPowerKB ();
}
