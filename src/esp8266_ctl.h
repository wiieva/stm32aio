#pragma once

enum {
    ESP8266_ResetStop,
    ESP8266_ResetRun,
    ESP8266_ResetFlash
};


void ESP_CTL_DoResetESP (int mode);
void ESP_CTL_Run ();
void ESP_CTL_Modem_SetLineState (uint16_t state);
void ESP_CTL_Modem_TxBuf (uint8_t *Buf,uint32_t len);

#define espDfuModeMagic (*((volatile uint32_t *)0x2000F000))

#define ESP_DFU_MODE_MAGIC 0xDEADBEEF