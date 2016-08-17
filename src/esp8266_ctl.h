#pragma once

enum {
    ESP8266_ResetStop,
    ESP8266_ResetRun,
    ESP8266_ResetFlash
};


void ESP_CTL_DoResetESP (int mode);
void ESP_CTL_Run ();
void ESP_CTL_Modem_SetLineState (uint16_t state);
