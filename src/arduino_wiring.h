#pragma once

void ESP_Wiring_Init ();
uint16_t ESP_Wiring_DigitalRead (uint16_t pin);
void ESP_Wiring_DigitalWrite (uint16_t pin,uint16_t val);
void ESP_Wiring_PinMode (uint16_t pin,uint16_t mode);
uint16_t ESP_Wiring_AnalogRead (uint16_t pin);
void ESP_Wiring_AnalogWrite (uint16_t pin,uint16_t val);
void ESP_Wiring_Init ();
void ESP_Wiring_ADC_Start ();


