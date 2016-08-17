#pragma once

#include "circular_buffer.h"

void ESP_Mic_Init ();
void ESP_Mic_Start (int sampleRate,int _speex_mode);
void ESP_Mic_Stop ();
void ESP_Mic_Encode_Run ();
circular_buffer *ESP_Mic_Buffer ();
