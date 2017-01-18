#pragma once
#include "circular_buffer.h"

void ESP_Speaker_Init ();
void ESP_Speaker_Start (int sampleRate,int mp3out);
void ESP_Speaker_Stop ();
void ESP_Speaker_Decode_Run ();
circular_buffer *ESP_Speaker_Buffer () ;

extern int audio_out_volume;
