#pragma once

#include "circular_buffer.h"
extern circular_buffer ir_sym_buf,ir_raw_buf;

void ESP_IRSend_Init ();
void ESP_IRSend_Run ();


void ir_sendSony(unsigned long data, int nbits);
void ir_sendNEC(unsigned long data, int nbits);
void ir_sendWhynter(unsigned long data, int nbits);
void ir_sendRC5(unsigned long data, int nbits);
void ir_sendRC6(unsigned long data, int nbits);
void ir_sendPanasonic(unsigned int address, unsigned long data);
void ir_sendJVC(unsigned long data, int nbits, int repeat);
void ir_sendSAMSUNG(unsigned long data, int nbits);
void ir_sendRaw(unsigned int buf[], int len, int hz);


