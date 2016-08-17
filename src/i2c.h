#pragma once

#include <stm32f10x.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>


// Send or not STOP condition
typedef enum {
	I2C_STOP   = 0,
	I2C_NOSTOP = !I2C_STOP
} I2C_STOP_TypeDef;


// I2C defines
#define I2C_WAIT_TIMEOUT  (uint32_t)0x00010000 // Timeout for I2C operations


// Function prototypes
ErrorStatus I2Cx_Init(I2C_TypeDef* I2Cx, uint32_t Clock);
ErrorStatus I2Cx_Write(I2C_TypeDef* I2Cx, const uint8_t* buf, uint32_t nbytes,uint8_t SlaveAddress, I2C_STOP_TypeDef stop);
ErrorStatus I2Cx_Read(I2C_TypeDef* I2Cx, uint8_t *buf, uint32_t nbytes, uint8_t SlaveAddress);
ErrorStatus I2Cx_IsDeviceReady(I2C_TypeDef* I2Cx, uint8_t SlaveAddress, uint32_t Trials);
