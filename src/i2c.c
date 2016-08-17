#include "i2c.h"

ErrorStatus I2Cx_Init (I2C_TypeDef* I2Cx, uint32_t Clock)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    // Reset
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);

    I2C_InitTypeDef i2c;

    i2c.I2C_Mode = I2C_Mode_I2C;
    i2c.I2C_ClockSpeed = Clock;
    i2c.I2C_Mode = I2C_Mode_I2C;
    i2c.I2C_DutyCycle = I2C_DutyCycle_2;
    i2c.I2C_OwnAddress1 = 0x0;
    i2c.I2C_Ack = I2C_Ack_Disable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

    I2C_Init(I2Cx, &i2c);

    I2C_Cmd(I2Cx,ENABLE);
    return SUCCESS;
}

ErrorStatus I2Cx_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_Event)
{
    uint32_t wait = I2C_WAIT_TIMEOUT;
    while (wait--)
        if (I2C_CheckEvent (I2Cx,I2C_Event))
            return SUCCESS;

    return ERROR;
}

ErrorStatus I2Cx_WaitFlagSet(I2C_TypeDef* I2Cx, uint32_t I2C_Flag)
{
    uint32_t wait = I2C_WAIT_TIMEOUT;
    while (wait--)
        if (I2C_GetFlagStatus(I2Cx,I2C_Flag))
            return SUCCESS;

    return ERROR;
}

ErrorStatus I2Cx_WaitFlagReset(I2C_TypeDef* I2Cx, uint32_t I2C_Flag)
{
    uint32_t wait = I2C_WAIT_TIMEOUT;
    while (wait--)
        if (!I2C_GetFlagStatus(I2Cx,I2C_Flag))
            return SUCCESS;

    return ERROR;
}


ErrorStatus I2Cx_Write(I2C_TypeDef* I2Cx, const uint8_t* buf, uint32_t nbytes, uint8_t SlaveAddress, I2C_STOP_TypeDef stop)
{
    I2C_GenerateSTART(I2Cx,ENABLE);
    if (I2Cx_WaitEvent(I2Cx,I2C_EVENT_MASTER_MODE_SELECT) == ERROR)
        return ERROR;

    I2C_Send7bitAddress(I2Cx,SlaveAddress,I2C_Direction_Transmitter);

    if (I2Cx_WaitEvent(I2Cx,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == ERROR)
        return ERROR;

    while (nbytes--) {
        I2C_SendData(I2Cx,*buf++);
        if (I2Cx_WaitEvent (I2Cx,I2C_EVENT_MASTER_BYTE_TRANSMITTED) == ERROR)
            return ERROR;
	}

	if (stop == I2C_STOP) {
        I2C_GenerateSTOP (I2Cx,ENABLE);
        if (I2Cx_WaitFlagReset(I2Cx,I2C_FLAG_STOPF) == ERROR)
            return ERROR;
	}

    return SUCCESS;
}

ErrorStatus I2Cx_Read(I2C_TypeDef* I2Cx, uint8_t *buf, uint32_t nbytes, uint8_t SlaveAddress) {

    I2C_AcknowledgeConfig(I2Cx,ENABLE);

    I2C_NACKPositionConfig (I2Cx,I2C_NACKPosition_Current);

    I2C_GenerateSTART(I2Cx,ENABLE);
    if (I2Cx_WaitEvent(I2Cx,I2C_EVENT_MASTER_MODE_SELECT) == ERROR)
        return ERROR;

    I2C_Send7bitAddress(I2Cx,SlaveAddress,I2C_Direction_Receiver);

    if (I2Cx_WaitFlagSet(I2Cx,I2C_FLAG_ADDR) == ERROR)
        return ERROR;

    if (nbytes == 1) {
        I2C_AcknowledgeConfig(I2Cx,DISABLE);

        __disable_irq();
        (void)I2Cx->SR1;
        (void)I2Cx->SR2;
        I2C_GenerateSTOP(I2Cx,ENABLE);
        __enable_irq();

        if (I2Cx_WaitFlagSet(I2Cx,I2C_FLAG_RXNE) == ERROR)
            return ERROR;

        *buf = I2C_ReceiveData(I2Cx);
    } else if (nbytes == 2) {
        I2C_NACKPositionConfig (I2Cx,I2C_NACKPosition_Next);

        __disable_irq();
        (void)I2Cx->SR2;
        I2C_AcknowledgeConfig(I2Cx,DISABLE);
        __enable_irq();

        if (I2Cx_WaitFlagSet(I2Cx,I2C_FLAG_BTF) == ERROR)
            return ERROR;

        __disable_irq();
        I2C_GenerateSTOP(I2Cx,ENABLE);
        *buf++ = I2C_ReceiveData (I2Cx);
        __enable_irq();

        *buf = I2C_ReceiveData(I2Cx);

    } else {
        (void)I2Cx->SR2;

        while (nbytes-- != 3) {
            if (I2Cx_WaitFlagSet(I2Cx,I2C_FLAG_BTF) == ERROR)
                return ERROR;
            *buf++ = I2C_ReceiveData(I2Cx);
        }

        if (I2Cx_WaitFlagSet(I2Cx,I2C_FLAG_BTF) == ERROR)
            return ERROR;

        I2C_AcknowledgeConfig(I2Cx,DISABLE);

        __disable_irq();
        *buf++ = I2C_ReceiveData(I2Cx);
        I2C_GenerateSTOP(I2Cx,ENABLE);
        __enable_irq();

        *buf++ = I2C_ReceiveData(I2Cx);

        if (I2Cx_WaitEvent(I2Cx,I2C_EVENT_MASTER_BYTE_RECEIVED) == ERROR)
            return ERROR;

        *buf = I2C_ReceiveData(I2Cx);
    }

    if (I2Cx_WaitFlagReset(I2Cx,I2C_FLAG_STOPF) == ERROR)
        return ERROR;

    return SUCCESS;
}


ErrorStatus I2Cx_IsDeviceReady(I2C_TypeDef* I2Cx, uint8_t SlaveAddress, uint32_t Trials) {
     uint32_t wait;
	uint16_t reg;

	do {
        I2C_GenerateSTART(I2Cx,ENABLE);
        if (I2Cx_WaitEvent(I2Cx,I2C_EVENT_MASTER_MODE_SELECT) == ERROR)
            return ERROR;

        I2C_Send7bitAddress(I2Cx,SlaveAddress,I2C_Direction_Transmitter);

		// Wait until ADDR or AF bit set
		wait = I2C_WAIT_TIMEOUT;
		do {
			reg = I2Cx->SR1;
		} while (!(reg & I2C_SR1_ADDR) && !(reg & I2C_SR1_AF) && --wait);

        I2C_GenerateSTOP (I2Cx,ENABLE);
        // Check if device responded
		if (reg & I2C_SR1_ADDR) {
			(void)I2Cx->SR1;
			(void)I2Cx->SR2;
            if (I2Cx_WaitFlagReset(I2Cx,I2C_FLAG_STOPF) == ERROR)
                return ERROR;
            if (I2Cx_WaitFlagReset(I2Cx,I2C_FLAG_BUSY) == ERROR)
                return ERROR;
            return SUCCESS;
		} else {
			I2Cx->SR1 &= ~I2C_SR1_AF;
            if (I2Cx_WaitFlagReset(I2Cx,I2C_FLAG_BUSY) == ERROR)
                return ERROR;
		}
	} while (--Trials);

    return ERROR;
}


