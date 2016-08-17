#include "stm32x.h"
#include "tools.h"

volatile unsigned long sysTickMs;

void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len) {
    uint8_t idx = 0;
  
    for( idx = 0 ; idx < len ; idx ++) {
        if( ((value >> 28)) < 0xA )
            pbuf[ 2* idx] = (value >> 28) + '0';
        else
            pbuf[2* idx] = (value >> 28) + 'A' - 10;
    
        value = value << 4;
        pbuf[ 2* idx + 1] = 0;
    }
}

unsigned long GetSysTickMS () {
    return sysTickMs;
}

void DelayMs (int Ms) {
    unsigned long to = GetSysTickMS() + Ms;
    while ((long)((to - GetSysTickMS())) > 0);
}

void SysTick_Handler(void) {
    ++sysTickMs;

}


void Get_SerialNum(uint8_t *pbuf1,uint8_t *pbuf2)
{
    uint32_t Device_Serial0, Device_Serial1, Device_Serial2;

    Device_Serial0 = *(__IO uint32_t*)(0x1FFFF7E8);
    Device_Serial1 = *(__IO uint32_t*)(0x1FFFF7EC);
    Device_Serial2 = *(__IO uint32_t*)(0x1FFFF7F0);

    Device_Serial0 += Device_Serial2;

    if (Device_Serial0 != 0) {
        IntToUnicode (Device_Serial0, pbuf1 , 8);
        IntToUnicode (Device_Serial1, pbuf2, 4);
    }
}


#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line) {
  while (1);
}
#endif

