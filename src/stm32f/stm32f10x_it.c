
#include "stm32f10x_it.h"
#include "usbd_cdc_core.h"

extern USB_OTG_CORE_HANDLE           USB_OTG_dev;
extern uint32_t USBD_OTG_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);
extern int usb_init_done;

void NMI_Handler(void) {
}

void HardFault_Handler(void) {
  while (1);
}

void MemManage_Handler(void) {
    while (1);
}

void BusFault_Handler(void) {
  while (1);
}

void UsageFault_Handler(void) {
  while (1);
}

void SVC_Handler(void) {
}

void DebugMon_Handler(void) {
}

void PendSV_Handler(void) {
}


#ifdef USE_USB_OTG_FS
void OTG_FS_IRQHandler(void) {
    if (usb_init_done) {
      USBD_OTG_ISR_Handler (&USB_OTG_dev);
    }
}

void OTG_FS_WKUP_IRQHandler(void)
{
  if(USB_OTG_dev.cfg.low_power)
  {
    *(uint32_t *)(0xE000ED10) &= 0xFFFFFFF9 ;
    SystemInit();
    USB_OTG_UngateClock(&USB_OTG_dev);
  }
}

#endif

