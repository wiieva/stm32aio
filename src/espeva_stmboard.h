#ifndef ESPEVA_STMBOARD_H
#define ESPEVA_STMBOARD_H

//#define DISCO_PROTO

// ESP8266 serial connection
// *************************
#define ESP_USART USART1
#define ESP_USART_TX_PORT GPIOB
#define ESP_USART_RX_PORT GPIOB
#define ESP_USART_TX_PIN GPIO_Pin_6
#define ESP_USART_RX_PIN GPIO_Pin_7
#define ESP_USART_IRQ USART1_IRQn
#define ESP_GPIO_AF_USART GPIO_AF_USART1

// ESP8266 boot & reset control
// *************************
#define ESP_RESET_PORT GPIOD
#define ESP_RESET_PIN GPIO_Pin_2
#define ESP_GPIO0_PORT GPIOA
#define ESP_GPIO0_PIN GPIO_Pin_8
#define ESP_GPIO2_PORT GPIOC
#define ESP_GPIO2_PIN GPIO_Pin_15
#define ESP_GPIO15_PORT GPIOB
#define ESP_GPIO15_PIN GPIO_Pin_2

// ESP8266 SPI
// *************************
#define ESP_SPI SPI1
#define ESP_SPI_MOSI_PORT GPIOB
#define ESP_SPI_MOSI_PIN  GPIO_Pin_5
#define ESP_SPI_MISO_PORT GPIOB
#define ESP_SPI_MISO_PIN  GPIO_Pin_4
#define ESP_SPI_SCK_PORT  GPIOB
#define ESP_SPI_SCK_PIN   GPIO_Pin_3
#define ESP_SPI_NSS_PORT  GPIOA
#define ESP_SPI_NSS_PIN   GPIO_Pin_15

#define ESP_SPI_IRQ SPI1_IRQn
#define ESP_GPIO_AF_SPI GPIO_AF_SPI1

// Speaker DAC
// ***********
#define ESP_SPEAKER_DAC_PIN GPIO_Pin_4
#define ESP_SPEAKER_DAC_PORT GPIOA
#define ESP_SPEAKER_DAC_CHANNEL DAC_Channel_1

// IR Tx PWM
// *********
#define ESP_IR_TX_PIN  GPIO_Pin_11
#define ESP_IR_TX_PIN_SOURCE  GPIO_PinSource11
#define ESP_IR_TX_PORT GPIOB

// Keys and touch screen
// *********************
#define ESP_TOUCH_YU_PIN GPIO_Pin_0
#define ESP_TOUCH_YU_PORT GPIOB
#define ESP_TOUCH_YU_ADC ADC_Channel_8
#define ESP_TOUCH_XL_PIN GPIO_Pin_1
#define ESP_TOUCH_XL_PORT GPIOB
#define ESP_TOUCH_XL_ADC ADC_Channel_9
#define ESP_TOUCH_YD_PIN GPIO_Pin_3
#define ESP_TOUCH_YD_PORT GPIOC
#define ESP_TOUCH_YD_ADC ADC_Channel_13
#define ESP_TOUCH_XR_PIN GPIO_Pin_4
#define ESP_TOUCH_XR_PORT GPIOC
#define ESP_TOUCH_XR_ADC ADC_Channel_14

#define ESP_4DIR_PIN GPIO_Pin_4
#define ESP_4DIR_PORT GPIOC
#define ESP_4DIR_ADC ADC_Channel_14

#define ESP_KEY_PWR_PIN GPIO_Pin_0
#define ESP_KEY_PWR_PORT GPIOA

#define ESP_KEY_KB2_PIN GPIO_Pin_13
#define ESP_KEY_KB2_PORT GPIOC

#define CPUFREQ_MHZ 72L
#define CPUFREQ_KHZ 72000L
#define CPUFREQ_HZ  72000000L

#endif // ESPEVA_STMBOARD_H
