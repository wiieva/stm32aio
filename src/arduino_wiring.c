#include "stm32x.h"
#include "arduino_wiring.h"
#include "espeva_stmboard.h"
#include "aioiface.h"

typedef struct {
    GPIO_TypeDef *port;
    int pin;
    TIM_TypeDef *tim;
    int channel;
    void (*tim_oc_init) (TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
    int tim_prescaler;
} ArduinoPinDef;

const ArduinoPinDef pins[] =
{
    {GPIOC,GPIO_Pin_11,0,    0,             0,           0}, // 0
    {GPIOC,GPIO_Pin_10,0,    0,             0,           0}, // 1
    {GPIOB,GPIO_Pin_12,0,    0,             0,           0}, // 2
    {GPIOC,GPIO_Pin_6, TIM3, TIM_Channel_1, TIM_OC1Init, 1000}, // 3
    {GPIOB,GPIO_Pin_14,0,    0,             0,           0}, // 4
    {GPIOC,GPIO_Pin_7, TIM3, TIM_Channel_2, TIM_OC2Init, 1000}, // 5
    {GPIOC,GPIO_Pin_8, TIM3, TIM_Channel_3, TIM_OC3Init, 1000}, // 6
    {GPIOA,GPIO_Pin_5, 0,    0,             0,           0}, // 7
    {GPIOC,GPIO_Pin_12,0,    0,             0,           0}, // 8
    {GPIOC,GPIO_Pin_9, TIM3, TIM_Channel_4, TIM_OC4Init, 1000}, // 9
    {GPIOA,GPIO_Pin_1, TIM5, TIM_Channel_2, TIM_OC2Init, 1000}, // 10
    {GPIOA,GPIO_Pin_2, TIM5, TIM_Channel_3, TIM_OC3Init, 1000}, // 11
    {GPIOA,GPIO_Pin_14,0,    0,             0,           0}, // 12
    {GPIOA,GPIO_Pin_13,0,    0,             0,           0}, // 13
    // Analog pins
    {GPIOC,GPIO_Pin_0, 0,    ADC_Channel_10,0,           0}, // A0
    {GPIOC,GPIO_Pin_1, 0,    ADC_Channel_11,0,           0}, // A1
    {GPIOC,GPIO_Pin_2, 0,    ADC_Channel_12,0,           0}, // A2
    {GPIOA,GPIO_Pin_3, 0,    ADC_Channel_3, 0,           0}, // A3
    {GPIOA,GPIO_Pin_6, 0,    ADC_Channel_6, 0,           0}, // A4
    {GPIOA,GPIO_Pin_7, 0,    ADC_Channel_7, 0,           0}, // A5
    {GPIOC,GPIO_Pin_5, 0,    ADC_Channel_15,0,           0}, // Batary meausure
    // additional pins
    {GPIOB,GPIO_Pin_10,TIM2, TIM_Channel_3, TIM_OC3Init, 1}, // Backlight
    {GPIOA,GPIO_Pin_0, 0,    0,             0,           0}, // KB2 (PWR)
    {GPIOC,GPIO_Pin_14,0,    0,             0,           0}, // Charging
    {GPIOB,GPIO_Pin_11,TIM2, TIM_Channel_4, TIM_OC4Init, 1}, // IR
    {GPIOA,GPIO_Pin_9,0,     0,             0,           0}  // USB Power
};

#define numPins (sizeof (pins)/sizeof(pins[0]))

static int8_t adcIdx[numPins];
static int16_t adc_buffer[numPins];
static uint16_t pwmFreq[numPins];
static uint16_t pwmLimit[numPins];
static int8_t pinModes[numPins];
static int16_t pwmVal[numPins];

#define PWM_DEF_HZ 1000
static int ESP_Wiring_Init_Pin (uint16_t pin, const ArduinoPinDef *_pin) {

    GPIO_InitTypeDef gpio;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_Pin = _pin->pin;
    gpio.GPIO_Mode = (pin==23)?GPIO_Mode_IPU:GPIO_Mode_IN_FLOATING;
    GPIO_Init(_pin->port, &gpio);

    if (_pin->tim) {
        if (_pin->tim_prescaler == 1){
            pwmFreq[pin] = 38000;
        } else {
            pwmFreq[pin] = PWM_DEF_HZ;
        }
        pwmLimit[pin] = 255;
        pwmVal[pin] = 0;

        TIM_TimeBaseInitTypeDef tim;
        tim.TIM_Period = ((CPUFREQ_HZ/_pin->tim_prescaler) / pwmFreq[pin]) - 1;
        tim.TIM_Prescaler = (_pin->tim_prescaler==1)?0:_pin->tim_prescaler;
        tim.TIM_ClockDivision = 0;
        tim.TIM_CounterMode = TIM_CounterMode_Up;
        TIM_TimeBaseInit(_pin->tim, &tim);
        TIM_Cmd(_pin->tim, ENABLE);

        TIM_OCInitTypeDef timerPWM;
        timerPWM.TIM_Pulse = 0,
        timerPWM.TIM_OCMode = TIM_OCMode_PWM1,
        timerPWM.TIM_OutputState = TIM_OutputState_Enable;
        timerPWM.TIM_OCPolarity = TIM_OCPolarity_High;

        _pin->tim_oc_init (_pin->tim, &timerPWM);
        TIM_SelectOCxM(_pin->tim, _pin->channel,TIM_OCMode_PWM1);
        TIM_CCxCmd(_pin->tim, _pin->channel, TIM_CCx_Enable);
    }
    return 1;
}

static int ESP_Wiring_Set_Pin_Mode (uint16_t pin, const ArduinoPinDef *_pin, int mode) {
     if (pin >= numPins)
        return 0;
//    if (mode == _pin->cur_mode)
//        return 1;

    GPIO_InitTypeDef gpio;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_Pin = _pin->pin;
    gpio.GPIO_Mode = (pin==23)?GPIO_Mode_IPU:GPIO_Mode_IN_FLOATING;

    switch (mode){
    case AIO_PIN_DIGITAL_OUT:
        gpio.GPIO_Mode = GPIO_Mode_Out_PP;
        break;
    case AIO_PIN_DIGITAL_IN:
    case AIO_PIN_ANALOG_IN:
        if (!_pin->tim && _pin->channel)
            gpio.GPIO_Mode = GPIO_Mode_AIN;
        else
            gpio.GPIO_Mode = (pin==23)?GPIO_Mode_IPU:GPIO_Mode_IN_FLOATING;
        break;
    case AIO_PIN_ANALOG_OUT:
        if (!_pin->tim)
            return 0;
        gpio.GPIO_Mode = GPIO_Mode_AF_PP;
        break;
    }

    GPIO_Init(_pin->port, &gpio);
    pinModes[pin] = mode;

    return 1;
}

static inline int ESP_Wiring_Check_Pin_Mode (uint16_t pin, int mode) {
    if (pin >= numPins || mode != pinModes[pin])
        return 0;
    return 1;
}

void ESP_Wiring_PinMode (uint16_t pin,uint16_t mode) {
    const ArduinoPinDef *_pin = pins + pin;
    if (pin >= numPins)
       return;
    ESP_Wiring_Set_Pin_Mode (pin,_pin,mode);
}

uint16_t ESP_Wiring_DigitalRead (uint16_t pin) {
    const ArduinoPinDef *_pin = pins + pin;
    if (pin >= numPins)
       return 0;

    return (_pin->port->IDR & _pin->pin)?1:0;
}

void ESP_Wiring_DigitalWrite (uint16_t pin,uint16_t val) {
    const ArduinoPinDef *_pin = pins + pin;
    if (pin >= numPins)
       return;

    if (val)
        _pin->port->BSRR = _pin->pin;
    else
        _pin->port->BRR = _pin->pin;
}

uint16_t ESP_Wiring_AnalogRead (uint16_t pin) {
    if (pin >= numPins)
       return 0;

    return adc_buffer[adcIdx[pin]];
}

void ESP_Wiring_AnalogWrite (uint16_t pin,uint16_t val) {
    const ArduinoPinDef *_pin = pins + pin;
    if (pin >= numPins || !_pin->tim)
       return;

    __IO uint16_t *ccr = &(_pin->tim->CCR1) + _pin->channel/2;
    *ccr = ((CPUFREQ_HZ/_pin->tim_prescaler) / pwmFreq[pin])*val/pwmLimit[pin];
    pwmVal[pin] = val;
}

void ESP_Wiring_SetPwmParms (uint16_t pin,uint16_t freq,uint16_t limit) {
    const ArduinoPinDef *_pin = pins + pin;
    if (pin >= numPins || !_pin->tim)
       return;
    if (!freq)
        freq = PWM_DEF_HZ;
    if (!limit)
        limit = 255;
    pwmFreq[pin] = freq;
    pwmLimit[pin] = limit;

    _pin->tim->ARR = ((CPUFREQ_HZ/_pin->tim_prescaler) / freq) - 1;
    ESP_Wiring_AnalogWrite(pin,pwmVal[pin]);
}

void ESP_Wiring_ADC_Start () {

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    ADC_InitTypeDef adc;
    adc.ADC_Mode = ADC_Mode_Independent;
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_ContinuousConvMode = ENABLE;

    int i,chnls=0;
    for (i = 0; i < numPins; ++i) {
        if (!pins[i].tim && pins[i].channel) {
            adcIdx[i] = chnls++;
            ADC_RegularChannelConfig(ADC1, pins[i].channel, chnls, ADC_SampleTime_55Cycles5);
        } else
            adcIdx[i] = numPins-1;
    }
    adc.ADC_ScanConvMode = (chnls==1)?DISABLE:ENABLE;
    adc.ADC_NbrOfChannel = chnls;
    ADC_Init (ADC1, &adc);
    ADC_Cmd(ADC1, ENABLE);

    DMA_InitTypeDef dma;
    dma.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);
    dma.DMA_MemoryBaseAddr = (uint32_t)adc_buffer;
    dma.DMA_DIR = DMA_DIR_PeripheralSRC;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_Medium;
    dma.DMA_M2M = DMA_M2M_Disable;
    dma.DMA_BufferSize = chnls;
    DMA_Init (DMA1_Channel1, &dma);
    ADC_DMACmd(ADC1, ENABLE);
    DMA_Cmd(DMA1_Channel1, ENABLE);
    ADC_SoftwareStartConvCmd(ADC1,ENABLE);
}

void ESP_Wiring_OffAll () {
    int i;
    for (i = 0; i < numPins; ++i)
        ESP_Wiring_Set_Pin_Mode (i,pins+i,AIO_PIN_DIGITAL_IN);
}

void ESP_Wiring_Init () {

    uint16_t i;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2|RCC_APB1Periph_TIM3|RCC_APB1Periph_TIM5, ENABLE);
    GPIO_PinRemapConfig (GPIO_FullRemap_TIM2,ENABLE);
    GPIO_PinRemapConfig (GPIO_FullRemap_TIM3,ENABLE);
    GPIO_PinRemapConfig (GPIO_Remap_SWJ_Disable,ENABLE);

    for (i = 0; i < numPins; ++i)
        ESP_Wiring_Init_Pin (i,pins+i);

    ESP_Wiring_ADC_Start ();
}
