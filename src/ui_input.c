#include "stm32x.h"
#include "espeva_stmboard.h"
#include "aioiface.h"
#include "ui_input.h"

#define adc_max_val           4096
#define adc_val_threshhold    100
#define adc_detect_threshhold 200
#define adc_avg_samples       2
#define adc_avg_threshhold    16
#define adc_touch_threshhold  500
enum { VCC,GND,INPUT };

AIO_InputState input_state = {0};
AIO_Sys_Info sys_info = {0};

void ESP_ADC_Common_Init () {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);

    ADC_InitTypeDef adc;
    adc.ADC_Mode = ADC_Mode_Independent;
    adc.ADC_ContinuousConvMode = ENABLE;
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_ScanConvMode = DISABLE;
    adc.ADC_NbrOfChannel = 1;
    ADC_Init (ADC2, &adc);
    ADC_Cmd(ADC2, ENABLE);
    ADC_StartCalibration(ADC2);
    while (ADC_GetCalibrationStatus (ADC2) == RESET);
}

uint16_t ESP_ADC_ReadChannelValue (int ch)
{
}

static int ESP_ReadTouchVal (int ch,int samples)
{
    ADC_SoftwareStartConvCmd(ADC2,DISABLE);
    ADC_RegularChannelConfig(ADC2, ch, 1, ADC_SampleTime_239Cycles5);
    ADC_SoftwareStartConvCmd(ADC2,ENABLE);
    ADC_GetConversionValue (ADC2);

    int16_t val,prev_val,i=0;
    int32_t avg = 0;
    for (i = 0; i < samples; ++i) {
        while (ADC_GetFlagStatus(ADC2,ADC_FLAG_EOC) == RESET);
        val = ADC_GetConversionValue (ADC2);;
        if (i > 0 && (val - prev_val > adc_avg_threshhold || prev_val - val > adc_avg_threshhold))
            return -1;
        prev_val = val;
        avg += val;
    }
    return avg/samples;
}

static void ESP_SetTouchLines (uint8_t yu,uint8_t xl,uint8_t yd,uint8_t xr)
{
    int values[]={yu,xl,yd,xr};
    GPIO_TypeDef *ports[] = {ESP_TOUCH_YU_PORT,ESP_TOUCH_XL_PORT,ESP_TOUCH_YD_PORT,ESP_TOUCH_XR_PORT};
    int pins[] =            {ESP_TOUCH_YU_PIN, ESP_TOUCH_XL_PIN, ESP_TOUCH_YD_PIN, ESP_TOUCH_XR_PIN};
    int i;

    GPIO_InitTypeDef gpio;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    for (i = 0; i < 4; ++i) {
        gpio.GPIO_Pin = pins[i];
        gpio.GPIO_Mode = (values[i]!=INPUT)?GPIO_Mode_Out_PP:GPIO_Mode_AIN;
        GPIO_Init (ports[i],&gpio);
        if (values[i]==VCC)
            GPIO_SetBits(ports[i],pins[i]);
        if (values[i]==GND)
            GPIO_ResetBits(ports[i],pins[i]);
    }
}

// GPIO connected key
static int ESP_UI_Input_CheckKeyDown (int key)
{
    GPIO_TypeDef *ports[] = {ESP_KEY_PWR_PORT,ESP_KEY_KB2_PORT };
    int pins[] =            {ESP_KEY_PWR_PIN, ESP_KEY_KB2_PIN };

    GPIO_InitTypeDef gpio;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_Pin = pins[key];
    gpio.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init (ports[key],&gpio);
    return (GPIO_ReadInputDataBit(ports[key],pins[key]))?1:0;
}


// nav key
static int ESP_UI_Input_CheckNavKey () {

    int i,keys[] = {AIO_KEY_CODE_DOWN,AIO_KEY_CODE_RIGHT,AIO_KEY_CODE_UP,AIO_KEY_CODE_OK,AIO_KEY_CODE_LEFT};
    int val = ESP_ReadTouchVal (ESP_4DIR_ADC,adc_avg_samples);
    if (val < 0)
        return -1;

    for (i = 0; i < 5; ++i) {
        // calculate expected ADC value for key
        int16_t exp = adc_max_val/(i+1)*i;
        if (val >= exp - adc_val_threshhold && val <= exp + adc_val_threshhold)
            return keys[i];
    }

    return -1;
}


static int ESP_UI_Input_ReadTC ()
{
    int x,y,z1,z2;

    //                 YU,   XL,   YD,    XR
    ESP_SetTouchLines( INPUT,VCC,  INPUT, GND );
    DelayMs (1);
    x = ESP_ReadTouchVal (ESP_TOUCH_YD_ADC,adc_avg_samples);

    ESP_SetTouchLines( VCC,  INPUT,GND,   INPUT );
    DelayMs (1);
    y = ESP_ReadTouchVal (ESP_TOUCH_XR_ADC,adc_avg_samples);

    ESP_SetTouchLines( VCC,  GND,  INPUT, INPUT );
    DelayMs (1);
    z1 = ESP_ReadTouchVal (ESP_TOUCH_YD_ADC,1);
    z2 = ESP_ReadTouchVal (ESP_TOUCH_XR_ADC,1);
    if (x < 0 || y < 0 || z1 < 0 || z2 < 0) {
        input_state.z = 0;
        return -1;
    }

    input_state.x = adc_max_val - x - 1;
    input_state.y = adc_max_val - y -1 ;
    input_state.z = adc_max_val - (z1-z2) -1 ;
    return 0;

}

void ESP_UI_Input_GetSysInfo ()
{
    int z,z1,z2;

    // detect touch screen
    do {
        // Set VCC to XL
        ESP_SetTouchLines( INPUT,VCC,  INPUT, INPUT );
        DelayMs (1);
        // Expect VCC on XR
        if (ESP_ReadTouchVal (ESP_TOUCH_XR_ADC,adc_avg_samples) < adc_max_val - adc_detect_threshhold)
            break;
        // Set GND to XL
        ESP_SetTouchLines( INPUT,GND,  INPUT, INPUT );
        DelayMs (1);
        // Expect GND on XL
        if (ESP_ReadTouchVal (ESP_TOUCH_XR_ADC,adc_avg_samples) > adc_detect_threshhold)
            break;
        sys_info.has_touch = 1;

        // Check is touch pressed for alternative boot mode
        ESP_SetTouchLines( VCC,  GND,  INPUT, INPUT );
        DelayMs (1);
        z1 = ESP_ReadTouchVal (ESP_TOUCH_YD_ADC,1);
        z2 = ESP_ReadTouchVal (ESP_TOUCH_XR_ADC,1);
        if (z1 < 0 || z2 < 0)
            break;
        z = adc_max_val -(z1-z2);
        if (z > adc_touch_threshhold)
            sys_info.boot_mode = 1;

        // TODO: accurate detection
        sys_info.tft_model = AIO_TFT_ILI9225B;
    } while (0);

    // detect 4-way key
   if (!sys_info.has_touch) {
       // Set all touch screen lines to INPU
       ESP_SetTouchLines( INPUT, INPUT,  INPUT, INPUT );
       DelayMs (1);
       // Expect pullup to VCC on 4DIR
       if (ESP_ReadTouchVal (ESP_4DIR_ADC,adc_avg_samples) >= adc_max_val - adc_detect_threshhold)
           sys_info.has_4way = 1;
       // Check if  is pressed for alternative boot mode
       if (ESP_UI_Input_CheckNavKey()== AIO_KEY_CODE_OK)
           sys_info.has_4way = sys_info.boot_mode = 1;
       // TODO: accurate detection
       sys_info.tft_model = AIO_TFT_ST7735;
   }

}

void ESP_UI_Input_Run () {
    int key;
    uint16_t keys = 0;
    if (sys_info.has_touch)
        ESP_UI_Input_ReadTC ();

    if (sys_info.has_4way) {
        key = ESP_UI_Input_CheckNavKey ();
        if (key >=0)
            keys = 1<<key;
    }
    keys |= ESP_UI_Input_CheckKeyDown (AIO_KEY_CODE_PWR)<<AIO_KEY_CODE_PWR;
    keys |= ESP_UI_Input_CheckKeyDown (AIO_KEY_CODE_PWR)<<AIO_KEY_CODE_PWR;
    input_state.keys = keys;
}


void ESP_UI_Input_Init () {

    ESP_ADC_Common_Init ();
}
