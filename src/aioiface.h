#pragma once

// Commands definition
enum AIOCommand {
    AIO_CMD_NOP               =    0x0,
    AIO_CMD_ANALOG_READ,
    AIO_CMD_DIGITAL_READ,
    AIO_CMD_ANALOG_WRITE,
    AIO_CMD_DIGITAL_WRITE,
    AIO_CMD_PIN_MODE,
    AIO_CMD_SET_PWM_PARMS,
    AIO_CMD_RD_STREAM,
    AIO_CMD_WR_STREAM,
    AIO_CMD_AUDIO_IN_MODE,
    AIO_CMD_AUDIO_OUT_MODE,
    AIO_CMD_AUDIO_IN_VOLUME,
    AIO_CMD_AUDIO_OUT_VOLUME,
    AIO_CMD_GET_VAD_STATE,
    AIO_CMD_SEND_IR_CMD,
    AIO_CMD_SEND_UART,
    AIO_CMD_RECV_UART,
    AIO_CMD_SEND_I2C,
    AIO_CMD_RECV_I2C,
    AIO_CMD_GET_SYSINFO,
    AIO_CMD_GET_INPUT_STATE,
    AIO_CMD_UART_MODE,
    AIO_CMD_GET_BME_DATA,
    AIO_CMD_TEST
};

typedef struct {
    uint8_t cmd : 5;
    uint8_t arg2 : 3;
    uint8_t arg1;
} __attribute__ ((packed)) AIO_Cmd_Body;

typedef struct {
    uint16_t freq;
    uint16_t limit;
} __attribute__ ((packed)) AIO_PwmParms;

// Audio modes definition for AIO_CMD_AUDIO_IN_MODE and AIO_CMD_AUDIO_OUT_MODE
enum AIOAudioMode {
    AIO_AUDIO_OFF,
    AIO_AUDIO_IN_SPEEX,
    AIO_AUDIO_OUT_MP3,
    AIO_AUDIO_RAW_8KHZ,
    AIO_AUDIO_RAW_16KHZ,
    AIO_AUDIO_RAW_22KHZ,
    AIO_AUDIO_RAW_44KHZ
};

// PIN modes definition for AIO_CMD_PIN_MODE
enum AIOPinMode{
    AIO_PIN_DIGITAL_IN,
    AIO_PIN_DIGITAL_OUT,
    AIO_PIN_ANALOG_IN,
    AIO_PIN_ANALOG_OUT
} ;

// Send IR modes definition for AIO_CMD_SEND_IR
enum AIOSendIRMode {
    AIO_SEND_IR_SONY,
    AIO_SEND_IR_NEC,
    AIO_SEND_IR_WHYNTER,
    AIO_SEND_IR_RC5,
    AIO_SEND_IR_RC6,
    AIO_SEND_IR_PANASONIC,
    AIO_SEND_IR_JVC,
    AIO_SEND_IR_SAMSUNG,
    AIO_SEND_IR_RAW
};

typedef struct {
    uint8_t mode;
    uint8_t n_bits : 7;
    uint8_t repeat : 1;
    uint16_t addr;
    uint32_t data;
} __attribute__ ((packed)) AIO_IR_Payload;


enum AIOTFTModel {
    AIO_TFT_NONE,
    AIO_TFT_ILI9225B,
    AIO_TFT_ST7735
};
typedef struct {
    uint16_t hw_ver;
    uint16_t aio_sw_ver;
    uint8_t tft_model;
    uint8_t has_touch :1;
    uint8_t has_4way : 1;
    uint8_t has_bme280 : 1;
    uint8_t has_bat : 1;
    uint8_t boot_mode : 2;
} __attribute__ ((packed)) AIO_Sys_Info;

typedef struct {
    uint16_t keys;
    uint16_t x;
    uint16_t y;
    uint16_t z;
} __attribute__ ((packed)) AIO_InputState;

enum AIOInputEvent {
    AIO_EVENT_KEY_DOWN,
    AIO_EVENT_KEY_UP,
    AIO_EVENT_TOUCH_DOWN,
    AIO_EVENT_TOUCH_UP
};

enum AIOKeyCode {
    AIO_KEY_CODE_PWR,
    AIO_KEY_CODE_KB2,
    AIO_KEY_CODE_UP,
    AIO_KEY_CODE_DOWN,
    AIO_KEY_CODE_LEFT,
    AIO_KEY_CODE_RIGHT,
    AIO_KEY_CODE_OK
};

typedef struct AIO_BME_Data {
    uint8_t ready;
    uint8_t valid;
    int16_t temperature;
    uint32_t pressure;
    uint32_t humidity;
} __attribute__  ((packed)) AIO_BME_Data;

typedef struct {
    int32_t baud_rate;
    int8_t parity;
    int8_t bits;
    int8_t stop_bits;
} __attribute__  ((packed)) AIO_SerialSetup;


