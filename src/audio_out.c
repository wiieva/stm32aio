#include "stm32x.h"
#include "espeva_stmboard.h"
#include "audio_out.h"
#include <string.h>
#include <malloc.h>
#include <mp3dec.h>
#include "usart.h"

#define mp3_frame_max_samples   1152
#define dac_dma_buf_size        (mp3_frame_max_samples*sizeof(int16_t)*2)
#define mp3_buf_size            2048
#define mp3_frame_max_size      4096
#define mp3_frame_headers_size  (32+4+2+2)

static circular_buffer  dac_dma_buf = {0};
static int              mp3mode = 0;
static int              out_sample_rate = 0;
static int              audio_out_started = 0;
static int              audio_out_do_start = -1;
static int              do_sample_rate = 0;
static int              do_mp3mode     = 0;

static MP3FrameInfo     mp3FrameInfo = {0};
static HMP3Decoder      hMP3Decoder = {0};
static circular_buffer  mp3_buf = {0};
static uint8_t         *mp3_frame_buf = 0;
static size_t           mp3_frame_len  = 0;

circular_buffer *ESP_Speaker_Buffer ()
{
    return mp3mode?&mp3_buf:&dac_dma_buf;
}

static TIM_TimeBaseInitTypeDef tim;
void InitTMR4() {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_TimeBaseInitTypeDef tim;
    tim.TIM_Period = (32000000L / 8000L) - 1, // 8 KHz, from 32 MHz TIM2CLK (ie APB1 = HCLK)
    tim.TIM_Prescaler = 0,
    tim.TIM_ClockDivision = 0,
    tim.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM4, &tim);

    TIM_SelectOutputTrigger(TIM4, TIM_TRGOSource_Update);
    TIM_Cmd(TIM4, ENABLE);
}

void ESP_ADC_DAC_SetSampleRate (int sampleRate)
{
    TIM_Cmd(TIM4, DISABLE);

    tim.TIM_Period = CPUFREQ_HZ / sampleRate - 1;
    TIM_TimeBaseInit(TIM4, &tim);
    TIM_Cmd(TIM4, ENABLE);
}

void ESP_Speaker_DAC_Init ()
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
    RCC_AHBPeriphClockCmd (RCC_AHBPeriph_DMA2,ENABLE);

    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = ESP_SPEAKER_DAC_PIN;
    gpio.GPIO_Mode = GPIO_Mode_AF_OD;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init (ESP_SPEAKER_DAC_PORT, &gpio);

    DAC_InitTypeDef  dac;
    dac.DAC_Trigger = DAC_Trigger_T4_TRGO;
    dac.DAC_WaveGeneration = DAC_WaveGeneration_None;
    dac.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
    dac.DAC_LFSRUnmask_TriangleAmplitude = 0;
    DAC_Init(ESP_SPEAKER_DAC_CHANNEL, &dac);

    DMA_InitTypeDef dma;
    dma.DMA_PeripheralBaseAddr = (uint32_t)&(DAC->DHR12L1);
    dma.DMA_MemoryBaseAddr = (uint32_t)dac_dma_buf.buf;
    dma.DMA_DIR = DMA_DIR_PeripheralDST;
    dma.DMA_BufferSize = dac_dma_buf.buf_size/2;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_High;
    dma.DMA_M2M = DMA_M2M_Disable;
    DMA_Init (DMA2_Channel3, &dma);
}

void DMA2_Channel3_IRQHandler(void)
{
    int was = 0;
    uint8_t *ptr;

    if(DMA_GetITStatus(DMA2_IT_HT3)) {
        DMA_ClearITPendingBit(DMA2_IT_HT3);
        was++;
        // Sync tail with DMA
        dac_dma_buf.tail = 0;
    }

    if(DMA_GetITStatus(DMA2_IT_TC3)) {
        DMA_ClearITPendingBit(DMA2_IT_TC3);
        was++;
    }

//    __disable_irq();
    if (!was)
        return;
    if (cbuf_read_ptr (&dac_dma_buf, &ptr,dac_dma_buf.buf_size/2) < dac_dma_buf.buf_size/2) {
        // underrun
        DMA_Cmd(DMA2_Channel3, DISABLE);
        cbuf_clear (&dac_dma_buf);
    } else
    {
        cbuf_read_commit (&dac_dma_buf);
    }
//    __enable_irq();
}

void ESP_Speaker_DAC_Interrupts_Init(void)
{
    NVIC_InitTypeDef nvic;

    nvic.NVIC_IRQChannelCmd = ENABLE;
    nvic.NVIC_IRQChannel =   DMA2_Channel3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&nvic);
}

void ESP_Speaker_Start (int sampleRate, int _mp3mode)
{
    audio_out_do_start = 1;
    do_sample_rate = sampleRate;
    do_mp3mode = _mp3mode;
}

static void ESP_Speaker_Do_Start ()
{
     if (audio_out_started)
        ESP_Speaker_Stop();

    cbuf_init (&dac_dma_buf,dac_dma_buf_size);
    memset (dac_dma_buf.buf,0,dac_dma_buf_size);
    DMA2_Channel3->CMAR = (uint32_t)dac_dma_buf.buf;
    DMA2_Channel3->CNDTR = dac_dma_buf.buf_size/2;
    ESP_ADC_DAC_SetSampleRate(do_sample_rate);

    DMA_ITConfig(DMA2_Channel3, DMA_IT_HT | DMA_IT_TC, ENABLE);

    DMA_Cmd(DMA2_Channel3, ENABLE);
    DAC_Cmd(ESP_SPEAKER_DAC_CHANNEL, ENABLE);
    DAC_DMACmd(ESP_SPEAKER_DAC_CHANNEL, ENABLE);

    mp3mode = do_mp3mode;
    if (mp3mode) {
        hMP3Decoder = MP3InitDecoder();
        cbuf_init (&mp3_buf,mp3_buf_size);
        mp3_frame_buf = (uint8_t*)malloc (mp3_frame_max_size);
        mp3_frame_len = 0;
        out_sample_rate = 0;
    }

    audio_out_started = 1;
}


void ESP_Speaker_Stop ()
{
    audio_out_do_start = 0;
}

static void ESP_Speaker_Do_Stop ()
{
    if (!audio_out_started)
        return;

    DAC_DMACmd(ESP_SPEAKER_DAC_CHANNEL, DISABLE);
    DAC_Cmd(ESP_SPEAKER_DAC_CHANNEL, DISABLE);
    DMA_Cmd(DMA2_Channel3, DISABLE);

    DMA_ITConfig(DMA2_Channel3, DMA_IT_HT | DMA_IT_TC, DISABLE);

    cbuf_destroy (&dac_dma_buf);
    if (mp3mode) {
        MP3FreeDecoder(hMP3Decoder);
        cbuf_destroy (&mp3_buf);
        free (mp3_frame_buf);
        hMP3Decoder = 0;
        mp3_frame_buf = 0;
    }

    if (mp3_buf.buf)
        for (;;) {}


    audio_out_started = 0;

}

//#define DEBUG ESP_USART_Debug_Printf

static int ESP_Speaker_MP3_decode ()
{
    uint8_t *mp3_frame_ptr = mp3_frame_buf;
    int mp3_frame_left = mp3_frame_len;
    int err;

    for (;;) {

        int offset = MP3FindSyncWord(mp3_frame_ptr, mp3_frame_left);
        if (offset < 0) {
            // Frame not found - clean buffer
            mp3_frame_len = 0;
            return 0;
        }

        mp3_frame_ptr += offset;
        mp3_frame_left -= offset;

        // check for filled size at least enought for mp3 headers
        if (mp3_frame_left < mp3_frame_headers_size )
            break;

        // Get next frame info. clean buffer if error
        err = MP3GetNextFrameInfo(hMP3Decoder,&mp3FrameInfo,mp3_frame_ptr);
        if (err < 0) {
//            DEBUG ("MP3: GetNextFrameInfo err=%d\r\n",err);
            mp3_frame_left -= 2;
            mp3_frame_ptr += 2;
            continue;
        }

        // Check for available free space in DMA buffer. out if not enougth space
        if (cbuf_free_count(&dac_dma_buf) < mp3FrameInfo.outputSamps/mp3FrameInfo.nChans * sizeof (int16_t))
            break;

        int left = mp3_frame_left;
        uint8_t *ptr = mp3_frame_ptr;

        int16_t mp3_samples_buf[mp3_frame_max_samples*2];

        err = MP3Decode (hMP3Decoder, &ptr, &left, mp3_samples_buf, 0);
        if (err == ERR_MP3_INDATA_UNDERFLOW || err == ERR_MP3_MAINDATA_UNDERFLOW)
            break;
        else if (err < 0) {
//            DEBUG ("MP3: Decode frame err=%d\r\n",err);
            mp3_frame_left -= 2;
            mp3_frame_ptr += 2;
            continue;
        }

        MP3GetLastFrameInfo(hMP3Decoder,&mp3FrameInfo);
//        DEBUG ("MP3: Decode frame ok, bitrate=%d,samplerate=%d,nChans=%d\r\n",mp3FrameInfo.bitrate,mp3FrameInfo.samprate,mp3FrameInfo.nChans);
        if (out_sample_rate != mp3FrameInfo.samprate) {
            ESP_ADC_DAC_SetSampleRate(mp3FrameInfo.samprate);
            out_sample_rate = mp3FrameInfo.samprate;
        }

        mp3_frame_ptr = ptr;
        mp3_frame_left = left;

        int samples = mp3FrameInfo.outputSamps/mp3FrameInfo.nChans;
        int i = 0,room = 0;

        while (samples && (room = cbuf_write_ptr (&dac_dma_buf,&ptr,samples*sizeof(int16_t))/sizeof(int16_t)) != 0) {

            int16_t *iptr = (int16_t*)ptr;
            for (;i < room; ++i)
                *iptr++  = mp3_samples_buf[i*mp3FrameInfo.nChans] + 0x8000;
            samples -= room;
            cbuf_write_commit (&dac_dma_buf);
        }
    }

    if (mp3_frame_left && mp3_frame_left != mp3_frame_len)
        memmove (mp3_frame_buf,mp3_frame_ptr,mp3_frame_left);
    mp3_frame_len = mp3_frame_left;

}

void ESP_Speaker_Decode_Run ()
{
    if (audio_out_do_start == 1)
        ESP_Speaker_Do_Start();
    else if (audio_out_do_start == 0)
        ESP_Speaker_Do_Stop();
    audio_out_do_start = -1;

    if (!mp3mode || !audio_out_started)
        return;

    int sz = cbuf_read (&mp3_buf,mp3_frame_buf+mp3_frame_len,mp3_frame_max_size-mp3_frame_len);
    mp3_frame_len += sz;
    ESP_Speaker_MP3_decode ();

    if (dac_dma_buf.full)
       DMA_Cmd(DMA2_Channel3,ENABLE);
}

void ESP_Speaker_Init ()
{
    InitTMR4();
    ESP_Speaker_DAC_Init ();
    ESP_Speaker_DAC_Interrupts_Init ();
}

