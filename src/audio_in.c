#include "stm32x.h"
#include "audio_in.h"
#include "espeva_stmboard.h"
#include <speex/speex.h>
#include <aioiface.h>

#include "circular_buffer.h"
#include "webrtc_vad.h"

#include "pdm_filter.h"


#define speex_encoded_frame_size 20
#define speex_in_frame_samples 160
#define speex_buf_size (speex_encoded_frame_size*4)
#define pdm_in_size 128
#define pdm_out_samples 16
#define pdm_dma_buf_size (pdm_in_size*2)
#define pdm_samples_buf_size (speex_in_frame_samples*sizeof(int16_t)*2)

static circular_buffer speex_buf;

static void *enc_state;
static SpeexBits bits;
static int quality = 3 , complexity = 1;
static int speex_mode = 0;

static circular_buffer pdm_dma_buf,pdm_samples_buf;
static PDMFilter_InitStruct pf;
static int sample_rate_div = 1;
static int audio_in_started = 0;

static int audio_in_do_start = -1;
static int do_sample_rate = 0;
static int do_speex_mode = 0;


int vad_state = 0;
static VadInst *vad;

circular_buffer *ESP_Mic_Buffer ()
{
    return speex_mode?&speex_buf:&pdm_samples_buf;
}

static void ESP_Mic_PDM_SPI_Init(/*uint32_t Freq*/)
{
    /* Enable the SPI clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);

    DMA_InitTypeDef dma;
    dma.DMA_PeripheralBaseAddr = (uint32_t)&(SPI2->DR);
    dma.DMA_MemoryBaseAddr = (uint32_t)pdm_dma_buf.buf;
    dma.DMA_DIR = DMA_DIR_PeripheralSRC;
    dma.DMA_BufferSize = pdm_dma_buf.buf_size/2;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_Medium;
    dma.DMA_M2M = DMA_M2M_Disable;
    DMA_Init (DMA1_Channel4, &dma);

    GPIO_InitTypeDef gpio;
    gpio.GPIO_Speed   = GPIO_Speed_50MHz;
    gpio.GPIO_Mode    = GPIO_Mode_AF_PP;
    gpio.GPIO_Pin = GPIO_Pin_13;
    GPIO_Init(GPIOB, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_15;
    gpio.GPIO_Mode    = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &gpio);

}

void ESP_Mic_Interrupts_Init ()
{
    NVIC_InitTypeDef nvic;

    nvic.NVIC_IRQChannelCmd = ENABLE;
    nvic.NVIC_IRQChannel =   DMA1_Channel4_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 3;
    nvic.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&nvic);
}


void ESP_Mic_Speex_init ()
{
    speex_bits_init(&bits);
    enc_state = speex_encoder_init(&speex_nb_mode);
    speex_encoder_ctl(enc_state, SPEEX_SET_QUALITY,&quality);
    speex_encoder_ctl(enc_state, SPEEX_SET_COMPLEXITY, &complexity);
}

void ESP_Mic_Speex_deinit ()
{
    speex_bits_destroy (&bits);
    speex_encoder_destroy(&enc_state);
}


int ESP_Mic_Speex_encode (int16_t *buf,uint16_t samples)
{
    uint8_t *ptr = 0;
    if (cbuf_write_ptr(&speex_buf,&ptr,speex_encoded_frame_size) == speex_encoded_frame_size) {

        vad_state = WebRtcVad_Process(vad, 8000, buf,samples);

        speex_bits_reset(&bits);
        speex_encode_int (enc_state, (spx_int16_t*)buf, &bits);
        speex_bits_write(&bits, (char *)ptr, speex_encoded_frame_size);

 //      __disable_irq();
        cbuf_write_commit(&speex_buf);
 //       __enable_irq();
        return 1;
    }
    return 0;
}
void ESP_Mic_Start (int sampleRate,int _speex_mode)
{
    do_sample_rate = sampleRate;
    do_speex_mode  = _speex_mode;
    audio_in_do_start = 1;
}

static void ESP_Mic_Do_Stop ();

static void ESP_Mic_Do_Start ()
{
    if (audio_in_started) {
       //ESP_Mic_Do_Stop ();
       return ;
    }

    speex_mode = do_speex_mode;

    cbuf_init (&pdm_dma_buf,pdm_dma_buf_size);
    cbuf_init (&pdm_samples_buf,pdm_samples_buf_size);
    DMA1_Channel4->CMAR = (uint32_t)pdm_dma_buf.buf;
    DMA1_Channel4->CNDTR = pdm_dma_buf.buf_size/2;

    if (do_sample_rate == 8000) {
        sample_rate_div = 2;
        do_sample_rate = do_sample_rate*sample_rate_div;
    } else
        sample_rate_div = 1;

    if (speex_mode) {
        pf.In_MicChannels = 1;
        pf.Out_MicChannels = 1;
        pf.HP_HZ = 4000;
        pf.LP_HZ = 250;
        pf.Fs = do_sample_rate;
        PDM_Filter_Init (&pf);

        ESP_Mic_Speex_init();
        cbuf_init (&speex_buf,speex_buf_size);
    }

    vad = WebRtcVad_Create ();
    WebRtcVad_Init (vad);
    WebRtcVad_set_mode (vad,0);

    SPI_I2S_DeInit(SPI2);
    I2S_InitTypeDef i2s;
    i2s.I2S_AudioFreq = do_sample_rate*2;//i2s two channel mode
    i2s.I2S_Standard = I2S_Standard_LSB;
    i2s.I2S_DataFormat = I2S_DataFormat_16b;
    i2s.I2S_CPOL = I2S_CPOL_High;
    i2s.I2S_Mode = I2S_Mode_MasterRx;
    i2s.I2S_MCLKOutput = I2S_MCLKOutput_Disable;
    /* Initialize the I2S peripheral with the structure above */
    I2S_Init(SPI2, &i2s);

    DMA_ITConfig(DMA1_Channel4,DMA_IT_TC|DMA_IT_HT, ENABLE);
    I2S_Cmd(SPI2, ENABLE);
    DMA_Cmd(DMA1_Channel4,ENABLE);
    SPI_I2S_DMACmd(SPI2,SPI_I2S_DMAReq_Rx,ENABLE);
    audio_in_started = 1;

}

void ESP_Mic_Stop ()
{
    audio_in_do_start = 0;
}

static void ESP_Mic_Do_Stop ()
{
    if (!audio_in_started)
       return;

    DMA_ITConfig(DMA1_Channel4,DMA_IT_TC|DMA_IT_HT, DISABLE);
    I2S_Cmd(SPI2, DISABLE);
    DMA_Cmd(DMA1_Channel4,DISABLE);
    SPI_I2S_DMACmd(SPI2,SPI_I2S_DMAReq_Rx,DISABLE);

    if (speex_mode) {
        ESP_Mic_Speex_deinit ();
        cbuf_destroy (&speex_buf);
    }

    cbuf_destroy (&pdm_dma_buf);
    cbuf_destroy (&pdm_samples_buf);

    speex_mode = 0;
    WebRtcVad_Free (vad);
    audio_in_started = 0;

}

void ESP_Mic_Encode_Run ()
{
    if (audio_in_do_start == 1)
        ESP_Mic_Do_Start();
    else if (audio_in_do_start == 0)
        ESP_Mic_Do_Stop();
    audio_in_do_start = -1;

    if (!speex_mode || !audio_in_started)
        return;
    uint8_t *ptr;
    if (cbuf_read_ptr (&pdm_samples_buf,&ptr,speex_in_frame_samples*2) == speex_in_frame_samples*2) {
        if (ESP_Mic_Speex_encode((int16_t *)ptr,speex_in_frame_samples))
            cbuf_read_commit (&pdm_samples_buf);
    }
}

void ESP_Mic_Init ()
{
    ESP_Mic_PDM_SPI_Init ();
    ESP_Mic_Interrupts_Init ();
}


void DMA1_Channel4_IRQHandler(void)
{
    int16_t outbuf[pdm_out_samples];
    int i,cur_buf = -1;
    if(DMA_GetITStatus(DMA1_IT_TC4)) {
        DMA_ClearITPendingBit(DMA1_IT_TC4);
        cur_buf =pdm_dma_buf_size/2;
    }
    if(DMA_GetITStatus(DMA1_IT_HT4)) {
        DMA_ClearITPendingBit(DMA1_IT_HT4);
        cur_buf =0;
    }

    if (cur_buf >=0) {

        uint16_t *p = (uint16_t *) (pdm_dma_buf.buf+cur_buf);
        for (i = 0; i < pdm_in_size/sizeof(uint16_t); i++)
            p[i] = HTONS (p[i]);

        PDM_Filter_64_LSB(pdm_dma_buf.buf+cur_buf,(uint16_t*)outbuf,64,&pf);
        if (sample_rate_div == 2)
        {
            for (i = 0; i < pdm_out_samples/sample_rate_div; ++i)
                outbuf[i] = outbuf[i*sample_rate_div]/sample_rate_div + outbuf[1+i*sample_rate_div]/sample_rate_div;
        }
        cbuf_write (&pdm_samples_buf,(uint8_t*)outbuf,sizeof(outbuf)/sample_rate_div);
    }
}
