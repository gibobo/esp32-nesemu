/* Copyright (c) 2020, Peter Barrett
**
** Permission to use, copy, modify, and/or distribute this software for
** any purpose with or without fee is hereby granted, provided that the
** above copyright notice and this permission notice appear in all copies.
**
** THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
** WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR
** BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES
** OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
** WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,
** ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
** SOFTWARE.
*/

#include "driver/dac.h"
#include "driver/gpio.h"
#include "driver/i2s.h"
#include "driver/periph_ctrl.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_intr_alloc.h"
#include "esp_types.h"
#include <esp_log.h>
#include "rom/gpio.h"
#include "rom/lldesc.h"
#include "soc/gpio_reg.h"
#include "soc/i2s_reg.h"
#include "soc/i2s_struct.h"
#include "soc/io_mux_reg.h"
#include "soc/ledc_struct.h"
#include "soc/rtc.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc.h"

#include "config.h"
#include "palette.h"

static const char *TAG = "VIDEO";
static int _pal_ = 0;
//====================================================================================================
// low level HW setup of DAC/DMA/APLL/PWM
//====================================================================================================
static lldesc_t _dma_desc[2] = {0};
static intr_handle_t _isr_handle;

uint8_t **_lines = 0; // filled in by emulator

volatile int _line_counter = 0;
volatile uint32_t _frame_counter = 0;

int _active_lines;
int _line_count;

int _line_width;
int _samples_per_cc;
const uint32_t *_palette;

float _sample_rate;
int _hsync;
int _hsync_long;
int _hsync_short;
int _burst_start;
int _burst_width;
int _active_start;

int16_t *_burst0 = 0; // pal bursts
int16_t *_burst1 = 0;

int _machine; // 2:1 3:2 4:3 3:4 input pixel to color clock ratio

#define DMA_BUFFER_UINT16 ((uint16_t *)((lldesc_t *)I2S0.out_eof_des_addr)->buf)
#define DMA_BUFFER_UINT8 ((uint8_t *)((lldesc_t *)I2S0.out_eof_des_addr)->buf)
#define DMA_BUFFER_UINT32 ((uint32_t *)((lldesc_t *)I2S0.out_eof_des_addr)->buf)

static inline IRAM_ATTR void video_isr(void);

// simple isr
static void IRAM_ATTR i2s_intr_handler_video(void *arg)
{
    if (I2S0.int_st.out_eof)
        video_isr();
    // reset the interrupt
    I2S0.int_clr.val = I2S0.int_st.val;
}

static esp_err_t start_dma(int line_width, int samples_per_cc)
{
    periph_module_enable(PERIPH_I2S0_MODULE);

    const size_t dma_buffer_size_bytes = line_width * 2;
    if (dma_buffer_size_bytes >= 4092)
    {
        printf("DMA chunk too big:%d\n", dma_buffer_size_bytes);
        return -1;
    }

    // setup interrupt
    if (esp_intr_alloc(ETS_I2S0_INTR_SOURCE, ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_INTRDISABLED,
                       i2s_intr_handler_video, 0, &_isr_handle) != ESP_OK)
        return -1;

    // reset conf
    I2S0.conf.val = 1;
    I2S0.conf.val = 0;
    I2S0.conf.tx_right_first = 1;
    I2S0.conf.tx_mono = 1;
    I2S0.conf2.lcd_en = 1;
    I2S0.fifo_conf.tx_fifo_mod_force_en = 1;
    I2S0.sample_rate_conf.tx_bits_mod = 16; // DAC uses MSB 8 bits of 16
    I2S0.conf_chan.tx_chan_mod = 1;         // Mono mode
    I2S0.clkm_conf.clkm_div_num = 1;        // I2S clock divider’s integral value.
    I2S0.clkm_conf.clkm_div_b = 0;          // Fractional clock divider’s numerator value.
    I2S0.clkm_conf.clkm_div_a = 1;          // Fractional clock divider’s denominator value
    I2S0.sample_rate_conf.tx_bck_div_num = 1;
    I2S0.clkm_conf.clka_en = 1;     // Set this bit to enable clk_apll.
    I2S0.fifo_conf.tx_fifo_mod = 1; // 16-bit single channel data

    // Create TX DMA buffers
    const size_t DMA_BUFFER_COUNT = sizeof(_dma_desc) / sizeof(lldesc_t);
    for (size_t i = 0; i < DMA_BUFFER_COUNT; i++)
    {
        _dma_desc[i].buf = (uint8_t *)heap_caps_calloc(dma_buffer_size_bytes, sizeof(uint8_t), MALLOC_CAP_DMA);
        assert(_dma_desc[i].buf != NULL);
        _dma_desc[i].owner = 1;
        _dma_desc[i].eof = 1;
        _dma_desc[i].length = dma_buffer_size_bytes;
        _dma_desc[i].size = dma_buffer_size_bytes;
        _dma_desc[i].empty = (uint32_t)(i == (DMA_BUFFER_COUNT - 1) ? &_dma_desc[0] : &_dma_desc[i + 1]);
    }
    I2S0.out_link.addr = (uint32_t)&_dma_desc[0];
    ESP_LOGI(TAG, "DMA buffers configured. Buffers: %u, Size: %u bytes each", DMA_BUFFER_COUNT, dma_buffer_size_bytes);

    //  Setup up the apll: See ref 3.2.7 Audio PLL
    //  f_xtal = (int)rtc_clk_xtal_freq_get() * 1000000;
    //  f_out = xtal_freq * (4 + sdm2 + sdm1/256 + sdm0/65536); // 250 < f_out < 500
    //  apll_freq = f_out/((o_div + 2) * 2)
    //  operating range of the f_out is 250 MHz ~ 500 MHz
    //  operating range of the apll_freq is 16 ~ 128 MHz.
    //  select sdm0,sdm1,sdm2 to produce nice multiples of colorburst frequencies

    //  see calc_freq() for math: (4+a)*10/((2 + b)*2) mhz
    //  up to 20mhz seems to work ok:
    //  rtc_clk_apll_enable(1,0x00,0x00,0x4,0);   // 20mhz for fancy DDS

    if (!_pal_)
    {
        switch (samples_per_cc)
        {
        case 3:
            rtc_clk_apll_enable(1, 0x46, 0x97, 0x4, 2);
            break; // 10.7386363636 3x NTSC (10.7386398315mhz)
        case 4:
            rtc_clk_apll_enable(1, 0x46, 0x97, 0x4, 1);
            break; // 14.3181818182 4x NTSC (14.3181864421mhz)
        }
    }
    else
    {
        rtc_clk_apll_enable(1, 0x04, 0xA4, 0x6, 1); // 17.734476mhz ~4x PAL
    }

    dac_output_enable(DAC_CHANNEL_1); // DAC, video on GPIO25
    dac_i2s_enable();                 // start DAC!

    I2S0.conf.tx_start = 1; // start DMA!
    I2S0.int_clr.val = UINT32_MAX;
    I2S0.int_ena.out_eof = 1;
    I2S0.out_link.start = 1;
    return esp_intr_enable(_isr_handle); // start interruprs!
}

void *MALLOC32(int x, const char *label)
{
    printf("MALLOC32 %d free, %d biggest, allocating %s:%d\n",
           heap_caps_get_free_size(MALLOC_CAP_32BIT), heap_caps_get_largest_free_block(MALLOC_CAP_32BIT), label, x);
    void *r = heap_caps_malloc(x, MALLOC_CAP_32BIT);
    if (!r)
    {
        printf("MALLOC32 FAILED allocation of %s:%d!!!!####################\n", label, x);
        esp_restart();
    }
    else
        printf("MALLOC32 allocation of %s:%d %08X\n", label, x, (unsigned int)r);
    return r;
}

//===================================================================================================
// ntsc tables
//===================================================================================================
// AA AA                // 2 pixels, 1 color clock - atari
// AA AB BB             // 3 pixels, 2 color clocks - nes
// AAA ABB BBC CCC      // 4 pixels, 3 color clocks - sms

// cc == 3 gives 684 samples per line, 3 samples per cc, 3 pixels for 2 cc
// cc == 4 gives 912 samples per line, 4 samples per cc, 2 pixels per cc
//====================================================================================================
// GLOBAL
//====================================================================================================

//====================================================================================================
//====================================================================================================

uint32_t cpu_ticks()
{
    return xthal_get_ccount();
}

uint32_t us()
{
    return cpu_ticks() / 240;
}

// Color clock frequency is 315/88 (3.57954545455)
// DAC_MHZ is 315/11 or 8x color clock
// 455/2 color clocks per line, round up to maintain phase
// HSYNCH period is 44/315*455 or 63.55555..us
// Field period is 262*44/315*455 or 16651.5555us

#define P0 (color >> 16)
#define P1 (color >> 8)
#define P2 (color)
#define P3 (color << 8)

#ifdef PERF
uint32_t _blit_ticks_min = 0;
uint32_t _blit_ticks_max = 0;
uint32_t _isr_us = 0;
#define BEGIN_TIMING() uint32_t t = cpu_ticks()
#define END_TIMING()                           \
    t = cpu_ticks() - t;                       \
    _blit_ticks_min = min(_blit_ticks_min, t); \
    _blit_ticks_max = max(_blit_ticks_max, t);
#define ISR_BEGIN() uint32_t t = cpu_ticks()
#define ISR_END()        \
    t = cpu_ticks() - t; \
    _isr_us += (t + 120) / 240;
#else
uint32_t _isr_us = 0;
#define BEGIN_TIMING()
#define END_TIMING()
#define ISR_BEGIN()
#define ISR_END()
#endif

static int usec(float us)
{
    return _samples_per_cc * round(us * _sample_rate / _samples_per_cc); // multiple of color clock, word align
}

#define NTSC_COLOR_CLOCKS_PER_SCANLINE 228 // really 227.5 for NTSC but want to avoid half phase fiddling for now
#define NTSC_FREQUENCY (315000000.0 / 88)
#define NTSC_LINES 262

#define PAL_COLOR_CLOCKS_PER_SCANLINE 284 // really 283.75 ?
#define PAL_FREQUENCY 4433618.75
#define PAL_LINES 312

// Wait for front and back buffers to swap before starting drawing
void video_sync()
{
    if (!_lines)
        return;

    // Tips 1
    // ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Tips 2
    int n = 0;
    // if (_line_counter < _active_lines)
    //   n = (_active_lines - _line_counter) * 1000 / 15600;
    vTaskDelay(n + 1);
}

uint32_t nes_3_phase[64] = {
    0x2C2C2C00, 0x241D2400, 0x221D2600, 0x1F1F2700, 0x1D222600, 0x1D242400, 0x1D262200, 0x1F271F00,
    0x22261D00, 0x24241D00, 0x26221D00, 0x271F1F00, 0x261D2200, 0x14141400, 0x14141400, 0x14141400,
    0x38383800, 0x2C252C00, 0x2A252E00, 0x27272F00, 0x252A2E00, 0x252C2C00, 0x252E2A00, 0x272F2700,
    0x2A2E2500, 0x2C2C2500, 0x2E2A2500, 0x2F272700, 0x2E252A00, 0x1F1F1F00, 0x15151500, 0x15151500,
    0x45454500, 0x3A323A00, 0x37333C00, 0x35353C00, 0x33373C00, 0x323A3A00, 0x333C3700, 0x353C3500,
    0x373C3300, 0x3A3A3200, 0x3C373300, 0x3C353500, 0x3C333700, 0x2B2B2B00, 0x16161600, 0x16161600,
    0x45454500, 0x423B4200, 0x403B4400, 0x3D3D4500, 0x3B404400, 0x3B424200, 0x3B444000, 0x3D453D00,
    0x40443B00, 0x42423B00, 0x44403B00, 0x453D3D00, 0x443B4000, 0x39393900, 0x17171700, 0x17171700};

uint32_t nes_4_phase[64] = {
    0x27272727, 0x1B16191E, 0x1D151921, 0x1C151920, 0x1A1A1F20, 0x171E231C, 0x171E231C, 0x171C201A,
    0x17191A18, 0x1B1A1819, 0x1D1C191A, 0x1D1C191A, 0x1C1A191C, 0x17171717, 0x17171717, 0x17171717,
    0x3A3A3A3A, 0x2C1F1F2C, 0x281A1E2D, 0x231D282E, 0x1E212F2C, 0x1A263428, 0x192A3423, 0x1D2A3023,
    0x23292822, 0x28261F20, 0x2A281F21, 0x2B271F23, 0x2D241F28, 0x17171717, 0x17171717, 0x17171717,
    0x50505050, 0x3E2D2A3B, 0x362B2F3A, 0x332E373C, 0x33374441, 0x313A443C, 0x303D4437, 0x2F414332,
    0x34413D30, 0x393E342E, 0x3D3A2B2F, 0x41352733, 0x43312839, 0x21212121, 0x17171717, 0x17171717,
    0x50505050, 0x48414047, 0x443F4146, 0x43404447, 0x44454B4A, 0x43464B47, 0x42484B45, 0x424A4B43,
    0x444A4842, 0x46494442, 0x48474142, 0x4A453F44, 0x4A433F46, 0x3C3C3C3C, 0x17171717, 0x17171717};

#if VIDEO_STANDARD == NTSC
//=====================================================================================
// NTSC VIDEO
//=====================================================================================
void video_init(int samples_per_cc, int machine, const uint32_t *palette, int ntsc)
{
    _pal_ = (ntsc) ? 0 : 1;
    _samples_per_cc = samples_per_cc;

    if (ntsc)
    {
        _sample_rate = 315.0 / 88 * samples_per_cc; // DAC rate
        _line_width = NTSC_COLOR_CLOCKS_PER_SCANLINE * samples_per_cc;
        _line_count = NTSC_LINES;
        _hsync_long = usec(63.555f - 4.7f);
        _active_start = usec(samples_per_cc == 4 ? 10.f : 10.5f);
        _hsync = usec(4.7f);
        _palette = ntsc_RGB332;
    }
    else
    {
        int cc_width = 4;
        _sample_rate = PAL_FREQUENCY * cc_width / 1000000.0; // DAC rate in mhz
        _line_width = PAL_COLOR_CLOCKS_PER_SCANLINE * cc_width;
        _line_count = PAL_LINES;
        _hsync_short = usec(2.f);
        _hsync_long = usec(30.f);
        _hsync = usec(4.7f);
        _burst_start = usec(5.6f);
        _burst_width = (int)(10 * cc_width + 4) & 0xFFFE;
        _active_start = usec(10.4f);
        _palette = pal_yuyv;

        // make colorburst tables for even and odd lines
        _burst0 = malloc(_burst_width * sizeof(int16_t));
        _burst1 = malloc(_burst_width * sizeof(int16_t));
        float phase = M_PI;
        for (int i = 0; i < _burst_width; i++)
        {
            _burst0[i] = BLANKING_LEVEL + sin(phase + 3.f * M_PI / 4.f) * BLANKING_LEVEL / 1.5f;
            _burst1[i] = BLANKING_LEVEL + sin(phase - 3.f * M_PI / 4.f) * BLANKING_LEVEL / 1.5f;
            phase += 2.f * M_PI / cc_width;
        }
    }
    _active_lines = 240;
    ESP_ERROR_CHECK(start_dma(_line_width, _samples_per_cc));
}

// draw a line of game in NTSC
static inline IRAM_ATTR void blit(uint8_t *src, uint16_t *dst)
{
    uint32_t *p = _palette;
    uint32_t color, c;
    uint32_t mask = 0xFF;
    int i;

    BEGIN_TIMING();

    switch (_machine)
    {
    case EMU_ATARI:
        // 2 pixels per color clock, 4 samples per cc, used by atari
        // AA AA
        // 192 color clocks wide
        // only show 336 pixels
        src += 24;
        dst += 16;
        for (i = 0; i < (384 - 48); i += 4, dst += 4, src += 4)
        {
            uint32_t c = *((uint32_t *)src); // screen may be in 32 bit mem
            dst[0] = p[(uint8_t)c];
            dst[1] = p[(uint8_t)(c >> 8)] << 8;
            dst[2] = p[(uint8_t)(c >> 16)];
            dst[3] = p[(uint8_t)(c >> 24)] << 8;
        }
        break;

        // case EMU_NES:
        //   // 3 pixels to 2 color clocks, 3 samples per cc, used by nes
        //   // could be faster with better tables: 2953 cycles ish
        //   // about 18% of the core at 240Mhz
        //   // 170 color clocks wide: not all that attractive
        //   // AA AB BB
        //   for (i = 0; i < 255; i += 3)
        //   {
        //     color = p[src[i + 0] & 0x3F];
        //     dst[0] = P0;
        //     dst[1] = P1;
        //     color = p[src[i + 1] & 0x3F];
        //     dst[2] = P2;
        //     dst[3] = P0;
        //     color = p[src[i + 2] & 0x3F];
        //     dst[4] = P1;
        //     dst[5] = P2;
        //     dst += 6;
        //   }
        //   // last pixel
        //   color = p[src[i + 0]];
        //   dst[0] = P0;
        //   dst[1] = P1;
        //   break;

    case EMU_NES:
        mask = 0x3F; // 63
        p = nes_3_phase;
    case EMU_SMS:
        // AAA ABB BBC CCC
        // 4 pixels, 3 color clocks, 4 samples per cc
        // each pixel gets 3 samples, 192 color clocks wide
        for (i = 0; i < 256; i += 4, dst += 12)
        {
            c = *((uint32_t *)(src + i));
            color = p[c & mask];
            dst[0] = P0;
            dst[1] = P1;
            dst[2] = P2;
            color = p[(c >> 8) & mask];
            dst[3] = P3;
            dst[4] = P0;
            dst[5] = P1;
            color = p[(c >> 16) & mask];
            dst[6] = P2;
            dst[7] = P3;
            dst[8] = P0;
            color = p[(c >> 24) & mask];
            dst[9] = P1;
            dst[10] = P2;
            dst[11] = P3;
        }
        break;
    }
    END_TIMING();
}

static inline IRAM_ATTR void burst(uint16_t *line)
{
    int i, phase;
    switch (_samples_per_cc)
    {
    case 4:
        // 4 samples per color clock
        // Breezeway (delay colorburst by two cycles following the sync pulse)
        for (int i = _hsync + 8; i < _hsync + 16; i++)
            line[i] = BLANKING_LEVEL;
        // Color burst 9 cycles
        for (i = _hsync + 16; i < _hsync + 16 + (4 * 9); i += 4)
        {
            line[i + 1] = BLANKING_LEVEL;
            line[i + 0] = BLANKING_LEVEL + BLANKING_LEVEL / 2;
            line[i + 3] = BLANKING_LEVEL;
            line[i + 2] = BLANKING_LEVEL - BLANKING_LEVEL / 2;
        }
        break;
    case 3:
        // 3 samples per color clock
        phase = 0.866025f * BLANKING_LEVEL / 2.f;
        for (i = _hsync; i < _hsync + (3 * 10); i += 6)
        {
            line[i + 1] = BLANKING_LEVEL;
            line[i + 0] = BLANKING_LEVEL + phase;
            line[i + 3] = BLANKING_LEVEL - phase;
            line[i + 2] = BLANKING_LEVEL;
            line[i + 5] = BLANKING_LEVEL + phase;
            line[i + 4] = BLANKING_LEVEL - phase;
        }
        break;
    }
}

static inline IRAM_ATTR void sync(uint16_t *line, int syncwidth)
{
    // Front porch
    for (int i = 0; i < 8; i++)
        line[i] = BLANKING_LEVEL;
    // Sync pulse
    for (int i = 8; i < syncwidth + 8; i++)
        line[i] = SYNC_LEVEL;
}

static inline IRAM_ATTR void blanking(uint16_t *line, bool vbl)
{
    int syncwidth = vbl ? _hsync_long : _hsync;
    sync(line, syncwidth);
    for (int i = syncwidth; i < _line_width; i++)
        line[i] = BLANKING_LEVEL;
    if (!vbl)
        burst(line); // no burst during vbl
}

// Workhorse ISR handles audio and video updates
static inline IRAM_ATTR void video_isr(void)
{
    if (!_lines)
        return;

    ISR_BEGIN();

    int i = _line_counter++;
    uint16_t *buf = DMA_BUFFER_UINT16;
    if (i < _active_lines)
    {
        // active video
        sync(buf, _hsync);
        burst(buf);
        blit(_lines[i], buf + _active_start);
    }
    else if (i < (_active_lines + 5))
    { // post render/black
        blanking(buf, false);
    }
    else if (i < (_active_lines + 8))
    { // vsync
        blanking(buf, true);
    }
    else
    { // pre render/black
        blanking(buf, false);
    }

    if (_line_counter == _line_count)
    {
        _line_counter = 0; // frame is done
        _frame_counter++;
    }

    ISR_END();
}

#else
//=====================================================================================
// PAL VIDEO
//=====================================================================================
static inline IRAM_ATTR void blit(uint8_t *src, uint16_t *dst)
{
    uint32_t c, color;
    bool even = _line_counter & 1;
    const uint32_t *p = even ? _palette : _palette + 256;
    int left = 0;
    int right = 256;
    uint8_t mask = 0xFF;
    uint8_t c0, c1, c2, c3, c4;
    uint8_t y1, y2, y3;

    switch (_machine)
    {
    case EMU_ATARI:
        // pal is 5/4 wider than ntsc to account for pal 288 color clocks per line vs 228 in ntsc
        // so do an ugly stretch on pixels (actually luma) to accomodate -> 384 pixels are now 240 pal color clocks wide
        left = 24;
        right = 384 - 24; // only show center 336 pixels
        dst += 40;
        for (int i = left; i < right; i += 4)
        {
            c = *((uint32_t *)(src + i));

            // make 5 colors out of 4 by interpolating y: 0000 0111 1122 2223 3333
            c0 = c;
            c1 = c >> 8;
            c3 = c >> 16;
            c4 = c >> 24;
            y1 = (((c1 & 0xF) << 1) + ((c0 + c1) & 0x1F) + 2) >> 2; // (c0 & 0xF)*0.25 + (c1 & 0xF)*0.75;
            y2 = ((c1 + c3 + 1) >> 1) & 0xF;                        // (c1 & 0xF)*0.50 + (c2 & 0xF)*0.50;
            y3 = (((c3 & 0xF) << 1) + ((c3 + c4) & 0x1F) + 2) >> 2; // (c2 & 0xF)*0.75 + (c3 & 0xF)*0.25;
            c1 = (c1 & 0xF0) + y1;
            c2 = (c1 & 0xF0) + y2;
            c3 = (c3 & 0xF0) + y3;

            color = p[c0];
            dst[0] = P0;
            dst[1] = P1;
            color = p[c1];
            dst[2] = P2;
            dst[3] = P3;
            color = p[c2];
            dst[4] = P0;
            dst[5] = P1;
            color = p[c3];
            dst[6] = P2;
            dst[7] = P3;
            color = p[c4];
            dst[8] = P0;
            dst[9] = P1;

            i += 4;
            c = *((uint32_t *)(src + i));

            // make 5 colors out of 4 by interpolating y: 0000 0111 1122 2223 3333
            c0 = c;
            c1 = c >> 8;
            c3 = c >> 16;
            c4 = c >> 24;
            y1 = (((c1 & 0xF) << 1) + ((c0 + c1) & 0x1F) + 2) >> 2; // (c0 & 0xF)*0.25 + (c1 & 0xF)*0.75;
            y2 = ((c1 + c3 + 1) >> 1) & 0xF;                        // (c1 & 0xF)*0.50 + (c2 & 0xF)*0.50;
            y3 = (((c3 & 0xF) << 1) + ((c3 + c4) & 0x1F) + 2) >> 2; // (c2 & 0xF)*0.75 + (c3 & 0xF)*0.25;
            c1 = (c1 & 0xF0) + y1;
            c2 = (c1 & 0xF0) + y2;
            c3 = (c3 & 0xF0) + y3;

            color = p[c0];
            dst[10] = P2;
            dst[11] = P3;
            color = p[c1];
            dst[12] = P0;
            dst[13] = P1;
            color = p[c2];
            dst[14] = P2;
            dst[15] = P3;
            color = p[c3];
            dst[16] = P0;
            dst[17] = P1;
            color = p[c4];
            dst[18] = P2;
            dst[19] = P3;
            dst += 20;
        }
        return;

    case EMU_NES:
        // 192 of 288 color clocks wide: roughly correct aspect ratio
        mask = 0x3F;
        if (!even)
            p = _palette + 64;
        dst += 88;
        break;

    case EMU_SMS:
        // 192 of 288 color clocks wide: roughly correct aspect ratio
        dst += 88;
        break;
    }

    // 4 pixels over 3 color clocks, 12 samples
    // do the blitting
    for (int i = left; i < right; i += 4)
    {
        c = *((uint32_t *)(src + i));
        color = p[c & mask];
        dst[0] = P0;
        dst[1] = P1;
        dst[2] = P2;
        color = p[(c >> 8) & mask];
        dst[3] = P3;
        dst[4] = P0;
        dst[5] = P1;
        color = p[(c >> 16) & mask];
        dst[6] = P2;
        dst[7] = P3;
        dst[8] = P0;
        color = p[(c >> 24) & mask];
        dst[9] = P1;
        dst[10] = P2;
        dst[11] = P3;
        dst += 12;
    }
}

static inline IRAM_ATTR void burst(uint16_t *line)
{
    line += _burst_start;
    int16_t *b = (_line_counter & 1) ? _burst0 : _burst1;
    for (int i = 0; i < _burst_width; i += 2)
    {
        line[i] = b[i];
        line[i + 1] = b[i + 1];
    }
}

static inline IRAM_ATTR void sync(uint16_t *line, int syncwidth)
{
    // Front porch
    for (int i = 0; i < 8; i++)
        line[i] = BLANKING_LEVEL;
    // Sync
    for (int i = 8; i < syncwidth; i++)
        line[i] = SYNC_LEVEL;
}

static inline IRAM_ATTR void blanking(uint16_t *line, bool vbl)
{
    int syncwidth = vbl ? _hsync_long : _hsync;
    sync(line, syncwidth);
    for (int i = syncwidth; i < _line_width; i++)
        line[i] = BLANKING_LEVEL;
    if (!vbl)
        burst(line); // no burst during vbl
}

// Fancy pal non-interlace
// http://martin.hinner.info/vga/pal.html
static inline IRAM_ATTR void vsync2(uint16_t *line, int width, int swidth)
{
    swidth = swidth ? _hsync_long : _hsync_short;
    int i;
    for (i = 0; i < swidth; i++)
        line[i] = SYNC_LEVEL;
    for (; i < width; i++)
        line[i] = BLANKING_LEVEL;
}

static uint8_t DRAM_ATTR _sync_type[8] = {0, 0, 0, 3, 3, 2, 0, 0};

static inline IRAM_ATTR void vsync(uint16_t *line, int i)
{
    uint8_t t = _sync_type[i - 304];
    vsync2(line, _line_width / 2, t & 2);
    vsync2(line + _line_width / 2, _line_width / 2, t & 1);
}

// Workhorse ISR handles audio and video updates
static inline IRAM_ATTR void video_isr(void)
{
    if (!_lines)
        return;

    ISR_BEGIN();

    int i = _line_counter++;
    uint16_t *buf = DMA_BUFFER_UINT16;
    if (i < 32)
    {
        blanking(buf, false); // pre render/black 0-32
    }
    else if (i < _active_lines + 32)
    { // active video 32-272
        sync(buf, _hsync);
        burst(buf);
        blit(_lines[i - 32], buf + _active_start);
    }
    else if (i < 304)
    { // post render/black 272-304
        blanking(buf, false);
    }
    else
    {
        vsync(buf, i); // 8 lines of sync 304-312
    }

    if (_line_counter == _line_count)
    {
        _line_counter = 0; // frame is done
        _frame_counter++;
    }

    ISR_END();
}

#endif
