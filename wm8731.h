/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Jerzy Kasenberg
 * Copyright (c) 2022 Angel Molina 
 * Copyright (c) 2023 Dhiru Kholia 
 * Copyright (c) 2026 Riyas Vettukattil  
 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */
#ifndef WM8731_H_
#define WM8731_H_

#include <stdint.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------
 * I2C address
 * CSB pin low  -> 0x1A, CSB pin high -> 0x1B
 * ------------------------------------------------------------------------- */
#define WM8731_I2C_ADDR         0x1A

/* -------------------------------------------------------------------------
 * GPIO pin definitions
 * ------------------------------------------------------------------------- */
#define WM8731_I2C_SDA_PIN      0
#define WM8731_I2C_SCL_PIN      1
#define WM8731_SDI_PIN          2   /* WM8731 -> RP2040  (din_pin_base)     */
#define WM8731_BCLK_PIN         3   /* RP2040 -> WM8731  (clock_pin_base)   */
#define WM8731_LRCK_PIN         4   /* RP2040 -> WM8731  (clock_pin_base+1) */
#define WM8731_SDO_PIN          5   /* RP2040 -> WM8731  (dout_pin)         */
#define WM8731_MCLK_PIN         6   /* PWM master clock                     */

/* Verify the consecutive-pin constraint at compile time */
_Static_assert(WM8731_BCLK_PIN == WM8731_SDI_PIN  + 1, "BCLK must be SDI+1");
_Static_assert(WM8731_LRCK_PIN == WM8731_BCLK_PIN + 1, "LRCK must be BCLK+1");

/* -------------------------------------------------------------------------
 * WM8731 Register addresses
 * ------------------------------------------------------------------------- */
#define WM8731_REG_LLINE_IN     0x00
#define WM8731_REG_RLINE_IN     0x01
#define WM8731_REG_LHPHONE_OUT  0x02
#define WM8731_REG_RHPHONE_OUT  0x03
#define WM8731_REG_ANALOG_PATH  0x04
#define WM8731_REG_DIGITAL_PATH 0x05
#define WM8731_REG_POWER_DOWN   0x06
#define WM8731_REG_DIGITAL_IF   0x07
#define WM8731_REG_SAMPLING     0x08
#define WM8731_REG_ACTIVE       0x09
#define WM8731_REG_RESET        0x0F

/* -------------------------------------------------------------------------
 * Register bit definitions
 * ------------------------------------------------------------------------- */

/* Left/Right Line In (0x00, 0x01) */
#define WM8731_LINVOL_MASK      0x1F
#define WM8731_LINMUTE          (1 << 7)
#define WM8731_LRINBOTH         (1 << 8)

/* Left/Right Headphone Out (0x02, 0x03) */
#define WM8731_HPVOL_MASK       0x7F
#define WM8731_HPZCEN           (1 << 7)
#define WM8731_HPBOTH           (1 << 8)

/* Analog Audio Path Control (0x04) */
#define WM8731_MICBOOST         (1 << 0)
#define WM8731_MUTEMIC          (1 << 1)
#define WM8731_INSEL            (1 << 2)
#define WM8731_BYPASS           (1 << 3)
#define WM8731_DACSEL           (1 << 4)
#define WM8731_SIDETONE         (1 << 5)
#define WM8731_SIDEATT_MASK     (3 << 6)

/* Digital Audio Path Control (0x05) */
#define WM8731_ADCHPD           (1 << 0)
#define WM8731_DEEMP_MASK       (3 << 1)
#define WM8731_DEEMP_DISABLE    (0 << 1)
#define WM8731_DEEMP_32K        (1 << 1)
#define WM8731_DEEMP_44K        (2 << 1)
#define WM8731_DEEMP_48K        (3 << 1)
#define WM8731_DACMU            (1 << 3)
#define WM8731_HPOR             (1 << 4)

/* Power Down Control (0x06) */
#define WM8731_LINEINPD         (1 << 0)
#define WM8731_MICPD            (1 << 1)
#define WM8731_ADCPD            (1 << 2)
#define WM8731_DACPD            (1 << 3)
#define WM8731_OUTPD            (1 << 4)
#define WM8731_OSCPD            (1 << 5)
#define WM8731_CLKOUTPD         (1 << 6)
#define WM8731_POWEROFF         (1 << 7)

/* Digital Audio Interface Format (0x07) */
#define WM8731_FORMAT_MASK      (3 << 0)
#define WM8731_FORMAT_RIGHT     (0 << 0)
#define WM8731_FORMAT_LEFT      (1 << 0)
#define WM8731_FORMAT_I2S       (2 << 0)
#define WM8731_FORMAT_DSP       (3 << 0)
#define WM8731_IWL_MASK         (3 << 2)
#define WM8731_IWL_16BIT        (0 << 2)
#define WM8731_IWL_20BIT        (1 << 2)
#define WM8731_IWL_24BIT        (2 << 2)
#define WM8731_IWL_32BIT        (3 << 2)
#define WM8731_LRP              (1 << 4)
#define WM8731_LRSWAP           (1 << 5)
#define WM8731_MS               (1 << 6)  /* 1=master, 0=slave */
#define WM8731_BCLKINV          (1 << 7)

/* Sampling Control (0x08) */
#define WM8731_USB_MODE         (1 << 0)
#define WM8731_BOSR             (1 << 1)
#define WM8731_SR_MASK          (0xF << 2)
#define WM8731_CLKIDIV2         (1 << 6)
#define WM8731_CLKODIV2         (1 << 7)

/* Sample rate SR field values */
#define WM8731_SR_48K_NORMAL    (0x00 << 2)
#define WM8731_SR_48K_USB       (0x00 << 2)
#define WM8731_SR_44K1_NORMAL   (0x08 << 2)
#define WM8731_SR_96K_NORMAL    (0x07 << 2)

/* Active Control (0x09) */
#define WM8731_ACTIVE           (1 << 0)

/* -------------------------------------------------------------------------
 * DMA double-buffer parameters
 *
 * WM8731_DMA_BUFFER_SIZE : number of int32_t words per buffer per direction.
 *   Each word is one stereo sample (L and R are interleaved: L=even, R=odd).
 *   1024 words = 512 stereo frames ≈ 10.7 ms @ 48 kHz.
 *
 * WM8731_DMA_NUM_BUFFERS : always 2 (ping-pong / double-buffer).
 * ------------------------------------------------------------------------- */
#define WM8731_DMA_BUFFER_SIZE  960
#define WM8731_DMA_NUM_BUFFERS  2

/*
 * TX buffer: written by the application (USB audio out), read by DMA -> PIO TX FIFO.
 * RX buffer: written by DMA (PIO RX FIFO -> buffer), read by the application.
 *
 * Layout: [buffer_index][sample_index]
 *   buffer_index 0 or 1 (ping-pong)
 *   sample_index even = left channel, odd = right channel
 */
extern int32_t wm8731_tx_buffer[WM8731_DMA_NUM_BUFFERS][WM8731_DMA_BUFFER_SIZE];
extern int32_t wm8731_rx_buffer[WM8731_DMA_NUM_BUFFERS][WM8731_DMA_BUFFER_SIZE];

/*
 * wm8731_current_tx_buffer – index of the buffer DMA is currently PLAYING.
 *   The application should write into the OTHER buffer.
 *
 * wm8731_current_rx_buffer – index of the buffer DMA is currently FILLING.
 *   The application should read from the OTHER buffer.
 *
 * wm8731_tx_ready – set by the DMA IRQ when the buffer just swapped for TX.
 * wm8731_rx_ready – set by the DMA IRQ when the buffer just swapped for RX.
 *   Both flags are cleared by the application after it has acted on them.
 */
extern volatile uint8_t wm8731_current_tx_buffer;
extern volatile uint8_t wm8731_current_rx_buffer;
extern volatile bool    wm8731_tx_ready;
extern volatile bool    wm8731_rx_ready;

/* -------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */
bool wm8731_init(void);
bool wm8731_write_reg(uint8_t reg, uint16_t val);
bool wm8731_configure(uint32_t sample_rate);
bool wm8731_set_volume(uint8_t volume);
bool wm8731_set_mute(bool mute);
void wm8731_i2s_init(uint32_t sample_rate);
void wm8731_start_dma(void);
void wm8731_stop_dma(void);
void wm8731_set_i2s_samplerate(uint32_t sample_rate);
void wm8731_enable_i2s(void);
void wm8731_disable_i2s(void);

#endif /* WM8731_H_ */
