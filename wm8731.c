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

#include "wm8731.h"
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "i2s.pio.h"
#include "cdc_app.h"

/* -------------------------------------------------------------------------
 * PIO configuration
 *
 * Both state machines run on pio1 and are started in sync.
 *   sm_tx (i2s_out_master) – clocked at BCK * 2
 *   sm_rx (i2s_in_slave)   – clocked at SCK * 2 = BCK * 8  (4x oversampling)
 * ------------------------------------------------------------------------- */
#define I2S_PIO         pio1
#define I2S_BIT_DEPTH   32          /* must match WM8731_IWL_32BIT */

static uint sm_tx;
static uint sm_rx;
static uint sm_mask;                /* bitmask of both SMs for sync-start */
static uint offset_tx;
static uint offset_rx;

/* -------------------------------------------------------------------------
 * DMA channels (claimed in wm8731_i2s_init)
 * ------------------------------------------------------------------------- */
static int dma_tx_ctrl = -1;
static int dma_tx_data = -1;
static int dma_rx_ctrl = -1;
static int dma_rx_data = -1;

/*
 * Control blocks for double-buffering.
 * Each array holds pointers to the two buffer halves.
 * MUST be 8-byte aligned so the DMA ring-wrap (ring=3 -> 2^3=8 bytes) works.
 */
static int32_t* tx_ctrl_blocks[2] __attribute__((aligned(8)));
static int32_t* rx_ctrl_blocks[2] __attribute__((aligned(8)));

/* -------------------------------------------------------------------------
 * Application-visible DMA buffers and state
 * ------------------------------------------------------------------------- */
int32_t wm8731_tx_buffer[WM8731_DMA_NUM_BUFFERS][WM8731_DMA_BUFFER_SIZE];
int32_t wm8731_rx_buffer[WM8731_DMA_NUM_BUFFERS][WM8731_DMA_BUFFER_SIZE];

volatile uint8_t wm8731_current_tx_buffer = 0;
volatile uint8_t wm8731_current_rx_buffer = 0;
volatile bool    wm8731_tx_ready          = false;
volatile bool    wm8731_rx_ready          = false;

/* Last non-muted headphone volume, so wm8731_set_headphone_mute(false)
 * knows what to restore to (see wm8731_set_volume() / that function). */
static uint8_t wm8731_hp_last_volume = 0x5F;

/* I2C instance */
#define WM8731_I2C  i2c0

/* =========================================================================
 * WM8731 register access
 * ========================================================================= */

bool wm8731_write_reg(uint8_t reg, uint16_t val) {
    uint8_t data[2];
    data[0] = (reg << 1) | ((val >> 8) & 0x01);
    data[1] = val & 0xFF;
    int ret = i2c_write_timeout_us(WM8731_I2C, WM8731_I2C_ADDR, data, 2, false, 2000);
    return (ret == 2);
}

/* =========================================================================
 * I2C initialisation (with bus-recovery)
 * ========================================================================= */

static bool wm8731_i2c_init(void) {
    /* Bus recovery: clock SCL 9 times to unstick any hung slave */
    gpio_init(WM8731_I2C_SCL_PIN);
    gpio_set_dir(WM8731_I2C_SCL_PIN, GPIO_OUT);
    gpio_init(WM8731_I2C_SDA_PIN);
    gpio_set_dir(WM8731_I2C_SDA_PIN, GPIO_IN);

    for (int i = 0; i < 9; i++) {
        gpio_put(WM8731_I2C_SCL_PIN, 1); sleep_us(5);
        gpio_put(WM8731_I2C_SCL_PIN, 0); sleep_us(5);
        if (gpio_get(WM8731_I2C_SDA_PIN)) break;
    }
    /* Issue STOP condition */
    gpio_set_dir(WM8731_I2C_SDA_PIN, GPIO_OUT);
    gpio_put(WM8731_I2C_SDA_PIN, 0); sleep_us(5);
    gpio_put(WM8731_I2C_SCL_PIN, 1); sleep_us(5);
    gpio_put(WM8731_I2C_SDA_PIN, 1); sleep_us(5);

    /* Hand pins over to hardware I2C */
    i2c_init(WM8731_I2C, 400 * 1000);
    gpio_set_function(WM8731_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(WM8731_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(WM8731_I2C_SDA_PIN);
    gpio_pull_up(WM8731_I2C_SCL_PIN);
    return true;
}

/* =========================================================================
 * MCLK via PWM on GP6
 *
 * At 120 MHz system clock, wrap=9 gives:
 *   120 MHz / 10 = 12 MHz  (close enough for USB-mode 48 kHz / 96 kHz)
 *
 * For a true 12.288 MHz you would need an external oscillator or use the
 * USB-mode setting in the WM8731 sampling register (which we do).
 * ========================================================================= */

static void wm8731_setup_mclk(void) {
    gpio_set_function(WM8731_MCLK_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(WM8731_MCLK_PIN);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 1.0f);
    pwm_config_set_wrap(&cfg, 9);           /* 120 MHz / 10 = 12 MHz         */
    pwm_init(slice, &cfg, true);
    pwm_set_gpio_level(WM8731_MCLK_PIN, 5); /* 50 % duty cycle               */
}

/* =========================================================================
 * PIO setup
 *
 * Clock dividers:
 *   TX (i2s_out_master) runs at 2 PIO clocks per BCK:
 *     div_tx = sys_clk / (fs * 64 * 2)
 *
 *   RX (i2s_in_slave) runs at 2 PIO clocks per SCK (= 4x BCK):
 *     div_rx = sys_clk / (fs * 256 * 2)
 *
 *   At 120 MHz / 48 kHz:  div_tx = 19.53125,  div_rx = 4.8828125
 *   At 120 MHz / 96 kHz:  div_tx = 9.765625,  div_rx = 2.44140625
 *   Both have exact 1/256 representations -> zero clock-ratio error.
 * ========================================================================= */

static void wm8731_setup_i2s_pio(uint32_t sample_rate) {
    cdc_printf("DEBUG: wm8731_setup_i2s_pio, fs=%lu\n", sample_rate);

    /* Load PIO programs */
    offset_tx = pio_add_program(I2S_PIO, &i2s_out_master_program);
    offset_rx = pio_add_program(I2S_PIO, &i2s_in_slave_program);

    /* Claim state machines */
    sm_tx = pio_claim_unused_sm(I2S_PIO, true);
    sm_rx = pio_claim_unused_sm(I2S_PIO, true);
    sm_mask = (1u << sm_tx) | (1u << sm_rx);

    /* Initialise out_master FIRST (it sets BCLK and LRCK as outputs).
     * init function does NOT start the SM. */
    i2s_out_master_program_init(I2S_PIO, sm_tx, offset_tx,
                                I2S_BIT_DEPTH,
                                WM8731_SDO_PIN,
                                WM8731_BCLK_PIN);   /* clock_pin_base */

    /* Initialise in_slave SECOND (it only sets SDI as input; BCLK and LRCK
     * directions are already correct from out_master init above). */
    i2s_in_slave_program_init(I2S_PIO, sm_rx, offset_rx,
                              WM8731_SDI_PIN);       /* din_pin_base   */

    /* Set clock dividers */
    wm8731_set_i2s_samplerate(sample_rate);
}

/* =========================================================================
 * Clock divider update (can be called at runtime to change sample rate)
 * ========================================================================= */

void wm8731_set_i2s_samplerate(uint32_t sample_rate) {
    float sys_hz = (float)clock_get_hz(clk_sys);

    /* TX: out_master needs 2 PIO clocks per BCK edge; BCK = fs * 64 */
    float div_tx = sys_hz / ((float)sample_rate * 64.0f * 2.0f);

    /* RX: in_slave polls at 4x BCK = SCK = fs * 256; 2 PIO clocks per SCK */
    float div_rx = sys_hz / ((float)sample_rate * 256.0f * 2.0f);

    pio_sm_set_clkdiv(I2S_PIO, sm_tx, div_tx);
    pio_sm_set_clkdiv(I2S_PIO, sm_rx, div_rx);

    /* Restart both dividers simultaneously so phase is aligned */
    pio_sm_clkdiv_restart(I2S_PIO, sm_mask);
}

/* =========================================================================
 * Codec initialisation
 * ========================================================================= */

bool wm8731_init(void) {
    wm8731_setup_mclk();
    sleep_ms(10);

    if (!wm8731_i2c_init()) return false;
    sleep_ms(10);

    if (!wm8731_write_reg(WM8731_REG_RESET, 0x00)) return false;
    sleep_ms(10);

    /* Power everything up */
    if (!wm8731_write_reg(WM8731_REG_POWER_DOWN, 0x00)) return false;

    return true;
}

bool wm8731_configure(uint32_t sample_rate) {
    bool ok = true;

    /* Line input: 0 dB */
    ok &= wm8731_write_reg(WM8731_REG_LLINE_IN, 0x017);
    ok &= wm8731_write_reg(WM8731_REG_RLINE_IN, 0x017);

    /* Headphone output volume — routed through wm8731_set_volume() so
     * wm8731_hp_last_volume (used to restore volume after a TX mute)
     * starts in sync with what's actually in the register. */
    wm8731_set_volume(0x5F);

    /*
     * Analog path default (RX state at boot): DAC -> output mixer
     * enabled (feeds the speaker amp in RX / QSE op-amps in TX — see
     * wm8731_set_input_route()), Line In selected for the ADC (I/Q from
     * the QSD), mic input muted, mic boost off, no bypass/sidetone.
     * radio_set_ptt() in cat_app.c calls wm8731_set_input_route() to
     * flip the ADC side between Line In and Mic whenever T/R switches.
     */
    ok &= wm8731_write_reg(WM8731_REG_ANALOG_PATH, WM8731_DACSEL);

    /* Digital path: no soft-mute, no de-emphasis */
    ok &= wm8731_write_reg(WM8731_REG_DIGITAL_PATH, 0x00);

    /*
     * Digital interface format:
     *   FORMAT = I2S (bit 1:0 = 10)
     *   IWL    = 32-bit (bit 3:2 = 11)
     *   MS     = 0 (slave mode – RP2040 is master)
     */
    ok &= wm8731_write_reg(WM8731_REG_DIGITAL_IF,
                           WM8731_FORMAT_I2S | WM8731_IWL_32BIT);

    /* Sampling control – USB mode, 48 kHz normal. This project only
     * supports 48 kHz (see sample_rates[] in uac2_app.c); there's
     * deliberately no 96 kHz register path to keep in sync. */
    ok &= wm8731_write_reg(WM8731_REG_SAMPLING, WM8731_SR_48K_NORMAL | WM8731_USB_MODE);

    /* Activate */
    ok &= wm8731_write_reg(WM8731_REG_ACTIVE, WM8731_ACTIVE);
    sleep_ms(10);

    return ok;
}

bool wm8731_set_volume(uint8_t volume) {
    if (volume > 127) volume = 127;
    wm8731_hp_last_volume = volume;
    uint16_t val = (volume & WM8731_HPVOL_MASK) | WM8731_HPBOTH | WM8731_HPZCEN;
    bool ok = true;
    ok &= wm8731_write_reg(WM8731_REG_LHPHONE_OUT, val);
    ok &= wm8731_write_reg(WM8731_REG_RHPHONE_OUT, val);
    return ok;
}

/*
 * wm8731_set_headphone_mute — mute/restore ONLY the analog headphone
 * output (LHPOUT/RHPOUT, registers 2/3). This does NOT touch the DAC's
 * digital mute or the analog Line Out (LOUT/ROUT): on this codec, Line
 * Out is a separate fixed-level tap off the output mixer with no volume
 * or mute control of its own, while Headphone Out is a second, entirely
 * independent driver stage with its own volume/mute register. Per the
 * datasheet, whichever headphone output is muted, Line Out keeps its DC
 * level and keeps outputting whatever the DAC is producing.
 *
 * This is what makes the LM386-on-headphone-out / QSE-on-line-out split
 * work without any external mute hardware: silence LHPOUT/RHPOUT during
 * TX and LOUT/ROUT (feeding the QSE) is never affected.
 */
void wm8731_set_headphone_mute(bool mute) {
    uint16_t val;
    if (mute) {
        // Per datasheet: any code below 0110000 (0x30) mutes the channel.
        val = 0x00 | WM8731_HPBOTH | WM8731_HPZCEN;
    } else {
        val = (wm8731_hp_last_volume & WM8731_HPVOL_MASK) | WM8731_HPBOTH | WM8731_HPZCEN;
    }
    wm8731_write_reg(WM8731_REG_LHPHONE_OUT, val);
    wm8731_write_reg(WM8731_REG_RHPHONE_OUT, val);
}

bool wm8731_set_mute(bool mute) {
    return wm8731_write_reg(WM8731_REG_DIGITAL_PATH,
                            mute ? WM8731_DACMU : 0x00);
}

/* =========================================================================
 * Input routing: Line In (QSD I/Q, RX) <-> Mic In (TX), via the Analog
 * Audio Path Control register's ADC-input mux. This is one of two
 * "free" switches the WM8731 gives you (the other being the headphone
 * mute in wm8731_set_headphone_mute() below) so no external audio mux
 * is needed on either the ADC input side or the speaker output side.
 *
 * Register 4 bit positions (WM8731 datasheet, "Analog Audio Path
 * Control"), defined locally rather than assumed from wm8731.h since
 * this file doesn't have visibility into that header's full contents:
 *   bit0 MICBOOST   bit1 MUTEMIC   bit2 INSEL (1=mic, 0=line)
 *   bit3 BYPASS      bit4 DACSEL    bit5 SIDETONE
 * DACSEL is always kept set here — the DAC output stays live in both RX
 * and TX. Line Out (feeding the QSE) has no mute of its own and is
 * unaffected either way; Headphone Out (feeding the speaker/LM386) is
 * muted separately, over in wm8731_set_headphone_mute().
 * ========================================================================= */

#define WM8731_AAPC_MICBOOST   (1 << 0)
#define WM8731_AAPC_MUTEMIC    (1 << 1)
#define WM8731_AAPC_INSEL_MIC  (1 << 2)
#define WM8731_AAPC_BYPASS     (1 << 3)
#define WM8731_AAPC_DACSEL     (1 << 4)
#define WM8731_AAPC_SIDETONE   (1 << 5)

void wm8731_set_input_route(bool tx) {
    uint16_t val = WM8731_AAPC_DACSEL;

    if (tx) {
        // TX: mic -> ADC -> USB (to PowerSDR as the mic source).
        val |= WM8731_AAPC_INSEL_MIC;
    } else {
        // RX: line in (I/Q from the QSD) -> ADC -> USB, mic muted.
        val |= WM8731_AAPC_MUTEMIC;
    }

    wm8731_write_reg(WM8731_REG_ANALOG_PATH, val);
}

/* =========================================================================
 * DMA IRQ handler
 *
 * With chained control-block DMA the hardware reloads the data channel
 * address automatically – we do NOT touch any DMA registers here.
 * We only update the tracking index (toggle 0<->1) and raise the ready flag.
 *
 * TX: IRQ fires when the data channel finishes consuming buffer[x].
 *     DMA is now consuming buffer[x^1].  Tell app to fill buffer[x^1^1]=x.
 *
 * RX: IRQ fires when the data channel finishes filling buffer[x].
 *     DMA is now filling buffer[x^1].  Tell app to read buffer[x].
 *     We set current_rx_buffer = x^1 (the one now being filled) so that
 *     uac2_app's "source_buf = (current_rx_buffer + 1) % 2" gives x. ✓
 * ========================================================================= */

static void wm8731_dma_irq_handler(void) {
    if (dma_channel_get_irq0_status(dma_tx_data)) {
        dma_channel_acknowledge_irq0(dma_tx_data);
        wm8731_current_tx_buffer ^= 1;
        wm8731_tx_ready = true;
    }

    if (dma_channel_get_irq0_status(dma_rx_data)) {
        dma_channel_acknowledge_irq0(dma_rx_data);
        wm8731_current_rx_buffer ^= 1;
        wm8731_rx_ready = true;
    }
}

/* =========================================================================
 * I2S + DMA initialisation
 *
 * Sets up PIO programs and the 4-channel chained-DMA topology.
 * Does NOT start the SMs or DMA transfers – call wm8731_start_dma() for that.
 * ========================================================================= */

void wm8731_i2s_init(uint32_t sample_rate) {
    cdc_printf("DEBUG: wm8731_i2s_init\n");

    wm8731_setup_i2s_pio(sample_rate);

    /* ---- Claim four DMA channels ---- */
    dma_tx_ctrl = dma_claim_unused_channel(true);
    dma_tx_data = dma_claim_unused_channel(true);
    dma_rx_ctrl = dma_claim_unused_channel(true);
    dma_rx_data = dma_claim_unused_channel(true);

    /* ---------------------------------------------------------------
     * TX ctrl channel
     *
     * Reads a pointer from tx_ctrl_blocks[] (8-byte ring) and writes
     * it to the TX data channel's al3_read_addr_trig register, which
     * simultaneously sets the read address and re-triggers the data
     * channel.  This happens automatically whenever the data channel
     * chains back here.
     * --------------------------------------------------------------- */
    {
        dma_channel_config c = dma_channel_get_default_config(dma_tx_ctrl);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        channel_config_set_read_increment(&c, true);
        channel_config_set_write_increment(&c, false);
        channel_config_set_ring(&c, false, 3);  /* read wraps every 8 bytes */
        dma_channel_configure(
            dma_tx_ctrl, &c,
            &dma_hw->ch[dma_tx_data].al3_read_addr_trig, /* dst: tx_data read+trig */
            tx_ctrl_blocks,                               /* src: ctrl block array  */
            1,                                            /* 1 word per trigger     */
            false);                                       /* don't start yet        */
    }

    /* ---------------------------------------------------------------
     * TX data channel
     *
     * Reads from the application TX buffer (incrementing), writes to
     * the PIO TX FIFO (fixed address), paced by the PIO TX DREQ.
     * Chains back to tx_ctrl when a full buffer has been transferred.
     * --------------------------------------------------------------- */
    {
        dma_channel_config c = dma_channel_get_default_config(dma_tx_data);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        channel_config_set_read_increment(&c, true);
        channel_config_set_write_increment(&c, false);
        channel_config_set_chain_to(&c, dma_tx_ctrl);
        channel_config_set_dreq(&c, pio_get_dreq(I2S_PIO, sm_tx, true));
        dma_channel_configure(
            dma_tx_data, &c,
            &I2S_PIO->txf[sm_tx],  /* dst: PIO TX FIFO (fixed)  */
            NULL,                   /* src: set by ctrl channel  */
            WM8731_DMA_BUFFER_SIZE,
            false);
    }

    /* ---------------------------------------------------------------
     * RX ctrl channel
     *
     * Reads a pointer from rx_ctrl_blocks[] (8-byte ring) and writes
     * it to the RX data channel's al2_write_addr_trig register.
     * --------------------------------------------------------------- */
    {
        dma_channel_config c = dma_channel_get_default_config(dma_rx_ctrl);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        channel_config_set_read_increment(&c, true);
        channel_config_set_write_increment(&c, false);
        channel_config_set_ring(&c, false, 3);
        dma_channel_configure(
            dma_rx_ctrl, &c,
            &dma_hw->ch[dma_rx_data].al2_write_addr_trig, /* dst: rx_data write+trig */
            rx_ctrl_blocks,
            1,
            false);
    }

    /* ---------------------------------------------------------------
     * RX data channel
     *
     * Reads from the PIO RX FIFO (fixed), writes to the application
     * RX buffer (incrementing), paced by the PIO RX DREQ.
     * Chains back to rx_ctrl when a full buffer has been filled.
     * --------------------------------------------------------------- */
    {
        dma_channel_config c = dma_channel_get_default_config(dma_rx_data);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        channel_config_set_read_increment(&c, false);
        channel_config_set_write_increment(&c, true);
        channel_config_set_chain_to(&c, dma_rx_ctrl);
        channel_config_set_dreq(&c, pio_get_dreq(I2S_PIO, sm_rx, false));
        dma_channel_configure(
            dma_rx_data, &c,
            NULL,                   /* dst: set by ctrl channel  */
            &I2S_PIO->rxf[sm_rx],  /* src: PIO RX FIFO (fixed)  */
            WM8731_DMA_BUFFER_SIZE,
            false);
    }

    /* ---- IRQ setup ---- */
    dma_channel_set_irq0_enabled(dma_tx_data, true);
    dma_channel_set_irq0_enabled(dma_rx_data, true);
    irq_set_exclusive_handler(DMA_IRQ_0, wm8731_dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);
}

/* =========================================================================
 * Start DMA transfers and PIO state machines
 * ========================================================================= */

void wm8731_start_dma(void) {
    /* Reset tracking state */
    wm8731_current_tx_buffer = 0;
    wm8731_current_rx_buffer = 0;
    wm8731_tx_ready           = false;
    wm8731_rx_ready           = false;

    /* Zero both TX and RX buffers */
    for (int b = 0; b < WM8731_DMA_NUM_BUFFERS; b++) {
        for (int s = 0; s < WM8731_DMA_BUFFER_SIZE; s++) {
            wm8731_tx_buffer[b][s] = 0;
            wm8731_rx_buffer[b][s] = 0;
        }
    }

    /*
     * Point control blocks at the two buffer rows.
     * tx_ctrl_blocks[0] -> buffer being played first, [1] -> second.
     * rx_ctrl_blocks[0] -> buffer being filled first, [1] -> second.
     *
     * The DMA ctrl channel reads [0], triggers data channel with that
     * address, then (ring wrap) reads [1] after the next chain-back, etc.
     */
    tx_ctrl_blocks[0] = wm8731_tx_buffer[0];
    tx_ctrl_blocks[1] = wm8731_tx_buffer[1];
    rx_ctrl_blocks[0] = wm8731_rx_buffer[0];
    rx_ctrl_blocks[1] = wm8731_rx_buffer[1];

    /* Reload ctrl channel read addresses to the start of the arrays.
     * (In case wm8731_start_dma is called a second time.) */
    dma_channel_set_read_addr(dma_tx_ctrl, tx_ctrl_blocks, false);
    dma_channel_set_read_addr(dma_rx_ctrl, rx_ctrl_blocks, false);

    /* Make sure SMs are stopped before touching FIFOs */
    pio_sm_set_enabled(I2S_PIO, sm_tx, false);
    pio_sm_set_enabled(I2S_PIO, sm_rx, false);
    pio_sm_clear_fifos(I2S_PIO, sm_tx);
    pio_sm_clear_fifos(I2S_PIO, sm_rx);

    /*
     * Pre-fill the TX FIFO with silence.
     * The out_master program uses "pull noblock" which substitutes OSR=0
     * when the FIFO is empty, but pre-filling guarantees clean startup
     * without any glitch on the very first BCK edges.
     */
    for (int i = 0; i < 8; i++) {
        pio_sm_put(I2S_PIO, sm_tx, 0);
    }

    /*
     * Start ctrl channels first.  Each ctrl channel immediately reads its
     * first control block, writes the buffer address into the corresponding
     * data channel's addr+trig register, which starts the data channel.
     * The data channels are then self-sustaining via the chain-back.
     */
    dma_channel_start(dma_tx_ctrl);
    dma_channel_start(dma_rx_ctrl);

    /*
     * Enable both SMs simultaneously and in-phase.
     * pio_enable_sm_mask_in_sync holds the SMs in reset until the moment
     * all selected SMs can be released together on the same PIO clock edge.
     */
    pio_enable_sm_mask_in_sync(I2S_PIO, sm_mask);
}

/* =========================================================================
 * Stop DMA transfers (SMs are left running; call wm8731_disable_i2s if needed)
 * ========================================================================= */

void wm8731_stop_dma(void) {
    if (dma_tx_data >= 0) dma_channel_abort(dma_tx_data);
    if (dma_tx_ctrl >= 0) dma_channel_abort(dma_tx_ctrl);
    if (dma_rx_data >= 0) dma_channel_abort(dma_rx_data);
    if (dma_rx_ctrl >= 0) dma_channel_abort(dma_rx_ctrl);
}

/* =========================================================================
 * Enable / disable PIO state machines
 * ========================================================================= */

void wm8731_enable_i2s(void) {
    pio_enable_sm_mask_in_sync(I2S_PIO, sm_mask);
}

void wm8731_disable_i2s(void) {
    pio_sm_set_enabled(I2S_PIO, sm_tx, false);
    pio_sm_set_enabled(I2S_PIO, sm_rx, false);
}
