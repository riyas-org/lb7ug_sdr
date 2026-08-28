/*
 * si5351.c — Minimal Si5351A driver for the RP2040 LMR-SDR QSD/QSE clock.
 * See si5351.h for the hardware role (CLK0 -> 74LVC74 /4 -> FST3253).
 *
 * PLL/MultiSynth math follows the standard Si5351 approach documented in
 * Silicon Labs AN619: pick an integer output MultiSynth divider (keeps
 * the final divider in low-jitter integer mode), then land the PLL on
 * the exact frequency needed using its fractional feedback divider
 * (a + b/c, c up to 20 bits). Only CLK0 / PLLA / MS0 are programmed;
 * PLLB and MS1/MS2 are left free for a future second clock output.
 */

#include "si5351.h"

#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

// -----------------------------------------------------------------------
// Si5351 register map (only what we use)
// -----------------------------------------------------------------------
#define SI5351_REG_OUTPUT_ENABLE   3
#define SI5351_REG_CLK0_CTRL       16
#define SI5351_REG_MSNA_BASE       26   // PLLA feedback MultiSynth
#define SI5351_REG_MS0_BASE        42   // Output MultiSynth 0 (CLK0)
#define SI5351_REG_PLL_RESET       177
#define SI5351_REG_XTAL_LOAD       183

#define SI5351_CLK_SRC_MULTISYNTH  (0x3 << 2)
#define SI5351_CLK_PLL_SRC_A       (0x0 << 5)
#define SI5351_CLK_IDRV_8MA        0x3
#define SI5351_CLK_POWERED         0x00
#define SI5351_MSx_INTEGER_MODE    (1 << 6)

#define SI5351_XTAL_LOAD_10PF      0xD2  // bits[7:6]=11, reserved bits per datasheet

#define SI5351_PLL_RESET_PLLA      (1 << 5)
#define SI5351_PLL_RESET_PLLB      (1 << 7)

#define SI5351_MS_DENOM_MAX        1048575UL  // 2^20 - 1, PLL feedback fractional denominator

// Valid PLL VCO range per datasheet
#define SI5351_PLL_FREQ_MIN_HZ     600000000ULL
#define SI5351_PLL_FREQ_MAX_HZ     900000000ULL

static bool si5351_present = false;

// -----------------------------------------------------------------------
// Low-level register access
// -----------------------------------------------------------------------

static bool si5351_write_reg(uint8_t reg, uint8_t val) {
    uint8_t data[2] = { reg, val };
    int ret = i2c_write_timeout_us(SI5351_I2C, SI5351_I2C_ADDR, data, 2, false, 2000);
    return (ret == 2);
}

static bool si5351_write_block(uint8_t start_reg, const uint8_t *vals, uint8_t len) {
    // Si5351 auto-increments the register pointer, so one burst write works.
    uint8_t buf[1 + 8];
    if (len > sizeof(buf) - 1) return false;
    buf[0] = start_reg;
    for (uint8_t i = 0; i < len; i++) buf[1 + i] = vals[i];
    int ret = i2c_write_timeout_us(SI5351_I2C, SI5351_I2C_ADDR, buf, len + 1, false, 2000);
    return (ret == (int)(len + 1));
}

// -----------------------------------------------------------------------
// Init
// -----------------------------------------------------------------------

bool si5351_init(void) {
    i2c_init(SI5351_I2C, 400 * 1000);
    gpio_set_function(SI5351_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SI5351_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SI5351_SDA_PIN);
    gpio_pull_up(SI5351_SCL_PIN);

    // Probe: disable all outputs first (also confirms the chip ACKs).
    si5351_present = si5351_write_reg(SI5351_REG_OUTPUT_ENABLE, 0xFF);
    if (!si5351_present) return false;

    si5351_write_reg(SI5351_REG_XTAL_LOAD, SI5351_XTAL_LOAD_10PF);

    // Power down all 8 clock outputs until explicitly enabled.
    for (uint8_t clk = 0; clk < 8; clk++) {
        si5351_write_reg(SI5351_REG_CLK0_CTRL + clk, 0x80); // CLKx_PDN=1
    }

    return true;
}

// -----------------------------------------------------------------------
// Frequency programming
// -----------------------------------------------------------------------

bool si5351_set_freq(uint32_t lo_freq_hz) {
    if (!si5351_present) return false;

    // Physical CLK0 frequency needed so the external /4 flip-flop lands
    // on the requested LO.
    uint64_t f_out = (uint64_t)lo_freq_hz * SI5351_EXT_QUADRATURE_DIVIDER;

    // Output MultiSynth divider: pick the integer divide ratio that puts
    // the PLL near the middle of its valid 600-900 MHz range.
    uint32_t ms_div = (uint32_t)((750000000ULL + f_out / 2) / f_out);
    if (ms_div < 8)   ms_div = 8;
    if (ms_div > 900) ms_div = 900;

    uint64_t pll_freq = f_out * ms_div;
    if (pll_freq < SI5351_PLL_FREQ_MIN_HZ || pll_freq > SI5351_PLL_FREQ_MAX_HZ) {
        return false; // out of supported LO range for this simple divider search
    }

    uint64_t xtal_hz = SI5351_XTAL_FREQ_HZ;
    xtal_hz += ((int64_t)xtal_hz * SI5351_XTAL_CORRECTION_PPB) / 1000000000LL;

    // PLL feedback multiplier = pll_freq / xtal_hz, split into integer (a)
    // and fractional (b/c) parts.
    uint32_t a = (uint32_t)(pll_freq / xtal_hz);
    uint64_t rem = pll_freq - (uint64_t)a * xtal_hz;
    uint32_t c = SI5351_MS_DENOM_MAX;
    uint32_t b = (uint32_t)((rem * c) / xtal_hz);

    // ---- PLLA feedback MultiSynth (fractional) register math ----
    uint32_t msna_p1 = 128 * a + (uint32_t)((128ULL * b) / c) - 512;
    uint32_t msna_p2 = (uint32_t)(128ULL * b - (uint64_t)c * ((128ULL * b) / c));
    uint32_t msna_p3 = c;

    uint8_t plla_regs[8] = {
        (uint8_t)(msna_p3 >> 8), (uint8_t)(msna_p3),
        (uint8_t)((msna_p1 >> 16) & 0x03),
        (uint8_t)(msna_p1 >> 8), (uint8_t)(msna_p1),
        (uint8_t)(((msna_p3 >> 16) & 0x0F) << 4 | ((msna_p2 >> 16) & 0x0F)),
        (uint8_t)(msna_p2 >> 8), (uint8_t)(msna_p2),
    };
    si5351_write_block(SI5351_REG_MSNA_BASE, plla_regs, 8);

    // ---- Output MultiSynth 0 (integer mode: b=0, c=1) ----
    uint32_t ms0_p1 = 128 * ms_div - 512;

    uint8_t ms0_regs[8] = {
        0x00, 0x01,                              // MS0_P3 = 1
        (uint8_t)((ms0_p1 >> 16) & 0x03),        // R0_DIV=0 (no extra division), MS0_P1[19:16]
        (uint8_t)(ms0_p1 >> 8), (uint8_t)(ms0_p1),
        0x00,                                    // MS0_P3[19:16]=0, MS0_P2[19:16]=0
        0x00, 0x00,                               // MS0_P2 = 0
    };
    si5351_write_block(SI5351_REG_MS0_BASE, ms0_regs, 8);

    // CLK0: MultiSynth source, PLLA, integer mode (even divider -> low jitter), 8 mA drive.
    uint8_t clk0_ctrl = SI5351_CLK_POWERED | SI5351_CLK_PLL_SRC_A |
                         SI5351_CLK_SRC_MULTISYNTH | SI5351_CLK_IDRV_8MA;
    if ((ms_div % 2) == 0) clk0_ctrl |= SI5351_MSx_INTEGER_MODE;
    si5351_write_reg(SI5351_REG_CLK0_CTRL, clk0_ctrl);

    // Reset PLLA so the new feedback divider takes effect cleanly.
    si5351_write_reg(SI5351_REG_PLL_RESET, SI5351_PLL_RESET_PLLA);

    return true;
}

void si5351_enable_output(bool enable) {
    if (!si5351_present) return;

    // Register 3 (Output Enable Control): bit0=0 lets CLK0 run, bit0=1
    // masks it. CLK1-7 stay masked (we don't use them yet).
    // This alone is enough to mute/unmute cleanly without disturbing the
    // CLK0_CTRL configuration (PLL source, integer-mode bit, drive
    // strength) that si5351_set_freq() already programmed.
    si5351_write_reg(SI5351_REG_OUTPUT_ENABLE, enable ? 0xFE : 0xFF);

    if (!enable) {
        // Also power the output driver down for a bit of extra current
        // savings when idle; si5351_set_freq() will power it back up
        // (and restore the correct control bits) the next time it runs.
        si5351_write_reg(SI5351_REG_CLK0_CTRL, 0x80);
    }
}

/*
 * EXTENDING: to drive a second Si5351 clock (e.g. a separate TX LO, or a
 * reference/beacon output), duplicate the MultiSynth1 register block
 * (registers 50-57) and CLK1 control (register 17), and either reuse
 * PLLA (SI5351_CLK_PLL_SRC_A) if both clocks can share one VCO frequency,
 * or program PLLB (registers 34-41) independently if they need different,
 * unrelated frequencies. Add a `clk_num` parameter to si5351_set_freq()
 * and si5351_enable_output() at that point rather than copy-pasting new
 * top-level functions.
 */
