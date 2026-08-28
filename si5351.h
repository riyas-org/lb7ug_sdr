/*
 * si5351.h — Minimal Si5351A driver for the RP2040 LMR-SDR QSD/QSE clock
 *
 * Hardware role in this design (same topology as LMR SDR v1.7):
 *   Si5351 CLK0  --> 74LVC74 (dual D-FF, wired as /4)  --> FST3253 mux
 *                    select lines (I/Q 0/90/180/270 switching for the
 *                    Tayloe-style QSD receiver and QSE transmitter, which
 *                    share the same analog switch).
 *
 * Because the external flip-flop divides by 4, the Si5351 must be told to
 * output 4x the wanted RF (LO) frequency. si5351_set_freq() below hides
 * that detail — callers always pass the actual LO/VFO frequency in Hz.
 *
 * This driver intentionally implements only what's needed today:
 *   - I2C bring-up
 *   - PLL + MultiSynth0 programming for CLK0 (the RX/TX shared LO)
 *   - CLK0 output enable/disable
 *
 * It is structured so a second clock (e.g. independent TX LO, a reference
 * output for a companion board, or a beacon/calibration clock) can be
 * added later by extending si5351_set_freq()/si5351_enable_output() with
 * a clk_num parameter and duplicating the MultiSynth1/2 register block —
 * see the "EXTENDING" note near the bottom of si5351.c.
 */

#ifndef SI5351_H
#define SI5351_H

#include <stdbool.h>
#include <stdint.h>

// ---------------------------------------------------------------------
// Pin / bus configuration.
// This board shares a single I2C bus (i2c0, GP0=SDA/GP1=SCL) between
// the WM8731 codec and the Si5351 — same pins wm8731.c's
// wm8731_i2c_init() already configures. si5351_init() re-issues
// i2c_init()/gpio_set_function() on the same bus; that's harmless
// (idempotent — same instance, same 400 kHz baud), it just means the
// bus only needs bringing up once in practice. Keep SI5351_I2C_ADDR
// (0x60) distinct from the WM8731's I2C address so both devices can
// coexist on the shared bus.
// ---------------------------------------------------------------------
#ifndef SI5351_I2C
#define SI5351_I2C          i2c0
#endif
#ifndef SI5351_SDA_PIN
#define SI5351_SDA_PIN      0       // GP0, shared with WM8731
#endif
#ifndef SI5351_SCL_PIN
#define SI5351_SCL_PIN      1       // GP1, shared with WM8731
#endif
#ifndef SI5351_I2C_ADDR
#define SI5351_I2C_ADDR     0x60    // Standard Si5351A breakout address
#endif

// Reference crystal on the Si5351 board. Common values: 25000000 or
// 27000000. Check the crystal marking / breakout silkscreen.
#ifndef SI5351_XTAL_FREQ_HZ
#define SI5351_XTAL_FREQ_HZ 27000000UL
#endif

// Parts-per-billion trim for the reference crystal, determined by
// calibrating CLK0 against a known-accurate frequency counter/receiver.
// Positive = crystal runs fast. Stored as a simple correction applied
// to SI5351_XTAL_FREQ_HZ during PLL math. Safe to leave at 0 initially.
#ifndef SI5351_XTAL_CORRECTION_PPB
#define SI5351_XTAL_CORRECTION_PPB 0
#endif

// The external /4 flip-flop between CLK0 and the FST3253. If a future
// board wires the mux directly to Si5351 CLKx (using its internal
// quadrature/phase-offset registers instead of an external divider),
// change this to 1 and switch si5351_set_freq() to use CLKx phase
// offsets — left as a documented extension point, not implemented here.
#ifndef SI5351_EXT_QUADRATURE_DIVIDER
#define SI5351_EXT_QUADRATURE_DIVIDER 4
#endif

/**
 * si5351_init — bring up I2C, sanity-check the chip, load crystal
 * capacitance, and disable all outputs (clean starting state).
 * Returns false if the chip did not ACK on the bus.
 */
bool si5351_init(void);

/**
 * si5351_set_freq — program CLK0 so the *receiver/transmitter LO*
 * ends up at lo_freq_hz. Internally multiplies by
 * SI5351_EXT_QUADRATURE_DIVIDER before computing PLL/MultiSynth
 * registers, since the external flip-flop divides back down.
 *
 * Valid roughly 1 MHz .. 40 MHz LO (i.e. 4 MHz .. 160 MHz at the Si5351
 * output), which comfortably covers HF. Returns false on an out-of-range
 * request or I2C failure.
 */
bool si5351_set_freq(uint32_t lo_freq_hz);

/**
 * si5351_enable_output — turn CLK0 on/off. Used for RX/TX muting of the
 * LO (e.g. while re-tuning) and to save power when idle.
 */
void si5351_enable_output(bool enable);

#endif // SI5351_H
