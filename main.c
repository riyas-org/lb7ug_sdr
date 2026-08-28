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

#include <stdio.h>
#include <string.h>

#include "bsp/board_api.h"
#include "tusb.h"
#include "common.h"

extern uint32_t blink_interval_ms;
extern void audio_init_codec(void);

#if (CFG_TUSB_MCU == OPT_MCU_RP2040)
#include "pico/stdlib.h"
#endif

#include "hardware/pio.h"
#include "ws2812.pio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "cdc_app.h"
#include "cat_app.h"
#include "si5351.h"
#include "hardware/pll.h"
#include "hardware/xosc.h"
#include "hardware/clocks.h"
#include "pico/multicore.h"
#include "dual_core_config.h"

// Default VFO on boot, before any CAT/PowerSDR client connects.
#define LMR_SDR_DEFAULT_FREQ_HZ  7074000UL

/*
 * cdc_app_process_rx_byte / cdc_app_check_greeting — implemented in
 * cdc_app.c. Not (yet) declared in cdc_app.h since that file wasn't part
 * of what's being edited here; same ad-hoc extern-declaration style used
 * elsewhere in this codebase (see cat_app.c's wm8731_* prototypes).
 */
extern void cdc_app_process_rx_byte(char c);
extern void cdc_app_check_greeting(void);

#define WS2812_PIN 16

// INIT BOARD LED
void init_ws2812_led(void) {
    PIO pio = pio0;
    int sm = 0;
    uint offset = pio_add_program(pio, &ws2812_program);
    ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, false);
} 


void ws2812_led_write(bool state) {
    // Green when "on", Red when "off" (or 0 for dark)
    uint32_t color = state ? 0x00FF0000 : 0xFF000000; 
    pio_sm_put_blocking(pio0, 0, color);
}


void set_audio_clocks_safe() {
    // 1. Park clk_sys on the 12MHz Crystal (REF) 
    // This keeps the CPU running while we stop and restart the PLLs
    clock_configure(clk_sys,
                    CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLK_REF,
                    0,          // No auxiliary mux needed for REF
                    12 * MHZ,
                    12 * MHZ);

    // 2. Initialize PLLs to Audio frequencies
    // PLL_SYS = 122.88 MHz (12 * 102.4)
    pll_init(pll_sys, 1, 1228800000, 5, 2);
    // PLL_USB = 48 MHz (Standard for TinyUSB)
    pll_init(pll_usb, 1, 480000000, 5, 2);

    // 3. Switch clk_sys to the new 122.88 MHz PLL
    clock_configure(clk_sys,
                    CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                    CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                    122880000,
                    122880000);

    // 4. Switch clk_usb to the 48 MHz PLL
    clock_configure(clk_usb,
                    0, 
                    CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                    48 * MHZ,
                    48 * MHZ);

    // 5. IMPORTANT: Re-init stdio because the UART baud rate is calculated from clk_sys
    stdio_init_all();
}



/*
 * =========================================================================
 * Core split (see dual_core_config.h to toggle this off)
 * =========================================================================
 * Core 1: TinyUSB, in its entirety — tusb_init(), tud_task(),
 *         audio_task() (the software glue between the WM8731's DMA
 *         ping-pong buffers and the USB isochronous endpoints), cdc_task()
 *         (drains cdc_app.c's ring buffer to tud_cdc_write), and
 *         led_blinking_task() (only reacts to TinyUSB's own mount/
 *         suspend/resume callbacks, which now fire here too). The USB
 *         controller IRQ binds to whichever core calls tusb_init(), and
 *         TinyUSB's internal CDC/audio FIFOs aren't safe to touch from
 *         two cores, so everything that calls into tud_*()/tusb_*()
 *         lives here together — nothing on Core 0 ever does.
 *
 * Core 0: everything else — Si5351/WM8731 init at boot, then in the main
 *         loop: draining the Core1->Core0 CDC byte relay (see below),
 *         CAT/debug parsing (cdc_app_process_rx_byte(), which calls into
 *         cat_app.c), and radio_task() applying the resulting Si5351/
 *         WM8731 I2C writes. Core 0 is the sole writer of radio_state
 *         and the sole owner of every I2C control-plane device — nothing
 *         on Core 1 ever touches them, so there's no cross-core lock
 *         needed for any of it.
 *
 * This is what actually solves the buffer-underrun class of problem
 * discussed earlier: however long a Si5351/WM8731 I2C transaction takes
 * on Core 0, it now has zero effect on Core 1's servicing of the codec's
 * DMA buffers, because they're not sharing a time budget anymore. The
 * single-core deferred-flag mechanism in cat_app.c (radio_task() and its
 * pending flags) still exists and still helps even here — it's what
 * keeps tud_cdc_rx_cb() itself fast — but this is the stronger fix.
 *
 * CDC RX -> Core 0 relay: CAT/debug input over USB CDC has to be parsed
 * on Core 0 (the same place that owns every I2C device it might act on).
 * Core 1 reads raw bytes off TinyUSB's CDC RX FIFO (in tud_cdc_rx_cb(),
 * cdc_app.c) and relays them to Core 0 one at a time over the SDK's
 * inter-core hardware FIFO; Core 0 drains that below and calls
 * cdc_app_process_rx_byte() for each one.
 *
 * The reverse direction (CAT replies/debug output going back to the
 * host) needs no relay of its own: cdc_app.c's ring buffer is already a
 * cross-core-safe single-producer/single-consumer queue — Core 0's
 * cdc_printf() calls (via cat_dispatch_command()/handle_debug_key()) are
 * its only producer, Core 1's cdc_task() its only consumer. See the
 * comment on cdc_printf() in cdc_app.c.
 *
 * Adapted from the same Core0/Core1 split used in a sister SDR project's
 * main.c, which hit and solved this exact class of problem first.
 * =========================================================================
 */

#if LMR_SDR_DUAL_CORE
static void core1_entry(void) {
    tusb_rhport_init_t dev_init = {
        .role  = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_AUTO,
    };
    tusb_init(BOARD_TUD_RHPORT, &dev_init);

    while (1) {
        tud_task(); // TinyUSB device task
        led_blinking_task();
        audio_task();
        cdc_task();
    }
}
#endif // LMR_SDR_DUAL_CORE

/*------------- MAIN -------------*/
int main(void)
{

set_audio_clocks_safe();


#if (CFG_TUSB_MCU == OPT_MCU_RP2040)
  stdio_init_all();
#endif

  
board_init();
init_ws2812_led();



  TU_LOG1("CDC UAC2 Stereo SDR example running\r\n");
  TU_LOG1("Stereo audio: 2 channels TX (mic), 2 channels RX (speaker)\r\n");

audio_init_codec();

//wm8731_start_dma();

#if (CFG_TUSB_MCU == OPT_MCU_RP2040)
  // Si5351 LO for the QSD/QSE (CLK0 -> 74LVC74 /4 -> FST3253, see si5351.h).
  // radio_init() also sets up the PTT GPIO and applies the default VFO.
  // Deliberately done here, before TinyUSB starts on either core, since
  // the exact timing of this one-time boot sequence doesn't matter —
  // only steady-state I2C writes needed to move off Core1, which
  // radio_task() below (and the Core0/Core1 split itself) handles.
  if (si5351_init()) {
    radio_init(LMR_SDR_DEFAULT_FREQ_HZ);
    TU_LOG1("Si5351 LO running at %lu Hz\r\n", (unsigned long)LMR_SDR_DEFAULT_FREQ_HZ);
  } else {
    TU_LOG1("WARNING: Si5351 not found on I2C - check wiring/address\r\n");
  }
#endif

#if LMR_SDR_DUAL_CORE
  // From here on, all USB/audio-timing-critical work runs on Core1;
  // Core0 (the loop below) is purely the control plane.
  multicore_launch_core1(core1_entry);
#else
  // Single-core fallback (see dual_core_config.h): TinyUSB runs right
  // here, same as this project did before the dual-core split.
  tusb_rhport_init_t dev_init = {
      .role  = TUSB_ROLE_DEVICE,
      .speed = TUSB_SPEED_AUTO,
  };
  tusb_init(BOARD_TUD_RHPORT, &dev_init);
#endif

  while (1)
  {
#if LMR_SDR_DUAL_CORE
    // Drain the Core1->Core0 CDC byte relay (see tud_cdc_rx_cb() in
    // cdc_app.c). multicore_fifo_rvalid() confirms data is present, so
    // the pop below never actually blocks.
    while (multicore_fifo_rvalid()) {
        uint32_t w = multicore_fifo_pop_blocking();
        cdc_app_process_rx_byte((char)(uint8_t)w);
    }

    cdc_app_check_greeting();
#endif

#if (CFG_TUSB_MCU == OPT_MCU_RP2040)
    // Applies any FA/TX/RX changes queued (non-blocking) by CAT commands.
    // In dual-core builds this — and every other I2C control-plane call
    // on Core0 — can take as long as it needs without ever affecting
    // Core1's audio/USB servicing. In single-core builds it's still
    // useful on its own (see the big comment on the pending flags in
    // cat_app.c), which is why it stays outside the #if above.
    radio_task();

    // Manual PTT button (GP14 by default — see RADIO_MANUAL_PTT_PIN in
    // cat_app.h). Polled here on Core0, same as CAT: cheap (one GPIO
    // read most iterations), and combines with any CAT TX;/RX; request
    // via OR — see the comment on cat_ptt_request in cat_app.c.
    radio_ptt_button_task();

    // Backup PTT via the serial port's RTS line, for CAT/logging
    // software that keys PTT that way instead of TX;/RX;. Same OR
    // combine as the CAT and manual-button sources.
    radio_rts_ptt_task();
#endif

#if !LMR_SDR_DUAL_CORE
    // Single-core fallback: everything shares this one loop, same as
    // before the dual-core split. CDC bytes go straight from
    // tud_cdc_rx_cb() to cdc_app_process_rx_byte() (see cdc_app.c) —
    // there's no second core to relay to.
    tud_task(); // TinyUSB device task
    led_blinking_task();
    audio_task();
    cdc_task();
    cdc_app_check_greeting();
#endif
    //run_bit_perfect_test();    
  }
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+
// In dual-core builds these fire on Core1 (TinyUSB lives there — see
// core1_entry() above). They only touch blink_interval_ms, which
// led_blinking_task() also now reads on Core1 (it moved there too), so
// this stays same-core and needs no extra synchronization.

// Invoked when device is mounted
void tud_mount_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
  TU_LOG1("Device mounted\r\n");
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  blink_interval_ms = BLINK_NOT_MOUNTED;
  TU_LOG1("Device unmounted\r\n");
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void)remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
  TU_LOG1("Device suspended\r\n");
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
  TU_LOG1("Device resumed\r\n");
}
