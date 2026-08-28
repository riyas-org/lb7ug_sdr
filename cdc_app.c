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
#include <stdarg.h>

#include "bsp/board_api.h"
#include "tusb.h"
#include "common.h"
#include "cdc_app.h"
#include "cat_app.h"
#include "pico/multicore.h"
#include "dual_core_config.h"

#if (CFG_TUSB_MCU == OPT_MCU_RP2040)
#include "wm8731.h"
#endif

// Underrun/overrun counters — defined in uac2_app.c (see the comment
// there). Declared extern here rather than in a shared header, matching
// the loose ad-hoc style already used elsewhere in this codebase.
extern volatile uint32_t audio_spk_underrun_count;
extern volatile uint32_t audio_mic_overrun_count;

/*
 * The CDC port is shared by two very different consumers:
 *
 *   1. CAT control (PowerSDR, hamlib, etc.) — a raw ASCII stream of
 *      "XXdata;" commands, parsed by cat_app.c. This is the default,
 *      because that's what a CAT program expects to see the moment it
 *      opens the port: no banners, no unsolicited text.
 *   2. A human debug console for bring-up/troubleshooting — single-key
 *      shortcuts ('?' status, 'm' mute, 'v' verbose toggle), reached by
 *      prefixing the key with '!' (e.g. "!m") so they can never collide
 *      with CAT command bytes (digits, letters, ';') flowing through at
 *      the same time.
 *
 * Debug/status printf output (cdc_debug_printf) is suppressed by default
 * so it can't corrupt a CAT session; toggle it on with "!v" while
 * connected to a terminal for bring-up work.
 *
 * Core placement (see dual_core_config.h and main.c's file header):
 * in dual-core builds, TinyUSB — and therefore tud_cdc_rx_cb() below —
 * runs entirely on Core1, but all the actual parsing (the '!'-escape
 * check, handle_debug_key(), cat_process_char()) needs to happen on
 * Core0, the same place that owns every I2C control-plane device and
 * radio_state. So tud_cdc_rx_cb() itself does nothing but relay raw
 * bytes to Core0 over the hardware inter-core FIFO; cdc_app_process_rx_byte()
 * — called from Core0's loop in main.c — is where the real logic now
 * lives. In single-core builds there's no relay: tud_cdc_rx_cb() calls
 * cdc_app_process_rx_byte() directly.
 */
static volatile bool cat_debug_verbose = false;

/**
 * cdc_debug_printf — like cdc_printf, but only emits when verbose debug
 * output has been enabled (see cat_debug_verbose above). Use this for
 * human-readable status/troubleshooting text; use cdc_printf directly
 * only for protocol bytes that must always go out (i.e. CAT replies).
 */
static int cdc_debug_printf(const char *fmt, ...) {
    if (!cat_debug_verbose) return 0;
    char tmp[256];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(tmp, sizeof(tmp), fmt, args);
    va_end(args);
    if (len <= 0) return len;
    return cdc_printf("%s", tmp);
}

// -----------------------------------------------------------------------
// Ring buffer for debug output
// -----------------------------------------------------------------------
#define CDC_RING_BUF_SIZE  2048   // Must be a power of two
#define CDC_RING_MASK      (CDC_RING_BUF_SIZE - 1)

static char              _ring[CDC_RING_BUF_SIZE];
static volatile uint32_t _ring_head = 0;   // Write index — Core0 only (cdc_printf)
static volatile uint32_t _ring_tail = 0;   // Read  index — cdc_task()'s core only

/** How many bytes are waiting to be sent */
static inline uint32_t ring_used(void) {
    return (_ring_head - _ring_tail) & CDC_RING_MASK;
}

/** How many bytes of free space remain */
static inline uint32_t ring_free(void) {
    return CDC_RING_BUF_SIZE - 1 - ring_used();
}

/**
 * cdc_printf — drop-in printf replacement routed to the CDC ring buffer.
 *
 * Thread/core safety: this must only ever be called from Core0 (or, in
 * single-core builds, from the one and only core). The ring buffer is a
 * single-producer/single-consumer queue — cdc_printf() is the sole
 * producer (writes _ring_head), cdc_task() is the sole consumer (writes
 * _ring_tail, on whichever core owns TinyUSB). Both indices are
 * volatile and naturally-aligned 32-bit words, so that split is safe
 * across cores with no extra locking — but only as long as nothing else
 * ever writes _ring_head. That's why tud_cdc_line_state_cb() defers its
 * greeting through cdc_greeting_pending instead of calling this
 * directly: in dual-core builds that callback runs on Core1, and a
 * second producer would break the single-producer assumption.
 */
int cdc_printf(const char *fmt, ...) {
    char tmp[256];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(tmp, sizeof(tmp), fmt, args);
    va_end(args);

    if (len <= 0) return len;
    if (len > (int)sizeof(tmp) - 1) len = sizeof(tmp) - 1;

    // Write into ring buffer, dropping bytes if full (debug drops are
    // preferable to blocking the audio task).
    uint32_t free = ring_free();
    uint32_t to_write = (len < (int)free) ? (uint32_t)len : free;

    for (uint32_t i = 0; i < to_write; i++) {
        _ring[_ring_head & CDC_RING_MASK] = tmp[i];
        _ring_head++;
    }

    return (int)to_write;
}

/**
 * cdc_task — call this from the main loop alongside tud_task().
 * Drains up to one CDC packet's worth of bytes per call.
 */
void cdc_task(void) {
    if (!tud_cdc_connected()) return;

    uint32_t avail = ring_used();
    if (avail == 0) return;

    // Send up to 64 bytes at a time (one FS bulk packet)
    uint8_t  buf[64];
    uint32_t to_send = avail < sizeof(buf) ? avail : sizeof(buf);

    for (uint32_t i = 0; i < to_send; i++) {
        buf[i] = (uint8_t)_ring[_ring_tail & CDC_RING_MASK];
        _ring_tail++;
    }

    uint32_t written = tud_cdc_write(buf, to_send);
    (void)written;
    tud_cdc_write_flush();
}

// -----------------------------------------------------------------------
// TinyUSB CDC callbacks
// -----------------------------------------------------------------------

/*
 * tud_cdc_line_state_cb() fires on whichever core owns TinyUSB (Core1 in
 * dual-core builds). It must not call cdc_debug_printf()/cdc_printf()
 * directly there — see the "single producer" note on cdc_printf() above
 * — so it just sets this flag; cdc_app_check_greeting(), called from
 * Core0's loop in main.c, does the actual printing. Same deferred-flag
 * pattern used throughout this codebase (radio_task()'s pending flags,
 * the reference dual-core design's terminal_connect_pending).
 */
static volatile bool cdc_greeting_pending = false;

/*
 * Backup PTT via the serial port's RTS control line — some CAT/logging
 * software keys PTT this way directly instead of (or alongside) sending
 * TX;/RX;. Not static: cat_app.c's radio_rts_ptt_task() reads this
 * (extern-declared there, same ad-hoc pattern used elsewhere in this
 * codebase) to fold it into the same OR'd PTT combine as CAT and the
 * manual button. Single writer (this callback, whichever core owns
 * TinyUSB) / single reader (Core0), plain volatile bool is enough —
 * same reasoning as wm8731_tx_ready/rx_ready in wm8731.c.
 */
volatile bool cdc_rts_active = false;

// Invoked when CDC line state changes (host connects / disconnects)
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
    (void)itf;

    cdc_rts_active = rts;

    if (dtr) {
        cdc_greeting_pending = true;
    }
}

/**
 * cdc_app_check_greeting — call once per iteration from whichever loop
 * is allowed to produce into the ring buffer (Core0 in dual-core builds,
 * the single loop in single-core builds). Prints the debug-console
 * greeting queued by tud_cdc_line_state_cb() above, if any. A no-op most
 * iterations (just a flag check).
 */
void cdc_app_check_greeting(void) {
    if (!cdc_greeting_pending) return;
    cdc_greeting_pending = false;

    // Host just opened the port. Say nothing unless verbose debug is on
    // — a CAT program doesn't expect a banner, and shouldn't get one.
    cdc_debug_printf("\r\n=== CDC debug console ready ===\r\n");
    cdc_debug_printf("Commands (prefix with '!'): '?' help  'm' mute  'v' verbose toggle  'u' reset underrun counters\r\n");
}

/** Handle one '!'-escaped debug shortcut key. */
static void handle_debug_key(char c) {
    switch (c) {
        case '?':
        case 'h':
            cdc_printf("--- Status ---\r\n");
#if (CFG_TUSB_MCU == OPT_MCU_RP2040)
            cdc_printf("TX ready: %d  RX ready: %d\r\n",
                       (int)wm8731_tx_ready, (int)wm8731_rx_ready);
            cdc_printf("TX buf: %d  RX buf: %d\r\n",
                       (int)wm8731_current_tx_buffer,
                       (int)wm8731_current_rx_buffer);
            cdc_printf("VFO: %lu Hz  Mode: %d  PTT: %s\r\n",
                       (unsigned long)radio_state.vfo_a_hz,
                       (int)radio_state.mode,
                       radio_state.ptt_active ? "TX" : "RX");
            cdc_printf("SPK underruns: %lu  MIC overruns: %lu\r\n",
                       (unsigned long)audio_spk_underrun_count,
                       (unsigned long)audio_mic_overrun_count);
#endif
            break;

        case 'u':
#if (CFG_TUSB_MCU == OPT_MCU_RP2040)
            audio_spk_underrun_count = 0;
            audio_mic_overrun_count  = 0;
            cdc_printf("Underrun/overrun counters reset\r\n");
#endif
            break;

        case 'v':
            cat_debug_verbose = !cat_debug_verbose;
            cdc_printf("Verbose debug: %s\r\n", cat_debug_verbose ? "ON" : "OFF");
            break;

        case 'm':
#if (CFG_TUSB_MCU == OPT_MCU_RP2040)
            {
                static bool muted = false;
                muted = !muted;
                wm8731_set_mute(muted);
                cdc_printf("DAC mute: %s\r\n", muted ? "ON" : "OFF");
            }
#endif
            break;

        default:
            // Unknown shortcut — ignore silently
            break;
    }
}

/*
 * cdc_app_process_rx_byte — the actual '!'-escape / CAT dispatch logic.
 * In dual-core builds this runs on Core0, called from main.c's loop
 * after draining the Core1->Core0 byte relay (see tud_cdc_rx_cb() below
 * and main.c's file header for the full picture). In single-core builds
 * it's called directly from tud_cdc_rx_cb(), same as this file always
 * did before the dual-core split.
 */
void cdc_app_process_rx_byte(char c) {
    static bool debug_escape_pending = false;

    if (debug_escape_pending) {
        debug_escape_pending = false;
        handle_debug_key(c);
        return;
    }

    if (c == '!') {
        debug_escape_pending = true;
        return;
    }

    // Everything else is CAT protocol traffic (PowerSDR, hamlib,
    // future QMX-style commands, ...) — see cat_app.c.
    cat_process_char(c);
}

// Invoked when CDC receives data from the host
void tud_cdc_rx_cb(uint8_t itf) {
    uint8_t buf[64];
    uint32_t count = tud_cdc_n_read(itf, buf, sizeof(buf));

    for (uint32_t i = 0; i < count; i++) {
#if LMR_SDR_DUAL_CORE
        // This callback runs on Core1 (TinyUSB lives there — see
        // main.c). Relay the raw byte to Core0 over the hardware
        // inter-core FIFO rather than parsing it here; non-blocking
        // with a short timeout, because this loop also has to keep
        // servicing the audio endpoints every iteration and must never
        // stall waiting on Core0. CAT/debug input is low-rate text, so
        // dropping an occasional byte under a genuine stall is an
        // acceptable trade — and should be rare to never in practice.
        if (!multicore_fifo_push_timeout_us((uint32_t)buf[i], 100)) {
            break;
        }
#else
        cdc_app_process_rx_byte((char)buf[i]);
#endif
    }
}
