/*
 * cat_app.c — Minimal Kenwood-style CAT parser (works with PowerSDR and
 * most hamlib "Kenwood" rig backends) plus the radio_state_t glue that
 * drives the Si5351 LO and PTT line.
 *
 * Implemented commands (deliberately small — extend the ladder in
 * cat_dispatch_command() as you need more):
 *   ID;        -> "ID019;"                  (identify as a TS-2000)
 *   FA;        -> "FA00007074000;"          (get VFO A, 11-digit Hz)
 *   FA<11d>;   -> set VFO A, reprograms Si5351
 *   MD;        -> "MD2;"                    (get mode, 2=USB)
 *   MDn;       -> set mode (stored; QSD/QSE doesn't need RF-side action)
 *   PS;        -> "PS1;"                    (power-on state)
 *   AI0;/AI1;  -> accepted, stored; AI1; enables unsolicited TX;/RX;
 *                 reports whenever PTT changes from ANY source (CAT,
 *                 the manual button, or RTS) — see radio_set_ptt().
 *   TX;        -> PTT on
 *   RX;        -> PTT off
 *
 * Anything else is silently ignored (matches real rigs: unknown/unsupported
 * commands are just not answered rather than erroring).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cat_app.h"
#include "cdc_app.h"
#include "si5351.h"
#include "hardware/gpio.h"
#include "pico/time.h"

/*
 * wm8731_set_input_route, wm8731_set_headphone_mute — implemented in
 * wm8731.c. Not (yet) declared in wm8731.h since that file wasn't part
 * of what's being edited here; these extern prototypes let cat_app.c
 * call them without needing to touch a header whose full existing
 * contents aren't visible. Feel free to move these declarations into
 * wm8731.h alongside wm8731_set_mute() etc. — same functions, tidier.
 *
 * wm8731_set_input_route(tx) switches the ADC input mux:
 *   tx=false (RX): Line In selected (I/Q from the QSD via the NE5532s),
 *                  mic input muted.
 *   tx=true  (TX): Mic input selected, line input effectively bypassed.
 *
 * wm8731_set_headphone_mute(mute) mutes/restores ONLY the analog
 * headphone output (LHPOUT/RHPOUT) — the WM8731's Line Out (LOUT/ROUT)
 * is a separate, always-live tap with no mute of its own, so this
 * silences the LM386/speaker during TX without touching the line-out
 * feed to the QSE. See the comment on that function in wm8731.c.
 */
extern void wm8731_set_input_route(bool tx);
extern void wm8731_set_headphone_mute(bool mute);

/*
 * cdc_rts_active — implemented in cdc_app.c, updated from
 * tud_cdc_line_state_cb() whenever the host toggles the serial port's
 * RTS line. Used here as a third, independent backup PTT source (see
 * radio_rts_ptt_task() below) for CAT/logging software that keys PTT
 * directly via RTS rather than sending TX;/RX;.
 */
extern volatile bool cdc_rts_active;

radio_state_t radio_state = {
    .vfo_a_hz    = 7074000,   // 40m FT8-ish default; change as you like
    .mode        = RADIO_MODE_USB,
    .ptt_active  = false,
};

static bool cat_ai_enabled = true; // gates unsolicited TX;/RX; reports — see radio_set_ptt()

/*
 * Deferred radio-hardware changes.
 *
 * cat_dispatch_command() runs inside tud_cdc_rx_cb(), which TinyUSB calls
 * synchronously from tud_task() in the main loop. si5351_set_freq() and
 * the WM8731 route/mute calls in radio_set_freq()/radio_set_ptt() are
 * blocking I2C transactions — if they ran directly from that callback,
 * they'd stall tud_task() (and with it audio_task(), which is what
 * actually shuttles samples between the codec DMA buffers and the USB
 * isochronous endpoints) for their full duration, risking an audible
 * glitch on every retune or PTT edge.
 *
 * So instead, cat_dispatch_command() only ever touches these flags (fast,
 * no I2C), and radio_task() — called from the main loop, not from inside
 * a USB callback — does the actual hardware writes.
 *
 * These are only ever written on core0's main-loop thread (both the rx
 * callback and radio_task() run there — TinyUSB isn't safe to call from
 * a second core), so there's no real concurrency here today; volatile is
 * just defensive hygiene, and matters more if this ever moves to a
 * two-core split later (control-plane I2C on core1, USB/audio on core0).
 */
static volatile bool     freq_change_pending = false;
static volatile uint32_t pending_freq_hz     = 0;
static volatile bool     ptt_change_pending  = false;
static volatile bool     pending_ptt_state   = false;

/*
 * Three independent PTT sources — CAT (TX;/RX;), the manual button on
 * RADIO_MANUAL_PTT_PIN, and the serial port's RTS line (a common backup
 * PTT method for older/simpler CAT software) — combine via OR: the rig
 * transmits if ANY source wants it to. Releasing one source alone never
 * drops TX out from under another still holding it (e.g. RTS asserted
 * for PTT while the operator also happens to be sending FA; over CAT).
 * cat_ptt_request and manual_ptt_request are Core0-only (CAT dispatch
 * and the button poll both run there); rts_ptt_request is written only
 * by radio_rts_ptt_task(), also Core0 — so no cross-core concern for
 * any of the three. cdc_rts_active (the actual RTS line state) IS
 * cross-core (set from Core1's tud_cdc_line_state_cb()), which is why
 * it's a separate volatile in cdc_app.c rather than folded in here
 * directly.
 */
static bool cat_ptt_request    = false;
static bool manual_ptt_request = false;
static bool rts_ptt_request    = false;

/** Recompute the combined PTT request and queue it for radio_task(). */
static void radio_update_ptt_request(void) {
    //pending_ptt_state  = cat_ptt_request || manual_ptt_request || rts_ptt_request;
    pending_ptt_state  = cat_ptt_request || manual_ptt_request ;
    ptt_change_pending = true; // applied by radio_task(), not here
}

// -----------------------------------------------------------------------
// Radio state helpers — the single place that touches hardware
// -----------------------------------------------------------------------

void radio_init(uint32_t default_freq_hz) {
    gpio_init(RADIO_PTT_PIN);
    gpio_set_dir(RADIO_PTT_PIN, GPIO_OUT);

    // Manual PTT button: momentary to GND, internal pull-up so idle
    // reads high — see radio_ptt_button_task() for the active-low read.
    gpio_init(RADIO_MANUAL_PTT_PIN);
    gpio_set_dir(RADIO_MANUAL_PTT_PIN, GPIO_IN);
    gpio_pull_up(RADIO_MANUAL_PTT_PIN);

    radio_state.vfo_a_hz = default_freq_hz;
    si5351_set_freq(radio_state.vfo_a_hz);
    si5351_enable_output(true);

    // Establish the RX default: T/R line low, ADC on Line In (QSD I/Q),
    // mic muted. Goes through radio_set_ptt() rather than poking the
    // GPIO directly so the codec routing stays in sync with the T/R
    // state from the very first moment, not just after the first TX/RX
    // CAT command.
    radio_set_ptt(false);
}

void radio_set_freq(uint32_t freq_hz) {
    radio_state.vfo_a_hz = freq_hz;
    si5351_set_freq(freq_hz);
}

void radio_set_ptt(bool tx) {
    bool state_changed = (tx != radio_state.ptt_active);
    radio_state.ptt_active = tx;

    // T/R switching: GP11 high = TX, low = RX. Drives the external
    // FST3253 mux between QSD (RX) and QSE (TX) operation.
    gpio_put(RADIO_PTT_PIN, tx ? 1 : 0);

    // ADC input routing: line-in (QSD I/Q) for RX, mic for TX.
    wm8731_set_input_route(tx);

    // Speaker mute: the WM8731's headphone output (LHPOUT/RHPOUT,
    // driving the LM386) has its own independent mute, completely
    // separate from Line Out (LOUT/ROUT, feeding the QSE op-amps) —
    // muting one never touches the other on this codec. So TX mutes
    // just the headphone/speaker path; the DAC keeps driving Line Out
    // the whole time, RX or TX, which is exactly what the QSE needs.
    wm8731_set_headphone_mute(tx);

    // Unsolicited status report (Kenwood "AI" behaviour): if the host
    // has asked for auto-info (AI1;), push TX;/RX; whenever PTT actually
    // changes, no matter which of the three sources (CAT, button, RTS)
    // triggered it. This is what lets PowerSDR/hamlib notice the rig
    // keyed itself from the button or RTS rather than from a CAT command
    // that software itself sent — every source funnels through this one
    // function, so this is the single place that needs to know. Only
    // fires on an actual transition (state_changed), not every time
    // radio_set_ptt() happens to get called with the same value (e.g.
    // one source re-asserting while another already holds PTT).
    if (state_changed && cat_ai_enabled) {
        cdc_printf(tx ? "TX;" : "RX;");
    }
}

void radio_task(void) {
    if (freq_change_pending) {
        freq_change_pending = false;
        radio_set_freq(pending_freq_hz);
    }
    if (ptt_change_pending) {
        ptt_change_pending = false;
        radio_set_ptt(pending_ptt_state);
    }
}

void radio_ptt_button_task(void) {
    //cdc_printf("GP14 raw=%d\r\n", gpio_get(RADIO_MANUAL_PTT_PIN)); 
    // Simple time-based debounce: a raw-reading change has to hold
    // steady for DEBOUNCE_MS before it's accepted as real. Button PTT
    // needs to feel instant, so this stays short — mechanical bounce is
    // typically well under 10ms, 25ms leaves comfortable margin without
    // adding noticeable keying lag.
    const uint32_t DEBOUNCE_MS = 25;
    static bool     last_stable    = false; // debounced, accepted state
    static bool     last_raw       = false; // most recent raw reading
    static uint32_t last_change_ms = 0;

    // Active-low: pressed pulls the pin to GND (see radio_init()'s
    // pull-up), so "pressed" is a logic 0 — invert here so the rest of
    // this function just deals in "is it pressed" (true = pressed).
    bool     raw = !gpio_get(RADIO_MANUAL_PTT_PIN);
    uint32_t now = to_ms_since_boot(get_absolute_time());

    if (raw != last_raw) {
        last_raw       = raw;
        last_change_ms = now;
    }

    if (raw != last_stable && (now - last_change_ms) >= DEBOUNCE_MS) {
        last_stable         = raw;
        manual_ptt_request  = raw;
        radio_update_ptt_request();
    }
}

void radio_rts_ptt_task(void) {
    // RTS is a control line driven by the host's serial stack, not a
    // mechanical contact — no bounce, so no debounce needed here, just
    // an edge check so we only touch the pending-flag machinery when
    // it actually changes.
    static bool last_rts = false;

    bool rts = cdc_rts_active;
    if (rts != last_rts) {
        last_rts         = rts;
        rts_ptt_request  = rts;
        radio_update_ptt_request();
    }
}

// -----------------------------------------------------------------------
// CAT command dispatch
// -----------------------------------------------------------------------

/*
 * NOTE: this runs inside tud_cdc_rx_cb(), called synchronously from
 * tud_task(). Keep it fast — no I2C, no blocking calls. Anything that
 * needs to touch the Si5351 or WM8731 goes through the pending flags
 * above and gets applied later by radio_task() from the main loop. See
 * the big comment on those flags for why.
 */
static void cat_dispatch_command(const char *cmd) {
    // cmd points at the mnemonic, NUL-terminated, ';' already stripped.
    if (cmd[0] == '\0') return;

    if (strncmp(cmd, "ID", 2) == 0) {
        cdc_printf("ID019;");
        return;
    }

    if (strncmp(cmd, "FA", 2) == 0) {
        if (cmd[2] == '\0') {
            cdc_printf("FA%011lu;", (unsigned long)radio_state.vfo_a_hz);
        } else {
            uint32_t hz = (uint32_t)strtoul(&cmd[2], NULL, 10);
            if (hz > 0) {
                pending_freq_hz = hz;
                freq_change_pending = true; // applied by radio_task(), not here
            }
        }
        return;
    }

    if (strncmp(cmd, "MD", 2) == 0) {
        if (cmd[2] == '\0') {
            cdc_printf("MD%d;", (int)radio_state.mode);
        } else {
            int m = cmd[2] - '0';
            if (m >= RADIO_MODE_LSB && m <= RADIO_MODE_CW) {
                radio_state.mode = (radio_mode_t)m;
            }
        }
        return;
    }

    if (strncmp(cmd, "PS", 2) == 0) {
        if (cmd[2] == '\0') cdc_printf("PS1;");
        return;
    }

    if (strncmp(cmd, "AI", 2) == 0) {
        if (cmd[2] == '\0') {
            cdc_printf("AI%d;", cat_ai_enabled ? 1 : 0);
        } else {
            cat_ai_enabled = (cmd[2] != '0');
        }
        return;
    }

    if (strcmp(cmd, "TX") == 0) {
        cat_ptt_request = true;
        radio_update_ptt_request();
        return;
    }

    if (strcmp(cmd, "RX") == 0) {
        cat_ptt_request = false;
        radio_update_ptt_request();
        return;
    }

    // Unknown command: ignore, matching real-rig behaviour.
}

// -----------------------------------------------------------------------
// Byte-at-a-time framing
// -----------------------------------------------------------------------

#define CAT_LINE_MAX 32

void cat_process_char(char c) {
    static char    line[CAT_LINE_MAX];
    static uint8_t len = 0;

    if (c == ';') {
        line[len] = '\0';
        cat_dispatch_command(line);
        len = 0;
        return;
    }

    if (c == '\r' || c == '\n') {
        // Some CAT libraries use bare CR/LF instead of ';'. Treat a
        // non-empty accumulated line the same way; ignore blank lines.
        if (len > 0) {
            line[len] = '\0';
            cat_dispatch_command(line);
            len = 0;
        }
        return;
    }

    if (len < CAT_LINE_MAX - 1) {
        line[len++] = c;
    } else {
        // Line too long (garbage/noise on the wire) — reset and resync.
        len = 0;
    }
}
