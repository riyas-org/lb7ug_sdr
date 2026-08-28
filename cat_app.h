/*
 * cat_app.h — CAT command interface (PowerSDR / Kenwood-style) and basic
 * radio state for the LMR-SDR QSD/QSE transceiver.
 *
 * Design intent (read this before extending):
 *   - radio_state holds the "truth" of what the rig is doing right now
 *     (VFO frequency, mode, PTT). Every control surface — CAT over CDC
 *     today, a future front-panel encoder/buttons for standalone
 *     operation, or a QMX-style command set — should read/write this one
 *     struct through the radio_* helper functions rather than poking
 *     hardware directly. That keeps CAT, standalone UI, and any future
 *     control paths consistent with each other for free.
 *   - cat_process_char() is a byte-at-a-time line accumulator so it can
 *     be fed from any transport (CDC today; a UART or the debug console
 *     tomorrow) without change.
 *   - cat_dispatch_command() is a small if/else ladder on purpose, not a
 *     table, because the command set is currently tiny. If/when this
 *     grows (full Kenwood IF;, QMX commands, etc.) switch it to a
 *     {mnemonic, handler} lookup table — the parsing/framing code above
 *     it does not need to change.
 */

#ifndef CAT_APP_H
#define CAT_APP_H

#include <stdbool.h>
#include <stdint.h>

// ---------------------------------------------------------------------
// PTT hardware — EDIT to match your wiring.
// Drives the T/R switching for the shared QSD/QSE FST3253 path (e.g. a
// relay, or an RF switch enable line). Active-high by default.
// ---------------------------------------------------------------------
#ifndef RADIO_PTT_PIN
#define RADIO_PTT_PIN   11   // GP11: +ve (high) = TX, -ve (low) = RX (idle/boot default)
#endif

// ---------------------------------------------------------------------
// Manual PTT button — a momentary switch to GND, using the RP2040's
// internal pull-up (so idle/unpressed reads high, pressed reads low).
// EDIT if your button is wired the other way (e.g. to 3V3 with a
// pull-down) — see radio_ptt_button_task() in cat_app.c.
// ---------------------------------------------------------------------
#ifndef RADIO_MANUAL_PTT_PIN
#define RADIO_MANUAL_PTT_PIN   14   // GP14
#endif

typedef enum {
    RADIO_MODE_LSB = 1,
    RADIO_MODE_USB = 2,
    RADIO_MODE_CW  = 3,
} radio_mode_t;

typedef struct {
    uint32_t     vfo_a_hz;
    radio_mode_t mode;
    bool         ptt_active;
} radio_state_t;

extern radio_state_t radio_state;

/** One-time setup: PTT GPIO direction, initial LO frequency at boot. */
void radio_init(uint32_t default_freq_hz);

/** Set VFO A and immediately reprogram the Si5351 LO. */
void radio_set_freq(uint32_t freq_hz);

/** Assert/de-assert PTT (T/R switching). */
void radio_set_ptt(bool tx);

/**
 * radio_ptt_button_task — call once per Core0 loop iteration (alongside
 * radio_task()) to poll and debounce the manual PTT button on
 * RADIO_MANUAL_PTT_PIN. Combines with any CAT-driven TX;/RX; request via
 * OR (either source can key the rig; releasing one doesn't cancel a TX
 * still held by the other) — see the comment above cat_ptt_request in
 * cat_app.c for the full reasoning.
 */
void radio_ptt_button_task(void);

/**
 * radio_rts_ptt_task — call once per Core0 loop iteration (alongside
 * radio_task() and radio_ptt_button_task()) to fold the serial port's
 * RTS control line into the same OR'd PTT combine, as a backup keying
 * method for CAT/logging software that toggles RTS directly instead of
 * sending TX;/RX;. See cdc_rts_active in cdc_app.c and the comment
 * above cat_ptt_request in cat_app.c.
 */
void radio_rts_ptt_task(void);

/** Call once per main-loop iteration (alongside audio_apply_pending_rate_change())
 *  to apply any CAT-triggered frequency/PTT changes queued by cat_process_char().
 *  This is what keeps the slow, blocking I2C writes (Si5351, WM8731) out of
 *  the TinyUSB rx callback — see the comment above cat_dispatch_command()
 *  in cat_app.c for why that matters. */
void radio_task(void);

// -----------------------------------------------------------------------
// CAT parser
// -----------------------------------------------------------------------

/** Feed one received byte (call from the CDC/UART rx callback). */
void cat_process_char(char c);

#endif // CAT_APP_H
