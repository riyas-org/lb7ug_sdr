/*
 * dual_core_config.h — one switch controlling the Core0/Core1 split.
 *
 * Adapted from a similar FEATURE_DUAL_CORE flag in a sister SDR project's
 * main.c, which used the same idea to solve the same class of problem:
 * a slow control-plane I2C write (there: Si4732; here: Si5351/WM8731)
 * stalling whatever's servicing USB audio, because both were sharing one
 * core's time budget.
 *
 *   1 (default): Core1 owns TinyUSB entirely — tusb_init(), tud_task(),
 *                audio_task(), cdc_task() — plus led_blinking_task()
 *                (it only reacts to TinyUSB's own connection callbacks,
 *                which now fire on Core1 too). Core0 owns every I2C
 *                control-plane device (Si5351, WM8731 mute/route) and
 *                all CAT/debug parsing, fed by a byte-at-a-time relay
 *                over the RP2040's hardware inter-core FIFO — see the
 *                file header comment in main.c for the full picture.
 *                Because the two live on separate cores, no I2C
 *                transaction on Core0, however long it runs, can ever
 *                delay Core1's servicing of the codec's DMA buffers.
 *
 *   0: single-core, everything on Core0 — the behaviour this project had
 *      before this switch existed. Useful for bring-up with fewer
 *      moving parts to reason about, or if you want Core1 free for
 *      something else later (extra DSP, a second capture path, ...) and
 *      are willing to accept that CAT/PTT-triggered I2C writes share a
 *      time budget with audio_task() again (see the deferred-flag
 *      pattern in cat_app.c, which still helps even here).
 */
#ifndef DUAL_CORE_CONFIG_H
#define DUAL_CORE_CONFIG_H

#ifndef LMR_SDR_DUAL_CORE
#define LMR_SDR_DUAL_CORE 1
#endif

#endif // DUAL_CORE_CONFIG_H
