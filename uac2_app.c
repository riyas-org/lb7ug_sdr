/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Jerzy Kasenberg
 * Copyright (c) 2022 Angel Molina 
 * Copyright (c) 2023 Dhiru Kholia 
 * Copyright (c) 2026 Riyas Vettukattil 
 *
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
 */
 
#if UAC2_DEBUG
#define UAC_LOG(...) UAC_LOG(__VA_ARGS__)
#else
#define UAC_LOG(...) do{}while(0)
#endif 

#include <stdio.h>
#include <string.h>

#include "bsp/board_api.h"
#include "tusb.h"
#include "usb_descriptors.h"
#include "common.h"
#include "cdc_app.h"
#include "wm8731.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTOTYPES
//--------------------------------------------------------------------+

#define N_SAMPLE_RATES                          TU_ARRAY_SIZE(sample_rates)
#define MIC_BUF_SIZE                            (WM8731_DMA_BUFFER_SIZE > (CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ / 4) ? \
                                                 WM8731_DMA_BUFFER_SIZE : (CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ / 4))

//--------------------------------------------------------------------+
// GLOBAL VARIABLES
//--------------------------------------------------------------------+

static uint16_t tx_buf_pos = 0;
static uint8_t  target_tx_buf = 1; // Start by filling the second buffer
static bool     codec_initialized = false;

uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;
int8_t   mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];
int16_t  volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];
int32_t  spk_buf[CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ / 4];
int32_t  mic_buf[MIC_BUF_SIZE];
uint16_t spk_data_size;

const uint8_t resolutions_per_format[CFG_TUD_AUDIO_FUNC_1_N_FORMATS] = {
    CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_RX
};

uint8_t current_resolution;
static bool streaming_spk_active = false;
static bool streaming_mic_active = false;
// Fixed at 48 kHz — this project only ever runs the codec at 48 kHz (no
// runtime rate switching), so a single-entry table is deliberate, not a
// placeholder for more rates later.
uint32_t sample_rates[] = {48000};
uint32_t current_sample_rate = 48000;

// Underrun/overrun instrumentation (see audio_task()). Deliberately not
// static — cdc_app.c's '?' debug status command reports these via an
// extern declaration, same ad-hoc pattern the rest of this codebase
// already uses (e.g. cat_app.c's extern wm8731_* prototypes) rather than
// a dedicated header. Use these to actually characterize an underrun
// report instead of guessing: reset with the '!u' debug key, do
// whatever you suspect triggers it (retune, PTT, leave it running),
// then check '!?' — which counter moved tells you which side to chase.
volatile uint32_t audio_spk_underrun_count = 0; // speaker buffer not refilled before DMA needed it (audible glitch on RX audio out)
volatile uint32_t audio_mic_overrun_count  = 0; // mic data dropped because the USB IN endpoint wasn't drained in time (glitch/gap in what PowerSDR hears)

//--------------------------------------------------------------------+
// HARDWARE LOOPBACK TASK
//--------------------------------------------------------------------+

/**
 * This function bypasses TinyUSB and directly mirrors the ADC input 
 * to the DAC output with a bitwise inversion.
 */
void run_bit_perfect_test(void) {
    // Check if the RX DMA has finished filling a buffer
    if (wm8731_rx_ready) {
        // Identify the buffer that was JUST filled by the ADC
        // Because the IRQ increments the index AFTER the transfer, 
        // the valid data is in (current - 1).
        uint8_t rx_idx = (wm8731_current_rx_buffer + WM8731_DMA_NUM_BUFFERS - 1) % WM8731_DMA_NUM_BUFFERS;
        
        // Identify the TX buffer that is currently idle (ready to be filled)
        // Since TX and RX are phase-locked, the buffer the DAC just finished 
        // playing is the same index as the one the ADC just finished filling.
        uint8_t tx_idx = rx_idx;

        // Process the data
        for (int i = 0; i < WM8731_DMA_BUFFER_SIZE; i++) {
            int32_t sample = wm8731_rx_buffer[rx_idx][i];

            /* --- THE INVERSION --- */
            // If the audio is bit-aligned, this is clean.
            // If the audio is shifted by 1 bit, this sounds like hell.
            // wm8731_tx_buffer[tx_idx][i] = ~sample;
            // wm8731_tx_buffer[tx_idx][i] = sample;
            wm8731_tx_buffer[tx_idx][i] = sample << 8;
        }

        // Clear the ready flag so we wait for the next DMA cycle
        wm8731_rx_ready = false;
    }
}

//--------------------------------------------------------------------+
// AUDIO Task
//--------------------------------------------------------------------+

void audio_task(void) {
    if (!codec_initialized) {
        static uint32_t last_print = 0;
        if (tusb_time_millis_api() - last_print > 2000) {            
            last_print = tusb_time_millis_api();
        }
        return;
    }

    // -----------------------------------------------------------------
    // 1. SPEAKER PATH (USB -> Codec)
    // -----------------------------------------------------------------
    if (streaming_spk_active) {
        // Check if the DMA IRQ just swapped buffers
        if (wm8731_tx_ready) {
            wm8731_tx_ready = false;

            // If we hadn't finished filling the buffer DMA just started
            // playing, its tail is stale/leftover data from a previous
            // cycle — an audible underrun. Count it before resetting
            // tx_buf_pos so we're checking the buffer that just got
            // played, not the new one we're about to fill.
            if (tx_buf_pos < WM8731_DMA_BUFFER_SIZE) {
                audio_spk_underrun_count++;
            }

            tx_buf_pos = 0;
            // The buffer we were filling (target_tx_buf) is now being played by DMA.
            // We must now start filling the OTHER buffer.
            target_tx_buf = (wm8731_current_tx_buffer + 1) % WM8731_DMA_NUM_BUFFERS;
        }

        // If our current target buffer isn't full yet, try to read from USB
        if (tx_buf_pos < WM8731_DMA_BUFFER_SIZE) {
            // Calculate how much space is left in the current DMA buffer (in bytes)
            uint32_t bytes_needed = (WM8731_DMA_BUFFER_SIZE - tx_buf_pos) * 4;
            uint32_t to_read = (bytes_needed < sizeof(spk_buf)) ? bytes_needed : sizeof(spk_buf);

            uint32_t n = tud_audio_read(spk_buf, to_read);
            if (n > 0) {
                int32_t *usb_samples = (int32_t *)spk_buf;
                uint32_t num_pairs = n / 8;
                
                for (uint32_t i = 0; i < num_pairs; i++) {
                    int32_t left  = usb_samples[i * 2];
                    int32_t right = usb_samples[i * 2 + 1];
                    
                    // Pack into the 32-bit DMA buffer word
                    wm8731_tx_buffer[target_tx_buf][tx_buf_pos++] = left;
                    wm8731_tx_buffer[target_tx_buf][tx_buf_pos++] = right;
                }
            }
        }
    }
    
    // -----------------------------------------------------------------
    // 2. MICROPHONE PATH (Codec -> USB)
    // -----------------------------------------------------------------
    if (streaming_mic_active) {
        if (wm8731_rx_ready) {
            uint8_t source_buf = (wm8731_current_rx_buffer + WM8731_DMA_NUM_BUFFERS - 1) % WM8731_DMA_NUM_BUFFERS;
            int32_t *codec_samples = (int32_t *)wm8731_rx_buffer[source_buf];
            int32_t *usb_mic_data = (int32_t *)mic_buf;
        
            // Correctly pass through both Left and Right channels for true Stereo
            for (uint16_t i = 0; i < WM8731_DMA_BUFFER_SIZE; i++) {
                usb_mic_data[i] = codec_samples[i]; 
            } 
            
            /* Alternative processing examples:
            for (uint16_t i = 0; i < WM8731_DMA_BUFFER_SIZE; i += 2) {
                // Manually shift the noisy sample left by 1 bit
                // This often "fixes" the click if the PIO missed the first bit
                usb_mic_data[i]     = codec_samples[i] << 1; 
                usb_mic_data[i + 1] = codec_samples[i + 1]; 
            }
            
            for (uint16_t i = 0; i < WM8731_DMA_BUFFER_SIZE; i += 2) {
                int32_t left_sample = codec_samples[i+1];
                usb_mic_data[i]     = left_sample; // USB Left
                usb_mic_data[i + 1] = left_sample; // USB Right (Mirroring Left)
            }
            */
        
            uint32_t bytes_total = WM8731_DMA_BUFFER_SIZE * 4;
            uint32_t chunk = (current_sample_rate / 1000) * 2 * 4; // 1 ms stereo (384 bytes @ 48k)
            
            for (uint32_t off = 0; off < bytes_total; off += chunk) {
                uint32_t want = TU_MIN(chunk, bytes_total - off);
                uint32_t wrote = tud_audio_write((uint8_t *)mic_buf + off, want);
                if (wrote < want) {
                    // Host isn't draining the mic IN endpoint fast enough
                    // (or it briefly wasn't ready) — this chunk's tail
                    // never reached PowerSDR. Count it.
                    audio_mic_overrun_count++;
                }
            }
            
            wm8731_rx_ready = false;
        }
    }
}

//--------------------------------------------------------------------+
// Codec Initialization
//--------------------------------------------------------------------+

void audio_init_codec(void) {
    if (codec_initialized) return;

    UAC_LOG("audio_init_codec: setting up I2S PIO/DMA...\r\n");
    wm8731_i2s_init(current_sample_rate);
    UAC_LOG("I2S PIO/DMA ready\r\n");

    if (wm8731_init()) {
        if (wm8731_configure(current_sample_rate)) {
            wm8731_set_volume(79);
            UAC_LOG("WM8731 configured at %" PRIu32 " Hz\r\n", current_sample_rate);
        } else {
            UAC_LOG("WARNING: wm8731_configure() failed\r\n");
        }
        codec_initialized = true;
        UAC_LOG("audio_init_codec: done\r\n");
    } else {
        UAC_LOG("WARNING: wm8731_init() failed - check I2C address/wiring\r\n");
    }
}

//--------------------------------------------------------------------+
// Volume / Mute Helpers — Windows Compatibility
//
// UAC2 Feature Unit channels:
//   0 = master,  1 = left,  2 = right
//
// Linux mixer sends SET on ch 0 (master).
// Windows sends SET on ch 1 and ch 2 independently.
//
// Strategy: always keep the volume[] / mute[] arrays up to date, then
// derive a single codec setting from them after every update.
//--------------------------------------------------------------------+

#if (CFG_TUSB_MCU == OPT_MCU_RP2040) && !defined(AUDIO_LOOPBACK)

/**
 * Convert the UAC2 volume array to a single WM8731 headphone register
 * value (0-127) and write it.
 *
 * Priority: if master (ch0) != 0, use it; otherwise average ch1 & ch2.
 * UAC2 volume is Q8.8 signed, 0x0000 = 0 dB, negative = attenuation.
 * The codec range we advertise is [−VOLUME_CTRL_50_DB .. 0] mapped
 * onto WM8731 codes [48 .. 127].
 */
static void apply_volume_to_codec(void) {
    int16_t vol_db;   // Q8.8

    if (volume[0] != 0) {
        vol_db = volume[0];
    } else {
        // Average left and right; guard divide-by-zero if array too small
        int32_t sum = 0;
        int     cnt = 0;
        for (int ch = 1; ch <= CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX; ch++) {
            sum += volume[ch];
            cnt++;
        }
        vol_db = (cnt > 0) ? (int16_t)(sum / cnt) : 0;
    }

    uint8_t codec_vol;
    if (vol_db <= -VOLUME_CTRL_50_DB) {
        codec_vol = 48;
    } else if (vol_db >= 0) {
        codec_vol = 127;
    } else {
        codec_vol = (uint8_t)(48 + ((int32_t)(vol_db + VOLUME_CTRL_50_DB) * 79) / VOLUME_CTRL_50_DB);
    }
    
    wm8731_set_volume(codec_vol);
    UAC_LOG("Codec vol: %u/127 (UAC %d dBx256)\r\n", codec_vol, (int)vol_db);
}

/**
 * Derive a single mute state for the codec:
 * muted if master (ch0) OR any per-channel flag is set.
 */
static void apply_mute_to_codec(void) {
    bool any_muted = (mute[0] != 0);
    for (int ch = 1; ch <= CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX && !any_muted; ch++) {
        if (mute[ch]) any_muted = true;
    }
    wm8731_set_mute(any_muted);
    UAC_LOG("Codec mute: %s\r\n", any_muted ? "ON" : "off");
}

#endif /* RP2040 && !AUDIO_LOOPBACK */

//--------------------------------------------------------------------+
// CLOCK ENTITY CONTROL HANDLERS
//--------------------------------------------------------------------+

static bool tud_audio_clock_get_request(uint8_t rhport, audio20_control_request_t const *request) {
    TU_ASSERT(request->bEntityID == UAC2_ENTITY_CLOCK);   

    if (request->bControlSelector == AUDIO20_CS_CTRL_SAM_FREQ) {
        if (request->bRequest == AUDIO20_CS_REQ_CUR) {
            UAC_LOG("Clock GET CUR freq: %" PRIu32 "\r\n", current_sample_rate);
            audio20_control_cur_4_t curf = { .bCur = (int32_t) tu_htole32(current_sample_rate) };
            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &curf, sizeof(curf));
        } else if (request->bRequest == AUDIO20_CS_REQ_RANGE) {
            audio20_control_range_4_n_t(N_SAMPLE_RATES) rangef;
            rangef.wNumSubRanges = tu_htole16(N_SAMPLE_RATES);
            UAC_LOG("Clock GET RANGE: %d rates\r\n", N_SAMPLE_RATES);
            for (uint8_t i = 0; i < N_SAMPLE_RATES; i++) {
                rangef.subrange[i].bMin = (int32_t) tu_htole32(sample_rates[i]);
                rangef.subrange[i].bMax = (int32_t) tu_htole32(sample_rates[i]);
                rangef.subrange[i].bRes = 0;
                UAC_LOG("  %u Hz\r\n", sample_rates[i]);
            }
            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &rangef, sizeof(rangef));
        }
    } else if (request->bControlSelector == AUDIO20_CS_CTRL_CLK_VALID && request->bRequest == AUDIO20_CS_REQ_CUR) {
        audio20_control_cur_1_t cur_valid = { .bCur = 1 };
        return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &cur_valid, sizeof(cur_valid));
    }

    UAC_LOG("Clock GET UNSUPPORTED: Ent=%u Sel=%u Req=%u\r\n",
               request->bEntityID, request->bControlSelector, request->bRequest);
    return false;
}

static bool tud_audio_clock_set_request(uint8_t rhport, audio20_control_request_t const *request, uint8_t const *buf) {
    (void)rhport;
    TU_ASSERT(request->bEntityID == UAC2_ENTITY_CLOCK);
    TU_VERIFY(request->bRequest == AUDIO20_CS_REQ_CUR);

    if (request->bControlSelector == AUDIO20_CS_CTRL_SAM_FREQ) {
        TU_VERIFY(request->wLength == sizeof(audio20_control_cur_4_t));
        current_sample_rate = (uint32_t) tu_le32toh(((audio20_control_cur_4_t const *)buf)->bCur);
        UAC_LOG("Clock SET CUR freq: %" PRIu32 "\r\n", current_sample_rate);
        
#if (CFG_TUSB_MCU == OPT_MCU_RP2040) && !defined(AUDIO_LOOPBACK)
        if (codec_initialized) {
            wm8731_configure(current_sample_rate);
            wm8731_set_i2s_samplerate(current_sample_rate); 
            UAC_LOG("WM8731 reconfigured for %" PRIu32 " Hz\r\n", current_sample_rate);
        }
#endif
        return true;
    }

    UAC_LOG("Clock SET UNSUPPORTED: Ent=%u Sel=%u\r\n",
               request->bEntityID, request->bControlSelector);
    return false;
}

//--------------------------------------------------------------------+
// FEATURE UNIT CONTROL HANDLERS
//--------------------------------------------------------------------+

static bool tud_audio_feature_unit_get_request(uint8_t rhport, audio20_control_request_t const *request) {
    TU_ASSERT(request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT);

    if (request->bControlSelector == AUDIO20_FU_CTRL_MUTE && request->bRequest == AUDIO20_CS_REQ_CUR) {
        audio20_control_cur_1_t m = { .bCur = mute[request->bChannelNumber] };
        UAC_LOG("FU GET MUTE Ch=%u val=%d\r\n", request->bChannelNumber, m.bCur);
        return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &m, sizeof(m));
    } else if (request->bControlSelector == AUDIO20_FU_CTRL_VOLUME) {
        if (request->bRequest == AUDIO20_CS_REQ_RANGE) {
            audio20_control_range_2_n_t(1) range_vol = { .wNumSubRanges = tu_htole16(1) };
            range_vol.subrange[0].bMin = tu_htole16(-VOLUME_CTRL_50_DB);
            range_vol.subrange[0].bMax = tu_htole16(VOLUME_CTRL_0_DB);
            range_vol.subrange[0].bRes = tu_htole16(256);
            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &range_vol, sizeof(range_vol));
        } else if (request->bRequest == AUDIO20_CS_REQ_CUR) {
            audio20_control_cur_2_t cur_vol = { .bCur = tu_htole16(volume[request->bChannelNumber]) };
            UAC_LOG("FU GET VOLUME Ch=%u val=%d dB\r\n",
                       request->bChannelNumber,
                       (int16_t)tu_le16toh(cur_vol.bCur) / 256);
            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &cur_vol, sizeof(cur_vol));
        }
    }

    UAC_LOG("FU GET UNSUPPORTED: Ch=%u Sel=%u Req=%u\r\n",
               request->bChannelNumber, request->bControlSelector, request->bRequest);
    return false;
}

static bool tud_audio_feature_unit_set_request(uint8_t rhport,
                                               audio20_control_request_t const *request,
                                               uint8_t const *buf) {
    (void)rhport;
    TU_ASSERT(request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT);
    TU_VERIFY(request->bRequest == AUDIO20_CS_REQ_CUR);

    uint8_t ch = request->bChannelNumber;

    if (request->bControlSelector == AUDIO20_FU_CTRL_MUTE) {
        TU_VERIFY(request->wLength == sizeof(audio20_control_cur_1_t));
        
        // Guard: don't write past the mute[] array
        if (ch <= CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX) {
            mute[ch] = ((audio20_control_cur_1_t const *)buf)->bCur;
            UAC_LOG("FU SET MUTE Ch=%d val=%d\r\n", ch, mute[ch]);
        }
        
#if (CFG_TUSB_MCU == OPT_MCU_RP2040) && !defined(AUDIO_LOOPBACK)
        // Windows fix: apply mute regardless of which channel changed
        // apply_mute_to_codec() checks all channels.
        if (codec_initialized) apply_mute_to_codec();
#endif
        return true;
    } else if (request->bControlSelector == AUDIO20_FU_CTRL_VOLUME) {
        TU_VERIFY(request->wLength == sizeof(audio20_control_cur_2_t));
        
        if (ch <= CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX) {
            volume[ch] = tu_le16toh(((audio20_control_cur_2_t const *)buf)->bCur);
            UAC_LOG("FU SET VOLUME Ch=%d val=%d dB*256\r\n", ch, volume[ch]);
        }
        
#if (CFG_TUSB_MCU == OPT_MCU_RP2040) && !defined(AUDIO_LOOPBACK)
        // Windows fix: apply volume whenever ANY channel is updated.
        // apply_volume_to_codec() derives a single codec value from all
        // channels, so both the Linux master path and the Windows
        // per-channel path produce a codec update.
        if (codec_initialized) apply_volume_to_codec();
#endif
        return true;
    }

    UAC_LOG("FU SET UNSUPPORTED: Ch=%u Sel=%u\r\n", ch, request->bControlSelector);
    return false;
}

//--------------------------------------------------------------------+
// Application Callback API
//--------------------------------------------------------------------+

bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    audio20_control_request_t const *request = (audio20_control_request_t const *)p_request;
    if (request->bEntityID == UAC2_ENTITY_CLOCK) {
        return tud_audio_clock_get_request(rhport, request);
    }
    if (request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT) {
        return tud_audio_feature_unit_get_request(rhport, request);
    }
    UAC_LOG("Audio GET unknown entity %d\r\n", request->bEntityID);
    return false;
}

bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *buf) {
    audio20_control_request_t const *request = (audio20_control_request_t const *)p_request;
    if (request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT) {
        return tud_audio_feature_unit_set_request(rhport, request, buf);
    }
    if (request->bEntityID == UAC2_ENTITY_CLOCK) {
        return tud_audio_clock_set_request(rhport, request, buf);
    }
    UAC_LOG("Audio SET unknown entity %d\r\n", request->bEntityID);
    return false;
}

bool tud_audio_set_itf_close_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    (void)rhport;
    uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
    uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

    if (ITF_NUM_AUDIO_STREAMING_SPK == itf && alt == 0) {
        streaming_spk_active = false;
        UAC_LOG("Audio streaming stopped\r\n");
        // Only stop hardware if the other direction isn't using it
        if (codec_initialized && !streaming_mic_active) {
            wm8731_stop_dma();
            blink_interval_ms = BLINK_MOUNTED;
        }
    }
    
    if (ITF_NUM_AUDIO_STREAMING_MIC == itf && alt == 0) {
        streaming_mic_active = false;
        UAC_LOG("Mic streaming stopped\r\n");
        // Only stop hardware if the other direction isn't using it
        if (codec_initialized && !streaming_spk_active) {
            wm8731_stop_dma();
            blink_interval_ms = BLINK_MOUNTED;
        }
    }
    
    return true;
}

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    (void)rhport;
    uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
    uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));
    

    if (ITF_NUM_AUDIO_STREAMING_SPK == itf) {
        if (alt != 0) {
            blink_interval_ms = BLINK_STREAMING;
            streaming_spk_active = true;
        } else {
            streaming_spk_active = false;
        }
        UAC_LOG("Speaker streaming %s\r\n", streaming_spk_active ? "ON" : "OFF");
        if (codec_initialized) wm8731_start_dma(); 
    }

    if (ITF_NUM_AUDIO_STREAMING_MIC == itf) {
        if (alt != 0) {
            blink_interval_ms = BLINK_STREAMING;
            streaming_mic_active = true;
        } else {
            streaming_mic_active = false;
        }
        UAC_LOG("Mic streaming %s\r\n", streaming_mic_active ? "ON" : "OFF");
        if (codec_initialized) wm8731_start_dma(); 
    }

    if (alt != 0) {
        current_resolution = resolutions_per_format[alt - 1];
        
    }

    return true;
}

//--------------------------------------------------------------------+
// LED Blinking Task
//--------------------------------------------------------------------+

void led_blinking_task(void) {
    static uint32_t start_ms = 0;
    static bool led_state = false;

    if (tusb_time_millis_api() - start_ms < blink_interval_ms) return;
    start_ms += blink_interval_ms;

    ws2812_led_write(led_state);
    led_state = 1 - led_state;
}
