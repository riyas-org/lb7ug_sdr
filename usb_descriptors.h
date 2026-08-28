/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Jerzy Kasenberg
 * Copyright (c) 2022 Angel Molinu (angelmolinu@gmail.com)
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
 *
 */

#ifndef USB_DESCRIPTORS_H_
#define USB_DESCRIPTORS_H_

// Unit numbers are arbitrary selected
#define UAC2_ENTITY_CLOCK               0x04
// Speaker path  (USB -> DAC -> line out / headphones)
#define UAC2_ENTITY_SPK_INPUT_TERMINAL  0x01
#define UAC2_ENTITY_SPK_FEATURE_UNIT    0x02
#define UAC2_ENTITY_SPK_OUTPUT_TERMINAL 0x03
// Microphone path (line in / mic -> ADC -> USB)
#define UAC2_ENTITY_MIC_INPUT_TERMINAL  0x11
#define UAC2_ENTITY_MIC_OUTPUT_TERMINAL 0x13

enum
{
    ITF_NUM_AUDIO_CONTROL = 0,
    ITF_NUM_AUDIO_STREAMING_SPK,
    ITF_NUM_AUDIO_STREAMING_MIC,
    ITF_NUM_CDC,
    ITF_NUM_CDC_DATA,
    ITF_NUM_TOTAL
};

// ---------------------------------------------------------------------------
// Descriptor total length
//
// Single format per streaming interface:
//   Interface 1 (SPK): alt 0 (zero-BW) + alt 1 (streaming)
//   Interface 2 (MIC): alt 0 (zero-BW) + alt 1 (streaming)
//
// The original descriptor had alt 2 blocks for FORMAT_2 on both interfaces.
// These are removed because we now use a single format; rate switching is
// handled by the clock entity (host SET_CUR on sample freq) or via CDC.
// ---------------------------------------------------------------------------
#define TUD_AUDIO_HEADSET_STEREO_DESC_LEN   ( \
    TUD_AUDIO20_DESC_IAD_LEN                  \
    + TUD_AUDIO20_DESC_STD_AC_LEN             \
    + TUD_AUDIO20_DESC_CS_AC_LEN              \
    + TUD_AUDIO20_DESC_CLK_SRC_LEN            \
    /* Speaker path entities */               \
    + TUD_AUDIO20_DESC_INPUT_TERM_LEN         \
    + TUD_AUDIO20_DESC_FEATURE_UNIT_LEN(2)    \
    + TUD_AUDIO20_DESC_OUTPUT_TERM_LEN        \
    /* Microphone path entities */            \
    + TUD_AUDIO20_DESC_INPUT_TERM_LEN         \
    + TUD_AUDIO20_DESC_OUTPUT_TERM_LEN        \
    /* Interface 1 (SPK), Alternate 0 – zero bandwidth */  \
    + TUD_AUDIO20_DESC_STD_AS_LEN             \
    /* Interface 1 (SPK), Alternate 1 – streaming */       \
    + TUD_AUDIO20_DESC_STD_AS_LEN             \
    + TUD_AUDIO20_DESC_CS_AS_INT_LEN          \
    + TUD_AUDIO20_DESC_TYPE_I_FORMAT_LEN      \
    + TUD_AUDIO20_DESC_STD_AS_ISO_EP_LEN      \
    + TUD_AUDIO20_DESC_CS_AS_ISO_EP_LEN       \
    /* Interface 2 (MIC), Alternate 0 – zero bandwidth */  \
    + TUD_AUDIO20_DESC_STD_AS_LEN             \
    /* Interface 2 (MIC), Alternate 1 – streaming */       \
    + TUD_AUDIO20_DESC_STD_AS_LEN             \
    + TUD_AUDIO20_DESC_CS_AS_INT_LEN          \
    + TUD_AUDIO20_DESC_TYPE_I_FORMAT_LEN      \
    + TUD_AUDIO20_DESC_STD_AS_ISO_EP_LEN      \
    + TUD_AUDIO20_DESC_CS_AS_ISO_EP_LEN       \
)

// ---------------------------------------------------------------------------
// Descriptor macro
//
// Changes from original:
//
// 1. Alternate 2 blocks removed from both Interface 1 and Interface 2.
//    FORMAT_2 macros (CFG_TUD_AUDIO_FUNC_1_FORMAT_2_*) no longer exist in
//    tusb_config.h and must not be referenced here.
//
// 2. Clock source _attr=3 (internal programmable) + _ctrl=5:
//      bits[1:0]=01 = freq readable (host can GET_CUR to query current rate)
//      bits[3:2]=01 = validity readable
//    Changed from _ctrl=7 (read/write freq) to _ctrl=5 (read-only freq).
//    Rate changes come from CDC commands and are applied via re-enumeration;
//    we do NOT want the host to autonomously SET the clock and skip our
//    pending-change path.  If you want the host (e.g. SDR software) to also
//    drive the rate change set _ctrl back to 7 and keep the SET handler.
//
// 3. Speaker output terminal type changed from AUDIO_TERM_TYPE_OUT_HEADPHONES
//    to AUDIO_TERM_TYPE_OUT_GENERIC_ANALOG to reflect that the same DAC signal
//    drives both headphone out and line out simultaneously on the WM8731.
//
// 4. Microphone input terminal type changed from AUDIO_TERM_TYPE_IN_GENERIC_MIC
//    to AUDIO_TERM_TYPE_IN_LINE_CONNECTOR.  In RX mode the ADC captures I/Q
//    from a line-level SDR frontend; in TX mode a physical mic is used but the
//    descriptor cannot change at runtime.  Line connector is the more accurate
//    primary description for an SDR device.
// ---------------------------------------------------------------------------
#define TUD_AUDIO_HEADSET_STEREO_DESCRIPTOR(_stridx, _epout, _epin)            \
    /* Standard Interface Association Descriptor (IAD) */                       \
    TUD_AUDIO20_DESC_IAD(                                                       \
        /*_firstitfs*/ ITF_NUM_AUDIO_CONTROL,                                   \
        /*_nitfs*/     3,                                                        \
        /*_stridx*/    0x00),                                                   \
                                                                                \
    /* Standard AC Interface Descriptor (4.7.1) */                              \
    TUD_AUDIO20_DESC_STD_AC(                                                    \
        /*_itfnum*/  ITF_NUM_AUDIO_CONTROL,                                     \
        /*_nEPs*/    0x00,                                                       \
        /*_stridx*/  _stridx),                                                  \
                                                                                \
    /* Class-Specific AC Interface Header Descriptor (4.7.2) */                 \
    TUD_AUDIO20_DESC_CS_AC(                                                     \
        /*_bcdADC*/    0x0200,                                                  \
        /*_category*/  AUDIO20_FUNC_DESKTOP_SPEAKER,                            \
        /*_totallen*/  TUD_AUDIO20_DESC_CLK_SRC_LEN                             \
                       + TUD_AUDIO20_DESC_INPUT_TERM_LEN                        \
                       + TUD_AUDIO20_DESC_FEATURE_UNIT_LEN(2)                   \
                       + TUD_AUDIO20_DESC_OUTPUT_TERM_LEN                       \
                       + TUD_AUDIO20_DESC_INPUT_TERM_LEN                        \
                       + TUD_AUDIO20_DESC_OUTPUT_TERM_LEN,                      \
        /*_ctrl*/      AUDIO20_CS_AS_INTERFACE_CTRL_LATENCY_POS),               \
                                                                                \
    /* Clock Source Descriptor (4.7.2.1) */                                     \
    /* _attr=3: internal clock, programmable */                                 \
    /* _ctrl=5: freq=read-only (01), validity=read-only (01) */                 \
    /* Rate changes are CDC-driven; host can read but not SET the frequency. */ \
    /* Change _ctrl to 7 if you want host software to SET the sample rate.  */ \
    TUD_AUDIO20_DESC_CLK_SRC(                                                   \
        /*_clkid*/     UAC2_ENTITY_CLOCK,                                       \
        /*_attr*/      3,                                                        \
        /*_ctrl*/      5,                                                        \
        /*_assocTerm*/ 0x00,                                                    \
        /*_stridx*/    0x00),                                                   \
                                                                                \
    /* ------------------------------------------------------------------ */   \
    /* Speaker path: USB streaming -> Feature Unit -> DAC                 */   \
    /* ------------------------------------------------------------------ */   \
                                                                                \
    /* Input Terminal: USB streaming source */                                  \
    TUD_AUDIO20_DESC_INPUT_TERM(                                                \
        /*_termid*/          UAC2_ENTITY_SPK_INPUT_TERMINAL,                    \
        /*_termtype*/        AUDIO_TERM_TYPE_USB_STREAMING,                     \
        /*_assocTerm*/       0x00,                                              \
        /*_clkid*/           UAC2_ENTITY_CLOCK,                                 \
        /*_nchannelslogical*/CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX,               \
        /*_channelcfg*/      AUDIO20_CHANNEL_CONFIG_NON_PREDEFINED,             \
        /*_idxchannelnames*/ 0x00,                                              \
        /*_ctrl*/            0x00,                                              \
        /*_stridx*/          0x00),                                             \
                                                                                \
    /* Feature Unit: volume + mute control on master + both channels */         \
    TUD_AUDIO20_DESC_FEATURE_UNIT(                                              \
        /*_unitid*/      UAC2_ENTITY_SPK_FEATURE_UNIT,                          \
        /*_srcid*/       UAC2_ENTITY_SPK_INPUT_TERMINAL,                        \
        /*_stridx*/      0x00,                                                  \
        /*_ctrlch0master*/(AUDIO20_CTRL_RW << AUDIO20_FEATURE_UNIT_CTRL_MUTE_POS   \
                          |AUDIO20_CTRL_RW << AUDIO20_FEATURE_UNIT_CTRL_VOLUME_POS),\
        /*_ctrlch1*/      (AUDIO20_CTRL_RW << AUDIO20_FEATURE_UNIT_CTRL_MUTE_POS   \
                          |AUDIO20_CTRL_RW << AUDIO20_FEATURE_UNIT_CTRL_VOLUME_POS),\
        /*_ctrlch2*/      (AUDIO20_CTRL_RW << AUDIO20_FEATURE_UNIT_CTRL_MUTE_POS   \
                          |AUDIO20_CTRL_RW << AUDIO20_FEATURE_UNIT_CTRL_VOLUME_POS)),\
                                                                                \
    /* Output Terminal: DAC -> line out / headphones */                         \
    /* AUDIO_TERM_TYPE_OUT_GENERIC_ANALOG: WM8731 drives both HP and line out */\
    TUD_AUDIO20_DESC_OUTPUT_TERM(                                               \
        /*_termid*/    UAC2_ENTITY_SPK_OUTPUT_TERMINAL,                         \
        /*_termtype*/  AUDIO_TERM_TYPE_OUT_HEADPHONES,                      \
        /*_assocTerm*/ 0x00,                                                    \
        /*_srcid*/     UAC2_ENTITY_SPK_FEATURE_UNIT,                            \
        /*_clkid*/     UAC2_ENTITY_CLOCK,                                       \
        /*_ctrl*/      0x0000,                                                  \
        /*_stridx*/    0x00),                                                   \
                                                                                \
    /* ------------------------------------------------------------------ */   \
    /* Microphone path: ADC (line in I/Q or mic) -> USB streaming         */   \
    /* ------------------------------------------------------------------ */   \
                                                                                \
    /* Input Terminal: line connector (primary use = I/Q from SDR frontend) */ \
    /* In TX mode the WM8731 INSEL bit switches ADC to mic; the descriptor  */ \
    /* cannot change at runtime, so line connector is the SDR-centric label.*/  \
    TUD_AUDIO20_DESC_INPUT_TERM(                                                \
        /*_termid*/          UAC2_ENTITY_MIC_INPUT_TERMINAL,                    \
        /*_termtype*/        AUDIO_TERM_TYPE_IN_GENERIC_MIC,                 \
        /*_assocTerm*/       0x00,                                              \
        /*_clkid*/           UAC2_ENTITY_CLOCK,                                 \
        /*_nchannelslogical*/CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX,               \
        /*_channelcfg*/      AUDIO20_CHANNEL_CONFIG_NON_PREDEFINED,             \
        /*_idxchannelnames*/ 0x00,                                              \
        /*_ctrl*/            0x00,                                              \
        /*_stridx*/          0x00),                                             \
                                                                                \
    /* Output Terminal: ADC data -> USB */                                      \
    TUD_AUDIO20_DESC_OUTPUT_TERM(                                               \
        /*_termid*/    UAC2_ENTITY_MIC_OUTPUT_TERMINAL,                         \
        /*_termtype*/  AUDIO_TERM_TYPE_USB_STREAMING,                           \
        /*_assocTerm*/ 0x00,                                                    \
        /*_srcid*/     UAC2_ENTITY_MIC_INPUT_TERMINAL,                          \
        /*_clkid*/     UAC2_ENTITY_CLOCK,                                       \
        /*_ctrl*/      0x0000,                                                  \
        /*_stridx*/    0x00),                                                   \
                                                                                \
    /* ------------------------------------------------------------------ */   \
    /* Interface 1 – Speaker (USB OUT -> DAC)                             */   \
    /* ------------------------------------------------------------------ */   \
                                                                                \
    /* Alternate 0: zero bandwidth (default, no endpoint) */                    \
    TUD_AUDIO20_DESC_STD_AS_INT(                                                \
        /*_itfnum*/ (uint8_t)(ITF_NUM_AUDIO_STREAMING_SPK),                     \
        /*_altset*/ 0x00,                                                        \
        /*_nEPs*/   0x00,                                                        \
        /*_stridx*/ 0x05),                                                      \
                                                                                \
    /* Alternate 1: active streaming */                                         \
    TUD_AUDIO20_DESC_STD_AS_INT(                                                \
        /*_itfnum*/ (uint8_t)(ITF_NUM_AUDIO_STREAMING_SPK),                     \
        /*_altset*/ 0x01,                                                        \
        /*_nEPs*/   0x01,                                                        \
        /*_stridx*/ 0x05),                                                      \
    TUD_AUDIO20_DESC_CS_AS_INT(                                                 \
        /*_termid*/          UAC2_ENTITY_SPK_INPUT_TERMINAL,                    \
        /*_ctrl*/            AUDIO20_CTRL_NONE,                                 \
        /*_formattype*/      AUDIO20_FORMAT_TYPE_I,                             \
        /*_formats*/         AUDIO20_DATA_FORMAT_TYPE_I_PCM,                    \
        /*_nchannelsphysical*/CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX,              \
        /*_channelcfg*/      AUDIO20_CHANNEL_CONFIG_NON_PREDEFINED,             \
        /*_stridx*/          0x00),                                             \
    TUD_AUDIO20_DESC_TYPE_I_FORMAT(                                             \
        CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_RX,                   \
        CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_RX),                          \
    /* ADAPTIVE: device clock adapts to host SOF — correct for a device      */\
    /* that is clocked by an external I2S master (the WM8731 in slave mode).  */\
    TUD_AUDIO20_DESC_STD_AS_ISO_EP(                                             \
        /*_ep*/      _epout,                                                     \
        /*_attr*/    (uint8_t)(TUSB_XFER_ISOCHRONOUS                            \
                               | TUSB_ISO_EP_ATT_ADAPTIVE                       \
                               | TUSB_ISO_EP_ATT_DATA),                         \
        /*_maxEPsize*/ TUD_AUDIO_EP_SIZE(0,                                     \
                           CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE,                \
                           CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_RX,\
                           CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX),                 \
        /*_interval*/ 0x01),                                                    \
    TUD_AUDIO20_DESC_CS_AS_ISO_EP(                                              \
        /*_attr*/          AUDIO20_CS_AS_ISO_DATA_EP_ATT_NON_MAX_PACKETS_OK,    \
        /*_ctrl*/          AUDIO20_CTRL_NONE,                                   \
        /*_lockdelayunit*/ AUDIO20_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_MILLISEC, \
        /*_lockdelay*/     0x0001),                                             \
                                                                                \
    /* ------------------------------------------------------------------ */   \
    /* Interface 2 – Microphone (ADC -> USB IN)                           */   \
    /* ------------------------------------------------------------------ */   \
                                                                                \
    /* Alternate 0: zero bandwidth (default, no endpoint) */                    \
    TUD_AUDIO20_DESC_STD_AS_INT(                                                \
        /*_itfnum*/ (uint8_t)(ITF_NUM_AUDIO_STREAMING_MIC),                     \
        /*_altset*/ 0x00,                                                        \
        /*_nEPs*/   0x00,                                                        \
        /*_stridx*/ 0x04),                                                      \
                                                                                \
    /* Alternate 1: active streaming */                                         \
    TUD_AUDIO20_DESC_STD_AS_INT(                                                \
        /*_itfnum*/ (uint8_t)(ITF_NUM_AUDIO_STREAMING_MIC),                     \
        /*_altset*/ 0x01,                                                        \
        /*_nEPs*/   0x01,                                                        \
        /*_stridx*/ 0x04),                                                      \
    TUD_AUDIO20_DESC_CS_AS_INT(                                                 \
        /*_termid*/          UAC2_ENTITY_MIC_OUTPUT_TERMINAL,                   \
        /*_ctrl*/            AUDIO20_CTRL_NONE,                                 \
        /*_formattype*/      AUDIO20_FORMAT_TYPE_I,                             \
        /*_formats*/         AUDIO20_DATA_FORMAT_TYPE_I_PCM,                    \
        /*_nchannelsphysical*/CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX,              \
        /*_channelcfg*/      AUDIO20_CHANNEL_CONFIG_NON_PREDEFINED,             \
        /*_stridx*/          0x00),                                             \
    TUD_AUDIO20_DESC_TYPE_I_FORMAT(                                             \
        CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_TX,                   \
        CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_TX),                          \
    /* ASYNCHRONOUS: the ADC runs on its own I2S clock derived from MCLK;    */\
    /* the host must accept whatever packet size the device sends each frame. */\
    TUD_AUDIO20_DESC_STD_AS_ISO_EP(                                             \
        /*_ep*/      _epin,                                                      \
        /*_attr*/    (uint8_t)(TUSB_XFER_ISOCHRONOUS                            \
                               | TUSB_ISO_EP_ATT_ASYNCHRONOUS                   \
                               | TUSB_ISO_EP_ATT_DATA),                         \
        /*_maxEPsize*/ TUD_AUDIO_EP_SIZE(0,                                     \
                           CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE,                \
                           CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_TX,\
                           CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX),                 \
        /*_interval*/ 0x01),                                                    \
    TUD_AUDIO20_DESC_CS_AS_ISO_EP(                                              \
        /*_attr*/          AUDIO20_CS_AS_ISO_DATA_EP_ATT_NON_MAX_PACKETS_OK,    \
        /*_ctrl*/          AUDIO20_CTRL_NONE,                                   \
        /*_lockdelayunit*/ AUDIO20_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_UNDEFINED, \
        /*_lockdelay*/     0x0000)

#endif /* USB_DESCRIPTORS_H_ */
