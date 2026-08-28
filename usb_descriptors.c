/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Ha Thach (tinyusb.org)
 * Copyright (c) 2020 Jerzy Kasenberg
 * Copyright (c) 2022 Angel Molinu 
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
 *
 */

#include "bsp/board_api.h"
#include "tusb.h"
#include "usb_descriptors.h"

/* A combination of interfaces must have a unique product id, since the PC
 * saves the device driver after the first plug.  Same VID/PID with different
 * interfaces can cause errors on Windows.
 *
 * Auto ProductID layout bitmap:
 *   [MSB]  AUDIO | MIDI | HID | MSC | CDC  [LSB]
 */
#define PID_MAP(itf, n)  ((CFG_TUD_##itf) ? (1 << (n)) : 0)
#define USB_PID          (0x4000 | PID_MAP(CDC, 0) | PID_MAP(MSC, 1) \
                         | PID_MAP(HID, 2) | PID_MAP(MIDI, 3)         \
                         | PID_MAP(AUDIO, 4) | PID_MAP(VENDOR, 5))

//--------------------------------------------------------------------+
// Device Descriptor
//--------------------------------------------------------------------+
static tusb_desc_device_t const desc_device =
{
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,

    // IAD requires class=Misc, subclass=Common, protocol=IAD
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = 0xCafe,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
};

uint8_t const *tud_descriptor_device_cb(void)
{
    return (uint8_t const *)&desc_device;
}

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+

// CONFIG_TOTAL_LEN is driven entirely by the TUD_AUDIO_HEADSET_STEREO_DESC_LEN
// macro in usb_descriptors.h, which now reflects a single-format layout.
// No manual adjustment needed here — just keep this formula.
#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN \
                            + CFG_TUD_AUDIO * TUD_AUDIO_HEADSET_STEREO_DESC_LEN \
                            + CFG_TUD_CDC   * TUD_CDC_DESC_LEN)

// Endpoint assignments
// RP2040 supports shared IN/OUT endpoint numbers so AUDIO_IN == AUDIO_OUT == 0x01.
#if CFG_TUSB_MCU == OPT_MCU_LPC175X_6X || CFG_TUSB_MCU == OPT_MCU_LPC177X_8X \
    || CFG_TUSB_MCU == OPT_MCU_LPC40XX
  #define EPNUM_AUDIO_IN    0x03
  #define EPNUM_AUDIO_OUT   0x03
  #define EPNUM_CDC_NOTIF   0x84
  #define EPNUM_CDC_OUT     0x05
  #define EPNUM_CDC_IN      0x85
#elif CFG_TUSB_MCU == OPT_MCU_NRF5X
  #define EPNUM_AUDIO_IN    0x08
  #define EPNUM_AUDIO_OUT   0x08
  #define EPNUM_CDC_NOTIF   0x81
  #define EPNUM_CDC_OUT     0x02
  #define EPNUM_CDC_IN      0x82
#elif defined(TUD_ENDPOINT_ONE_DIRECTION_ONLY)
  #define EPNUM_AUDIO_IN    0x01
  #define EPNUM_AUDIO_OUT   0x02
  #define EPNUM_CDC_NOTIF   0x83
  #define EPNUM_CDC_OUT     0x04
  #define EPNUM_CDC_IN      0x85
#else
  // RP2040 default — EP1 shared for audio IN/OUT
  #define EPNUM_AUDIO_IN    0x01
  #define EPNUM_AUDIO_OUT   0x01
  #define EPNUM_CDC_NOTIF   0x83
  #define EPNUM_CDC_OUT     0x04
  #define EPNUM_CDC_IN      0x84
#endif

static uint8_t const desc_fs_configuration[] =
{
    // Config number, interface count, string index, total length,
    // attributes (bus powered), power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),

    // Audio interfaces + CDC
    TUD_AUDIO_HEADSET_STEREO_DESCRIPTOR(2, EPNUM_AUDIO_OUT, EPNUM_AUDIO_IN | 0x80),
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 6, EPNUM_CDC_NOTIF, 8,
                       EPNUM_CDC_OUT, EPNUM_CDC_IN, 64)
};

#if TUD_OPT_HIGH_SPEED
static uint8_t const desc_hs_configuration[] =
{
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),
    TUD_AUDIO_HEADSET_STEREO_DESCRIPTOR(2, EPNUM_AUDIO_OUT, EPNUM_AUDIO_IN | 0x80),
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 6, EPNUM_CDC_NOTIF, 8,
                       EPNUM_CDC_OUT, EPNUM_CDC_IN, 512)
};

static uint8_t desc_other_speed_config[CONFIG_TOTAL_LEN];

static tusb_desc_device_qualifier_t const desc_device_qualifier =
{
    .bLength            = sizeof(tusb_desc_device_qualifier_t),
    .bDescriptorType    = TUSB_DESC_DEVICE_QUALIFIER,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .bNumConfigurations = 0x01,
    .bReserved          = 0x00
};

uint8_t const *tud_descriptor_device_qualifier_cb(void)
{
    return (uint8_t const *)&desc_device_qualifier;
}

uint8_t const *tud_descriptor_other_speed_configuration_cb(uint8_t index)
{
    (void)index;
    memcpy(desc_other_speed_config,
           (tud_speed_get() == TUSB_SPEED_HIGH) ? desc_fs_configuration
                                                 : desc_hs_configuration,
           CONFIG_TOTAL_LEN);
    desc_other_speed_config[1] = TUSB_DESC_OTHER_SPEED_CONFIG;
    return desc_other_speed_config;
}
#endif

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
#if TUD_OPT_HIGH_SPEED
    return (tud_speed_get() == TUSB_SPEED_HIGH) ? desc_hs_configuration
                                                 : desc_fs_configuration;
#else
    return desc_fs_configuration;
#endif
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
};

// String index assignments (must match iManufacturer/iProduct/iSerial above
// and the _stridx arguments in the descriptor macro):
//   0 = language ID
//   1 = manufacturer
//   2 = product
//   3 = serial (auto-generated from chip ID)
//   4 = mic streaming interface  (Interface 2 _stridx = 0x04)
//   5 = speaker streaming interface (Interface 1 _stridx = 0x05)
//   6 = CDC interface
static char const *string_desc_arr[] =
{
    (const char[]){ 0x09, 0x04 },   // 0: English (0x0409)
    "RP2040 SDR",                   // 1: Manufacturer
    "RP2040 SDR Audio",             // 2: Product
    NULL,                           // 3: Serial — filled from chip unique ID
    "I/Q Capture (RX) / Mic (TX)", // 4: Mic streaming interface
    "I/Q Playback (TX) / Audio Out",// 5: Speaker streaming interface
    "SDR Control",                  // 6: CDC control interface
};

static uint16_t _desc_str[32 + 1];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;
    size_t chr_count;

    switch (index)
    {
        case STRID_LANGID:
            memcpy(&_desc_str[1], string_desc_arr[0], 2);
            chr_count = 1;
            break;

        case STRID_SERIAL:
            chr_count = board_usb_get_serial(_desc_str + 1, 32);
            break;

        default:
            if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))
                return NULL;

            const char *str = string_desc_arr[index];
            if (!str) return NULL;

            chr_count = strlen(str);
            size_t const max_count = sizeof(_desc_str) / sizeof(_desc_str[0]) - 1;
            if (chr_count > max_count) chr_count = max_count;

            for (size_t i = 0; i < chr_count; i++)
                _desc_str[1 + i] = (uint16_t)str[i];
            break;
    }

    // Header: length (including header word) + string descriptor type
    _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return _desc_str;
}
