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

#ifndef CDC_APP_H_
#define CDC_APP_H_

#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * printf-compatible function that queues output into a ring buffer and
 * drains it to the TinyUSB CDC endpoint whenever the host is connected.
 * Used as CFG_TUSB_DEBUG_PRINTF so all TU_LOG* output appears on the
 * CDC serial port.
 *
 * Safe to call before the USB stack is ready — data is held in the ring
 * buffer and sent once the host opens the port.
 */
int cdc_printf(const char *fmt, ...);

/**
 * Must be called from the main loop (alongside tud_task).
 * Drains any pending bytes from the ring buffer to the CDC endpoint.
 */

extern uint32_t sample_rates[];     
extern uint32_t current_sample_rate; 
static volatile bool sample_rate_change_pending = false;
static volatile uint32_t pending_sample_rate = 0; 
void cdc_task(void);
void audio_apply_pending_rate_change(void); 

#ifdef __cplusplus
}
#endif

#endif /* CDC_APP_H_ */
