/* Copyright (c) 2010 - 2017, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef DEBUG_PINS_H__
#define DEBUG_PINS_H__

#include <stdint.h>

/* Define debug pins that fit the standard development kits for the two chips. Placed to avoid
 * overlapping functionality. */
#ifdef NRF51
#define DEBUG_PIN0 12
#define DEBUG_PIN1 13
#define DEBUG_PIN2 14
#define DEBUG_PIN3 15
#define DEBUG_PIN4 16
#define DEBUG_PIN5 17
#define DEBUG_PIN6 18
#define DEBUG_PIN7 19
#elif defined(NRF52_SERIES) || defined(HOST)
#define DEBUG_PIN0 22
#define DEBUG_PIN1 23
#define DEBUG_PIN2 24
#define DEBUG_PIN3 25
#define DEBUG_PIN4 26
#define DEBUG_PIN5 27
#define DEBUG_PIN6 28
#define DEBUG_PIN7 29
#endif

#ifdef DEBUG_PINS_ENABLED

#define DEBUG_PIN_ON(p)               NRF_GPIO->OUTSET = (1UL << (p))
#define DEBUG_PIN_OFF(p)              NRF_GPIO->OUTCLR = (1UL << (p))
#define DEBUG_PIN_TOGGLE(p)           do{DEBUG_PIN_ON(p);__NOP();DEBUG_PIN_OFF(p);}while(0)
#define DEBUG_PIN_TOGGLE_NTIMES(p, n) do{uint8_t CTR = (n);do{DEBUG_PIN_TOGGLE(p);}while(CTR--);}while(0)

/* The inline function auto-generator. This macro should be called for each user of the debug_pins. */
#define DEBUG_PIN_MODULE(MODULE, ENABLED) \
    static inline __attribute__((unused)) void DEBUG_PIN_##MODULE##_ON(uint32_t pin)  { if (ENABLED) { DEBUG_PIN_ON(pin); } } \
    static inline __attribute__((unused)) void DEBUG_PIN_##MODULE##_OFF(uint32_t pin) { if (ENABLED) { DEBUG_PIN_OFF(pin); } } \
    static inline __attribute__((unused)) void DEBUG_PIN_##MODULE##_TOGGLE(uint32_t pin) { if (ENABLED) { DEBUG_PIN_TOGGLE(pin); } } \
    static inline __attribute__((unused)) void DEBUG_PIN_##MODULE##_TOGGLE_NTIMES(uint32_t pin, uint32_t n) { if (ENABLED) { DEBUG_PIN_TOGGLE_NTIMES(pin, n); } }

#else

#define DEBUG_PIN_ON(p)
#define DEBUG_PIN_OFF(p)
#define DEBUG_PIN_TOGGLE(p)
#define DEBUG_PIN_TOGGLE_NTIMES(p, n)

#define DEBUG_PIN_MODULE(MODULE, ENABLED) \
    static inline __attribute__((unused)) void DEBUG_PIN_##MODULE##_ON(uint32_t pin) {} \
    static inline __attribute__((unused)) void DEBUG_PIN_##MODULE##_OFF(uint32_t pin) {} \
    static inline __attribute__((unused)) void DEBUG_PIN_##MODULE##_TOGGLE(uint32_t pin) {} \
    static inline __attribute__((unused)) void DEBUG_PIN_##MODULE##_TOGGLE_NTIMES(uint32_t pin, uint32_t n) {}

#endif //DEBUG_PINS_ENABLED

DEBUG_PIN_MODULE(TIMESLOT, false)
#define DEBUG_PIN_TS_SIGNAL_CALLBACK         DEBUG_PIN0
#define DEBUG_PIN_TS_IN_TIMESLOT             DEBUG_PIN1
#define DEBUG_PIN_TS_EXTEND_HANDLER          DEBUG_PIN2
#define DEBUG_PIN_TS_TIMER_HANDLER           DEBUG_PIN3
#define DEBUG_PIN_TS_SD_EVT_HANDLER          DEBUG_PIN4
#define DEBUG_PIN_TS_END_TIMER_HANDLER       DEBUG_PIN5
#define DEBUG_PIN_TS_TIMESLOT_OPERATION      DEBUG_PIN6
#define DEBUG_PIN_TS_EXTEND_SUCCEEDED        DEBUG_PIN7

DEBUG_PIN_MODULE(BEARER_HANDLER, false)
#define DEBUG_PIN_BEARER_HANDLER_ACTION         DEBUG_PIN0
#define DEBUG_PIN_BEARER_HANDLER_SCANNER        DEBUG_PIN1
#define DEBUG_PIN_BEARER_HANDLER_SCANNER_STOP   DEBUG_PIN2

DEBUG_PIN_MODULE(SCANNER, false)
#define DEBUG_PIN_SCANNER_RADIO_IN_RX       DEBUG_PIN0
//#define DEBUG_PIN_SCANNER_                  DEBUG_PIN1
//#define DEBUG_PIN_SCANNER_                  DEBUG_PIN2
#define DEBUG_PIN_SCANNER_END_EVENT         DEBUG_PIN3
#define DEBUG_PIN_SCANNER_IN_ACTION         DEBUG_PIN4
#define DEBUG_PIN_SCANNER_START             DEBUG_PIN5
#define DEBUG_PIN_SCANNER_STOP              DEBUG_PIN6
#define DEBUG_PIN_SCANNER_RADIO_IRQ         DEBUG_PIN7

DEBUG_PIN_MODULE(BROADCAST, true)
#define DEBUG_PIN_BROADCAST_START           DEBUG_PIN0
#define DEBUG_PIN_BROADCAST_RADIO_EVT       DEBUG_PIN1
#define DEBUG_PIN_BROADCAST_RADIO_EVT_END   DEBUG_PIN2
#define DEBUG_PIN_BROADCAST_RADIO_EVT_READY DEBUG_PIN3
#define DEBUG_PIN_BROADCAST_ADV_CB          DEBUG_PIN4
#define DEBUG_PIN_BROADCAST_ACTIVE          DEBUG_PIN5

#endif //DEBUG_PINS_H__
