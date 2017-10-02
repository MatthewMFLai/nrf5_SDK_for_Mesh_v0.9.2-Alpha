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
#include <string.h>
#include <stdbool.h>
#include <stddef.h>

#include "bearer.h"
#include "bearer_adv.h"
#include "internal_event.h"
#include "bearer_event.h"
#include "fifo.h"
#include "packet_mgr.h"
#include "nrf_mesh_assert.h"
#include "log.h"
#include "nrf_error.h"

#if (BEARER_TYPES & BEARER_ADV_RADIO)
#include "radio.h"
#endif

/* Constants used for AD type filtering */
#define FILTER_SIZE (256/32) /* AD type is 8 bits , so we need 256 bits to encode all possible values as a bit field, we will store them as 32 bit numbers (words), thus we need 256/32 words */
#define ADTYPE_FILTER_INDEX_BITMASK 0xE0 /* Used for determining the index of the word in m_adtype_filter array when looking for a value's bitfield */
#define ADTYPE_FILTER_INDEX_BITMASK_LOCATION 5 /* Number of bits to represent a word (32: 1<<5) */
#define ADTYPE_FILTER_VALUE_MASK 31 /* Number of bits in each word */
#define ADTYPE_FILTER_INDEX(type) (((type) & ADTYPE_FILTER_INDEX_BITMASK) >> ADTYPE_FILTER_INDEX_BITMASK_LOCATION)
#define ADTYPE_FILTER_VALUE(type) (1UL << ((type) & ADTYPE_FILTER_VALUE_MASK))

static uint32_t m_adtype_filter[FILTER_SIZE] = {0};
static bool m_adtype_filtering = false;

bool m_adtype_valid(uint8_t type)
{
    bool adtype_valid = false;
    if (m_adtype_filtering)
    {
        uint8_t filter_index = ADTYPE_FILTER_INDEX(type);
        uint32_t filter_value = ADTYPE_FILTER_VALUE(type);

        if ( filter_value & m_adtype_filter[filter_index] )
        {
            adtype_valid = true;
        }
    }
    else
    {
        adtype_valid = true;
    }
    return adtype_valid;
}

void bearer_adtype_filtering_set(bool filter)
{
    m_adtype_filtering = filter;
}

void bearer_adtype_add(uint8_t type)
{
    uint8_t filter_index = ADTYPE_FILTER_INDEX(type);
    uint32_t filter_value = ADTYPE_FILTER_VALUE(type);
    m_adtype_filter[filter_index] |= filter_value;
}

void bearer_adtype_remove(uint8_t type)
{
    uint8_t filter_index = ADTYPE_FILTER_INDEX(type);
    uint32_t filter_value = ADTYPE_FILTER_VALUE(type);
    m_adtype_filter[filter_index] &= ~filter_value;
}
