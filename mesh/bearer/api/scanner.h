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
#ifndef SCANNER_H__
#define SCANNER_H__

#include <stdint.h>
#include <stdbool.h>
#include "nrf_mesh.h"
#include "radio_config.h"
#include "packet.h"

/**
 * @defgroup SCANNER Scanner
 * @ingroup MESH_API_GROUP_BEARER
 * @{
 */

#define SCANNER_CHANNELS_MAX           (3)
#define SCANNER_ACCESS_ADDR_INVALID    (0x00000000)
#define SCANNER_CHANNELS_DEFAULT       {37, 38, 39}

/** Content of a packet received by the scanner. */
typedef struct
{
    nrf_mesh_rx_metadata_scanner_t metadata; /**< Packet metadata. */
    packet_t packet;                         /**< Packet content. */
} scanner_packet_t;

/** Scanner statistics structure. Recorded since last reset. */
typedef struct
{
    uint32_t successful_receives;           /**< Number of received packets. */
    uint32_t crc_failures;                  /**< Number of CRC failures. */
    uint32_t length_out_of_bounds;          /**< Number of packets with length out of bounds. */
} scanner_stats_t;

/**
 * Initializes the scanner module.
 */
void scanner_init(void);

/**
 * Enables the scanner module.
 */
void scanner_enable(void);

/**
 * Disables the scanner module.
 */
void scanner_disable(void);

/**
 * Checks if the scanner module is enabled.
 *
 * @return         true if enabled, false if disabled.
 */
bool scanner_is_enabled(void);

/**
 * Returns the next packet that has been received by the scanner.
 *
 * @note The returned packet must be released using scanner_packet_release().
 *
 * @return         Pointer to received packet, or NULL if no packet has been received.
 */
const scanner_packet_t * scanner_rx(void);

/**
 * Releases a packet that has previously been returned by scanner_rx().
 *
 * @param[in]      p_packet  Packet to be released.
 */
void scanner_packet_release(const scanner_packet_t * p_packet);

/**
 * Returns statistics related to the scanner module.
 *
 * @return         Pointer to statistics structure.
 */
const scanner_stats_t * scanner_stats_get(void);

/**
 * Sets scanner radio mode (data rate and modulation).
 *
 * @param[in]      radio_mode  New radio mode.
 */
void scanner_config_radio_mode_set(radio_mode_t radio_mode);

/**
 * Sets scanner timing parameters.
 *
 * @param[in]      scan_interval_us  Scan interval duration.
 * @param[in]      scan_window_us    Scan window duration.
 */
void scanner_config_scan_time_set(uint32_t scan_interval_us, uint32_t scan_window_us);

/**
 * Sets which radio channels are to be used by the scanner.
 *
 * @param[in]      p_channels     Array of radio channels.
 * @param[in]      channel_count  Number of radio channels.
 */
void scanner_config_channels_set(const uint8_t * p_channels, uint8_t channel_count);

/**
 * Sets access addresses to be used by the scanner, and corresponding logical address.
 *
 * @note Set address to SCANNER_ACCESS_ADDR_INVALID if corresponding logical address is not to be
 *       used.
 *
 * @param[in]      p_access_addresses  Array of access addresses. Address index defines
 *                                     corresponding logical address.
 * @param[in]      address_count       Number of access addresses.
 */
void scanner_config_access_addresses_set(const uint32_t * p_access_addresses, uint8_t address_count);

/**
 * Resets the scanner module.
 */
void scanner_config_reset(void);

/**
 * @defgroup SCANNER_INTERNAL Internal scanner functions
 * Should not be called by the application.
 * @{
 * @internal
 */

/**
 * Start scanner radio.
 *
 * @warning Only to be used by the bearer module.
 */
void scanner_radio_start(void);

/**
 * Stop scanner radio.
 *
 * @warning Only to be used by the bearer module.
 */
void scanner_radio_stop(void);

/**
 * Scanner radio IRQ handler.
 *
 * @warning Only to be used by the bearer module.
 */
void scanner_radio_irq_handler(void);

/** @} */

/** @} */

#endif /* SCANNER_H__ */
