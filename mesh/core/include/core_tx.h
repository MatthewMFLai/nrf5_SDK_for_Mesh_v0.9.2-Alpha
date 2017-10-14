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
#ifndef CORE_TX_H__
#define CORE_TX_H__

#include <stdint.h>
#include <stdbool.h>
#include "nrf_mesh.h"

/**
 * @defgroup CORE_TX Core Transmission Handler
 * Handler for transmission of core mesh packets.
 * @{
 */

/** Available roles each packet can be transmitted for. */
typedef enum
{
    CORE_TX_ROLE_ORIGINATOR, /**< The packet originated in this device. */
    CORE_TX_ROLE_RELAY, /**< The packet came from a different device and is being relayed. */

    CORE_TX_ROLE_COUNT /**< Number of roles available, not a valid role itself. */
} core_tx_role_t;

typedef enum
{
    CORE_TX_BEARER_ADV = (1 << 0),
    //CORE_TX_BEARER_GATT = (1 << 1),
    //CORE_TX_BEARER_FRIENDSHIP = (1 << 2)
} core_tx_bearer_t;

typedef struct
{
    core_tx_bearer_t bearer;
    core_tx_role_t role;
} core_tx_metadata_t;


/**
 * Transmission complete callback type used to notify the user of ended transmissions.
 *
 * @warning The transmission complete callback executes in an interrupt, and should do as little as
 * possible before returning.
 *
 * @param[in] p_metadata Metadata for the transmission.
 * @param[in] timestamp Timestamp of the packet transmission in microseconds.
 * @param[in] token Token set in the packet alloc call.
 */
typedef void (*core_tx_complete_cb_t)(const core_tx_metadata_t * p_metadata, uint32_t timestamp, nrf_mesh_tx_token_t token);

/**
 * Initialize the core tx module.
 */
void core_tx_init(void);

/**
 * Set the core tx complete callback.
 *
 * @param[in] tx_complete_callback Callback to call at the end of a successful transmission, or NULL
 * if no callback is needed.
 */
void core_tx_complete_cb_set(core_tx_complete_cb_t tx_complete_callback);

/**
 * Allocate a network packet for transmission.
 *
 * @param[in] net_packet_len Desired network packet length, including header and MIC.
 * @param[in] p_metadata Metadata for the packet.
 * @param[in,out] pp_packet Packet pointer to populate.
 * @param[in] token User token that can be used to identify the packet in the TX complete callback.
 *
 * @returns A bitmask of the bearers that were able to allocate data.
 */
core_tx_bearer_t core_tx_packet_alloc(uint32_t net_packet_len,
                                      const core_tx_metadata_t * p_metadata,
                                      uint8_t ** pp_packet,
                                      nrf_mesh_tx_token_t token);

/**
 * Send a network packet acquired through the @ref core_tx_packet_alloc() function.
 *
 * @param[in] p_metadata Metadata used when allocating the packet.
 * @param[in,out] p_packet Packet pointer to send.
 */
void core_tx_packet_send(const core_tx_metadata_t * p_metadata, uint8_t * p_packet);

/**
 * Discard a packet buffer acquired through the @ref core_tx_packet_alloc() function. The packet
 * memory will be made available for reallocation, and its contents discarded.
 *
 * @param[in] p_metadata Metadata used when allocating the packet.
 * @param[in,out] p_packet Packet pointer to send.
 */
void core_tx_packet_discard(const core_tx_metadata_t * p_metadata, uint8_t * p_packet);

/**
 * Set the number of transmissions to do for a single packet, when transmitted in the given role.
 *
 * @param[in] role Role to configure the TX count for.
 * @param[in] tx_count The number of transmissions to do per packet. Must be larger than 0. Setting
 * this to @ref ADVERTISER_REPEAT_INFINITE causes the packet to be transmitted until a new packet
 * has been added to the queue.
 */
void core_tx_adv_count_set(core_tx_role_t role, uint8_t tx_count);

/**
 * Get the number of transmissions that will be done per packet when transmitted in the given role.
 *
 * @param[in] role Role to get the configuration for.
 *
 * @returns The configured number of transmissions per packet for the given role.
 */
uint8_t core_tx_adv_count_get(core_tx_role_t role);

/**
 * Set the advertisement interval for the specific role.
 *
 * @param[in] role Role to set the advertisement interval for.
 * @param[in] interval_ms Advertisement interval in milliseconds.
 */
void core_tx_adv_interval_set(core_tx_role_t role, uint32_t interval_ms);

/**
 * Get the advertisement interval in milliseconds for the given role.
 *
 * @param[in] role Role to get the advertisement interval for.
 *
 * @returns The advertisement interval for the given role in milliseconds.
 */
uint32_t core_tx_adv_interval_get(core_tx_role_t role);

/**
 * Set the advertisement address for the specific role.
 *
 * @param[in] role Role to set the advertisement address for.
 * @param[in] p_addr New GAP advertisement address.
 */
void core_tx_adv_address_set(core_tx_role_t role, const ble_gap_addr_t * p_addr);

/** @} */

#endif /* CORE_TX_H__ */
