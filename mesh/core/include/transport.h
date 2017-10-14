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
#ifndef MESH_TRANSPORT_H__
#define MESH_TRANSPORT_H__

#include <stdint.h>
#include <stdbool.h>

#include "packet.h"
#include "timer.h"
#include "utils.h"

#include "nrf_mesh.h"
#include "nrf_mesh_opt.h"
#include "nrf_mesh_defines.h"
#include "network.h"
#include "packet_mesh.h"
#include "utils.h"

#include "nrf_mesh_config_core.h"

/**
 * @defgroup TRANSPORT Transport Layer
 * @ingroup MESH_CORE
 * @{
 */

/** Default TX timeout. */
#define TRANSPORT_SAR_RX_TIMEOUT_DEFAULT_US SEC_TO_US(10)

/** Default base RX acknowledgement timeout. */
#define TRANSPORT_SAR_RX_ACK_BASE_TIMEOUT_DEFAULT_US MS_TO_US(150)

/** Default per hop RX acknowledgement timeout addition. */
#define TRANSPORT_SAR_RX_ACK_PER_HOP_ADDITION_DEFAULT_US MS_TO_US(50)

/** Default base TX retry timeout. */
#define TRANSPORT_SAR_TX_RETRY_BASE_TIMEOUT_DEFAULT_US MS_TO_US(500)

/** Default per hop TX retry timeout addition. */
#define TRANSPORT_SAR_TX_RETRY_PER_HOP_ADDITION_DEFAULT_US MS_TO_US(50)

/** Default number of retries before cancelling SAR TX session. */
#define TRANSPORT_SAR_TX_RETRIES_DEFAULT (4)


/**
 * Allocation function type for transport SAR buffer allocation. Matches
 * stdlib's malloc.
 *
 * @param[in] size Number of bytes to allocate.
 *
 * @returns A pointer to a valid buffer of at least size @p size, or NULL if
 * the allocation failed.
 */
typedef void* (*transport_sar_alloc_t)(size_t size);

/**
 * Release function type for transport SAR buffer deallocation. Matches
 * stdlib's free.
 *
 * @param[in] ptr A pointer to a previously allocated buffer, that is to be released.
 */
typedef void (*transport_sar_release_t)(void* ptr);

/**
 * Initializes the transport layer.
 *
 * @param[in] p_init_params  Generic initialization parameters pointer.
 */
void transport_init(const nrf_mesh_init_params_t * p_init_params);

/**
 * Set the SAR buffer allocation and release functions. Defaults to stdlib's
 * malloc and free. The transport layer has to allocate a temporary buffer
 * for transport packets that span multiple network packets, in order to put
 * them together (RX) or split them (TX). The transport module takes no
 * precautions to prevent overlapping memory regions for different buffers,
 * although this will cause undefined behavior.
 *
 * @note If both parameters are NULL, the function behaves like @ref
 * transport_sar_mem_funcs_reset. If only one of them is NULL, The function
 * does nothing and returns @c NRF_ERROR_NULL.
 *
 * @param[in] alloc_func Function pointer to the wanted allocation function.
 * @param[in] release_func Function pointer to the wanted release function.
 *
 * @retval NRF_SUCCESS The allocation and release functions were successfully
 * set.
 * @retval NRF_ERROR_NULL One of the given function pointers were NULL, but not both.
 */
uint32_t transport_sar_mem_funcs_set(transport_sar_alloc_t alloc_func, transport_sar_release_t release_func);

/**
 * Reset the SAR buffer allocation and release functions to malloc and free.
 */
void transport_sar_mem_funcs_reset(void);
/**
 * Function for passing packets from the network layer to the transport layer.
 *
 * @param[in] p_packet Pointer to the transport packet.
 * @param[in] trs_packet_len Length of the transport packet.
 * @param[in] p_net_metadata Pre-filled network metadata structure. Note that the transport layer
 * expects the destination address to have its value field set, and will fill in the rest of the
 * address data itself.
 * @param[in] p_rx_metadata RX metadata tied to the packet.
 *
 * @retval NRF_ERROR_INVALID_ADDR The destination address is not valid.
 * @retval NRF_ERROR_NOT_FOUND    The packet could not be decryptet with any application key.
 * @retval NRF_ERROR_NULL         One or more of the input parameters was NULL.
 * @retval NRF_SUCCESS            The packet was successfully decrypted and sent up the stack.
 */
uint32_t transport_packet_in(const packet_mesh_trs_packet_t * p_packet,
                             uint32_t trs_packet_len,
                             const network_packet_metadata_t * p_net_metadata,
                             const nrf_mesh_rx_metadata_t * p_rx_metadata);

/**
 * Convert a transport address context structure to a 16-bit short address.
 *
 * @param[in]  p_addr          Transport address context pointer.
 * @param[out] p_short_addr    16-bit little-endian short address.
 *
 * @retval NRF_SUCCESS            Successfully converted address.
 * @retval NRF_ERROR_INVALID_ADDR Invalid address.
 */
uint32_t transport_addr_to_short(nrf_mesh_address_t * p_addr, uint16_t * p_short_addr);

/**
 * Transmit a transport message.
 *
 * The transport layer processes a message from the application, encrypts it and
 * passes it on to the network layer.
 *
 * @param[in]  p_params           Message parameters.
 * @param[out] p_packet_reference Reference to SAR buffer (for TX complete event).
 *
 * @retval NRF_SUCCESS            The packet was successfully queued for transmission.
 * @retval NRF_ERROR_INVALID_ADDR Invalid address supplied.
 * @retval NRF_ERROR_NULL         Null-pointer supplied.
 * @retval NRF_ERROR_NO_MEM       Insufficient amount of available memory.
 * @retval NRF_ERROR_FORBIDDEN    Failed to allocate a sequence number from network. Only occurs
 *                                with unsegmented packets.
 */
uint32_t transport_tx(const nrf_mesh_tx_params_t * p_params, uint32_t * const p_packet_reference);

/**
 * Set transport layer options.
 *
 * @param[in] id    Identifier for option to set.
 * @param[in] p_opt Pointer to option struct.
 *
 * @retval NRF_SUCCESS         Successfully set option.
 * @retval NRF_ERROR_NOT_FOUND Could not find the ID requested.
 * @retval NRF_ERROR_NULL      NULL pointer supplied.
 */
uint32_t transport_opt_set(nrf_mesh_opt_id_t id, const nrf_mesh_opt_t * const p_opt);

/**
 * Get transport layer options.
 *
 * @param[in] id    Identifier for option to set.
 * @param[in] p_opt Pointer to option struct.
 *
 * @retval NRF_SUCCESS         Successfully set option.
 * @retval NRF_ERROR_NOT_FOUND Could not find the ID requested.
 * @retval NRF_ERROR_NULL      NULL pointer supplied.
 */
uint32_t transport_opt_get(nrf_mesh_opt_id_t id, nrf_mesh_opt_t * const p_opt);

/** @} */

#endif
