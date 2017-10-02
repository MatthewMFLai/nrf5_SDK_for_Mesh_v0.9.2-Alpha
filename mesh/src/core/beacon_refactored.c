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
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "packet_mgr.h"
#include "packet.h"
#include "log.h"
#include "bearer.h"
#include "nrf_mesh.h"
#include "prov_beacon.h"
#include "bearer_adv.h"
#include "network.h"
#include "nrf_mesh_assert.h"
#include "net_beacon.h"

#include "beacon.h"
#include "beacon_refactored.h"
/************************
 * Packet Type Typedefs *
 ************************/

/** Generic beacon packet */
typedef struct
{
    uint8_t beacon_type;  /**< Beacon type, see @ref BEACON_TYPE */
    uint8_t payload[];    /**< Beacon payload. */
} beacon_packet_t;

/********************/
/* Static variables */
/********************/
static advertiser_t m_advertiser; /**< Advertiser instance used to transmit beacon packets. */
/**************/
/* Public API */
/**************/
advertiser_t *beacon_get_advertiser(void)
{
	return (&m_advertiser);
}

uint32_t beacon_tx(uint8_t beacon_type, const void* p_payload, uint8_t payload_len, uint8_t count)
{
    if (p_payload == NULL ||
            payload_len == 0 ||
            beacon_type == BEACON_TYPE_INVALID ||
            payload_len > BEACON_DATA_MAXLEN ||
            count == 0)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    uint32_t status;
    packet_t * p_packet = beacon_create(beacon_type, p_payload, payload_len);
    if (p_packet == NULL)
    {
        status = NRF_ERROR_NO_MEM;
    }
    else
    {
        status = bearer_adv_tx(&m_advertiser, p_packet, count);
        if (status != NRF_SUCCESS)
        {
            packet_mgr_free(p_packet);
        }
    }
    return status;
}

packet_t* beacon_create(uint8_t beacon_type, const void* p_payload, uint8_t payload_len)
{
    if (p_payload == NULL || payload_len == 0 || beacon_type == BEACON_TYPE_INVALID || payload_len > BEACON_DATA_MAXLEN)
    {
        return NULL;
    }
    packet_t* p_packet = NULL;
    uint8_t beacon_len = BEACON_PACKET_AD_LEN_OVERHEAD + payload_len;
    uint32_t status = packet_mgr_alloc((packet_generic_t **) &p_packet,
            BLE_ADV_PACKET_HEADER_LENGTH + BLE_ADV_PACKET_OVERHEAD + BLE_AD_DATA_OVERHEAD + beacon_len);

    if (status == NRF_SUCCESS)
    {
        packet_payload_size_set(p_packet, BLE_AD_DATA_OVERHEAD + beacon_len);

        ble_ad_data_t* p_ad_data = (ble_ad_data_t*) &p_packet->payload[0];
        p_ad_data->length = beacon_len;
        p_ad_data->type = AD_TYPE_BEACON;
        beacon_packet_t* p_beacon = (beacon_packet_t*) p_ad_data->data;
        p_beacon->beacon_type = beacon_type;
        memcpy(&p_beacon->payload[0], p_payload, payload_len);
    }

    return p_packet;
}
