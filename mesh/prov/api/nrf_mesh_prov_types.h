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

#ifndef NRF_MESH_PROV_TYPES_H_
#define NRF_MESH_PROV_TYPES_H_

#include <stdint.h>
#include <stdbool.h>
#include "nrf_mesh_defines.h"
#include "prov_bearer_adv.h"
#include "prov_bearer.h"

/** Size of the elliptic curve public key. */
#define NRF_MESH_PROV_PUBKEY_SIZE      64
/** Size of the elliptic curve private key. */
#define NRF_MESH_PROV_PRIVKEY_SIZE     32
/** Size of the elliptic curve secret key. */
#define NRF_MESH_PROV_ECDHSECRET_SIZE  32
/** Size of the elliptic curve secret key. */
#define NRF_MESH_PROV_DATANONCE_SIZE  13

/** Max OOB size permitted by the Mesh Profile Specification v1.0 table 5.21 and 5.23. */
#define NRF_MESH_PROV_OOB_SIZE_MAX    8


/**
 * @defgroup PROV_FIELD_LENGTHS Provisioning field lengths
 * Length of various provisioning fields.
 * @{
 */

/** Length of Random value. */
#define PROV_RANDOM_LEN         (16)
/** Length of Confirmation value. */
#define PROV_CONFIRMATION_LEN   (NRF_MESH_KEY_SIZE)
/** Length of Auth value. */
#define PROV_AUTH_LEN           (NRF_MESH_KEY_SIZE)
/** Length of Salt value. */
#define PROV_SALT_LEN           (NRF_MESH_KEY_SIZE)
/** Length of Nonce. */
#define PROV_NONCE_LEN          (13)
/** Length of confirmation inputs */
#define PROV_CONFIRMATION_INPUT_LEN     ((sizeof(prov_pdu_invite_t) - 1) + (sizeof(prov_pdu_caps_t) - 1) + (sizeof(prov_pdu_prov_start_t) - 1))
/** @} */

/**
 * Out-of-band authentication methods for provisioning.
 */
 typedef enum
 {
     NRF_MESH_PROV_OOB_METHOD_NONE   = 0x00, /**< No authentication method. */
     NRF_MESH_PROV_OOB_METHOD_STATIC = 0x01, /**< Static OOB authentication method. */
     NRF_MESH_PROV_OOB_METHOD_OUTPUT = 0x02, /**< Output OOB authentication method. */
     NRF_MESH_PROV_OOB_METHOD_INPUT  = 0x03, /**< Input OOB authentication method. */
 } nrf_mesh_prov_oob_method_t;

 /**
  * Enumeration for the OOB input actions.
  */
 typedef enum
 {
     NRF_MESH_PROV_INPUT_ACTION_NONE         = 0x00, /**< No input action specified. */
     NRF_MESH_PROV_INPUT_ACTION_PUSH         = 0x01, /**< The user should do a push action as input action. */
     NRF_MESH_PROV_INPUT_ACTION_TWIST        = 0x02, /**< The user should do a twist action as input action. */
     NRF_MESH_PROV_INPUT_ACTION_ENTER_NUMBER = 0x03, /**< The user should enter a number into the device as input action. */
     NRF_MESH_PROV_INPUT_ACTION_ENTER_STRING = 0x04, /**< The user should enter a string into the device as input action. */
 } nrf_mesh_prov_input_action_t;

 /**
  * Enumeration for the OOB output actions.
  */
 typedef enum
 {
     NRF_MESH_PROV_OUTPUT_ACTION_NONE            = 0x00, /**< No output action specified. */
     NRF_MESH_PROV_OUTPUT_ACTION_BLINK           = 0x01, /**< The device should use blinking as output action. */
     NRF_MESH_PROV_OUTPUT_ACTION_BEEP            = 0x02, /**< The device should use beeping as output action. */
     NRF_MESH_PROV_OUTPUT_ACTION_VIBRATE         = 0x03, /**< The device should vibrate as output action. */
     NRF_MESH_PROV_OUTPUT_ACTION_DISPLAY_NUMERIC = 0x04, /**< The device should display a number as output action. */
     NRF_MESH_PROV_OUTPUT_ACTION_ALPHANUMERIC    = 0x05, /**< The device should display an alpha-numberic value as output action. */
 } nrf_mesh_prov_output_action_t;

 /**
  * Reason for why a provisioning link was closed.
  */
 typedef enum
 {
     NRF_MESH_PROV_LINK_CLOSE_REASON_SUCCESS = 0,    /**< The link was closed because provisioning was completed successfully. */
     NRF_MESH_PROV_LINK_CLOSE_REASON_TIMEOUT = 1,    /**< The link timed out and was closed. */
     NRF_MESH_PROV_LINK_CLOSE_REASON_ERROR   = 2,   /**< The link was closed beacuse of an error. */
     NRF_MESH_PROV_LINK_CLOSE_REASON_LAST = NRF_MESH_PROV_LINK_CLOSE_REASON_ERROR /**< The highest known link close reason, values higher are unrecognized */
 } nrf_mesh_prov_link_close_reason_t;

 /**
  * Provisioning failure codes.
  */
 typedef enum
 {
     NRF_MESH_PROV_FAILURE_CODE_INVALID_PDU         = 0x01, /**< An invalid provisioning PDU was received. */
     NRF_MESH_PROV_FAILURE_CODE_INVALID_FORMAT      = 0x02, /**< An incoming provisioning packet had an invalid format. */
     NRF_MESH_PROV_FAILURE_CODE_UNEXPECTED_PDU      = 0x03, /**< The incoming packet was a different packet type than what was expected. */
     NRF_MESH_PROV_FAILURE_CODE_CONFIRMATION_FAILED = 0x04, /**< The OOB authentication between provisioner and provisionee failed. */
     NRF_MESH_PROV_FAILURE_CODE_OUT_OF_RESOURCES    = 0x05, /**< The device does not have enough resources (memory, CPU time) to complete the provisioning. */
     NRF_MESH_PROV_FAILURE_CODE_DECRYPTION_FAILED   = 0x06, /**< The provisioning data could not be decrypted. */
     NRF_MESH_PROV_FAILURE_CODE_UNEXPECTED_ERROR    = 0x07, /**< An unexpected error occured. */
     NRF_MESH_PROV_FAILURE_CODE_CANNOT_ASSIGN_ADDR  = 0x08, /**< Consecutive unicast addresses could not be assigned. */
 } nrf_mesh_prov_failure_code_t;

/**
 * Provisioning bearer types.
 */
typedef enum
{
    NRF_MESH_PROV_BEARER_ADV,         /**< Advertising-based provisioning bearer, PB-ADV. */
    NRF_MESH_PROV_BEARER_GATT,        /**< GATT-based provisioning bearer, PB-GATT. */
    NRF_MESH_PROV_BEARER_MESH_CLIENT /**< PB-Mesh client bearer. Uses PB-Mesh for communicating with a target device, and reports incoming traffic to the provisioner state machine. */
} nrf_mesh_prov_bearer_type_t;


/** Provisioning role */
typedef enum
{
    NRF_MESH_PROV_ROLE_PROVISIONER, /**< The device will act as a provisioner, distributing provisioning data. */
    NRF_MESH_PROV_ROLE_PROVISIONEE  /**< The device will act as a provisionee, receiving provisioning data. */
} nrf_mesh_prov_role_t;

/*lint -align_max(push) -align_max(1) */

/**
 * Provisioning authentication capabilities.
 * This structure is filled in with the preferred values for how authentication is to be performed.
 * Only one method can be chosen for each of input/output authentication.
 */
typedef struct __attribute((packed))
{
    uint8_t  num_elements;       /**< Number of elements in the device. */
    uint16_t algorithms;         /**< Supported authentication algorithms. */
    uint8_t  pubkey_type;        /**< Supported public key types. */
    uint8_t  oob_static_types;   /**< Supported static OOB types. */
    uint8_t  oob_output_size;    /**< Output OOB data size. */
    uint16_t oob_output_actions; /**< Supported output OOB actions. */
    uint8_t  oob_input_size;     /**< Input OOB data size. */
    uint16_t oob_input_actions;  /**< Supported input OOB actions. */
} nrf_mesh_prov_oob_caps_t;

/**
 * Provisioning state machine states.
 * These are used internally in the provisioning stack to run the provisioning protocol.
 */
typedef enum
{
    NRF_MESH_PROV_STATE_IDLE,                   /**< Idle state, no action is taken. */
    NRF_MESH_PROV_STATE_WAIT_ACK,               /**< Waiting for an ACK to a previously sent packet. */
    NRF_MESH_PROV_STATE_WAIT_LINK,              /**< Waiting for a link to be established. */
    NRF_MESH_PROV_STATE_INVITE,                 /**< Waiting for a provisioning invitation message. */
    NRF_MESH_PROV_STATE_WAIT_CAPS,              /**< Waiting for a provisioning capabilities message. */
    NRF_MESH_PROV_STATE_WAIT_CAPS_CONFIRM,      /**< Waiting for the application to confirm the capabilities to use. */
    NRF_MESH_PROV_STATE_WAIT_START,             /**< Waiting for a provisioning start message. */
    NRF_MESH_PROV_STATE_WAIT_START_ACK,         /**< Waiting for the message acknowledgement for the start message. */
    NRF_MESH_PROV_STATE_WAIT_PUB_KEY_ACK,       /**< Waiting for the public key to be received by the peer. */
    NRF_MESH_PROV_STATE_WAIT_PUB_KEY,           /**< Waiting for the peer node to send its public key. */
    NRF_MESH_PROV_STATE_WAIT_OOB_PUB_KEY,       /**< Waiting for the OOB public key to be retrieved. */
    NRF_MESH_PROV_STATE_WAIT_EXTERNAL_ECDH,     /**< Waiting for the offloaded ECDH calculation to complete. */
    NRF_MESH_PROV_STATE_WAIT_OOB_INPUT,         /**< Waiting for OOB input. */
    NRF_MESH_PROV_STATE_WAIT_OOB_STATIC,        /**< Waiting for static OOB data. */
    NRF_MESH_PROV_STATE_WAIT_OOB_STATIC_C_RCVD, /**< Waiting for static OOB data, confirmation already received. */
    NRF_MESH_PROV_STATE_WAIT_CONFIRMATION_ACK,  /**< Waiting for the confirmation to be received by the peer. */
    NRF_MESH_PROV_STATE_WAIT_CONFIRMATION,      /**< Waiting for a provisioning confirmation message. */
    NRF_MESH_PROV_STATE_WAIT_INPUT_COMPLETE,    /**< Waiting for an input complete message. */
    NRF_MESH_PROV_STATE_WAIT_RANDOM,            /**< Waiting for a provisioning random message. */
    NRF_MESH_PROV_STATE_WAIT_DATA,              /**< Waiting for the provisioning data message. */
    NRF_MESH_PROV_STATE_WAIT_COMPLETE,          /**< Waiting for the provisioning complete message. */
    NRF_MESH_PROV_STATE_COMPLETE,               /**< Provisioning complete state. */
    NRF_MESH_PROV_STATE_FAILED,                 /**< Provisioning failed state. */
} nrf_mesh_prov_state_t;



/**
 * Provisioning data to transmit to a device.
 */
 typedef struct __attribute((packed))
 {
     /** Network key for the device. */
     uint8_t  netkey[NRF_MESH_KEY_SIZE];
     /** Network key index. */
     uint16_t netkey_index;
     /** IV_index value for the device, in little endian format. */
     uint32_t iv_index;
     /** Unicast address for the device. */
     uint16_t address;
     /** Flags. */
     struct __attribute((packed)) {
         /** IV update in progress flag. */
         uint8_t iv_update   : 1;
         /** Key refresh in progress flag. */
         uint8_t key_refresh : 1;
     } flags;
 } nrf_mesh_prov_provisioning_data_t;

 /*lint -align_max(pop) */


/**
 * Provisioning bearer context.
 */
typedef struct prov_bearer_t
{
    union
    {
        prov_bearer_adv_t pb_adv;                 /**< Context for the PB-ADV bearer. */
    } bearer;                                     /**< Bearer specific context. */
    const struct prov_bearer_callbacks *  p_callbacks; /**< Provisioning module user callbacks. */
    const struct prov_bearer_interface *  p_interface; /**< Provisioning bearer interface. */
    uint32_t            timeout;                  /**< Link timeout value (in us). */
} prov_bearer_t;


/**
 * Common provisioning context.
 */
typedef struct nrf_mesh_prov_ctx
{
    prov_bearer_t bearer; /**< Provisioning bearer information. */

    const uint8_t * p_public_key;  /**< Public key of this node. */
    const uint8_t * p_private_key; /**< Private key of this node. */

    uint8_t peer_public_key[NRF_MESH_PROV_PUBKEY_SIZE];   /**< Public key of the peer node. */
    uint8_t shared_secret[NRF_MESH_PROV_ECDHSECRET_SIZE]; /**< ECDH shared secret: P-256(private key, peer public key). */

    uint8_t device_key[NRF_MESH_KEY_SIZE];  /**< Node device key. */
    uint8_t session_key[NRF_MESH_KEY_SIZE]; /**< Provisioning session key. */
    uint8_t data_nonce[PROV_NONCE_LEN];  /**< Provisioning data nonce. Only 13 bytes are used. */

    uint8_t node_random[PROV_RANDOM_LEN]; /**< Random number for the current node. */
    uint8_t peer_random[PROV_RANDOM_LEN]; /**< Random number for the peer node. */
    uint8_t auth_value[PROV_AUTH_LEN];    /**< Authentication value. */

    uint8_t confirmation_salt[PROV_SALT_LEN];   /**< Confirmation salt value. */
    uint8_t peer_confirmation[PROV_CONFIRMATION_LEN];   /**< Confirmation value for the peer node. */
    uint8_t confirmation_inputs[PROV_CONFIRMATION_INPUT_LEN]; /**< Confirmation inputs, used to calculate the confirmation key. */

    uint8_t oob_size;   /**< Size of the chosen OOB authentication data. */
    uint8_t oob_action; /**< Chosen OOB action. */
    bool pubkey_oob;    /**< Uses out-of-band public key. */

    nrf_mesh_prov_role_t role;                 /**< Provisioning role, provisioner or provisionee. */
    nrf_mesh_prov_failure_code_t failure_code; /**< Error code sent with the previous provisioning failed packet. */
    nrf_mesh_prov_state_t state;               /**< Provisioning state machine state. */
    nrf_mesh_prov_oob_method_t oob_method;     /**< Chosen OOB authentication method. */
    nrf_mesh_prov_oob_caps_t capabilities;     /**< Node OOB and authentication capabilities. */
    nrf_mesh_prov_provisioning_data_t data;    /**< Provisioning data to send to the provisionee or received from the provisioner. */

    struct nrf_mesh_prov_ctx * p_next;  /**< Pointer to the next provisioner context structure, used for parallel provisioning. */
} nrf_mesh_prov_ctx_t;


#endif /* NRF_MESH_PROV_TYPES_H__ */
