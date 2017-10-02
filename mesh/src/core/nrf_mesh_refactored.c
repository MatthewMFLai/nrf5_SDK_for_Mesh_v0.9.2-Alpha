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
#include "nrf_mesh.h"
#include "nrf_mesh_utils.h"
#include "nrf_mesh_events.h"
#include "nrf_mesh_assert.h"
#include "nrf_mesh_configure.h"
#include "nrf_mesh_utils.h"

#include <string.h>
#include <stddef.h>

#include "nrf.h"
#include <nrf_error.h>
#include "nrf_sdm.h"

#include "bearer.h"
#include "bearer_event.h"
#include "enc.h"
#include "event.h"
#include "fifo.h"
#include "msg_cache.h"
#include "network.h"
#include "packet.h"
#include "packet_mgr.h"
#include "nrf_mesh_dfu.h"
#include "dfu_types_internal.h"
#include "ticker.h"
#include "timer_scheduler.h"
#include "timeslot.h"
#include "toolchain.h"
#include "transport.h"
#include "beacon.h"
#include "dfu_types_internal.h"
#include "list.h"
#include "utils.h"
#include "log.h"
#include "mesh_flash.h"
#include "flash_manager.h"

#include "prov_bearer_adv.h"

/** Function pointer to mesh assertion handler. */
nrf_mesh_assertion_handler_t m_assertion_handler;

