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
#include <stdio.h>
#include <string.h>
#include "flash_manager.h"
#include "flash_manager_internal.h"
#include "flash_manager_defrag.h"
#include "nrf.h"
#include "nrf_error.h"
#include "toolchain.h"
#include "bearer_event.h"
#include "nrf_mesh_assert.h"
#include "internal_event.h"
#include "queue.h"

#include "hal.h"
#include "utils.h"

//-------------------------------------------------------------------
/*****************************************************************************
* Local defines
*****************************************************************************/
#define DEFRAG_RECOVER_STEP (erase_source)
#define MAX_PROCEDURE_STEP (sizeof(m_procedure_steps) / sizeof(m_procedure_steps[0]))

/*****************************************************************************
* Local typedefs
*****************************************************************************/
/** Global defrag states */
typedef enum
{
    DEFRAG_STATE_IDLE, /**< No defrag is currently in progress. */
    DEFRAG_STATE_PROCESSING, /**< Currently in the middle of a defrag procedure. */
    DEFRAG_STATE_STABILIZING /**< Stabilizing flash contents after a finished defrag procedure. */
} defrag_state_t;

/** Procedure step end action. Returned by each procedure step to indicate what the next step in
 *  the procedure should be. */
typedef enum
{
    PROCEDURE_STAY,     /**< Wait for a flash operation to finish, and repeat the current step. */
    PROCEDURE_CONTINUE, /**< Move on to the next step in the procedure. */
    PROCEDURE_END,      /**< End the procedure. */
    PROCEDURE_RESTART,  /**< Start the procedure from the beginning. */
} procedure_action_t;

typedef struct
{
    defrag_state_t state;
    uint32_t step; /**< Current step number in the backup procedure. */
    const flash_manager_t * p_manager; /**< Flash manager owning the page currently being defragged. */
    const flash_manager_page_t * p_storage_page; /**< Page being defragmented. */
    const fm_entry_t * p_src; /**< Next entry to copy */
    const fm_entry_t * p_dst; /**< Next destination in recovery page. */
    bool wait_for_idle;       /**< Flag, that when set makes the procedure wait for all flash operations to end before proceeding. */
    bool found_all_entries;   /**< Whether we've ran through all entries in the original area. */
} defrag_t;

/** Single chunk of entries. */
typedef struct
{
    const fm_entry_t * p_start; /**< First entry in the chunk. */
    uint32_t length; /**< Length of the chunk in bytes. */
} fm_entry_chunk_t;

typedef procedure_action_t (*defrag_procedure_step_t)(void);

/*****************************************************************************
* Static globals
*****************************************************************************/
static flash_manager_recovery_area_t * mp_recovery_area; /**< Recovery area pointer into flash. */
static defrag_t m_defrag; /**< Global defrag state. */
static uint16_t m_token; /**< Flash operation token returned from the mesh flash module. */

/* We're iterating through pages with the assumption that one flash_manager_page_t and
 * flash_manager_recovery_area_t are exactly one page long, and that fm_entry_t is exactly one word.
 * Ensure these assumptions are true: */
NRF_MESH_STATIC_ASSERT(sizeof(flash_manager_page_t) == PAGE_SIZE);
NRF_MESH_STATIC_ASSERT(sizeof(flash_manager_recovery_area_t) == PAGE_SIZE);
NRF_MESH_STATIC_ASSERT(sizeof(fm_entry_t) == WORD_SIZE);

/*****************************************************************************
* Local utility functions
*****************************************************************************/
/**
 * Get the largest possible continuous chunk of data entries starting at the first entry
 * representing data after the @p p_src pointer.
 *
 * @param[out] Chunk object to populate.
 * @param[in] p_src Entry to start looking for a chunk from.
 * @param[in] p_end The upper boundary of memory to look through.
 * @param[in] max_length The maximum length of the chunk. The resulting chunk will never exceed
 * this length.
 *
 * @returns The first entry after the chunk, or NULL if there were no more valid data entries
 * before meeting the boundary conditions.
 */
static inline const fm_entry_t * get_chunk_of_data_entries(fm_entry_chunk_t * p_chunk,
                                                           const fm_entry_t * p_src,
                                                           const void *       p_end,
                                                           uint32_t           max_length)
{
    p_chunk->length = 0;
    /* find start of chunk */
    if (!handle_represents_data(p_src->header.handle))
    {
        p_src = get_next_data_entry(p_src, p_end);
        if (p_src == NULL)
        {
            p_chunk->p_start = NULL;
            return NULL;
        }
    }
    p_chunk->p_start = p_src;

    /* find end of chunk */
    while (((const void *) p_src < p_end) &&
           (p_chunk->length + p_src->header.len_words * WORD_SIZE < max_length) &&
           (PAGE_START_ALIGN(p_src) == PAGE_START_ALIGN(p_chunk->p_start)) &&
           handle_represents_data(p_src->header.handle))
    {
        /* include p_src in the chunk */
        p_chunk->length += p_src->header.len_words * WORD_SIZE;

        p_src = get_next_entry(p_src);
    }
    return p_src;
}

/*****************************************************************************
* Defrag procedure m_procedure_steps
*****************************************************************************/
static procedure_action_t check_for_invalid_entries(void)
{
    /* If we can't find any invalid entries in this page, we should skip it. */
    if (entry_get(get_first_entry(m_defrag.p_storage_page),
                  m_defrag.p_storage_page + 1,
                  FLASH_MANAGER_HANDLE_INVALID) == NULL)
    {
        if (m_defrag.p_storage_page == get_last_page(m_defrag.p_storage_page))
        {
            return PROCEDURE_END;
        }
        else
        {
            /* Jump to the next page */
            m_defrag.p_storage_page++;
            return PROCEDURE_RESTART;
        }
    }
    else
    {
        return PROCEDURE_CONTINUE;
    }
}

static procedure_action_t erase_recovery_area(void)
{
    if (erase(mp_recovery_area, PAGE_SIZE, &m_token) == NRF_SUCCESS)
    {
        return PROCEDURE_CONTINUE;
    }
    else
    {
        /* retry later */
        return PROCEDURE_STAY;
    }
}

static procedure_action_t copy_metadata(void)
{
    uint32_t metadata_length = m_defrag.p_storage_page->metadata.metadata_len;
    if (flash(&mp_recovery_area->data[0], m_defrag.p_storage_page, metadata_length, &m_token) == NRF_SUCCESS)
    {
        /* ready to start copying entries */
        m_defrag.p_dst = (const fm_entry_t *) &mp_recovery_area->data[metadata_length / sizeof(mp_recovery_area->data[0])];
        m_defrag.p_src = get_first_entry(m_defrag.p_storage_page);
        m_defrag.found_all_entries = false;
        return PROCEDURE_CONTINUE;
    }
    else
    {
        /* retry later */
        return PROCEDURE_STAY;
    }
}

/**
 * Copy valid handles from storage area to backup page.
 */
static procedure_action_t backup_entries(void)
{
    NRF_MESH_ASSERT(m_defrag.p_src != NULL);
    NRF_MESH_ASSERT(m_defrag.p_dst != NULL);

    bool recovery_page_is_full = false;
    /* Copy entries in chunks of sequential data-entries until we get to the end or are unable to
       fit any more in the recovery page. */
    while (!m_defrag.found_all_entries && !recovery_page_is_full)
    {
        fm_entry_chunk_t chunk;
        uint32_t remaining_space = (PAGE_START_ALIGN(m_defrag.p_dst) + PAGE_SIZE) - (uint32_t) m_defrag.p_dst;
        const fm_entry_t * p_next =
            get_chunk_of_data_entries(&chunk,
                                      m_defrag.p_src,
                                      get_area_end(m_defrag.p_storage_page),
                                      remaining_space);

        if (chunk.length > 0)
        {
            if (NRF_SUCCESS == flash(m_defrag.p_dst, chunk.p_start, chunk.length, &m_token))
            {
                m_defrag.p_dst += (chunk.length / sizeof(fm_entry_t));
                NRF_MESH_ASSERT(p_next != NULL);
                m_defrag.p_src = p_next;
            }
            else
            {
                /* Early return */
                return PROCEDURE_STAY;
            }
        }
        else
        {
            recovery_page_is_full = true;
        }
        /* Getting a NULL return from the chunk getter means there were no more valid data chunks left. */
        if (p_next == NULL || p_next->header.handle == HANDLE_SEAL)
        {
            m_defrag.found_all_entries = true;
        }
    }

    return PROCEDURE_CONTINUE;
}


static procedure_action_t write_defrag_start_pointer(void)
{
    if (flash(&mp_recovery_area->p_storage_page, (void *) &m_defrag.p_storage_page, WORD_SIZE, &m_token) == NRF_SUCCESS)
    {
        return PROCEDURE_CONTINUE;
    }
    else
    {
        /* retry later */
        return PROCEDURE_STAY;
    }
}

static procedure_action_t erase_source(void)
{
    if (erase(m_defrag.p_storage_page, PAGE_SIZE, &m_token) == NRF_SUCCESS)
    {
        return PROCEDURE_CONTINUE;
    }
    else
    {
        /* retry later */
        return PROCEDURE_STAY;
    }
}

static procedure_action_t write_back(void)
{
    if (flash(m_defrag.p_storage_page, mp_recovery_area->data, sizeof(mp_recovery_area->data), &m_token) == NRF_SUCCESS)
    {
        /* In the next step, we'll iterate through the memory we write here, so we need this entire
           flash operation to finish before moving on: */
        m_defrag.wait_for_idle = true;
        return PROCEDURE_CONTINUE;
    }
    else
    {
        return PROCEDURE_STAY;
    }
}

/**
 * Write a seal or padding entry at the end of the current storage page.
 */
static procedure_action_t seal_storage_page(void)
{
    /* Put seal at the first blank header: */
    const fm_entry_t * p_seal_location = entry_get(get_first_entry(m_defrag.p_storage_page),
                                                   m_defrag.p_storage_page + 1,
                                                   HANDLE_BLANK);
    if (p_seal_location == NULL || p_seal_location == get_first_entry(m_defrag.p_storage_page))
    {
        /* Don't seal if the target page is either completely full or completely empty. */
        return PROCEDURE_CONTINUE;
    }
    /* If there are more entries in the area, pad the page, else seal it. */
    const fm_header_t * p_header;
    if (m_defrag.found_all_entries)
    {
        p_header = &SEAL_HEADER;
    }
    else
    {
        p_header = &PADDING_HEADER;
    }

    if (flash(p_seal_location, p_header, sizeof(fm_header_t), &m_token) == NRF_SUCCESS)
    {
        m_defrag.wait_for_idle = true;
        return PROCEDURE_CONTINUE;
    }
    else
    {
        return PROCEDURE_STAY;
    }
}

/**
 * Step through the entries on the page AFTER the one we're recovering, and
 * remove any entries that also occur in the backup, as these are duplicates.
 */
static procedure_action_t invalidate_duplicate_entries(void)
{
    if (m_defrag.p_storage_page->metadata.page_index ==
        m_defrag.p_storage_page->metadata.pages_in_area - 1)
    {
        /* The current page was the last page in the area, and there won't be any duplicates. */
        return PROCEDURE_CONTINUE;
    }

    const fm_entry_t * p_area_entry = get_first_entry(m_defrag.p_storage_page + 1);
    const fm_entry_t * p_recovery_entry =
        get_first_entry((const flash_manager_page_t *) mp_recovery_area->data);
    const fm_entry_t * p_end = (const fm_entry_t *) (get_area_end(m_defrag.p_storage_page));

    if (!handle_represents_data(p_area_entry->header.handle))
    {
        p_area_entry = get_next_data_entry(p_area_entry, p_end);
    }

    while (p_area_entry != NULL)
    {
        p_recovery_entry =
            entry_get(p_recovery_entry, mp_recovery_area + 1, p_area_entry->header.handle);
        if (p_recovery_entry == NULL)
        {
            /* Since the entries would have been added to the recovery area in-order, we can assume
               that once we find an entry that's not duplicated in the recovery area, we won't find
               any more duplicates. */
            break;
        }

        /* Wait for all invalidations to finish before moving on to the next step in the procedure,
           as they will alter the way we'll traverse the page on the next round: */
        m_defrag.wait_for_idle = true;

        if (flash(p_area_entry, &INVALID_HEADER, sizeof(INVALID_HEADER), &m_token) !=
            NRF_SUCCESS)
        {
            return PROCEDURE_STAY;
        }

        p_area_entry = get_next_data_entry(p_area_entry, p_end);
    }
    return PROCEDURE_CONTINUE;
}

static procedure_action_t post_process(void)
{
    if (m_defrag.p_storage_page == get_last_page(m_defrag.p_storage_page))
    {
        /* Invalidate area pointer */
        static const uint32_t * p_null_ptr = NULL;
        if (flash(&mp_recovery_area->p_storage_page, &p_null_ptr, sizeof(p_null_ptr), &m_token) == NRF_SUCCESS)
        {
            m_defrag.wait_for_idle = true;
            return PROCEDURE_END;
        }
        else
        {
            return PROCEDURE_STAY;
        }
    }
    else
    {
        /** Start the procedure from the beginning, operating on the next page in the area. */
        m_defrag.p_storage_page++;
        return PROCEDURE_RESTART;
    }
}

static const defrag_procedure_step_t m_procedure_steps[] =
{
    check_for_invalid_entries,
    erase_recovery_area,
    copy_metadata,
    backup_entries,
    write_defrag_start_pointer,
    erase_source,
    write_back,
    seal_storage_page,
    invalidate_duplicate_entries,
    post_process
};
/*****************************************************************************
* Static functions
*****************************************************************************/
static void defrag_on_end(void)
{
    const flash_manager_t * p_manager = m_defrag.p_manager;
    memset(&m_defrag, 0, sizeof(m_defrag));
    flash_manager_on_defrag_end((flash_manager_t *) p_manager);
}

static void jump_to_step(defrag_procedure_step_t step)
{
    for (uint32_t i = 0; i < MAX_PROCEDURE_STEP; i++)
    {
        if (m_procedure_steps[i] == step)
        {
            m_defrag.step = i;
            return;
        }
    }
    /* Couldn't find step */
    NRF_MESH_ASSERT(false);
}

static void execute_procedure_step(void)
{
    /* Verify the validity of the state */
    NRF_MESH_ASSERT(m_defrag.p_storage_page != NULL);
    NRF_MESH_ASSERT(!m_defrag.wait_for_idle);
    NRF_MESH_ASSERT(m_defrag.step < MAX_PROCEDURE_STEP);
    procedure_action_t action;

    do
    {
        action = m_procedure_steps[m_defrag.step]();
        switch (action)
        {
            case PROCEDURE_STAY:
                break;
            case PROCEDURE_CONTINUE:
                m_defrag.step++;
                NRF_MESH_ASSERT(m_defrag.step < MAX_PROCEDURE_STEP);
                break;
            case PROCEDURE_RESTART:
                m_defrag.step = 0;
                break;
            case PROCEDURE_END:
                if (m_defrag.wait_for_idle)
                {
                    m_defrag.state = DEFRAG_STATE_STABILIZING;
                }
                else
                {
                    defrag_on_end();
                }
                break;
        }
    } while ((action == PROCEDURE_CONTINUE || action == PROCEDURE_RESTART) &&
             !m_defrag.wait_for_idle);
}

static void on_flash_op_end(mesh_flash_user_t user, const flash_operation_t * p_op, uint16_t token)
{
    /*lint -esym(715, user, token) Ignore unused tokens */

    if (p_op->type == FLASH_OP_TYPE_ALL)
    {
        m_defrag.wait_for_idle = false;
    }

    if (m_defrag.state == DEFRAG_STATE_PROCESSING)
    {
        if (!m_defrag.wait_for_idle)
        {
            execute_procedure_step();
        }
    }
    else if (m_defrag.state == DEFRAG_STATE_STABILIZING && p_op->type == FLASH_OP_TYPE_ALL)
    {
        defrag_on_end();
    }
}

/**
 * Checks if a defrag was interrupted by a power cycle, and continues where it left off.
 *
 * @returns Whether there's a defrag in progress.
 */
static bool recover_defrag_progress(void)
{
    if (mp_recovery_area->p_storage_page != NULL &&
        mp_recovery_area->p_storage_page != (void *) BLANK_FLASH_WORD &&
        IS_PAGE_ALIGNED(mp_recovery_area->p_storage_page))
    {
        m_defrag.p_storage_page = mp_recovery_area->p_storage_page;
        m_defrag.wait_for_idle = false;
        m_defrag.found_all_entries = false;
        m_defrag.state = DEFRAG_STATE_PROCESSING;
        m_defrag.p_manager = NULL; /* Can't know which manager this is. */
        jump_to_step(DEFRAG_RECOVER_STEP);
        mesh_flash_user_callback_set(FLASH_MANAGER_FLASH_USER, on_flash_op_end);
        execute_procedure_step();
        return true;
    }
    else
    {
        return false;
    }
}

/*****************************************************************************
* Interface functions
*****************************************************************************/

bool flash_manager_defrag_init(void)
{
    flash_manager_recovery_area_t * p_flash_end;
#ifdef FLASH_MANAGER_RECOVERY_PAGE
    mp_recovery_area = (flash_manager_recovery_area_t *) FLASH_MANAGER_RECOVERY_PAGE;
#else
    if (BOOTLOADERADDR() != BLANK_FLASH_WORD &&
        BOOTLOADERADDR() != 0)
    {
        p_flash_end = (flash_manager_recovery_area_t *) BOOTLOADERADDR();
    }
    else
    {
        p_flash_end = (flash_manager_recovery_area_t *) DEVICE_FLASH_END_GET();
    }
    /* Recovery area is last page of application controlled flash */
    mp_recovery_area = p_flash_end - 1 ; /* pointer arithmetic */
#endif

    return recover_defrag_progress();
}

bool flash_manager_defragging(const flash_manager_t * p_manager)
{
    NRF_MESH_ASSERT(p_manager != NULL);
    return (m_defrag.state != DEFRAG_STATE_IDLE &&
            m_defrag.p_storage_page >= get_first_page(p_manager->config.p_area) &&
            m_defrag.p_storage_page <= get_last_page(p_manager->config.p_area));
}

bool flash_manager_defrag_is_running(void)
{
    return (m_defrag.state != DEFRAG_STATE_IDLE);
}

void flash_manager_defrag(const flash_manager_t * p_manager)
{
    NRF_MESH_ASSERT(m_defrag.state == DEFRAG_STATE_IDLE);
    NRF_MESH_ASSERT(p_manager->internal.state == FM_STATE_DEFRAG);

    m_defrag.p_manager = p_manager;
    m_defrag.p_storage_page = p_manager->config.p_area;
    m_defrag.step = 0;
    m_defrag.wait_for_idle = false;
    m_defrag.state = DEFRAG_STATE_PROCESSING;
    m_defrag.found_all_entries = false;

    mesh_flash_user_callback_set(FLASH_MANAGER_FLASH_USER, on_flash_op_end);

    execute_procedure_step();
    __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_FM_DEFRAG, 0, 0, NULL);
}

const void * flash_manager_defrag_recovery_page_get(void)
{
    return mp_recovery_area;
}

#ifdef UNIT_TEST
void flash_manager_defrag_reset(void)
{
    memset((uint8_t*)&m_defrag, 0, sizeof(m_defrag));
    mp_recovery_area = NULL;
    m_token = 0;
}
#endif
//-------------------------------------------------------------------
#define HEADER_LEN       (sizeof(fm_header_t))
#define ACTION_BUFFER_SIZE_NO_PARAMS     (offsetof(action_t, params))
#define ACTION_BUFFER_SIZE_ENTRY_NO_DATA (offsetof(action_t, params.entry_data.entry.data))
#define ACTION_BUFFER_SIZE_METADATA      (offsetof(action_t, params.metadata) + sizeof(flash_manager_metadata_t))
#define ACTION_QUEUE_BUFFER_LENGTH       (sizeof(packet_buffer_packet_t) + FLASH_MANAGER_POOL_SIZE)

NRF_MESH_STATIC_ASSERT(HEADER_LEN == WORD_SIZE);
NRF_MESH_STATIC_ASSERT(IS_WORD_ALIGNED(sizeof(flash_manager_metadata_t)));

/** Action to perform */
typedef enum
{
    ACTION_TYPE_REPLACE, /**< Replace an existing entry, or create a new. */
    ACTION_TYPE_INVALIDATE, /**< Invalidate an existing entry. */
    ACTION_TYPE_BUILD_METADATA, /**< Build page metadata. */
    ACTION_TYPE_RECOVER_SEAL, /**< Recover seal at end of entries. */
    ACTION_TYPE_ERASE_AREA, /**< Erase the entire manager area. */
} action_type_t;

/**
 * States an action go through while being processed.
 */
typedef enum
{
    ACTION_STATE_IDLE,           /**< No action is being processed. */
    ACTION_STATE_PROCESSING,     /**< This module is processing an action. */
    ACTION_STATE_WAIT_FOR_FLASH, /**< Waiting for flash operations to finish. */
    ACTION_STATE_DONE,           /**< Done processing the event. */
} action_state_t;

/** Flash manager action buffer, used to keep an entry waiting to be flashed. */
typedef struct
{
    flash_manager_t * p_manager; /**< Flash manager doing the write. */
    action_type_t       action;    /**< Action to perform. */
    union
    {
        struct
        {
            const fm_entry_t * p_target; /**< Pointer to target entry in flash. Used to pass the location to the user. */
            fm_entry_t         entry;    /**< Entry data to write. */
        } entry_data;
        flash_manager_metadata_t metadata; /**< Metadata to write. */
    } params;
} action_t;

static packet_buffer_t m_action_queue;
static uint8_t         m_action_queue_buffer[ACTION_QUEUE_BUFFER_LENGTH] __attribute__((aligned(WORD_SIZE)));

/* Short, common flash entries: */
const fm_header_t INVALID_HEADER __attribute__((aligned(WORD_SIZE))) = {0xFFFF, FLASH_MANAGER_HANDLE_INVALID};
const fm_header_t PADDING_HEADER __attribute__((aligned(WORD_SIZE))) = {0xFFFF, HANDLE_PADDING};
const fm_header_t SEAL_HEADER    __attribute__((aligned(WORD_SIZE))) = {0xFFFF, HANDLE_SEAL};

static bearer_event_flag_t   m_processing_flag;
static fm_state_t            m_state;
static action_state_t        m_action_state;
static uint16_t              m_token; /**< Token dealt by mesh flash that marks all flash operations complete for the current action. */
static queue_t               m_memory_listener_queue;
/******************************************************************************
* Static functions
******************************************************************************/

static inline void schedule_processing(void)
{
    bearer_event_flag_set(m_processing_flag);
}

/**
 * Get the packet buffer containing the given action in the action queue.
 *
 * @param[in] p_action Action to get packet buffer of
 *
 * @returns Pointer to packet buffer containing the given action.
 */
static inline packet_buffer_packet_t * get_packet_buffer(const action_t * p_action)
{
    NRF_MESH_ASSERT(p_action >= (action_t *) &m_action_queue_buffer[0] &&
                    p_action <= (action_t *) &m_action_queue_buffer[ACTION_QUEUE_BUFFER_LENGTH]);
    return (packet_buffer_packet_t *) ((uint32_t) p_action - offsetof(packet_buffer_packet_t, packet));
}
/**
 * Get the action containing the given entry in the action queue.
 *
 * @param[in] p_entry Entry to get action of.
 *
 * @returns Pointer to action containing the given entry.
 */
static inline action_t * get_entry_action(const fm_entry_t * p_entry)
{
    NRF_MESH_ASSERT(p_entry >= (fm_entry_t *) &m_action_queue_buffer[0] &&
                    p_entry <= (fm_entry_t *) &m_action_queue_buffer[ACTION_QUEUE_BUFFER_LENGTH]);
    return (action_t *) ((uint32_t) p_entry - offsetof(action_t, params.entry_data.entry));
}

static action_t * reserve_action_buffer(uint32_t size)
{
    packet_buffer_packet_t * p_packet_buffer = NULL;
    uint32_t status = packet_buffer_reserve(&m_action_queue,
                    &p_packet_buffer,
                    size);
    if (status == NRF_SUCCESS)
    {
        return (action_t *) p_packet_buffer->packet;
    }
    else
    {
        return NULL;
    }
}

static inline void commit_action_buffer(action_t * p_action)
{
    packet_buffer_packet_t * p_buffer = get_packet_buffer(p_action);
    packet_buffer_commit(&m_action_queue, p_buffer, p_buffer->size);
}

static const fm_entry_t * get_last_entry(const flash_manager_t * p_manager)
{
    const fm_entry_t * p_entry = NULL;
    const fm_entry_t * p_next  = get_first_entry(p_manager->config.p_area);
    while ((void *) p_next < get_area_end(p_manager->config.p_area) &&
           (p_next->header.handle != HANDLE_BLANK))
    {
        p_entry = p_next;
        p_next = get_next_entry(p_next);
    }
    return p_entry;
}

static inline const void * get_defrag_threshold(const flash_manager_t * p_manager)
{
    return (const void *) ((uint32_t) get_area_end(p_manager->config.p_area) -
                           p_manager->config.min_available_space);
}

/** Get the number of bytes remaining before the defragmentation threshold. The seal could be past
 * the threshold, so we need to do this as a signed integer. */
static int32_t get_remaining_free_space(const flash_manager_t * p_manager)
{
    const void * p_defrag_threshold =
        get_defrag_threshold(p_manager);

    return ((uint8_t *) p_defrag_threshold - (uint8_t *) p_manager->internal.p_seal);
}

static bool flash_area_is_blank(const void * p_area, uint32_t size)
{
    NRF_MESH_ASSERT(IS_WORD_ALIGNED(p_area));
    NRF_MESH_ASSERT(IS_WORD_ALIGNED(size));
    for (uint32_t i = 0; i < size / sizeof(uint32_t); i++)
    {
        if (((uint32_t *) p_area)[i] != BLANK_FLASH_WORD)
        {
            return false;
        }
    }
    return true;
}

static inline bool metadata_is_valid(const flash_manager_metadata_t * p_metadata)
{
    return (p_metadata->metadata_len != 0xFF &&
            p_metadata->metadata_len >= 8 &&
            p_metadata->page_index != 0xFF &&
            p_metadata->pages_in_area != 0xFF);
}

static bool flash_area_is_valid(const flash_manager_t * p_manager)
{
    for (uint32_t i = 0; i < p_manager->config.page_count; i++)
    {
        if (metadata_is_valid(&p_manager->config.p_area[i].metadata))
        {
            /* Check that the existing metadata aligns with ours */
            NRF_MESH_ASSERT(p_manager->config.p_area[i].metadata.page_index == i);
            NRF_MESH_ASSERT(p_manager->config.p_area[i].metadata.pages_in_area == p_manager->config.page_count);
        }
        else
        {
            return false;
        }
    }

    return true;
}

static uint32_t flash_area_build(flash_manager_t * p_manager)
{
    for (uint32_t i = 0; i < p_manager->config.page_count; i++)
    {
        /* Only flash pages without complete metadata. */
        if (flash_area_is_blank(&p_manager->config.p_area[i].raw[WORD_SIZE], PAGE_SIZE - WORD_SIZE))
        {
            action_t * p_action = reserve_action_buffer(ACTION_BUFFER_SIZE_METADATA);
            if (p_action == NULL)
            {
                return NRF_ERROR_NO_MEM;
            }
            else
            {
                p_action->action = ACTION_TYPE_BUILD_METADATA;
                p_action->params.metadata.metadata_len = sizeof(flash_manager_metadata_t);
                p_action->params.metadata.entry_header_length = sizeof(fm_header_t);
                p_action->params.metadata.entry_type_length_bits = sizeof(fm_handle_t) * 8;
                p_action->params.metadata.entry_len_length_bits = sizeof(((fm_header_t *) NULL)->len_words) * 8;
                p_action->params.metadata.page_index = i;
                p_action->params.metadata.pages_in_area = p_manager->config.page_count;
                p_action->params.metadata._padding = 0xFFFF;
                p_action->p_manager = p_manager;
                commit_action_buffer(p_action);
            }
        }
        else
        {
            /* If the page isn't blank, it has to contain valid metadata. If
             * this fails, the selected area contains data not managed by the
             * flash manager. */
            NRF_MESH_ASSERT(metadata_is_valid(&p_manager->config.p_area[i].metadata));
            /* The metadata page index and count must align with the page index
             * and count set for the given manager. If this fails, we are
             * trying to build two overlapping areas. */
            NRF_MESH_ASSERT(p_manager->config.p_area[i].metadata.page_index == i);
            NRF_MESH_ASSERT(p_manager->config.p_area[i].metadata.pages_in_area == p_manager->config.page_count);
        }
    }
    schedule_processing();
    return NRF_SUCCESS;
}

static uint32_t recover_seal(flash_manager_t * p_manager)
{
    uint32_t status = NRF_SUCCESS;
    p_manager->internal.p_seal = get_last_entry(p_manager);
    if (p_manager->internal.p_seal == NULL)
    {
        /* The area only contained metadata. */
        p_manager->internal.p_seal = get_first_entry(p_manager->config.p_area);
    }
    else if (p_manager->internal.p_seal->header.handle != HANDLE_SEAL)
    {
        /* The last entry isn't a seal, so we power cycled in the middle of
         * writing an entry to the area. Invalidate this entry if necessary,
         * and add a seal after it.
         */
        action_t * p_action = reserve_action_buffer(ACTION_BUFFER_SIZE_NO_PARAMS);

        if (p_action == NULL)
        {
            status = NRF_ERROR_NO_MEM;
        }
        else
        {
            p_action->action = ACTION_TYPE_RECOVER_SEAL;
            p_action->p_manager = p_manager;
            commit_action_buffer(p_action);

            schedule_processing();
        }
    }
    return status;
}

/** If the last entry is a duplicate of another entry, we should invalidate its duplicate, as it
 * got a power loss before doing the final step of invalidating in a replace action.
 */
static uint32_t invalidate_duplicate_of_last_entry(flash_manager_t * p_manager)
{
    uint32_t status = NRF_SUCCESS;
    /* get the last entry before the seal */
    const fm_entry_t * p_entry = get_first_entry(p_manager->config.p_area);
    if (p_manager->internal.p_seal <= p_entry)
    {
        /* there is no entry before the seal */
        return status;
    }
    const fm_entry_t * p_next = get_next_entry(p_entry);
    while (p_next != p_manager->internal.p_seal)
    {
        p_entry = p_next;
        p_next  = get_next_entry(p_next);
    }

    if (handle_represents_data(p_entry->header.handle))
    {
        const fm_entry_t * p_duplicate =
            entry_get(get_first_entry(p_manager->config.p_area), p_entry, p_entry->header.handle);

        if (p_duplicate != NULL && p_duplicate != p_entry)
        {
            status = flash_manager_entry_invalidate(p_manager, p_duplicate->header.handle);
        }
    }
    return status;
}

static uint32_t get_invalid_bytes(const flash_manager_page_t * p_area, uint32_t pages)
{
    uint32_t invalid_bytes = 0;
    const fm_entry_t * p_end = (fm_entry_t *) get_area_end(p_area);
    const fm_entry_t * p_entry = get_first_entry(p_area);

    while ((p_entry->header.handle != HANDLE_BLANK) && (p_entry->header.handle != HANDLE_SEAL))
    {
        if (p_entry->header.handle == FLASH_MANAGER_HANDLE_INVALID)
        {
            invalid_bytes += p_entry->header.len_words * WORD_SIZE;
        }
        p_entry = get_next_entry(p_entry);
        /* Check that we haven't gone out of bounds before finding the seal or a blank entry,
         * which means the area is invalid. */
        NRF_MESH_ASSERT(p_entry < p_end);
    }

    return invalid_bytes;
}

static bool validate_result(const action_t * p_action)
{
    uint32_t entry_length = p_action->params.entry_data.entry.header.len_words * WORD_SIZE;
    switch (p_action->action)
    {
        case ACTION_TYPE_REPLACE:
            return (memcmp(p_action->params.entry_data.p_target,
                           &p_action->params.entry_data.entry,
                           entry_length) == 0);
        case ACTION_TYPE_INVALIDATE:
            return (p_action->params.entry_data.p_target->header.handle ==
                    FLASH_MANAGER_HANDLE_INVALID);
        case ACTION_TYPE_BUILD_METADATA:
            return (memcmp(&p_action->p_manager->config.p_area[p_action->params.metadata.page_index].metadata,
                           &p_action->params.metadata,
                           sizeof(flash_manager_metadata_t)) == 0);
        default:
            return true;
    }
}

static void free_packet_buffer(packet_buffer_packet_t * p_packet_buffer)
{
    packet_buffer_free(&m_action_queue, p_packet_buffer);

    /* Notify all memory listeners. Run until we find the last entry, in case some of the entries
     * get re-added in their callbacks. */
    fm_mem_listener_t * p_back = (fm_mem_listener_t *) m_memory_listener_queue.p_back;
    fm_mem_listener_t * p_listener = NULL;
    do
    {
        p_listener = (fm_mem_listener_t *) queue_pop(&m_memory_listener_queue);
        if (p_listener != NULL)
        {
            p_listener->queue_elem.p_data = NULL;
            p_listener->callback(p_listener->p_args);
        }
        else
        {
            NRF_MESH_ASSERT(p_back == NULL);
        }
    } while (p_listener != p_back);
}

static void end_action(action_t * p_action, fm_result_t result, const fm_entry_t * p_entry)
{
    if (result == FM_RESULT_SUCCESS && !validate_result(p_action))
    {
        result = FM_RESULT_ERROR_FLASH_MALFUNCTION;
    }

    if (result != FM_RESULT_SUCCESS)
    {
        /* report in-RAM entry on failure: */
        p_entry = &p_action->params.entry_data.entry;
    }

    flash_manager_t * p_manager = p_action->p_manager;
    switch (p_action->action)
    {
        case ACTION_TYPE_REPLACE:
            if (result == FM_RESULT_SUCCESS)
            {
                /* Need to reset the seal */
                p_manager->internal.p_seal = get_next_entry(p_action->params.entry_data.p_target);
                NRF_MESH_ASSERT(p_manager->internal.p_seal->header.handle == HANDLE_SEAL);
            }
            if (p_manager->config.write_complete_cb != NULL)
            {
                p_manager->config.write_complete_cb(p_manager, p_entry, result);
            }
            break;
        case ACTION_TYPE_INVALIDATE:
            if (p_manager->config.invalidate_complete_cb != NULL)
            {
                p_manager->config.invalidate_complete_cb(p_manager,
                        p_action->params.entry_data.entry.header.handle,
                        result);
            }
            break;
        case ACTION_TYPE_BUILD_METADATA:
            NRF_MESH_ASSERT(p_manager->internal.state == FM_STATE_BUILDING);
            if (p_action->params.metadata.page_index == p_action->params.metadata.pages_in_area - 1)
            {
                /* Done building the metadata on the last page. */
                p_manager->internal.state = FM_STATE_READY;
            }
            break;
        case ACTION_TYPE_ERASE_AREA:
            NRF_MESH_ASSERT(result == FM_RESULT_SUCCESS);
            p_manager->internal.state = FM_STATE_UNINITIALIZED;
            if (p_manager->config.remove_complete_cb != NULL)
            {
                p_manager->config.remove_complete_cb(p_manager);
            }
            break;
        default:
            break;
    }

    free_packet_buffer(get_packet_buffer(p_action));
}

static bool defrag_required(action_t * p_next_action)
{
    if (p_next_action->action == ACTION_TYPE_REPLACE ||
        p_next_action->action == ACTION_TYPE_INVALIDATE)
    {
        int remaining_space = get_remaining_free_space(p_next_action->p_manager);
        int required_space;

        if (p_next_action->action == ACTION_TYPE_REPLACE)
        {
            required_space = p_next_action->params.entry_data.entry.header.len_words * WORD_SIZE;
        }
        else
        {
            required_space = 0;
        }

        if (remaining_space <= required_space &&
            p_next_action->p_manager->internal.invalid_bytes != 0)
        {
            return true;
        }
    }
    return false;
}
/******************************************************************************
* Action execution
******************************************************************************/
static fm_result_t execute_action_replace(action_t * p_action)
{
    const fm_entry_t * p_old_entry = entry_get(get_first_entry(p_action->p_manager->config.p_area),
                                               get_area_end(p_action->p_manager->config.p_area),
                                               p_action->params.entry_data.entry.header.handle);

    const flash_manager_page_t * p_next_page =
        (const flash_manager_page_t *) (PAGE_START_ALIGN(p_action->p_manager->internal.p_seal) + PAGE_SIZE);
    const fm_entry_t * p_new_seal = NULL;
    const fm_entry_t * p_new_entry = NULL;

    uint32_t remaining_space = ((uint32_t) p_next_page - (uint32_t) p_action->p_manager->internal.p_seal);

    if (remaining_space > (p_action->params.entry_data.entry.header.len_words * WORD_SIZE))
    {
        /* The entry and the seal can fit right after the previous entry. */
        p_new_entry = p_action->p_manager->internal.p_seal;
        p_new_seal = &p_action->p_manager->internal.p_seal[p_action->params.entry_data.entry.header.len_words];
    }
    else
    {
        if (p_next_page == get_area_end(p_action->p_manager->config.p_area))
        {
            /* No room for the packet */
            return FM_RESULT_ERROR_AREA_FULL;
        }

        if (remaining_space == (p_action->params.entry_data.entry.header.len_words * WORD_SIZE))
        {
            /* The entry can fills the page completely. Flash the seal on the next page */
            p_new_entry = p_action->p_manager->internal.p_seal;
            p_new_seal = get_first_entry(p_next_page);
        }
        else
        {
            /* The entry can't fit in the current page. Pad the page, and place the
             * entry and seal on the next. */
            NRF_MESH_ERROR_CHECK(flash(p_action->p_manager->internal.p_seal,
                        &PADDING_HEADER,
                        sizeof(PADDING_HEADER),
                        NULL));

            p_new_entry = get_first_entry(p_next_page);
            p_new_seal = p_new_entry + p_action->params.entry_data.entry.header.len_words;
        }
    }

    /* Flash the data */
    NRF_MESH_ERROR_CHECK(flash(p_new_entry,
                &p_action->params.entry_data.entry,
                p_action->params.entry_data.entry.header.len_words * WORD_SIZE,
                NULL));

    /* flash a seal at the end */
    NRF_MESH_ERROR_CHECK(flash(p_new_seal,
                &SEAL_HEADER,
                sizeof(SEAL_HEADER),
                &m_token));

    if (p_old_entry != NULL)
    {
        /* invalidate old entry */
        p_action->p_manager->internal.invalid_bytes += p_old_entry->header.len_words * WORD_SIZE;
        NRF_MESH_ERROR_CHECK(flash(p_old_entry,
                    &INVALID_HEADER,
                    sizeof(INVALID_HEADER),
                    &m_token));
    }

    p_action->params.entry_data.p_target = p_new_entry;
    return FM_RESULT_SUCCESS;
}

static fm_result_t execute_action_invalidate(action_t * p_action)
{
    const fm_entry_t * p_old_entry = entry_get(get_first_entry(p_action->p_manager->config.p_area),
                                               get_area_end(p_action->p_manager->config.p_area),
                                               p_action->params.entry_data.entry.header.handle);

    if (p_old_entry == NULL)
    {
        return FM_RESULT_ERROR_NOT_FOUND;
    }

    p_action->params.entry_data.p_target = p_old_entry;
    p_action->p_manager->internal.invalid_bytes += p_old_entry->header.len_words * WORD_SIZE;

    NRF_MESH_ERROR_CHECK(flash(p_old_entry,
                &INVALID_HEADER,
                sizeof(INVALID_HEADER),
                &m_token));
    return FM_RESULT_SUCCESS;
}

static fm_result_t execute_action_build_metadata(action_t * p_action)
{
    const flash_manager_page_t * p_page =
        &p_action->p_manager->config.p_area[p_action->params.metadata.page_index];

    /* Ensure that the metadata part of the page is clean or identical to what we are adding. */
    if (!flash_area_is_blank(&p_page->metadata, sizeof(flash_manager_metadata_t)))
    {
        /* It's not enough to memcmp the new and existing metadata, as the existing could be
         * partially written. */
        for (uint32_t i = 0; i < sizeof(flash_manager_metadata_t); ++i)
        {
            /* If this assert fails, we have two overlapping flash manager areas. */
            NRF_MESH_ASSERT(p_page->raw[i] == ((uint8_t *) &p_action->params.metadata)[i] ||
                            p_page->raw[i] == 0xFF);
        }
    }

    NRF_MESH_ERROR_CHECK(flash(p_page,
                               &p_action->params.metadata,
                               sizeof(flash_manager_metadata_t),
                               &m_token));
    return FM_RESULT_SUCCESS;
}

static fm_result_t execute_action_recover_seal(action_t * p_action)
{
    const fm_entry_t * p_last_entry = get_last_entry(p_action->p_manager);
    NRF_MESH_ASSERT(p_last_entry != NULL);

    bool last_entry_is_invalid = (p_last_entry->header.handle == FLASH_MANAGER_HANDLE_INVALID);

    if (!last_entry_is_invalid)
    {
        NRF_MESH_ERROR_CHECK(flash(&p_last_entry->header,
                    &INVALID_HEADER,
                    sizeof(INVALID_HEADER),
                    &m_token));
    }

    /* place seal at end */
    p_action->p_manager->internal.p_seal = get_next_entry(p_last_entry);
    if ((void *) p_action->p_manager->internal.p_seal < get_area_end(p_action->p_manager->config.p_area))
    {
        NRF_MESH_ERROR_CHECK(flash(get_next_entry(p_last_entry),
                    &SEAL_HEADER,
                    sizeof(SEAL_HEADER),
                    &m_token));
    }
    return FM_RESULT_SUCCESS;
}

static fm_result_t execute_action_erase_area(action_t * p_action)
{
    NRF_MESH_ERROR_CHECK(erase(p_action->p_manager->config.p_area,
                               p_action->p_manager->config.page_count * PAGE_SIZE,
                               &m_token));
    return FM_RESULT_SUCCESS;
}

static fm_result_t execute_action(action_t * p_action)
{
    __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_FM_ACTION, p_action->action, 0, NULL);
    switch (p_action->action)
    {
        case ACTION_TYPE_REPLACE:
            return execute_action_replace(p_action);
        case ACTION_TYPE_INVALIDATE:
            return execute_action_invalidate(p_action);
        case ACTION_TYPE_BUILD_METADATA:
            return execute_action_build_metadata(p_action);
        case ACTION_TYPE_RECOVER_SEAL:
            return execute_action_recover_seal(p_action);
        case ACTION_TYPE_ERASE_AREA:
            return execute_action_erase_area(p_action);
    }
    NRF_MESH_ASSERT(false);
    return FM_RESULT_SUCCESS;
}

static void process_action_queue(void)
{
    static action_t * p_current = NULL;
    static fm_result_t result;

    while (m_state == FM_STATE_READY)
    {
        switch (m_action_state)
        {
            case ACTION_STATE_IDLE:
            {
                packet_buffer_packet_t * p_buffer = NULL;
                if (packet_buffer_pop(&m_action_queue,
                            &p_buffer) != NRF_SUCCESS)
                {
                    return;
                }
                p_current = (action_t *) p_buffer->packet;
                m_action_state = ACTION_STATE_PROCESSING;
                if (defrag_required(p_current))
                {
                    /* do defrag, then come back once its finished */
                    m_state = FM_STATE_DEFRAG;
                    p_current->p_manager->internal.state = FM_STATE_DEFRAG;
                    flash_manager_defrag(p_current->p_manager);
                }
                break;
            }
            case ACTION_STATE_PROCESSING:
                result = execute_action(p_current);
                if (result == FM_RESULT_SUCCESS)
                {
                    m_action_state = ACTION_STATE_WAIT_FOR_FLASH;
                }
                else
                {
                    /* The action failed, and we should skip the WAIT_FOR_FLASH state. */
                    m_action_state = ACTION_STATE_DONE;
                }
                break;

            case ACTION_STATE_WAIT_FOR_FLASH:
                /* Exit the loop to let the flash process the action. */
                return;

            case ACTION_STATE_DONE:
                NRF_MESH_ASSERT(p_current != NULL);
                /* report to user */
                end_action(p_current,
                        result,
                        p_current->params.entry_data.p_target);
                m_action_state = ACTION_STATE_IDLE;
                break;

            default:
                break;
        }
    }
}

static void flash_op_ended_callback(mesh_flash_user_t user, const flash_operation_t * p_op, uint16_t token)
{
    if (token == m_token && m_action_state == ACTION_STATE_WAIT_FOR_FLASH)
    {
        /* we've received the last token for this action */
        m_action_state = ACTION_STATE_DONE;
        process_action_queue();
    }
}

/******************************************************************************
* Interface functions
******************************************************************************/

void flash_manager_init(void)
{
    packet_buffer_init(&m_action_queue, m_action_queue_buffer, sizeof(m_action_queue_buffer));
    mesh_flash_user_callback_set(MESH_FLASH_USER_MESH, flash_op_ended_callback);
    m_processing_flag = bearer_event_flag_add(process_action_queue);
    m_action_state = ACTION_STATE_IDLE;
    m_token = 0;
    queue_init(&m_memory_listener_queue);

    if (flash_manager_defrag_init())
    {
        m_state = FM_STATE_DEFRAG;
    }
    else
    {
        m_state = FM_STATE_READY;
    }
}

uint32_t flash_manager_add(flash_manager_t * p_manager,
        const flash_manager_config_t * p_config)
{
    uint32_t status = NRF_SUCCESS;
    p_manager->internal.state = FM_STATE_UNINITIALIZED;

    NRF_MESH_ASSERT(IS_PAGE_ALIGNED(p_config->p_area));
    NRF_MESH_ASSERT(p_config->page_count < FLASH_MANAGER_PAGE_COUNT_MAX);

    memcpy(&p_manager->config, p_config, sizeof(flash_manager_config_t));
    p_manager->internal.p_seal = NULL;
    p_manager->internal.invalid_bytes = 0;

    if (flash_area_is_valid(p_manager))
    {
        p_manager->internal.invalid_bytes = get_invalid_bytes(p_manager->config.p_area, p_manager->config.page_count);
        status = recover_seal(p_manager);
        if (status == NRF_SUCCESS)
        {
            p_manager->internal.state = FM_STATE_READY;
            status = invalidate_duplicate_of_last_entry(p_manager);
        }
    }
    else
    {
        p_manager->internal.state = FM_STATE_BUILDING;
        /* If the area build fails, it's because we can't fit all the metadata
         * in the action queue. Consider increasing its size. */
        status = flash_area_build(p_manager);
        if (status == NRF_SUCCESS)
        {
            p_manager->internal.p_seal =
                (const fm_entry_t *) &p_manager->config.p_area->raw[sizeof(flash_manager_metadata_t)];
        }
    }
    return status;
}

uint32_t flash_manager_remove(flash_manager_t * p_manager)
{
    action_t * p_action = reserve_action_buffer(ACTION_BUFFER_SIZE_NO_PARAMS);
    if (p_action == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }
    else
    {
        p_manager->internal.state = FM_STATE_REMOVING;
        p_action->action = ACTION_TYPE_ERASE_AREA;
        p_action->p_manager = p_manager;
        commit_action_buffer(p_action);
        schedule_processing();
        return NRF_SUCCESS;
    }
}

const fm_entry_t * flash_manager_entry_get(const flash_manager_t * p_manager, fm_handle_t handle)
{
    NRF_MESH_ASSERT(p_manager != NULL);
    if (!handle_represents_data(handle) || p_manager->internal.state != FM_STATE_READY)
    {
        return NULL;
    }
    return entry_get(get_first_entry(p_manager->config.p_area),
                     get_area_end(p_manager->config.p_area),
                     handle);
}

const fm_entry_t * flash_manager_entry_next_get(const flash_manager_t * p_manager,
        const fm_handle_filter_t * p_filter,
        const fm_entry_t * p_start)
{
    NRF_MESH_ASSERT(p_manager != NULL);
    if (p_manager->internal.state != FM_STATE_READY)
    {
        return NULL;
    }
    const fm_entry_t * p_entry;
    if (p_start == NULL)
    {
        p_entry = get_first_entry(p_manager->config.p_area);
    }
    else if (p_start < get_first_entry(p_manager->config.p_area) ||
             p_start > (const fm_entry_t *) get_area_end(p_manager->config.p_area))
    {
        return NULL;
    }
    else
    {
        p_entry = get_next_entry(p_start);
    }
    const fm_entry_t * p_end = get_area_end(p_manager->config.p_area);
    while (p_entry < p_end &&
           !handle_matches_filter(p_entry->header.handle, p_filter))
    {
        p_entry = get_next_entry(p_entry);
    }

    if (p_entry < p_end && handle_matches_filter(p_entry->header.handle, p_filter))
    {
        return p_entry;
    }
    else
    {
        return NULL;
    }
}

uint32_t flash_manager_entry_count_get(const flash_manager_t *    p_manager,
                                       const fm_handle_filter_t * p_filter)
{
    NRF_MESH_ASSERT(p_manager != NULL);
    if (p_manager->internal.state != FM_STATE_READY)
    {
        return 0;
    }
    const fm_entry_t * p_entry = get_first_entry(p_manager->config.p_area);
    const fm_entry_t * p_end   = get_area_end(p_manager->config.p_area);
    uint32_t count = 0;
    while (p_entry->header.handle != HANDLE_SEAL &&
            p_entry < p_end)
    {
        if (handle_matches_filter(p_entry->header.handle, p_filter))
        {
            count++;
        }
        p_entry = get_next_entry(p_entry);
    }
    return count;
}

fm_entry_t * flash_manager_entry_alloc(flash_manager_t * p_manager,
        fm_handle_t handle,
        uint32_t data_length)
{
    NRF_MESH_ASSERT(p_manager != NULL);
    NRF_MESH_ASSERT(handle_is_valid(handle));
    NRF_MESH_ASSERT(data_length <= FLASH_MANAGER_ENTRY_MAX_SIZE);

    if (p_manager->internal.state == FM_STATE_UNINITIALIZED)
    {
        return NULL;
    }

    uint32_t buffer_length = data_length + ACTION_BUFFER_SIZE_ENTRY_NO_DATA;

    action_t * p_action = reserve_action_buffer(buffer_length);
    if (p_action == NULL)
    {
        return NULL;
    }
    else
    {
        p_action->action = ACTION_TYPE_REPLACE;
        p_action->params.entry_data.entry.header.handle = handle;
        /* length should be in padded words: */
        p_action->params.entry_data.entry.header.len_words = 1 + ALIGN_VAL(data_length, WORD_SIZE) / WORD_SIZE;
        p_action->p_manager = p_manager;

        return &p_action->params.entry_data.entry;
    }
}

void flash_manager_entry_commit(const fm_entry_t * p_entry)
{
    commit_action_buffer(get_entry_action(p_entry));
    schedule_processing();
}

uint32_t flash_manager_entry_invalidate(flash_manager_t * p_manager, fm_handle_t handle)
{
    NRF_MESH_ASSERT(p_manager != NULL);
    NRF_MESH_ASSERT(handle_is_valid(handle));

    action_t * p_action = reserve_action_buffer(ACTION_BUFFER_SIZE_ENTRY_NO_DATA);
    if (p_action == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }
    else
    {
        p_action->action = ACTION_TYPE_INVALIDATE;
        p_action->params.entry_data.entry.header.handle = handle;
        p_action->params.entry_data.entry.header.len_words = 0xFFFF; /* do not alter length of the entry */
        p_action->p_manager = p_manager;
        commit_action_buffer(p_action);
        schedule_processing();
        return NRF_SUCCESS;
    }
}

void flash_manager_entry_release(fm_entry_t * p_entry)
{
    packet_buffer_packet_t * p_packet_buffer = get_packet_buffer(get_entry_action(p_entry));

    free_packet_buffer(p_packet_buffer);

}

void flash_manager_mem_listener_register(fm_mem_listener_t * p_listener)
{
    NRF_MESH_ASSERT(p_listener != NULL);
    NRF_MESH_ASSERT(p_listener->callback != NULL);
    if (p_listener->queue_elem.p_data == NULL)
    {
        p_listener->queue_elem.p_data = p_listener;
        queue_push(&m_memory_listener_queue, &p_listener->queue_elem);
    }
}

bool flash_manager_is_stable(void)
{
    return (!packet_buffer_can_pop(&m_action_queue) && !flash_manager_defrag_is_running());
}

void flash_manager_on_defrag_end(flash_manager_t * p_manager)
{
    if (p_manager != NULL)
    {
        p_manager->internal.p_seal = entry_get(get_first_entry(p_manager->config.p_area),
                                      get_area_end(p_manager->config.p_area),
                                      HANDLE_SEAL);
        NRF_MESH_ASSERT(p_manager->internal.p_seal != NULL);
        p_manager->internal.state = FM_STATE_READY;
        p_manager->internal.invalid_bytes = 0;
    }
    m_state = FM_STATE_READY;
    mesh_flash_user_callback_set(MESH_FLASH_USER_MESH, flash_op_ended_callback);
    schedule_processing();
}

const void * flash_manager_recovery_page_get(void)
{
    return flash_manager_defrag_recovery_page_get();
}
