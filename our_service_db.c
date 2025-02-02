/**
 * Copyright (c) 2012 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
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
 *
 */
#include "sdk_common.h"
#include "nrf_log.h"
#include "our_service_db.h"


typedef struct
{
    bool          in_use_flag;
    ble_os_rec_t record;
} database_entry_t;

static database_entry_t m_database[BLE_OS_DB_MAX_RECORDS];
static uint16_t          m_database_crossref[BLE_OS_DB_MAX_RECORDS];
static uint16_t         m_num_records;
#define TIME_BEFORE_TIME_UPDATE     (2*365*24*60*60)    // 2 years will be sufficient to determine whether it's an not-updated time


uint32_t ble_os_db_init(void)
{
    int i;

//    NRF_LOG_DEBUG("ble_os_db_init, sizeof(database_entry_t) = %d", sizeof(database_entry_t));

    for (i = 0; i < BLE_OS_DB_MAX_RECORDS; i++)
    {
        m_database[i].in_use_flag = false;
        m_database_crossref[i]    = 0xFF;
    }

    m_num_records = 0;

    return NRF_SUCCESS;
}

uint32_t ble_os_db_update_time_stamps(time_t timedelta)
{
    for (int i = 0; i < m_num_records; i++)
    {
        ASSERT(m_database[m_database_crossref[i]].record.meas.time_stamp < TIME_BEFORE_TIME_UPDATE);
        m_database[m_database_crossref[i]].record.meas.time_stamp += timedelta;
    }

    return NRF_SUCCESS;
}

uint16_t ble_os_db_num_records_get(void)
{
    return m_num_records;
}

uint16_t ble_os_db_num_free_entries_get(void)
{
    return BLE_OS_DB_MAX_RECORDS - m_num_records;
}


uint32_t ble_os_db_record_get(uint16_t rec_ndx, ble_os_rec_t * p_rec)
{
    if (rec_ndx >= m_num_records)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    // copy record to the specified memory
    *p_rec = m_database[m_database_crossref[rec_ndx]].record;

    return NRF_SUCCESS;
}

uint32_t ble_os_db_record_add(ble_os_rec_t * p_rec)
{
    int i;

    if (m_num_records == BLE_OS_DB_MAX_RECORDS)
    {
        return NRF_ERROR_NO_MEM;
    }

    // find next available database entry
    for (i = 0; i < BLE_OS_DB_MAX_RECORDS; i++)
    {
        if (!m_database[i].in_use_flag)
        {
            m_database[i].in_use_flag = true;
            m_database[i].record      = *p_rec;

            m_database_crossref[m_num_records] = i;
            m_num_records++;

            return NRF_SUCCESS;
        }
    }

    return NRF_ERROR_NO_MEM;
}


uint32_t ble_os_db_record_delete(uint16_t rec_ndx)
{
    int i;

    if (rec_ndx >= m_num_records)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    // free entry
    m_database[m_database_crossref[rec_ndx]].in_use_flag = false;

    // decrease number of records
    m_num_records--;

    // remove cross reference index
    for (i = rec_ndx; i < m_num_records; i++)
    {
        m_database_crossref[i] = m_database_crossref[i + 1];
    }

    return NRF_SUCCESS;
}
