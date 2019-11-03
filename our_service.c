/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
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

#include <stdint.h>
#include <string.h>
#include "ble.h"
#include "nrf_log.h"
#include "our_service.h"
#include "ble_srv_common.h"
#include "ble_racp.h"
#include "our_service_db.h"
#include "app_error.h"
#include "SEGGER_RTT.h"

// https://devzone.nordicsemi.com/f/nordic-q-a/45679/loss-of-notifications-when-sending-them-from-peripheral-to-central-as-bulk-transfer
// https://devzone.nordicsemi.com/f/nordic-q-a/25637/can-you-not-call-sd_ble_gatts_hvx-in-a-loop



#define OPERAND_FILTER_TYPE_SEQ_NUM     0x01                                     /**< Filter data using Sequence Number criteria. */
#define OPERAND_FILTER_TYPE_FACING_TIME 0x02                                     /**< Filter data using User Facing Time criteria. */
#define OPERAND_FILTER_TYPE_RFU_START   0x07                                     /**< Start of filter types reserved For Future Use range */
#define OPERAND_FILTER_TYPE_RFU_END     0xFF                                     /**< End of filter types reserved For Future Use range */

#define OPCODE_LENGTH 1                                                          /**< Length of opcode inside Our Service Measurement packet. */
#define HANDLE_LENGTH 2                                                          /**< Length of handle inside Our Service Measurement packet. */
#define MAX_OSM_LEN   (BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH) /**< Maximum size of a transmitted Our Service Measurement. */

#define OS_NACK_PROC_ALREADY_IN_PROGRESS   BLE_GATT_STATUS_ATTERR_APP_BEGIN + 0 /**< Reply when a requested procedure is already in progress. */
#define OS_NACK_CCCD_IMPROPERLY_CONFIGURED BLE_GATT_STATUS_ATTERR_APP_BEGIN + 1 /**< Reply when the a s CCCD is improperly configured. */

/**@brief Our Service communication state. */
typedef enum
{
    STATE_NO_COMM,                                                      /**< The service is not in a communicating state. */
    STATE_RACP_PROC_ACTIVE,                                             /**< Processing requested data. */
    STATE_RACP_RESPONSE_PENDING,                                        /**< There is a RACP indication waiting to be sent. */
    STATE_RACP_RESPONSE_IND_VERIF                                       /**< Waiting for a verification of a RACP indication. */
} os_state_t;

static os_state_t       m_os_state;                                     /**< Current communication state. */
static uint16_t         m_next_seq_num;                                 /**< Sequence number of the next database record. */
static uint8_t          m_racp_proc_operator;                           /**< Operator of current request. */
static uint16_t         m_racp_proc_seq_num;                            /**< Sequence number of current request. */
static uint8_t          m_racp_proc_record_ndx;                         /**< Current record index. */
static uint8_t          m_racp_proc_records_reported;                   /**< Number of reported records. */
static uint8_t          m_racp_proc_records_reported_since_txcomplete;  /**< Number of reported records since last TX_COMPLETE event. */
static ble_racp_value_t m_pending_racp_response;                        /**< RACP response to be sent. */
static uint8_t          m_pending_racp_response_operand[2];             /**< Operand of RACP response to be sent. */



/**@brief Function for setting the OS communication state.
 *
 * @param[in] new_state  New communication state.
 */
static void state_set(os_state_t new_state)
{
    m_os_state = new_state;
}


/**@brief Function for setting the next sequence number by reading the last record in the data base.
 *
 * @return NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
static uint32_t next_sequence_number_set(void)
{
    uint16_t      num_records;
    ble_os_rec_t rec;

    num_records = ble_os_db_num_records_get();
    if (num_records > 0)
    {
        // Get last record
        uint32_t err_code = ble_os_db_record_get(num_records - 1, &rec);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
        m_next_seq_num = rec.meas.sequence_number + 1;
    }
    else
    {
        m_next_seq_num = 0;
    }

    return NRF_SUCCESS;
}


/**@brief Function for encoding a Our Service measurement.
 *
 * @param[in]  p_meas            Measurement to be encoded.
 * @param[out] p_encoded_buffer  Pointer to buffer where the encoded measurement is to be stored.
 *
 * @return Size of encoded measurement.
 */
static uint8_t os_meas_encode(const ble_os_meas_t * p_meas, uint8_t * p_encoded_buffer)
{
    uint8_t len = 0;
// TODO
/*
    p_encoded_buffer[len++] = p_meas->flags;

    len += uint16_encode(p_meas->sequence_number, &p_encoded_buffer[len]);
    len += ble_date_time_encode(&p_meas->base_time, &p_encoded_buffer[len]);

    if (p_meas->flags & BLE_GLS_MEAS_FLAG_TIME_OFFSET)
    {
        len += uint16_encode(p_meas->time_offset, &p_encoded_buffer[len]);
    }

    if (p_meas->flags & BLE_GLS_MEAS_FLAG_CONC_TYPE_LOC)
    {
        uint16_t encoded_concentration;

        encoded_concentration = ((p_meas->glucose_concentration.exponent << 12) & 0xF000) |
                                ((p_meas->glucose_concentration.mantissa <<  0) & 0x0FFF);

        p_encoded_buffer[len++] = (uint8_t)(encoded_concentration);
        p_encoded_buffer[len++] = (uint8_t)(encoded_concentration >> 8);
        p_encoded_buffer[len++] = (p_meas->sample_location << 4) | (p_meas->type & 0x0F);
    }

    if (p_meas->flags & BLE_GLS_MEAS_FLAG_SENSOR_STATUS)
    {
        len += uint16_encode(p_meas->sensor_status_annunciation, &p_encoded_buffer[len]);
    }
*/
    return len;
}


uint32_t ble_os_init(ble_os_t * p_os, const ble_os_init_t * p_os_init)
{
    uint32_t                err_code;
    uint8_t                 num_recs;
    uint8_t                 init_value_encoded[MAX_OSM_LEN];
    ble_uuid128_t           base_uuid = BLE_UUID_OUR_BASE_UUID; // *
    ble_uuid_t              ble_uuid;
    ble_add_char_params_t   add_char_params;
    ble_os_rec_t            initial_os_rec_value;

    // Initialize data base
    err_code = ble_os_db_init();
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = next_sequence_number_set();
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }


    // Initialize service structure
    p_os->evt_handler          = p_os_init->evt_handler;
    p_os->error_handler        = p_os_init->error_handler;
    p_os->feature              = p_os_init->feature;
    p_os->conn_handle          = BLE_CONN_HANDLE_INVALID;


    // Initialize global variables
    state_set(STATE_NO_COMM);
    m_racp_proc_records_reported_since_txcomplete = 0;

    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_OUR_SERVICE);
    err_code = sd_ble_uuid_vs_add(&base_uuid, &ble_uuid.type);
    APP_ERROR_CHECK(err_code);    

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_os->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add Our Service measurement characteristic
    memset(&add_char_params, 0, sizeof(add_char_params));
    memset(&initial_os_rec_value, 0, sizeof(initial_os_rec_value));

    num_recs = ble_os_db_num_records_get();
    if (num_recs > 0)
    {
        err_code = ble_os_db_record_get(num_recs - 1, &initial_os_rec_value);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }

    add_char_params.uuid              = BLE_UUID_OUR_SERVICE_MEASUREMENT_CHAR;
    add_char_params.max_len           = MAX_OSM_LEN;
    add_char_params.init_len          = os_meas_encode(&initial_os_rec_value.meas, init_value_encoded);
    add_char_params.is_var_len        = true;
    add_char_params.char_props.notify = 1;
    add_char_params.cccd_write_access = p_os_init->os_meas_cccd_wr_sec;
    add_char_params.p_init_value      = init_value_encoded;

    err_code = characteristic_add(p_os->service_handle, &add_char_params, &p_os->osm_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add Our Service measurement feature characteristic
    memset(&add_char_params, 0, sizeof(add_char_params));

    add_char_params.uuid              = BLE_UUID_OUR_SERVICE_FEATURE_CHAR;
    add_char_params.max_len           = sizeof (uint16_t);
    add_char_params.init_len          = uint16_encode(p_os->feature, init_value_encoded);
    add_char_params.p_init_value      = init_value_encoded;
    add_char_params.char_props.read   = 1;
    add_char_params.read_access       = p_os_init->os_feature_rd_sec;

    err_code = characteristic_add(p_os->service_handle, &add_char_params, &p_os->osf_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add record control access point characteristic
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid                = BLE_UUID_RECORD_ACCESS_CONTROL_POINT_CHAR;
    add_char_params.max_len             = BLE_GATT_ATT_MTU_DEFAULT;
    add_char_params.is_var_len          = true;
    add_char_params.char_props.indicate = 1;
    add_char_params.char_props.write    = 1;
    add_char_params.cccd_write_access   = p_os_init->racp_cccd_wr_sec;
    add_char_params.write_access        = p_os_init->racp_wr_sec;
    add_char_params.is_defered_write    = true;

    err_code = characteristic_add(p_os->service_handle,
                                  &add_char_params,
                                  &p_os->racp_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}



/**@brief Function for sending a response from the Record Access Control Point.
 *
 * @param[in] p_os       Service instance.
 * @param[in] p_racp_val  RACP value to be sent.
 */
static void racp_send(ble_os_t * p_os, ble_racp_value_t * p_racp_val)
{
    uint32_t               err_code;
    uint8_t                encoded_resp[25];
    uint8_t                len;
    uint16_t               hvx_len;
    ble_gatts_hvx_params_t hvx_params;

    if (
        (m_os_state != STATE_RACP_RESPONSE_PENDING)
        &&
        (m_racp_proc_records_reported_since_txcomplete > 0)
       )
    {
        state_set(STATE_RACP_RESPONSE_PENDING);
        return;
    }

    // Send indication
    len     = ble_racp_encode(p_racp_val, encoded_resp);
    hvx_len = len;

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_os->racp_handles.value_handle;
    hvx_params.type   = BLE_GATT_HVX_INDICATION;
    hvx_params.offset = 0;
    hvx_params.p_len  = &hvx_len;
    hvx_params.p_data = encoded_resp;

    err_code = sd_ble_gatts_hvx(p_os->conn_handle, &hvx_params);

    // Error handling
    if ((err_code == NRF_SUCCESS) && (hvx_len != len))
    {
        err_code = NRF_ERROR_DATA_SIZE;
    }
    switch (err_code)
    {
        case NRF_SUCCESS:
            // Wait for HVC event
            state_set(STATE_RACP_RESPONSE_IND_VERIF);
            break;

        case NRF_ERROR_BUSY:
            // Wait for BLE_GATTS_EVT_HVC event to retry transmission
            state_set(STATE_RACP_RESPONSE_PENDING);
            break;

        case NRF_ERROR_INVALID_STATE:
            // Make sure state machine returns to the default state
            state_set(STATE_NO_COMM);
            break;

        default:
            // Report error to application
            if (p_os->error_handler != NULL)
            {
                p_os->error_handler(err_code);
            }

            // Make sure state machine returns to the default state
            state_set(STATE_NO_COMM);
            break;
    }
}


/**@brief Function for sending a RACP response containing a Response Code Op Code and a Response Code Value.
 *
 * @param[in] p_os   Service instance.
 * @param[in] opcode  RACP Op Code.
 * @param[in] value   RACP Response Code Value.
 */
static void racp_response_code_send(ble_os_t * p_os, uint8_t opcode, uint8_t value)
{
    m_pending_racp_response.opcode      = RACP_OPCODE_RESPONSE_CODE;
    m_pending_racp_response.operator    = RACP_OPERATOR_NULL;
    m_pending_racp_response.operand_len = 2;
    m_pending_racp_response.p_operand   = m_pending_racp_response_operand;

    m_pending_racp_response_operand[0] = opcode;
    m_pending_racp_response_operand[1] = value;

    racp_send(p_os, &m_pending_racp_response);
}



/**@brief Function for sending a Our Service measurement.
 *
 * @param[in] p_os   Service instance.
 * @param[in] p_rec  Measurement to be sent.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t our_service_meas_send(ble_os_t * p_os, ble_os_rec_t * p_rec)
{
    uint32_t               err_code;
    uint8_t                encoded_osm[MAX_OSM_LEN];
    uint16_t               len;
    uint16_t               hvx_len;
    ble_gatts_hvx_params_t hvx_params;

    len     = os_meas_encode(&p_rec->meas, encoded_osm);
    hvx_len = len;

    memset(&hvx_params, 0, sizeof (hvx_params));

    hvx_params.handle = p_os->osm_handles.value_handle;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.offset = 0;
    hvx_params.p_len  = &hvx_len;
    hvx_params.p_data = encoded_osm;

    err_code = sd_ble_gatts_hvx(p_os->conn_handle, &hvx_params);
    if (err_code == NRF_SUCCESS)
    {
        if (hvx_len != len)
        {
            err_code = NRF_ERROR_DATA_SIZE;
        }
        else
        {
            // Measurement successfully sent
            m_racp_proc_records_reported++;
            m_racp_proc_records_reported_since_txcomplete++;
        }
    }

    return err_code;
}


/**@brief Function for responding to the ALL operation.
 *
 * @param[in] p_os  Service instance.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t racp_report_records_all(ble_os_t * p_os)
{
    uint16_t total_records = ble_os_db_num_records_get();

    if (m_racp_proc_record_ndx >= total_records)
    {
        state_set(STATE_NO_COMM);
    }
    else
    {
        uint32_t      err_code;
        ble_os_rec_t rec;

        err_code = ble_os_db_record_get(m_racp_proc_record_ndx, &rec);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }

        err_code = our_service_meas_send(p_os, &rec);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }

    return NRF_SUCCESS;
}


/**@brief Function for responding to the FIRST or the LAST operation.
 *
 * @param[in] p_os  Service instance.
 *
 * @return  NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t racp_report_records_first_last(ble_os_t * p_os)
{
    uint32_t      err_code;
    ble_os_rec_t rec;
    uint16_t      total_records;

    total_records = ble_os_db_num_records_get();

    if ((m_racp_proc_records_reported != 0) || (total_records == 0))
    {
        state_set(STATE_NO_COMM);
    }
    else
    {
        if (m_racp_proc_operator == RACP_OPERATOR_FIRST)
        {
            err_code = ble_os_db_record_get(0, &rec);
            if (err_code != NRF_SUCCESS)
            {
                return err_code;
            }
        }
        else if (m_racp_proc_operator == RACP_OPERATOR_LAST)
        {
            err_code = ble_os_db_record_get(total_records - 1, &rec);
            if (err_code != NRF_SUCCESS)
            {
                return err_code;
            }
        }

        err_code = our_service_meas_send(p_os, &rec);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }

    return NRF_SUCCESS;
}


/**@brief Function for responding to the GREATER_OR_EQUAL operation.
 *
 * @param[in] p_os  Service instance.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t racp_report_records_greater_or_equal(ble_os_t * p_os)
{
    uint16_t total_records = ble_os_db_num_records_get();

    while (m_racp_proc_record_ndx < total_records)
    {
        uint32_t      err_code;
        ble_os_rec_t rec;

        err_code = ble_os_db_record_get(m_racp_proc_record_ndx, &rec);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }

        if (rec.meas.sequence_number >= m_racp_proc_seq_num)
        {
            err_code = our_service_meas_send(p_os, &rec);
            if (err_code != NRF_SUCCESS)
            {
                return err_code;
            }
            break;
        }
        m_racp_proc_record_ndx++;
    }
    if (m_racp_proc_record_ndx == total_records)
    {
        state_set(STATE_NO_COMM);
    }

    return NRF_SUCCESS;
}


/**@brief Function for informing that the REPORT RECORDS procedure is completed.
 *
 * @param[in] p_os  Service instance.
 */
static void racp_report_records_completed(ble_os_t * p_os)
{
    uint8_t resp_code_value;

    if (m_racp_proc_records_reported > 0)
    {
        resp_code_value = RACP_RESPONSE_SUCCESS;
    }
    else
    {
        resp_code_value = RACP_RESPONSE_NO_RECORDS_FOUND;
    }

    racp_response_code_send(p_os, RACP_OPCODE_REPORT_RECS, resp_code_value);
}


/**@brief Function for the RACP report records procedure.
 *
 * @param[in] p_os  Service instance.
 */
static void racp_report_records_procedure(ble_os_t * p_os)
{
    uint32_t err_code;

    while (m_os_state == STATE_RACP_PROC_ACTIVE)
    {
        // Execute requested procedure
        switch (m_racp_proc_operator)
        {
            case RACP_OPERATOR_ALL:
                err_code = racp_report_records_all(p_os);
                break;

            case RACP_OPERATOR_FIRST:
            case RACP_OPERATOR_LAST:
                err_code = racp_report_records_first_last(p_os);
                break;

            case RACP_OPERATOR_GREATER_OR_EQUAL:
                err_code = racp_report_records_greater_or_equal(p_os);
                break;

            default:
                // Report error to application
                if (p_os->error_handler != NULL)
                {
                    p_os->error_handler(NRF_ERROR_INTERNAL);
                }

                // Make sure state machine returns to the default state
                state_set(STATE_NO_COMM);
                return;
        }

        // Error handling
        switch (err_code)
        {
            case NRF_SUCCESS:
                if (m_os_state == STATE_RACP_PROC_ACTIVE)
                {
                    m_racp_proc_record_ndx++;
                }
                else
                {
                    racp_report_records_completed(p_os);
                }
                break;

            case NRF_ERROR_RESOURCES:
                // Wait for TX_COMPLETE event to resume transmission
                return;

            case NRF_ERROR_INVALID_STATE:
                // Notification is probably not enabled. Ignore request.
                state_set(STATE_NO_COMM);
                return;

            default:
                // Report error to application
                if (p_os->error_handler != NULL)
                {
                    p_os->error_handler(err_code);
                }

                // Make sure state machine returns to the default state
                state_set(STATE_NO_COMM);
                return;
        }
    }
}


/**@brief Function for testing if the received request is to be executed.
 *
 * @param[in]  p_racp_request   Request to be checked.
 * @param[out] p_response_code  Response code to be sent in case the request is rejected.
 *                              RACP_RESPONSE_RESERVED is returned if the received message is
 *                              to be rejected without sending a response.
 *
 * @return TRUE if the request is to be executed, FALSE if it is to be rejected.
 *         If it is to be rejected, p_response_code will contain the response code to be
 *         returned to the central.
 */
static bool is_request_to_be_executed(ble_racp_value_t const * p_racp_request,
                                      uint8_t                * p_response_code)
{
    *p_response_code = RACP_RESPONSE_RESERVED;

    if (p_racp_request->opcode == RACP_OPCODE_ABORT_OPERATION)
    {
        if (m_os_state == STATE_RACP_PROC_ACTIVE)
        {
            if (p_racp_request->operator != RACP_OPERATOR_NULL)
            {
                *p_response_code = RACP_RESPONSE_INVALID_OPERATOR;
            }
            else if (p_racp_request->operand_len != 0)
            {
                *p_response_code = RACP_RESPONSE_INVALID_OPERAND;
            }
            else
            {
                *p_response_code = RACP_RESPONSE_SUCCESS;
            }
        }
        else
        {
            *p_response_code = RACP_RESPONSE_ABORT_FAILED;
        }
    }
    else if (m_os_state != STATE_NO_COMM)
    {
        return false;
    }
    // Supported opcodes.
    else if ((p_racp_request->opcode == RACP_OPCODE_REPORT_RECS) ||
             (p_racp_request->opcode == RACP_OPCODE_REPORT_NUM_RECS))
    {
        switch (p_racp_request->operator)
        {
            // Operators WITHOUT a filter.
            case RACP_OPERATOR_ALL:
            case RACP_OPERATOR_FIRST:
            case RACP_OPERATOR_LAST:
                if (p_racp_request->operand_len != 0)
                {
                    *p_response_code = RACP_RESPONSE_INVALID_OPERAND;
                }
                break;

            // Operators WITH a filter.
            case RACP_OPERATOR_GREATER_OR_EQUAL:
                if (p_racp_request->p_operand[0] == OPERAND_FILTER_TYPE_SEQ_NUM)
                {
                    if (p_racp_request->operand_len != 3)
                    {
                        *p_response_code = RACP_RESPONSE_INVALID_OPERAND;
                    }
                }
                else if (p_racp_request->p_operand[0] == OPERAND_FILTER_TYPE_FACING_TIME)
                {
                    *p_response_code = RACP_RESPONSE_OPERAND_UNSUPPORTED;
                }
                else if (p_racp_request->p_operand[0] >= OPERAND_FILTER_TYPE_RFU_START)
                {
                    *p_response_code = RACP_RESPONSE_OPERAND_UNSUPPORTED;
                }
                else
                {
                    *p_response_code = RACP_RESPONSE_INVALID_OPERAND;
                }
                break;

            // Unsupported operators.
            case RACP_OPERATOR_LESS_OR_EQUAL:
            case RACP_OPERATOR_RANGE:
                *p_response_code = RACP_RESPONSE_OPERATOR_UNSUPPORTED;
                 break;

            // Invalid operators.
            case RACP_OPERATOR_NULL:
            default:
                if (p_racp_request->operator >= RACP_OPERATOR_RFU_START)
                {
                    *p_response_code = RACP_RESPONSE_OPERATOR_UNSUPPORTED;
                }
                else
                {
                    *p_response_code = RACP_RESPONSE_INVALID_OPERATOR;
                }
                break;
        }
    }
    // Unsupported opcodes,
    else if (p_racp_request->opcode == RACP_OPCODE_DELETE_RECS)
    {
        *p_response_code = RACP_RESPONSE_OPCODE_UNSUPPORTED;
    }
    // Unknown opcodes.
    else
    {
        *p_response_code = RACP_RESPONSE_OPCODE_UNSUPPORTED;
    }

    // NOTE: The computation of the return value will change slightly when deferred write has been
    //       implemented in the stack.
    return (*p_response_code == RACP_RESPONSE_RESERVED);
}


/**@brief Function for processing a REPORT RECORDS request.
 *
 * @param[in] p_os           Service instance.
 * @param[in] p_racp_request  Request to be executed.
 */
static void report_records_request_execute(ble_os_t * p_os, ble_racp_value_t * p_racp_request)
{
    uint16_t seq_num = (p_racp_request->p_operand[2] << 8) | p_racp_request->p_operand[1];

    state_set(STATE_RACP_PROC_ACTIVE);

    m_racp_proc_record_ndx       = 0;
    m_racp_proc_operator         = p_racp_request->operator;
    m_racp_proc_records_reported = 0;
    m_racp_proc_seq_num          = seq_num;

    racp_report_records_procedure(p_os);
}


/**@brief Function for processing a REPORT NUM RECORDS request.
 *
 * @param[in] p_os           Service instance.
 * @param[in] p_racp_request  Request to be executed.
 */
static void report_num_records_request_execute(ble_os_t * p_os, ble_racp_value_t * p_racp_request)
{
    uint16_t total_records;
    uint16_t num_records;

    total_records = ble_os_db_num_records_get();
    num_records   = 0;

    if (p_racp_request->operator == RACP_OPERATOR_ALL)
    {
        num_records = total_records;
    }
    else if (p_racp_request->operator == RACP_OPERATOR_GREATER_OR_EQUAL)
    {
        uint16_t seq_num;
        uint16_t i;

        seq_num = (p_racp_request->p_operand[2] << 8) | p_racp_request->p_operand[1];

        for (i = 0; i < total_records; i++)
        {
            uint32_t      err_code;
            ble_os_rec_t rec;

            err_code = ble_os_db_record_get(i, &rec);
            if (err_code != NRF_SUCCESS)
            {
                if (p_os->error_handler != NULL)
                {
                    p_os->error_handler(err_code);
                }
                return;
            }

            if (rec.meas.sequence_number >= seq_num)
            {
                num_records++;
            }
        }
    }
    else if ((p_racp_request->operator == RACP_OPERATOR_FIRST) ||
             (p_racp_request->operator == RACP_OPERATOR_LAST))
    {
        if (total_records > 0)
        {
            num_records = 1;
        }
    }

    m_pending_racp_response.opcode      = RACP_OPCODE_NUM_RECS_RESPONSE;
    m_pending_racp_response.operator    = RACP_OPERATOR_NULL;
    m_pending_racp_response.operand_len = sizeof(uint16_t);
    m_pending_racp_response.p_operand   = m_pending_racp_response_operand;

    m_pending_racp_response_operand[0] = num_records & 0xFF;
    m_pending_racp_response_operand[1] = num_records >> 8;

    racp_send(p_os, &m_pending_racp_response);
}


/**@brief Function for checking if the CCCDs are configured.
 *
 * @param[in] p_os                  Service instance.
 * @param[in] p_are_cccd_configured  boolean indicating if both cccds are configured
 */
uint32_t ble_os_are_cccd_configured(ble_os_t * p_os, bool * p_are_cccd_configured)
{
    uint32_t err_code;
    uint8_t  cccd_value_buf[BLE_CCCD_VALUE_LEN];
    bool     is_osm_notif_enabled  = false;
    bool     is_racp_indic_enabled = false;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = BLE_CCCD_VALUE_LEN;
    gatts_value.offset  = 0;
    gatts_value.p_value = cccd_value_buf;

    err_code = sd_ble_gatts_value_get(p_os->conn_handle,
                                      p_os->osm_handles.cccd_handle,
                                      &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    is_osm_notif_enabled = ble_srv_is_notification_enabled(cccd_value_buf);

    err_code = sd_ble_gatts_value_get(p_os->conn_handle,
                                      p_os->racp_handles.cccd_handle,
                                      &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    is_racp_indic_enabled = ble_srv_is_indication_enabled(cccd_value_buf);
    if (is_racp_indic_enabled & is_osm_notif_enabled)
    {
        *p_are_cccd_configured = true;
    }
    else
    {
        *p_are_cccd_configured = false;
    }
    return NRF_SUCCESS;
}


/**@brief Function for handling a write event to the Record Access Control Point.
 *
 * @param[in] p_os        Service instance.
 * @param[in] p_evt_write  WRITE event to be handled.
 */
static void on_racp_value_write(ble_os_t * p_os, ble_gatts_evt_write_t const * p_evt_write)
{
    ble_racp_value_t                      racp_request;
    uint8_t                               response_code;
    ble_gatts_rw_authorize_reply_params_t auth_reply;
    bool                                  are_cccd_configured;
    uint32_t                              err_code;

    auth_reply.type                = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
    auth_reply.params.write.offset = 0;
    auth_reply.params.write.len    = 0;
    auth_reply.params.write.p_data = NULL;

    err_code = ble_os_are_cccd_configured(p_os, &are_cccd_configured);
    if (err_code != NRF_SUCCESS)
    {
        if (p_os->error_handler != NULL)
        {
            p_os->error_handler(err_code);
        }
        return;
    }

    if (!are_cccd_configured)
    {
        auth_reply.params.write.gatt_status = OS_NACK_CCCD_IMPROPERLY_CONFIGURED;
        err_code                            = sd_ble_gatts_rw_authorize_reply(p_os->conn_handle,
                                                                              &auth_reply);

        if (err_code != NRF_SUCCESS)
        {
            if (p_os->error_handler != NULL)
            {
                p_os->error_handler(err_code);
            }
        }
        return;
    }

    // Decode request.
    ble_racp_decode(p_evt_write->len, (uint8_t*)p_evt_write->data, &racp_request);

    // Check if request is to be executed.
    if (is_request_to_be_executed(&racp_request, &response_code))
    {
        auth_reply.params.write.gatt_status = BLE_GATT_STATUS_SUCCESS;
        auth_reply.params.write.update      = 1;

        err_code = sd_ble_gatts_rw_authorize_reply(p_os->conn_handle,
                                                   &auth_reply);

        if (err_code != NRF_SUCCESS)
        {
            if (p_os->error_handler != NULL)
            {
                p_os->error_handler(err_code);
            }
            return;
        }
        // Execute request.
        if (racp_request.opcode == RACP_OPCODE_REPORT_RECS)
        {
            report_records_request_execute(p_os, &racp_request);
        }
        else if (racp_request.opcode == RACP_OPCODE_REPORT_NUM_RECS)
        {
            report_num_records_request_execute(p_os, &racp_request);
        }
    }
    else if (response_code != RACP_RESPONSE_RESERVED)
    {
        auth_reply.params.write.gatt_status = BLE_GATT_STATUS_SUCCESS;
        auth_reply.params.write.update      = 1;
        err_code                            = sd_ble_gatts_rw_authorize_reply(p_os->conn_handle,
                                                                              &auth_reply);

        if (err_code != NRF_SUCCESS)
        {
            if (p_os->error_handler != NULL)
            {
                p_os->error_handler(err_code);
            }
            return;
        }

        // Abort any running procedure.
        state_set(STATE_NO_COMM);

        // Respond with error code.
        racp_response_code_send(p_os, racp_request.opcode, response_code);
    }
    else
    {
        auth_reply.params.write.gatt_status = OS_NACK_PROC_ALREADY_IN_PROGRESS;
        err_code                            = sd_ble_gatts_rw_authorize_reply(p_os->conn_handle,
                                                                              &auth_reply);

        if (err_code != NRF_SUCCESS)
        {
            if (p_os->error_handler != NULL)
            {
                p_os->error_handler(err_code);
            }
            return;
        }
    }
}


/**@brief Function for handling the Our Service measurement CCCD write event.
 *
 * @param[in] p_os        Service instance.
 * @param[in] p_evt_write  WRITE event to be handled.
 */
static void on_osm_cccd_write(ble_os_t * p_os, ble_gatts_evt_write_t const * p_evt_write)
{
    if (p_evt_write->len == 2)
    {
        // CCCD written, update notification state
        ble_os_evt_t evt;

        if (ble_srv_is_notification_enabled(p_evt_write->data))
        {
            evt.evt_type = BLE_OS_EVT_NOTIFICATION_ENABLED;
        }
        else
        {
            evt.evt_type = BLE_OS_EVT_NOTIFICATION_DISABLED;
        }

        if (p_os->evt_handler != NULL)
        {
            p_os->evt_handler(p_os, &evt);
        }
    }
}


/**@brief Function for handling the WRITE event.
 *
 * @details Handles WRITE events from the BLE stack.
 *
 * @param[in] p_os       Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_write(ble_os_t * p_os, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_evt_write->handle == p_os->osm_handles.cccd_handle)
    {
        on_osm_cccd_write(p_os, p_evt_write);
    }
    else if (p_evt_write->handle == p_os->racp_handles.value_handle)
    {
        on_racp_value_write(p_os, p_evt_write);
    }
}


/**@brief Function for handling the TX_COMPLETE event.
 *
 * @details Handles TX_COMPLETE events from the BLE stack.
 *
 * @param[in] p_os       Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_tx_complete(ble_os_t * p_os, ble_evt_t const * p_ble_evt)
{
    m_racp_proc_records_reported_since_txcomplete = 0;

    if (m_os_state == STATE_RACP_RESPONSE_PENDING)
    {
        racp_send(p_os, &m_pending_racp_response);
    }
    else if (m_os_state == STATE_RACP_PROC_ACTIVE)
    {
        racp_report_records_procedure(p_os);
    }
}


/**@brief Function for handling the HVC event.
 *
 * @details Handles HVC events from the BLE stack.
 *
 * @param[in] p_os       Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_hvc(ble_os_t * p_os, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_hvc_t const * p_hvc = &p_ble_evt->evt.gatts_evt.params.hvc;

    if (p_hvc->handle == p_os->racp_handles.value_handle)
    {
        if (m_os_state == STATE_RACP_RESPONSE_IND_VERIF)
        {
            // Indication has been acknowledged. Return to default state.
            state_set(STATE_NO_COMM);
        }
        else if (m_os_state == STATE_RACP_RESPONSE_PENDING)
        {
            racp_send(p_os, &m_pending_racp_response);
        }
        else
        {
            // We did not expect this event in this state. Report error to application.
            if (p_os->error_handler != NULL)
            {
                p_os->error_handler(NRF_ERROR_INVALID_STATE);
            }
        }
    }
}


static void on_rw_authorize_request(ble_os_t * p_os, ble_gatts_evt_t const * p_gatts_evt)
{
    ble_gatts_evt_rw_authorize_request_t const * p_auth_req =
        &p_gatts_evt->params.authorize_request;

    if (p_auth_req->type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
    {
        if (   (p_gatts_evt->params.authorize_request.request.write.op
                != BLE_GATTS_OP_PREP_WRITE_REQ)
            && (p_gatts_evt->params.authorize_request.request.write.op
                != BLE_GATTS_OP_EXEC_WRITE_REQ_NOW)
            && (p_gatts_evt->params.authorize_request.request.write.op
                != BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL)
           )
        {
            if (p_auth_req->request.write.handle == p_os->racp_handles.value_handle)
            {
                on_racp_value_write(p_os, &p_auth_req->request.write);
            }
        }
    }
}

void ble_os_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_os_t * p_os = (ble_os_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            p_os->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            state_set(STATE_NO_COMM);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            p_os->conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_os, p_ble_evt);
            break;

        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
            on_tx_complete(p_os, p_ble_evt);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
            on_rw_authorize_request(p_os, &p_ble_evt->evt.gatts_evt);
            break;

        case BLE_GATTS_EVT_HVC:
            on_hvc(p_os, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


uint32_t ble_os_new_meas(ble_os_t * p_os, ble_os_rec_t * p_rec)
{
    p_rec->meas.sequence_number = m_next_seq_num++;
    return ble_os_db_record_add(p_rec);
}




// TODO


//
///**@brief Function for handling Our Service event.
// *
// * @param[in]   p_our_service   Our Service structure.
// * @param[in]   p_ble_evt       Event received from the BLE stack.
// */
//static void on_our_service_evt_handler(ble_os_t * p_our_service, ble_os_evt_t * p_ble_evt)
//{
//    NRF_LOG_DEBUG("Our Service event handler called with event 0x%X", p_ble_evt->evt_type);
//
//    // Implement switch case handling BLE events related to our service. 
//    switch (p_ble_evt->evt_type)
//    {
//        case BLE_OS_EVT_NOTIFICATION_ENABLED:
//            NRF_LOG_DEBUG("on_our_service_evt_handler: BLE_OS_EVT_NOTIFICATION_ENABLED");
//            break;
//
//        case BLE_OS_EVT_NOTIFICATION_DISABLED:
//            NRF_LOG_DEBUG("on_our_service_evt_handler: BLE_OS_EVT_NOTIFICATION_DISABLED");
//            break;
//
//        default:
//            // No implementation needed.
//            break;
//    }		
//}


//
//
///**@brief Function for handling write events to the Our Service characteristic.
// *
// * @param[in]   p_our_service   Our Service structure.
// * @param[in]   p_evt_write     Write event received from the BLE stack.
// */
//static void on_our_service_cccd_write(ble_os_t * p_our_service, ble_gatts_evt_write_t const * p_evt_write)
//{
//    if (p_evt_write->len == 2)
//    {
//        // CCCD written, update notification state
//        if (p_our_service->evt_handler != NULL)
//        {
//            ble_os_evt_t evt;
//
//            if (ble_srv_is_notification_enabled(p_evt_write->data))
//            {
//                evt.evt_type = BLE_OS_EVT_NOTIFICATION_ENABLED;
//            }
//            else
//            {
//                evt.evt_type = BLE_OS_EVT_NOTIFICATION_DISABLED;
//            }
//                NRF_LOG_DEBUG("on_hrm_cccd_write");
//            p_our_service->evt_handler(p_our_service, &evt);
//        }
//    }
//}

//
///**@brief Function for handling the Write event.
// *
// * @param[in]   p_our_service   Our Service structure.
// * @param[in]   p_ble_evt       Event received from the BLE stack.
// */
//static void on_write(ble_os_t * p_our_service, ble_evt_t const * p_ble_evt)
//{
//    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
//
//    if (p_evt_write->handle == p_our_service->os_handles.cccd_handle)
//    {
//        on_our_service_cccd_write(p_our_service, p_evt_write);
//    }
//}

//
///**@brief Function for adding our new characterstic to service
// *
// * @param[in]   p_our_service        Our Service structure.
// *
// */
//static uint32_t our_char_add(ble_os_t * p_our_service)
//{
//    ble_add_char_params_t add_char_params;
//
//    // Add a custom characteristic UUID
//    uint32_t            err_code;
//    ble_uuid_t          char_uuid;
//    ble_uuid128_t       base_uuid = BLE_UUID_OUR_BASE_UUID;
//    char_uuid.uuid      = BLE_UUID_VAL_CHAR;
//    err_code = sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
//    APP_ERROR_CHECK(err_code); 
//    
//    // Add read/write properties to our characteristic
//    ble_gatts_char_md_t char_md;
//    memset(&char_md, 0, sizeof(char_md));
//    char_md.char_props.read = 1;
//    char_md.char_props.write = 1;
//    
//    // Configuring Client Characteristic Configuration Descriptor metadata and add to char_md structure
//    ble_gatts_attr_md_t cccd_md;
//    memset(&cccd_md, 0, sizeof(cccd_md));
//    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
//    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
//    cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
//    char_md.p_cccd_md           = &cccd_md;
//    char_md.char_props.notify   = 1;
//    
//    // Configure the attribute metadata
//    ble_gatts_attr_md_t attr_md;
//    memset(&attr_md, 0, sizeof(attr_md));
//    attr_md.vloc        = BLE_GATTS_VLOC_STACK;
// 
//    // Set read/write security levels to our characteristic
//    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
//    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
//    
//    // Configure the characteristic value attribute
//    ble_gatts_attr_t    attr_char_value;
//    memset(&attr_char_value, 0, sizeof(attr_char_value));
//    attr_char_value.p_uuid      = &char_uuid;
//    attr_char_value.p_attr_md   = &attr_md;
//   
//    // Set characteristic length in number of bytes
//    attr_char_value.max_len     = 8;
//    attr_char_value.init_len    = 8;
//    uint8_t value[8]            = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
//    attr_char_value.p_value     = value;
//
//    // Add our new characteristic to the service
//    err_code = sd_ble_gatts_characteristic_add(p_our_service->service_handle,
//                                   &char_md,
//                                   &attr_char_value,
//                                   &p_our_service->os_handles);
//    APP_ERROR_CHECK(err_code);
//
//        // Add record control access point characteristic
//    memset(&add_char_params, 0, sizeof(add_char_params));
//    add_char_params.uuid                = BLE_UUID_RECORD_ACCESS_CONTROL_POINT_CHAR;
//    add_char_params.max_len             = BLE_GATT_ATT_MTU_DEFAULT;
//    add_char_params.is_var_len          = true;
//    add_char_params.char_props.indicate = 1;
//    add_char_params.char_props.write    = 1;
//    add_char_params.cccd_write_access   = SEC_JUST_WORKS; // p_os_init->racp_cccd_wr_sec;
//    add_char_params.write_access        = SEC_JUST_WORKS; //p_os_init->racp_wr_sec;
//    add_char_params.is_defered_write    = true;
//
//    err_code = characteristic_add(p_our_service->service_handle,
//                                  &add_char_params,
//                                  &p_our_service->racp_handles);
//
//
//    return NRF_SUCCESS;
//}
//
///**@brief Function for initiating our new service.
// *
// * @param[in]   p_our_service        Our Service structure.
// *
// */
//void our_service_init(ble_os_t * p_our_service)
//{
//    uint32_t   err_code;
//
//    ble_uuid_t        service_uuid;
//    ble_uuid128_t     base_uuid = BLE_UUID_OUR_BASE_UUID;
//    service_uuid.uuid = BLE_UUID_OUR_SERVICE;
//    err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
//    APP_ERROR_CHECK(err_code);    
//	
//    // Set our service connection handle to default value. I.e. an invalid handle since we are not yet in a connection.
//    p_our_service->conn_handle = BLE_CONN_HANDLE_INVALID;
//
//    p_our_service->evt_handler = on_our_service_evt_handler;
//
//    // Add our service
//    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
//                                        &service_uuid,
//                                        &p_our_service->service_handle);
//    
//    APP_ERROR_CHECK(err_code);
//    
//    // Call the function our_char_add() to add our new characteristic to the service. 
//    our_char_add(p_our_service);
//}
//

// 
///**@brief Function to be called when updating characteristic value
// *
// * @param[in]   p_our_service       Our Service structure.
// * @param[in]   p_data              Data to use to update characteristic
// * @param[in]   data_len            Length of p_data to be send.
// * @param[out]  
// *
// */
//uint32_t our_service_characteristic_update(ble_os_t *p_our_service, uint8_t *p_data, uint8_t data_len)
//{
//    uint32_t        err_code;
//
//    // NRF_LOG_DEBUG("our_service_characteristic_update");
//    
//    // Update characteristic value
//    if (p_our_service->conn_handle != BLE_CONN_HANDLE_INVALID)
//    {
//        uint16_t len = data_len;
//        ble_gatts_hvx_params_t hvx_params;
//        memset(&hvx_params, 0, sizeof(hvx_params));
//
//        hvx_params.handle = p_our_service->os_handles.value_handle;
//        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
//        hvx_params.offset = 0;
//        hvx_params.p_len  = &len;
//        hvx_params.p_data = (uint8_t *)p_data;  
//        
//        err_code = sd_ble_gatts_hvx(p_our_service->conn_handle, &hvx_params);
//        NRF_LOG_DEBUG("sd_ble_gatts_hvx, err code %d (0x%X)", err_code);
//
//        return err_code;
//    } else {
//        return BLE_ERROR_INVALID_CONN_HANDLE;
//    }
//}
//
///**@brief Function to be called to send the offline buffer as notifications
// *
// * @param[out]  Returns true if all data send, otherwise false;
// *
// */
//void our_service_send_data_control(ble_os_t *p_our_service,
//                                    uint8_t *p_data, uint8_t num_entries, uint8_t data_len_entry,
//                                    bool restart)
//{
//    ret_code_t err_code;
//	
//    static uint8_t data_counter = 0;
//    if(restart)
//        data_counter = 0;
//    
//    NRF_LOG_DEBUG("our_service_send_data_control()");
//
//    while(data_counter < num_entries) {
//        err_code = our_service_characteristic_update(p_our_service, 
//                        &p_data[data_counter * data_len_entry], data_len_entry);
//        if (err_code == NRF_ERROR_INVALID_STATE 
//            || err_code == BLE_ERROR_GATTS_SYS_ATTR_MISSING)
//        { 
//            break;
//        } else if (err_code == NRF_ERROR_RESOURCES)
//        { 
//            // We will try to maximize throughput, so we don't stop sending (quit the while loop) before we reach BLE_ERROR_NO_TX_BUFFERS state
//            break; 
//        } else if (err_code != NRF_SUCCESS) 
//        { 
//            APP_ERROR_HANDLER(err_code);
//        }
//        
//        // At this point the data entry is considered send, prepare the next 
//        data_counter++;
//    }
//}
//
//// TODO
//
//
///**@brief Function for encoding a Glucose measurement.
// *
// * @param[in]  p_meas            Measurement to be encoded.
// * @param[out] p_encoded_buffer  Pointer to buffer where the encoded measurement is to be stored.
// *
// * @return Size of encoded measurement.
// */
//static uint8_t gls_meas_encode(const ble_gls_meas_t * p_meas, uint8_t * p_encoded_buffer)
//{
//    uint8_t len = 0;
//
//    p_encoded_buffer[len++] = p_meas->flags;
//
//    len += uint16_encode(p_meas->sequence_number, &p_encoded_buffer[len]);
//    len += ble_date_time_encode(&p_meas->base_time, &p_encoded_buffer[len]);
//
//    if (p_meas->flags & BLE_GLS_MEAS_FLAG_TIME_OFFSET)
//    {
//        len += uint16_encode(p_meas->time_offset, &p_encoded_buffer[len]);
//    }
//
//    if (p_meas->flags & BLE_GLS_MEAS_FLAG_CONC_TYPE_LOC)
//    {
//        uint16_t encoded_concentration;
//
//        encoded_concentration = ((p_meas->glucose_concentration.exponent << 12) & 0xF000) |
//                                ((p_meas->glucose_concentration.mantissa <<  0) & 0x0FFF);
//
//        p_encoded_buffer[len++] = (uint8_t)(encoded_concentration);
//        p_encoded_buffer[len++] = (uint8_t)(encoded_concentration >> 8);
//        p_encoded_buffer[len++] = (p_meas->sample_location << 4) | (p_meas->type & 0x0F);
//    }
//
//    if (p_meas->flags & BLE_GLS_MEAS_FLAG_SENSOR_STATUS)
//    {
//        len += uint16_encode(p_meas->sensor_status_annunciation, &p_encoded_buffer[len]);
//    }
//
//    return len;
//}
//
//
//uint32_t ble_gls_init(ble_gls_t * p_gls, const ble_gls_init_t * p_gls_init)
//{
//    uint32_t              err_code;
//    uint8_t               num_recs;
//    uint8_t               init_value_encoded[MAX_OSM_LEN];
//    ble_uuid_t            ble_uuid;
//    ble_add_char_params_t add_char_params;
//    ble_gls_rec_t         initial_gls_rec_value;
//
//    // Initialize data base
//    err_code = ble_gls_db_init();
//    if (err_code != NRF_SUCCESS)
//    {
//        return err_code;
//    }
//
//    err_code = next_sequence_number_set();
//    if (err_code != NRF_SUCCESS)
//    {
//        return err_code;
//    }
//
//    // Initialize service structure
//    p_gls->evt_handler          = p_gls_init->evt_handler;
//    p_gls->error_handler        = p_gls_init->error_handler;
//    p_gls->feature              = p_gls_init->feature;
//    p_gls->is_context_supported = p_gls_init->is_context_supported;
//    p_gls->conn_handle          = BLE_CONN_HANDLE_INVALID;
//
//
//    // Initialize global variables
//    state_set(STATE_NO_COMM);
//    m_racp_proc_records_reported_since_txcomplete = 0;
//
//    // Add service
//    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_GLUCOSE_SERVICE);
//
//    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_gls->service_handle);
//    if (err_code != NRF_SUCCESS)
//    {
//        return err_code;
//    }
//
//    // Add glucose measurement characteristic
//    memset(&add_char_params, 0, sizeof(add_char_params));
//    memset(&initial_gls_rec_value, 0, sizeof(initial_gls_rec_value));
//
//    num_recs = ble_gls_db_num_records_get();
//    if (num_recs > 0)
//    {
//        err_code = ble_gls_db_record_get(num_recs - 1, &initial_gls_rec_value);
//        if (err_code != NRF_SUCCESS)
//        {
//            return err_code;
//        }
//    }
//
//    add_char_params.uuid              = BLE_UUID_GLUCOSE_MEASUREMENT_CHAR;
//    add_char_params.max_len           = MAX_OSM_LEN;
//    add_char_params.init_len          = gls_meas_encode(&initial_gls_rec_value.meas, init_value_encoded);
//    add_char_params.is_var_len        = true;
//    add_char_params.char_props.notify = 1;
//    add_char_params.cccd_write_access = p_gls_init->gl_meas_cccd_wr_sec;
//    add_char_params.p_init_value      = init_value_encoded;
//
//    err_code = characteristic_add(p_gls->service_handle, &add_char_params, &p_gls->osm_handles);
//    if (err_code != NRF_SUCCESS)
//    {
//        return err_code;
//    }
//
//    // Add glucose measurement feature characteristic
//    memset(&add_char_params, 0, sizeof(add_char_params));
//
//    add_char_params.uuid              = BLE_UUID_GLUCOSE_FEATURE_CHAR;
//    add_char_params.max_len           = sizeof (uint16_t);
//    add_char_params.init_len          = uint16_encode(p_gls->feature, init_value_encoded);
//    add_char_params.p_init_value      = init_value_encoded;
//    add_char_params.char_props.read   = 1;
//    add_char_params.read_access       = p_gls_init->gl_feature_rd_sec;
//
//    err_code = characteristic_add(p_gls->service_handle, &add_char_params, &p_gls->glf_handles);
//    if (err_code != NRF_SUCCESS)
//    {
//        return err_code;
//    }
//
//    // Add record control access point characteristic
//    memset(&add_char_params, 0, sizeof(add_char_params));
//    add_char_params.uuid                = BLE_UUID_RECORD_ACCESS_CONTROL_POINT_CHAR;
//    add_char_params.max_len             = BLE_GATT_ATT_MTU_DEFAULT;
//    add_char_params.is_var_len          = true;
//    add_char_params.char_props.indicate = 1;
//    add_char_params.char_props.write    = 1;
//    add_char_params.cccd_write_access   = p_gls_init->racp_cccd_wr_sec;
//    add_char_params.write_access        = p_gls_init->racp_wr_sec;
//    add_char_params.is_defered_write    = true;
//
//    err_code = characteristic_add(p_gls->service_handle,
//                                  &add_char_params,
//                                  &p_gls->racp_handles);
//    if (err_code != NRF_SUCCESS)
//    {
//        return err_code;
//    }
//
//    return NRF_SUCCESS;
//}
//
//
//
//
///**@brief Function for sending a response from the Record Access Control Point.
// *
// * @param[in] p_gls       Service instance.
// * @param[in] p_racp_val  RACP value to be sent.
// */
//static void racp_send(ble_gls_t * p_gls, ble_racp_value_t * p_racp_val)
//{
//    uint32_t               err_code;
//    uint8_t                encoded_resp[25];
//    uint8_t                len;
//    uint16_t               hvx_len;
//    ble_gatts_hvx_params_t hvx_params;
//
//    if (
//        (m_gls_state != STATE_RACP_RESPONSE_PENDING)
//        &&
//        (m_racp_proc_records_reported_since_txcomplete > 0)
//       )
//    {
//        state_set(STATE_RACP_RESPONSE_PENDING);
//        return;
//    }
//
//    // Send indication
//    len     = ble_racp_encode(p_racp_val, encoded_resp);
//    hvx_len = len;
//
//    memset(&hvx_params, 0, sizeof(hvx_params));
//
//    hvx_params.handle = p_gls->racp_handles.value_handle;
//    hvx_params.type   = BLE_GATT_HVX_INDICATION;
//    hvx_params.offset = 0;
//    hvx_params.p_len  = &hvx_len;
//    hvx_params.p_data = encoded_resp;
//
//    err_code = sd_ble_gatts_hvx(p_gls->conn_handle, &hvx_params);
//
//    // Error handling
//    if ((err_code == NRF_SUCCESS) && (hvx_len != len))
//    {
//        err_code = NRF_ERROR_DATA_SIZE;
//    }
//    switch (err_code)
//    {
//        case NRF_SUCCESS:
//            // Wait for HVC event
//            state_set(STATE_RACP_RESPONSE_IND_VERIF);
//            break;
//
//        case NRF_ERROR_BUSY:
//            // Wait for BLE_GATTS_EVT_HVC event to retry transmission
//            state_set(STATE_RACP_RESPONSE_PENDING);
//            break;
//
//        case NRF_ERROR_INVALID_STATE:
//            // Make sure state machine returns to the default state
//            state_set(STATE_NO_COMM);
//            break;
//
//        default:
//            // Report error to application
//            if (p_gls->error_handler != NULL)
//            {
//                p_gls->error_handler(err_code);
//            }
//
//            // Make sure state machine returns to the default state
//            state_set(STATE_NO_COMM);
//            break;
//    }
//}
//
//
///**@brief Function for sending a RACP response containing a Response Code Op Code and a Response Code Value.
// *
// * @param[in] p_gls   Service instance.
// * @param[in] opcode  RACP Op Code.
// * @param[in] value   RACP Response Code Value.
// */
//static void racp_response_code_send(ble_gls_t * p_gls, uint8_t opcode, uint8_t value)
//{
//    m_pending_racp_response.opcode      = RACP_OPCODE_RESPONSE_CODE;
//    m_pending_racp_response.operator    = RACP_OPERATOR_NULL;
//    m_pending_racp_response.operand_len = 2;
//    m_pending_racp_response.p_operand   = m_pending_racp_response_operand;
//
//    m_pending_racp_response_operand[0] = opcode;
//    m_pending_racp_response_operand[1] = value;
//
//    racp_send(p_gls, &m_pending_racp_response);
//}
//
//
//
///**@brief Function for responding to the ALL operation.
// *
// * @param[in] p_gls  Service instance.
// *
// * @return NRF_SUCCESS on success, otherwise an error code.
// */
//static uint32_t racp_report_records_all(ble_gls_t * p_gls)
//{
//    uint16_t total_records = ble_gls_db_num_records_get();
//
//    if (m_racp_proc_record_ndx >= total_records)
//    {
//        state_set(STATE_NO_COMM);
//    }
//    else
//    {
//        uint32_t      err_code;
//        ble_gls_rec_t rec;
//
//        err_code = ble_gls_db_record_get(m_racp_proc_record_ndx, &rec);
//        if (err_code != NRF_SUCCESS)
//        {
//            return err_code;
//        }
//
//        err_code = glucose_meas_send(p_gls, &rec);
//        if (err_code != NRF_SUCCESS)
//        {
//            return err_code;
//        }
//    }
//
//    return NRF_SUCCESS;
//}
//
//
///**@brief Function for responding to the FIRST or the LAST operation.
// *
// * @param[in] p_gls  Service instance.
// *
// * @return  NRF_SUCCESS on success, otherwise an error code.
// */
//static uint32_t racp_report_records_first_last(ble_gls_t * p_gls)
//{
//    uint32_t      err_code;
//    ble_gls_rec_t rec;
//    uint16_t      total_records;
//
//    total_records = ble_gls_db_num_records_get();
//
//    if ((m_racp_proc_records_reported != 0) || (total_records == 0))
//    {
//        state_set(STATE_NO_COMM);
//    }
//    else
//    {
//        if (m_racp_proc_operator == RACP_OPERATOR_FIRST)
//        {
//            err_code = ble_gls_db_record_get(0, &rec);
//            if (err_code != NRF_SUCCESS)
//            {
//                return err_code;
//            }
//        }
//        else if (m_racp_proc_operator == RACP_OPERATOR_LAST)
//        {
//            err_code = ble_gls_db_record_get(total_records - 1, &rec);
//            if (err_code != NRF_SUCCESS)
//            {
//                return err_code;
//            }
//        }
//
//        err_code = glucose_meas_send(p_gls, &rec);
//        if (err_code != NRF_SUCCESS)
//        {
//            return err_code;
//        }
//    }
//
//    return NRF_SUCCESS;
//}
//
//
///**@brief Function for responding to the GREATER_OR_EQUAL operation.
// *
// * @param[in] p_gls  Service instance.
// *
// * @return NRF_SUCCESS on success, otherwise an error code.
// */
//static uint32_t racp_report_records_greater_or_equal(ble_gls_t * p_gls)
//{
//    uint16_t total_records = ble_gls_db_num_records_get();
//
//    while (m_racp_proc_record_ndx < total_records)
//    {
//        uint32_t      err_code;
//        ble_gls_rec_t rec;
//
//        err_code = ble_gls_db_record_get(m_racp_proc_record_ndx, &rec);
//        if (err_code != NRF_SUCCESS)
//        {
//            return err_code;
//        }
//
//        if (rec.meas.sequence_number >= m_racp_proc_seq_num)
//        {
//            err_code = glucose_meas_send(p_gls, &rec);
//            if (err_code != NRF_SUCCESS)
//            {
//                return err_code;
//            }
//            break;
//        }
//        m_racp_proc_record_ndx++;
//    }
//    if (m_racp_proc_record_ndx == total_records)
//    {
//        state_set(STATE_NO_COMM);
//    }
//
//    return NRF_SUCCESS;
//}
//
//
///**@brief Function for informing that the REPORT RECORDS procedure is completed.
// *
// * @param[in] p_gls  Service instance.
// */
//static void racp_report_records_completed(ble_gls_t * p_gls)
//{
//    uint8_t resp_code_value;
//
//    if (m_racp_proc_records_reported > 0)
//    {
//        resp_code_value = RACP_RESPONSE_SUCCESS;
//    }
//    else
//    {
//        resp_code_value = RACP_RESPONSE_NO_RECORDS_FOUND;
//    }
//
//    racp_response_code_send(p_gls, RACP_OPCODE_REPORT_RECS, resp_code_value);
//}
//
//
///**@brief Function for the RACP report records procedure.
// *
// * @param[in] p_gls  Service instance.
// */
//static void racp_report_records_procedure(ble_gls_t * p_gls)
//{
//    uint32_t err_code;
//
//    while (m_gls_state == STATE_RACP_PROC_ACTIVE)
//    {
//        // Execute requested procedure
//        switch (m_racp_proc_operator)
//        {
//            case RACP_OPERATOR_ALL:
//                err_code = racp_report_records_all(p_gls);
//                break;
//
//            case RACP_OPERATOR_FIRST:
//            case RACP_OPERATOR_LAST:
//                err_code = racp_report_records_first_last(p_gls);
//                break;
//
//            case RACP_OPERATOR_GREATER_OR_EQUAL:
//                err_code = racp_report_records_greater_or_equal(p_gls);
//                break;
//
//            default:
//                // Report error to application
//                if (p_gls->error_handler != NULL)
//                {
//                    p_gls->error_handler(NRF_ERROR_INTERNAL);
//                }
//
//                // Make sure state machine returns to the default state
//                state_set(STATE_NO_COMM);
//                return;
//        }
//
//        // Error handling
//        switch (err_code)
//        {
//            case NRF_SUCCESS:
//                if (m_gls_state == STATE_RACP_PROC_ACTIVE)
//                {
//                    m_racp_proc_record_ndx++;
//                }
//                else
//                {
//                    racp_report_records_completed(p_gls);
//                }
//                break;
//
//            case NRF_ERROR_RESOURCES:
//                // Wait for TX_COMPLETE event to resume transmission
//                return;
//
//            case NRF_ERROR_INVALID_STATE:
//                // Notification is probably not enabled. Ignore request.
//                state_set(STATE_NO_COMM);
//                return;
//
//            default:
//                // Report error to application
//                if (p_gls->error_handler != NULL)
//                {
//                    p_gls->error_handler(err_code);
//                }
//
//                // Make sure state machine returns to the default state
//                state_set(STATE_NO_COMM);
//                return;
//        }
//    }
//}
//
//
///**@brief Function for testing if the received request is to be executed.
// *
// * @param[in]  p_racp_request   Request to be checked.
// * @param[out] p_response_code  Response code to be sent in case the request is rejected.
// *                              RACP_RESPONSE_RESERVED is returned if the received message is
// *                              to be rejected without sending a response.
// *
// * @return TRUE if the request is to be executed, FALSE if it is to be rejected.
// *         If it is to be rejected, p_response_code will contain the response code to be
// *         returned to the central.
// */
//static bool is_request_to_be_executed(ble_racp_value_t const * p_racp_request,
//                                      uint8_t                * p_response_code)
//{
//    *p_response_code = RACP_RESPONSE_RESERVED;
//
//    if (p_racp_request->opcode == RACP_OPCODE_ABORT_OPERATION)
//    {
//        if (m_gls_state == STATE_RACP_PROC_ACTIVE)
//        {
//            if (p_racp_request->operator != RACP_OPERATOR_NULL)
//            {
//                *p_response_code = RACP_RESPONSE_INVALID_OPERATOR;
//            }
//            else if (p_racp_request->operand_len != 0)
//            {
//                *p_response_code = RACP_RESPONSE_INVALID_OPERAND;
//            }
//            else
//            {
//                *p_response_code = RACP_RESPONSE_SUCCESS;
//            }
//        }
//        else
//        {
//            *p_response_code = RACP_RESPONSE_ABORT_FAILED;
//        }
//    }
//    else if (m_gls_state != STATE_NO_COMM)
//    {
//        return false;
//    }
//    // Supported opcodes.
//    else if ((p_racp_request->opcode == RACP_OPCODE_REPORT_RECS) ||
//             (p_racp_request->opcode == RACP_OPCODE_REPORT_NUM_RECS))
//    {
//        switch (p_racp_request->operator)
//        {
//            // Operators WITHOUT a filter.
//            case RACP_OPERATOR_ALL:
//            case RACP_OPERATOR_FIRST:
//            case RACP_OPERATOR_LAST:
//                if (p_racp_request->operand_len != 0)
//                {
//                    *p_response_code = RACP_RESPONSE_INVALID_OPERAND;
//                }
//                break;
//
//            // Operators WITH a filter.
//            case RACP_OPERATOR_GREATER_OR_EQUAL:
//                if (p_racp_request->p_operand[0] == OPERAND_FILTER_TYPE_SEQ_NUM)
//                {
//                    if (p_racp_request->operand_len != 3)
//                    {
//                        *p_response_code = RACP_RESPONSE_INVALID_OPERAND;
//                    }
//                }
//                else if (p_racp_request->p_operand[0] == OPERAND_FILTER_TYPE_FACING_TIME)
//                {
//                    *p_response_code = RACP_RESPONSE_OPERAND_UNSUPPORTED;
//                }
//                else if (p_racp_request->p_operand[0] >= OPERAND_FILTER_TYPE_RFU_START)
//                {
//                    *p_response_code = RACP_RESPONSE_OPERAND_UNSUPPORTED;
//                }
//                else
//                {
//                    *p_response_code = RACP_RESPONSE_INVALID_OPERAND;
//                }
//                break;
//
//            // Unsupported operators.
//            case RACP_OPERATOR_LESS_OR_EQUAL:
//            case RACP_OPERATOR_RANGE:
//                *p_response_code = RACP_RESPONSE_OPERATOR_UNSUPPORTED;
//                 break;
//
//            // Invalid operators.
//            case RACP_OPERATOR_NULL:
//            default:
//                if (p_racp_request->operator >= RACP_OPERATOR_RFU_START)
//                {
//                    *p_response_code = RACP_RESPONSE_OPERATOR_UNSUPPORTED;
//                }
//                else
//                {
//                    *p_response_code = RACP_RESPONSE_INVALID_OPERATOR;
//                }
//                break;
//        }
//    }
//    // Unsupported opcodes,
//    else if (p_racp_request->opcode == RACP_OPCODE_DELETE_RECS)
//    {
//        *p_response_code = RACP_RESPONSE_OPCODE_UNSUPPORTED;
//    }
//    // Unknown opcodes.
//    else
//    {
//        *p_response_code = RACP_RESPONSE_OPCODE_UNSUPPORTED;
//    }
//
//    // NOTE: The computation of the return value will change slightly when deferred write has been
//    //       implemented in the stack.
//    return (*p_response_code == RACP_RESPONSE_RESERVED);
//}
//
//
///**@brief Function for processing a REPORT RECORDS request.
// *
// * @param[in] p_gls           Service instance.
// * @param[in] p_racp_request  Request to be executed.
// */
//static void report_records_request_execute(ble_gls_t * p_gls, ble_racp_value_t * p_racp_request)
//{
//    uint16_t seq_num = (p_racp_request->p_operand[2] << 8) | p_racp_request->p_operand[1];
//
//    state_set(STATE_RACP_PROC_ACTIVE);
//
//    m_racp_proc_record_ndx       = 0;
//    m_racp_proc_operator         = p_racp_request->operator;
//    m_racp_proc_records_reported = 0;
//    m_racp_proc_seq_num          = seq_num;
//
//    racp_report_records_procedure(p_gls);
//}
//
//
///**@brief Function for processing a REPORT NUM RECORDS request.
// *
// * @param[in] p_gls           Service instance.
// * @param[in] p_racp_request  Request to be executed.
// */
//static void report_num_records_request_execute(ble_gls_t * p_gls, ble_racp_value_t * p_racp_request)
//{
//    uint16_t total_records;
//    uint16_t num_records;
//
//    total_records = ble_gls_db_num_records_get();
//    num_records   = 0;
//
//    if (p_racp_request->operator == RACP_OPERATOR_ALL)
//    {
//        num_records = total_records;
//    }
//    else if (p_racp_request->operator == RACP_OPERATOR_GREATER_OR_EQUAL)
//    {
//        uint16_t seq_num;
//        uint16_t i;
//
//        seq_num = (p_racp_request->p_operand[2] << 8) | p_racp_request->p_operand[1];
//
//        for (i = 0; i < total_records; i++)
//        {
//            uint32_t      err_code;
//            ble_gls_rec_t rec;
//
//            err_code = ble_gls_db_record_get(i, &rec);
//            if (err_code != NRF_SUCCESS)
//            {
//                if (p_gls->error_handler != NULL)
//                {
//                    p_gls->error_handler(err_code);
//                }
//                return;
//            }
//
//            if (rec.meas.sequence_number >= seq_num)
//            {
//                num_records++;
//            }
//        }
//    }
//    else if ((p_racp_request->operator == RACP_OPERATOR_FIRST) ||
//             (p_racp_request->operator == RACP_OPERATOR_LAST))
//    {
//        if (total_records > 0)
//        {
//            num_records = 1;
//        }
//    }
//
//    m_pending_racp_response.opcode      = RACP_OPCODE_NUM_RECS_RESPONSE;
//    m_pending_racp_response.operator    = RACP_OPERATOR_NULL;
//    m_pending_racp_response.operand_len = sizeof(uint16_t);
//    m_pending_racp_response.p_operand   = m_pending_racp_response_operand;
//
//    m_pending_racp_response_operand[0] = num_records & 0xFF;
//    m_pending_racp_response_operand[1] = num_records >> 8;
//
//    racp_send(p_gls, &m_pending_racp_response);
//}
//
//
//
///**@brief Function for handling a write event to the Record Access Control Point.
// *
// * @param[in] p_gls        Service instance.
// * @param[in] p_evt_write  WRITE event to be handled.
// */
//static void on_racp_value_write(ble_gls_t * p_gls, ble_gatts_evt_write_t const * p_evt_write)
//{
//    ble_racp_value_t                      racp_request;
//    uint8_t                               response_code;
//    ble_gatts_rw_authorize_reply_params_t auth_reply;
//    bool                                  are_cccd_configured;
//    uint32_t                              err_code;
//
//    auth_reply.type                = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
//    auth_reply.params.write.offset = 0;
//    auth_reply.params.write.len    = 0;
//    auth_reply.params.write.p_data = NULL;
//
//    err_code = ble_gls_are_cccd_configured(p_gls, &are_cccd_configured);
//    if (err_code != NRF_SUCCESS)
//    {
//        if (p_gls->error_handler != NULL)
//        {
//            p_gls->error_handler(err_code);
//        }
//        return;
//    }
//
//    if (!are_cccd_configured)
//    {
//        auth_reply.params.write.gatt_status = GLS_NACK_CCCD_IMPROPERLY_CONFIGURED;
//        err_code                            = sd_ble_gatts_rw_authorize_reply(p_gls->conn_handle,
//                                                                              &auth_reply);
//
//        if (err_code != NRF_SUCCESS)
//        {
//            if (p_gls->error_handler != NULL)
//            {
//                p_gls->error_handler(err_code);
//            }
//        }
//        return;
//    }
//
//    // Decode request.
//    ble_racp_decode(p_evt_write->len, (uint8_t*)p_evt_write->data, &racp_request);
//
//    // Check if request is to be executed.
//    if (is_request_to_be_executed(&racp_request, &response_code))
//    {
//        auth_reply.params.write.gatt_status = BLE_GATT_STATUS_SUCCESS;
//        auth_reply.params.write.update      = 1;
//
//        err_code = sd_ble_gatts_rw_authorize_reply(p_gls->conn_handle,
//                                                   &auth_reply);
//
//        if (err_code != NRF_SUCCESS)
//        {
//            if (p_gls->error_handler != NULL)
//            {
//                p_gls->error_handler(err_code);
//            }
//            return;
//        }
//        // Execute request.
//        if (racp_request.opcode == RACP_OPCODE_REPORT_RECS)
//        {
//            report_records_request_execute(p_gls, &racp_request);
//        }
//        else if (racp_request.opcode == RACP_OPCODE_REPORT_NUM_RECS)
//        {
//            report_num_records_request_execute(p_gls, &racp_request);
//        }
//    }
//    else if (response_code != RACP_RESPONSE_RESERVED)
//    {
//        auth_reply.params.write.gatt_status = BLE_GATT_STATUS_SUCCESS;
//        auth_reply.params.write.update      = 1;
//        err_code                            = sd_ble_gatts_rw_authorize_reply(p_gls->conn_handle,
//                                                                              &auth_reply);
//
//        if (err_code != NRF_SUCCESS)
//        {
//            if (p_gls->error_handler != NULL)
//            {
//                p_gls->error_handler(err_code);
//            }
//            return;
//        }
//
//        // Abort any running procedure.
//        state_set(STATE_NO_COMM);
//
//        // Respond with error code.
//        racp_response_code_send(p_gls, racp_request.opcode, response_code);
//    }
//    else
//    {
//        auth_reply.params.write.gatt_status = GLS_NACK_PROC_ALREADY_IN_PROGRESS;
//        err_code                            = sd_ble_gatts_rw_authorize_reply(p_gls->conn_handle,
//                                                                              &auth_reply);
//
//        if (err_code != NRF_SUCCESS)
//        {
//            if (p_gls->error_handler != NULL)
//            {
//                p_gls->error_handler(err_code);
//            }
//            return;
//        }
//    }
//}
//
//
///**@brief Function for checking if the CCCDs are configured.
// *
// * @param[in] p_gls                  Service instance.
// * @param[in] p_are_cccd_configured  boolean indicating if both cccds are configured
// */
//uint32_t ble_gls_are_cccd_configured(ble_gls_t * p_gls, bool * p_are_cccd_configured)
//{
//    uint32_t err_code;
//    uint8_t  cccd_value_buf[BLE_CCCD_VALUE_LEN];
//    bool     is_osm_notif_enabled  = false;
//    bool     is_racp_indic_enabled = false;
//    ble_gatts_value_t gatts_value;
//
//    // Initialize value struct.
//    memset(&gatts_value, 0, sizeof(gatts_value));
//
//    gatts_value.len     = BLE_CCCD_VALUE_LEN;
//    gatts_value.offset  = 0;
//    gatts_value.p_value = cccd_value_buf;
//
//    err_code = sd_ble_gatts_value_get(p_gls->conn_handle,
//                                      p_gls->osm_handles.cccd_handle,
//                                      &gatts_value);
//    if (err_code != NRF_SUCCESS)
//    {
//        return err_code;
//    }
//    is_osm_notif_enabled = ble_srv_is_notification_enabled(cccd_value_buf);
//
//    err_code = sd_ble_gatts_value_get(p_gls->conn_handle,
//                                      p_gls->racp_handles.cccd_handle,
//                                      &gatts_value);
//    if (err_code != NRF_SUCCESS)
//    {
//        return err_code;
//    }
//    is_racp_indic_enabled = ble_srv_is_indication_enabled(cccd_value_buf);
//    if (is_racp_indic_enabled & is_osm_notif_enabled)
//    {
//        *p_are_cccd_configured = true;
//    }
//    else
//    {
//        *p_are_cccd_configured = false;
//    }
//    return NRF_SUCCESS;
//}
//
//
///**@brief Function for handling the Glucose measurement CCCD write event.
// *
// * @param[in] p_gls        Service instance.
// * @param[in] p_evt_write  WRITE event to be handled.
// */
//static void on_osm_cccd_write(ble_gls_t * p_gls, ble_gatts_evt_write_t const * p_evt_write)
//{
//    if (p_evt_write->len == 2)
//    {
//        // CCCD written, update notification state
//        ble_gls_evt_t evt;
//
//        if (ble_srv_is_notification_enabled(p_evt_write->data))
//        {
//            evt.evt_type = BLE_GLS_EVT_NOTIFICATION_ENABLED;
//        }
//        else
//        {
//            evt.evt_type = BLE_GLS_EVT_NOTIFICATION_DISABLED;
//        }
//
//        if (p_gls->evt_handler != NULL)
//        {
//            p_gls->evt_handler(p_gls, &evt);
//        }
//    }
//}
//
//
///**@brief Function for handling the WRITE event.
// *
// * @details Handles WRITE events from the BLE stack.
// *
// * @param[in] p_gls      Glucose Service structure.
// * @param[in] p_ble_evt  Event received from the BLE stack.
// */
//static void on_write(ble_gls_t * p_gls, ble_evt_t const * p_ble_evt)
//{
//    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
//
//    if (p_evt_write->handle == p_gls->osm_handles.cccd_handle)
//    {
//        on_osm_cccd_write(p_gls, p_evt_write);
//    }
//    else if (p_evt_write->handle == p_gls->racp_handles.value_handle)
//    {
//        on_racp_value_write(p_gls, p_evt_write);
//    }
//}
//
//
///**@brief Function for handling the TX_COMPLETE event.
// *
// * @details Handles TX_COMPLETE events from the BLE stack.
// *
// * @param[in] p_gls      Glucose Service structure.
// * @param[in] p_ble_evt  Event received from the BLE stack.
// */
//static void on_tx_complete(ble_gls_t * p_gls, ble_evt_t const * p_ble_evt)
//{
//    m_racp_proc_records_reported_since_txcomplete = 0;
//
//    if (m_gls_state == STATE_RACP_RESPONSE_PENDING)
//    {
//        racp_send(p_gls, &m_pending_racp_response);
//    }
//    else if (m_gls_state == STATE_RACP_PROC_ACTIVE)
//    {
//        racp_report_records_procedure(p_gls);
//    }
//}
//
//
///**@brief Function for handling the HVC event.
// *
// * @details Handles HVC events from the BLE stack.
// *
// * @param[in] p_gls      Glucose Service structure.
// * @param[in] p_ble_evt  Event received from the BLE stack.
// */
//static void on_hvc(ble_gls_t * p_gls, ble_evt_t const * p_ble_evt)
//{
//    ble_gatts_evt_hvc_t const * p_hvc = &p_ble_evt->evt.gatts_evt.params.hvc;
//
//    if (p_hvc->handle == p_gls->racp_handles.value_handle)
//    {
//        if (m_gls_state == STATE_RACP_RESPONSE_IND_VERIF)
//        {
//            // Indication has been acknowledged. Return to default state.
//            state_set(STATE_NO_COMM);
//        }
//        else if (m_gls_state == STATE_RACP_RESPONSE_PENDING)
//        {
//            racp_send(p_gls, &m_pending_racp_response);
//        }
//        else
//        {
//            // We did not expect this event in this state. Report error to application.
//            if (p_gls->error_handler != NULL)
//            {
//                p_gls->error_handler(NRF_ERROR_INVALID_STATE);
//            }
//        }
//    }
//}
//
//
//static void on_rw_authorize_request(ble_gls_t * p_gls, ble_gatts_evt_t const * p_gatts_evt)
//{
//    ble_gatts_evt_rw_authorize_request_t const * p_auth_req =
//        &p_gatts_evt->params.authorize_request;
//
//    if (p_auth_req->type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
//    {
//        if (   (p_gatts_evt->params.authorize_request.request.write.op
//                != BLE_GATTS_OP_PREP_WRITE_REQ)
//            && (p_gatts_evt->params.authorize_request.request.write.op
//                != BLE_GATTS_OP_EXEC_WRITE_REQ_NOW)
//            && (p_gatts_evt->params.authorize_request.request.write.op
//                != BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL)
//           )
//        {
//            if (p_auth_req->request.write.handle == p_gls->racp_handles.value_handle)
//            {
//                on_racp_value_write(p_gls, &p_auth_req->request.write);
//            }
//        }
//    }
//}
//
//
///**@brief Declaration of a function that will take care of some housekeeping of ble connections related to our service and characteristic 
// *
// * @param[in]   p_ble_evt       ble event.
// * @param[in]   p_context       context for the event
// *
// */
//void ble_our_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
//{
//    ble_os_t * p_our_service = (ble_os_t *) p_context;  
//
//    NRF_LOG_DEBUG("BLE our service event handler called with event 0x%X", p_ble_evt->header.evt_id);
//
//    // Implement switch case handling BLE events related to our service. 
//    switch (p_ble_evt->header.evt_id)
//    {
//        case BLE_GAP_EVT_CONNECTED:
//            NRF_LOG_DEBUG("ble_our_service_on_ble_evt: BLE_GAP_EVT_CONNECTED");
//            p_our_service->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
//            state_set(STATE_NO_COMM);
//            break;
//
//        case BLE_GAP_EVT_DISCONNECTED:
//            NRF_LOG_DEBUG("ble_our_service_on_ble_evt: BLE_GAP_EVT_DISCONNECTED");
//            p_our_service->conn_handle = BLE_CONN_HANDLE_INVALID;
//            break;
//
//        case BLE_GATTS_EVT_WRITE:
//            NRF_LOG_DEBUG("ble_our_service_on_ble_evt: BLE_GATTS_EVT_WRITE");
//            on_write(p_our_service, p_ble_evt);
//            break;
//
//        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
//            NRF_LOG_DEBUG("ble_our_service_on_ble_evt: BLE_GATTS_EVT_HVN_TX_COMPLETE");
//            on_tx_complete(p_gls, p_ble_evt);
//            break;
//
//        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
//            on_rw_authorize_request(p_gls, &p_ble_evt->evt.gatts_evt);
//            break;
//
//        case BLE_GATTS_EVT_HVC:
//            on_hvc(p_gls, p_ble_evt);
//            break;
//
//        default:
//            // No implementation needed.
//            break;
//    }		
//}
