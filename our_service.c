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
#include "app_error.h"
#include "SEGGER_RTT.h"

// https://devzone.nordicsemi.com/f/nordic-q-a/45679/loss-of-notifications-when-sending-them-from-peripheral-to-central-as-bulk-transfer
// https://devzone.nordicsemi.com/f/nordic-q-a/25637/can-you-not-call-sd_ble_gatts_hvx-in-a-loop


/**@brief Function for handling our service event.
 *
 * @param[in]   p_our_service   Our Service structure.
 * @param[in]   p_ble_evt       Event received from the BLE stack.
 */
static void on_our_service_evt_handler(ble_os_t * p_our_service, ble_os_evt_t * p_ble_evt)
{
    NRF_LOG_DEBUG("our service event handler called with event 0x%X", p_ble_evt->evt_type);

    // Implement switch case handling BLE events related to our service. 
    switch (p_ble_evt->evt_type)
    {
        case BLE_OS_EVT_NOTIFICATION_ENABLED:
            NRF_LOG_DEBUG("on_our_service_evt_handler: BLE_OS_EVT_NOTIFICATION_ENABLED");
            break;

        case BLE_OS_EVT_NOTIFICATION_DISABLED:
            NRF_LOG_DEBUG("on_our_service_evt_handler: BLE_OS_EVT_NOTIFICATION_DISABLED");
            break;

        default:
            // No implementation needed.
            break;
    }		
}


/**@brief Function for handling write events to the Our Service characteristic.
 *
 * @param[in]   p_our_service   Our Service structure.
 * @param[in]   p_evt_write     Write event received from the BLE stack.
 */
static void on_our_service_cccd_write(ble_os_t * p_our_service, ble_gatts_evt_write_t const * p_evt_write)
{
    if (p_evt_write->len == 2)
    {
        // CCCD written, update notification state
        if (p_our_service->evt_handler != NULL)
        {
            ble_os_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_OS_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_OS_EVT_NOTIFICATION_DISABLED;
            }
                NRF_LOG_DEBUG("on_hrm_cccd_write");
            p_our_service->evt_handler(p_our_service, &evt);
        }
    }
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_our_service   Our Service structure.
 * @param[in]   p_ble_evt       Event received from the BLE stack.
 */
static void on_write(ble_os_t * p_our_service, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_evt_write->handle == p_our_service->os_handles.cccd_handle)
    {
        on_our_service_cccd_write(p_our_service, p_evt_write);
    }
}


/**@brief Declaration of a function that will take care of some housekeeping of ble connections related to our service and characteristic 
 *
 * @param[in]   p_ble_evt       ble event.
 * @param[in]   p_context       context for the event
 *
 */
void ble_our_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_os_t * p_our_service = (ble_os_t *) p_context;  

    NRF_LOG_DEBUG("BLE our service event handler called with event 0x%X", p_ble_evt->header.evt_id);

    // Implement switch case handling BLE events related to our service. 
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_DEBUG("ble_our_service_on_ble_evt: BLE_GAP_EVT_CONNECTED");
            p_our_service->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_DEBUG("ble_our_service_on_ble_evt: BLE_GAP_EVT_DISCONNECTED");
            p_our_service->conn_handle = BLE_CONN_HANDLE_INVALID;
            break;
        case BLE_GATTS_EVT_WRITE:
            NRF_LOG_DEBUG("ble_our_service_on_ble_evt: BLE_GATTS_EVT_WRITE");
            on_write(p_our_service, p_ble_evt);
            break;
        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
            NRF_LOG_DEBUG("ble_our_service_on_ble_evt: BLE_GATTS_EVT_HVN_TX_COMPLETE");
            break;
        default:
            // No implementation needed.
            break;
    }		
}

/**@brief Function for adding our new characterstic to service
 *
 * @param[in]   p_our_service        Our Service structure.
 *
 */
static uint32_t our_char_add(ble_os_t * p_our_service)
{
    // Add a custom characteristic UUID
    uint32_t            err_code;
    ble_uuid_t          char_uuid;
    ble_uuid128_t       base_uuid = BLE_UUID_OUR_BASE_UUID;
    char_uuid.uuid      = BLE_UUID_VAL_CHAR;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
    APP_ERROR_CHECK(err_code); 
    
    // Add read/write properties to our characteristic
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = 1;
    char_md.char_props.write = 1;
    
    // Configuring Client Characteristic Configuration Descriptor metadata and add to char_md structure
    ble_gatts_attr_md_t cccd_md;
    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
    char_md.p_cccd_md           = &cccd_md;
    char_md.char_props.notify   = 1;
    
    // Configure the attribute metadata
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));
    attr_md.vloc        = BLE_GATTS_VLOC_STACK;
 
    // Set read/write security levels to our characteristic
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    // Configure the characteristic value attribute
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;
   
    // Set characteristic length in number of bytes
    attr_char_value.max_len     = 8;
    attr_char_value.init_len    = 8;
    uint8_t value[8]            = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    attr_char_value.p_value     = value;

    // Add our new characteristic to the service
    err_code = sd_ble_gatts_characteristic_add(p_our_service->service_handle,
                                   &char_md,
                                   &attr_char_value,
                                   &p_our_service->os_handles);
    APP_ERROR_CHECK(err_code);

    return NRF_SUCCESS;
}

/**@brief Function for initiating our new service.
 *
 * @param[in]   p_our_service        Our Service structure.
 *
 */
void our_service_init(ble_os_t * p_our_service)
{
    uint32_t   err_code;

    ble_uuid_t        service_uuid;
    ble_uuid128_t     base_uuid = BLE_UUID_OUR_BASE_UUID;
    service_uuid.uuid = BLE_UUID_OUR_SERVICE;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
    APP_ERROR_CHECK(err_code);    
	
    // Set our service connection handle to default value. I.e. an invalid handle since we are not yet in a connection.
    p_our_service->conn_handle = BLE_CONN_HANDLE_INVALID;

    p_our_service->evt_handler = on_our_service_evt_handler;

    // Add our service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &p_our_service->service_handle);
    
    APP_ERROR_CHECK(err_code);
    
    // Call the function our_char_add() to add our new characteristic to the service. 
    our_char_add(p_our_service);
}


// 
/**@brief Function to be called when updating characteristic value
 *
 * @param[in]   p_our_service       Our Service structure.
 * @param[in]   p_data              Data to use to update characteristic
 * @param[in]   data_len            Length of p_data to be send.
 * @param[out]  
 *
 */
uint32_t our_service_characteristic_update(ble_os_t *p_our_service, uint8_t *p_data, uint8_t data_len)
{
    uint32_t        err_code;

    NRF_LOG_INFO("our_service_characteristic_update");
    
    // Update characteristic value
    if (p_our_service->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint16_t len = data_len;
        ble_gatts_hvx_params_t hvx_params;
        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_our_service->os_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &len;
        hvx_params.p_data = (uint8_t *)p_data;  
        
        NRF_LOG_INFO("sd_ble_gatts_hvx");
        err_code = sd_ble_gatts_hvx(p_our_service->conn_handle, &hvx_params);
        
        return err_code;
    } else {
        return BLE_ERROR_INVALID_CONN_HANDLE;
    }
}

/**@brief Function to be called to send the offline buffer as notifications
 *
 * @param[out]  Returns true if all data send, otherwise false;
 *
 */
void our_service_send_data_control(ble_os_t *p_our_service,
                                    uint8_t *p_data, uint8_t num_entries, uint8_t data_len_entry,
                                    bool restart)
{
    ret_code_t err_code;
	
    static uint8_t data_counter = 0;
    if(restart)
        data_counter = 0;
    
    NRF_LOG_INFO("our_service_send_data_control()");

    while(data_counter < num_entries) {
        err_code = our_service_characteristic_update(p_our_service, 
                        &p_data[data_counter * data_len_entry], data_len_entry);
        if (err_code == NRF_ERROR_INVALID_STATE 
            || err_code == BLE_ERROR_GATTS_SYS_ATTR_MISSING)
        { 
            break;
        } else if (err_code == NRF_ERROR_RESOURCES)
        { 
            // We will try to maximize throughput, so we don't stop sending (quit the while loop) before we reach BLE_ERROR_NO_TX_BUFFERS state
            break; 
        } else if (err_code != NRF_SUCCESS) 
        { 
            APP_ERROR_HANDLER(err_code);
        }
        
        // At this point the data entry is considered send, prepare the next 
        data_counter++;
    }
}
