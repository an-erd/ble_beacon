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
 
#ifndef OUR_SERVICE_H__
#define OUR_SERVICE_H__

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"


/**@brief   Macro for defining a ble_cus instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_OS_BLE_OBSERVER_PRIO    3

#define BLE_OS_DEF(_name)                                                                          \
static ble_os_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_OS_BLE_OBSERVER_PRIO,                                                     \
                     ble_our_service_on_ble_evt, &_name)


/**@brief Our Service event type. */
typedef enum
{
    BLE_OS_EVT_NOTIFICATION_ENABLED,   /**< Our service value notification enabled event. */
    BLE_OS_EVT_NOTIFICATION_DISABLED   /**< Our service value notification disabled event. */
} ble_os_evt_type_t;

/**@brief Our Service event. */
typedef struct
{
    ble_os_evt_type_t evt_type;    /**< Type of event. */
} ble_os_evt_t;

// Forward declaration of the ble_os_t type.
typedef struct ble_os_s ble_os_t;

/**@brief Our Service event handler type. */
typedef void (*ble_os_evt_handler_t) (ble_os_t * p_os, ble_os_evt_t * p_evt);


/**@brief Custom Service init structure. This contains all options and data needed for
 *        initialization of the service. */
//typedef struct
//{
//    ble_os_evt_handler_t        evt_handler;                                          /**< Event handler to be called for handling events in the Custom Service. */
//    security_req_t              os_cccd_wr_sec;                                      /**< Security requirement for writing the CUS characteristic CCCD. */
//} ble_os_init_t;
//
/**@brief Custom service structure. This contains various status information for the service. */
struct ble_os_s
{
    ble_os_evt_handler_t        evt_handler;                                /**< Event handler to be called for handling events in the Custom Service. */
    uint16_t                    service_handle;                             /**< Handle of Custom Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t    os_handles;                                 /**< Handles related to the our service characteristic. */
    uint16_t                    conn_handle;                                /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint8_t                     max_os_len;                                 /**< Current maximum custom measurement length, adjusted according to the current ATT MTU. */
};


// randomly generated 128-bit base UUID: 612f3d33-37f5-4c4f-9ff2-320c4ba2b73c
#define BLE_UUID_OUR_BASE_UUID      {0x3C, 0xB7, 0xA2, 0x4B, 0x0C, 0x32, 0xF2, 0x9F, 0x4F, 0x4C, 0xF5, 0x37, 0x00, 0x00, 0x2F, 0x61} 
#define BLE_UUID_OUR_SERVICE        0x1400  // 16-bit service UUIDs
#define BLE_UUID_VAL_CHAR           0x1401  // 16-bit characteristic UUID, 
#define BLE_UUID_VAL_COUNT_CHAR     0x1402  // 16-bit characteristic UUID for the count of data entries available for BLE_UUID_VAL_CHAR
#define BLE_UUID_VAL_CMD_CHAR       0x140F  // 16-bit characteristic UUID for commands to the BLE_UUID_VAL_CHAR

///**
// * @brief This structure contains various status information for our service. 
// * It only holds one entry now, but will be populated with more items as we go.
// * The name is based on the naming convention used in Nordic's SDKs. 
// * 'ble’ indicates that it is a Bluetooth Low Energy relevant structure and 
// * ‘os’ is short for Our Service). 
// */
//typedef struct
//{
//    uint16_t                    conn_handle;            /**< Handle of the current connection (is BLE_CONN_HANDLE_INVALID if not in a connection).*/
//    uint16_t                    service_handle;         /**< Handle of Our Service  */
//    ble_gatts_char_handles_t    char_handles;           /**< Handles of our characteristic */
//    ble_gatts_char_handles_t    char_val_handles;       /**< Handles of our characteristic BLE_UUID_VAL_CHAR */
//    ble_gatts_char_handles_t    char_val_count_handles; /**< Handles of our characteristic BLE_UUID_VAL_COUNT_CHAR */
//    ble_gatts_char_handles_t    char_val_cmd_handles;   /**< Handles of our characteristic BLE_UUID_VAL_CMD_CHAR */
//} ble_os_t;
//

// (ble_os_t * p_os, ble_os_evt_t * p_evt);

/**@brief Function for handling our service event.
 *
 * @param[in]   p_our_service   Our Service structure.
 * @param[in]   p_ble_evt       Event received from the BLE stack.
 */
static void on_our_service_evt_handler(ble_os_t * p_our_service, ble_os_evt_t * p_ble_evt);

/**@brief Function for handling the GATT module's events.
 *
 * @details Handles all events from the GATT module of interest to the Custom Service.
 *
 * @param[in]   p_os       Our Service structure.
 * @param[in]   p_gatt_evt  Event received from the GATT module.
 */
void ble_os_on_gatt_evt(ble_os_t * p_os, nrf_ble_gatt_evt_t const * p_gatt_evt);


/**@brief Function for handling BLE Stack events related to our service and characteristic.
 *
 * @details Handles all events from the BLE stack of interest to Our Service.
 *
 * @param[in]   p_our_service       Our Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_our_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

/**@brief Function for initializing our new service.
 *
 * @param[in]   p_our_service       Pointer to Our Service structure.
 */
void our_service_init(ble_os_t * p_our_service);

/**@brief Function for updating and sending new characteristic values
 *
 * @details The application calls this function whenever our timer_timeout_handler triggers
 *
 * @param[in]   p_our_service                     Our Service structure.
 * @param[in]   characteristic_value     New characteristic value.
 */
uint32_t our_service_characteristic_update(ble_os_t *p_our_service, uint8_t *p_data, uint8_t data_len);
//void our_service_characteristic_update(ble_os_t *p_our_service, int32_t *temperature_value);
//void our_service_characteristic_update(ble_os_t *p_our_service, int8_t *p_transfer_dataset);

void our_service_send_data_control(ble_os_t *p_our_service, uint8_t *p_data, uint8_t num_entries, uint8_t data_len_entry, bool restart);

#endif  /* _ OUR_SERVICE_H__ */
