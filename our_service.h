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
#include "time.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "ble_date_time.h"
#include "nrf_ble_gatt.h"

#ifdef __cplusplus
extern "C" {
#endif


/**@brief   Macro for defining a ble_os instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_OS_BLE_OBSERVER_PRIO    2
#define BLE_OS_DEF(_name)                                       \
static ble_os_t _name;                                          \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                             \
                     BLE_OS_BLE_OBSERVER_PRIO,                  \
                     ble_os_on_ble_evt, &_name)

// Offline Buffer
//  with RACP/DB now we have 16 byte/entry
//      20 KB = 20.000 Byte -> 1.250 entries
//#ifdef USE_OFFLINE_FUNCTION
#define OFFLINE_BUFFER_RESERVED_BYTE    20000   // 20 KB RAM reserved
#define OFFLINE_BUFFER_SIZE_PER_ENTRY   16      // uint8_t
#define OFFLINE_BUFFER_SIZE             (OFFLINE_BUFFER_RESERVED_BYTE / OFFLINE_BUFFER_SIZE_PER_ENTRY)
//#endif // USE_OFFLINE_FUCTION


/**@brief Our Service feature */
#define BLE_OS_FEATURE_LOW_BATT                         0x0001  /**< Low Battery Detection During Measurement Supported */
#define BLE_OS_FEATURE_TEMPERATURE                      0x0002  /**< Temperature Measurement Supported */
#define BLE_OS_FEATURE_TEMPERATURE_ALERT                0x0004  /**< Temperature Alert Supported */
#define BLE_OS_FEATURE_HUMIDITY                         0x0008  /**< Humidity Measurement Supported */
#define BLE_OS_FEATURE_HUMIDITY_ALERT                   0x0010  /**< Humidity Alert Supported */
#define BLE_OS_FEATURE_ACCEL                            0x0020  /**< Accel Alert Supported */
#define BLE_OS_FEATURE_ACCEL_ALERT                      0x0040  /**< Accel Alert Supported */
#define BLE_OS_FEATURE_MALFUNC                          0x0100  /**< Sensor Malfunction Detection Supported */
#define BLE_OS_FEATURE_TIME_FAULT                       0x0200  /**< Time Fault Supported */
#define BLE_OS_FEATURE_MULTI_BOND                       0x0400  /**< Multiple Bond Supported */

/**@brief Our Service sensor status annunciation */
#define BLE_OS_MEAS_STATUS_BATT_LOW                     0x0001  /**< Device battery low at time of measurement */
#define BLE_OS_MEAS_STATUS_SENSOR_FAULT                 0x0002  /**< Sensor malfunction or faulting at time of measurement */
#define BLE_OS_MEAS_STATUS_GENERAL_FAULT                0x0004  /**< General device fault has occurred in the sensor */
#define BLE_OS_MEAS_STATUS_TIME_FAULT                   0x0008  /**< Time fault has occurred in the sensor and time may be inaccurate */
#define BLE_OS_MEAS_STATUS_TIME_NOT_SET                 0x0010  /**< Time is not set by CTS, thus showing seconds since startup */


/**@brief Our Service event type. */
typedef enum
{
    BLE_OS_EVT_NOTIFICATION_ENABLED,                            /**< Our service value notification enabled event. */
    BLE_OS_EVT_NOTIFICATION_DISABLED                            /**< Our service value notification disabled event. */
} ble_os_evt_type_t;

/**@brief Our Service event. */
typedef struct
{
    ble_os_evt_type_t evt_type;                                 /**< Type of event. */
} ble_os_evt_t;

// Forward declaration of the ble_os_t type.
typedef struct ble_os_s ble_os_t;

/**@brief Our Service event handler type. */
typedef void (*ble_os_evt_handler_t) (ble_os_t * p_os, ble_os_evt_t * p_evt);

/**@brief Our Service structure. This contains our service measurement value. */
typedef struct
{
    uint16_t        sequence_number;                            /**< Sequence number */
    time_t          time_stamp;                                 /**< Time stamp */
    uint16_t        temperature;                                /**< Sensor temperature value */
    uint16_t        humidity;                                   /**< Sensor humidity value */
} ble_os_meas_t;

/**@brief Our service measurement record */
typedef struct
{
    ble_os_meas_t   meas;                                       /**< Our Service measurement */
} ble_os_rec_t;

/**@brief Our service status data and update record. Order and content as in our_service_db record */
typedef struct
{
   uint8_t          flags;                                      /**< flags and update request */
   uint8_t          sequence_number[2];                         /**< Sequence number, LSB first */
   uint8_t          time_stamp[4];                              /**< Time stamp, LSB first */
   uint8_t          available_db_entries[2];                    /**< Number of availalbe DB entries */
   uint8_t          max_db_entries[2];                          /**< Max. number of DB entries = DB size */
} ble_os_status_data_update_t;


/**@brief Our Service init structure. This contains all options and data needed for
 *        initialization of the service. */
typedef struct
{
    ble_os_evt_handler_t        evt_handler;                    /**< Event handler to be called for handling events in Our Service. */
    ble_srv_error_handler_t     error_handler;                  /**< Function to be called in case of an error. */
    uint16_t                    feature;                        /**< Feature value indicating supported features. */
    uint16_t                    annunciation;                   /**< Initial annunciation value. */
    ble_os_status_data_update_t status_update;                  /**< Initial data and update */
    security_req_t              os_meas_cccd_wr_sec;            /**< Security requirement for writing our service measurement characteristic CCCD. */
    security_req_t              os_feature_rd_sec;              /**< Security requirement for reading our service feature characteristic. */
    security_req_t              os_annunciation_rd_sec;         /**< Security requirement for reading our service annunciation characteristic. */
    security_req_t              os_annunciation_wr_sec;         /**< Security requirement for writing our service annunciation characteristic. */
    security_req_t              racp_cccd_wr_sec;               /**< Security requirement for writing RACP Characteristic CCCD. */
    security_req_t              racp_wr_sec;                    /**< Security requirement for writing RACP Characteristic. (Service specification mandates authentication) */
} ble_os_init_t;

/**@brief Our Service service structure. This contains various status information for the service. */
struct ble_os_s
{
    ble_os_evt_handler_t        evt_handler;                    /**< Event handler to be called for handling events in Our Service. */
    ble_srv_error_handler_t     error_handler;                  /**< Function to be called in case of an error. */
    uint16_t                    service_handle;                 /**< Handle of Our Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t    osm_handles;                    /**< Handles related to the Our Service Measurement characteristic. */
    ble_gatts_char_handles_t    osf_handles;                    /**< Handles related to the Our Service Feature characteristic. */
    ble_gatts_char_handles_t    racp_handles;                   /**< Handles related to the Record Access Control Point characteristic. */
    ble_gatts_char_handles_t    osa_handles;                    /**< Handles related to the Our Service Annunciation characteristic. */
    ble_gatts_char_handles_t    ossu_handles;                   /**< Handles related to the Our Service status data and update characteristic. */
    uint16_t                    conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint16_t                    feature;                        /**< Features provided by the device. */
    uint16_t                    annunciation;                   /**< Annunciation given by the device. Will be reset if read. (TODO) */
    ble_os_status_data_update_t status_update;                  /**< Status data and update */
};

// randomly generated 128-bit base UUID: 612f3d33-37f5-4c4f-9ff2-320c4ba2b73c
#define BLE_UUID_OUR_BASE_UUID                  {0x3C, 0xB7, 0xA2, 0x4B, 0x0C, 0x32, 0xF2, 0x9F, 0x4F, 0x4C, 0xF5, 0x37, 0x00, 0x00, 0x2F, 0x61} 
#define BLE_UUID_OUR_SERVICE                    0x1400  // 16-bit service UUIDs
#define BLE_UUID_OUR_SERVICE_MEASUREMENT_CHAR   0x1401  // 16-bit characteristic UUID for the masurement values
#define BLE_UUID_OUR_SERVICE_FEATURE_CHAR       0x1402  // 16-bit characteristic UUID for the feature values
#define BLE_UUID_OUR_SERVICE_ANNUNCIATION_CHAR  0x1403  // 16-bit characteristic UUID for the annunciation values
#define BLE_UUID_OUR_SERVICE_STATUS_DATA_CHAR   0x1404  // 16-bit characteristic UUID for status data and update


/**@brief Function for initializing the Our Service service.
 *
 * @details This call allows the application to initialize the Our Service service.
 *
 * @param[out]  p_os        Our Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_os_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
ret_code_t ble_os_init(ble_os_t * p_os, ble_os_init_t const * p_os_init);


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Glucose Service.
 *
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 * @param[in]   p_context   Our Service structure.
 */
void ble_os_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief Function for reporting a new Our Service measurement to the our service module.
 *
 * @details The application calls this function after having performed a new Our Service measurement.
 *          The new measurement is recorded in the RACP database.
 *
 * @param[in]   p_os                    Our Service structure.
 * @param[in]   p_rec                   Pointer to Our Service record.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
ret_code_t ble_os_sensor_new_meas(ble_os_t * p_os, ble_os_rec_t * p_rec);


/**@brief Function for reporting a new Our Service annunciation.
 *
 * @details The application calls this function if a new annunciation is available.
 *
 * @param[in]   p_os                    Our Service structure.
 * @param[in]   p_rec                   Pointer to Our Service record.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
ret_code_t ble_os_new_annunciation(ble_os_t * p_os, uint16_t new_annunciation);



/**@brief Function for reporting a new Our Service status data update.
 *
 * @details The application calls this function if a new status data is available.
 *
 * @param[in]   p_os                    Our Service structure.
 * @param[in]   new_status_update       Pointer to Our Service status data update record.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
ret_code_t ble_os_new_status_data_update(ble_os_t * p_os, ble_os_status_data_update_t new_status_update);


#ifdef __cplusplus
}
#endif

#endif // OUR_SERVICE_H__

/** @} */

