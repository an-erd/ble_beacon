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
/** @file
 *
 * @defgroup ble_beacon main.c
 * @{
 * @ingroup ble_beacon
 * @brief Beacon Transmitter Application main file.
 *
 * This file contains the source code for an Beacon transmitter which sends
 * temperature, humidity, accelerator data and battery level in an
 * low power configuration.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "nrf_dfu_ble_svci_bond_sharing.h"
#include "nrf_svci_async_function.h"
#include "nrf_svci_async_handler.h"

#include "boards.h"
#include "nordic_common.h"


#include "nrf.h"
#include "nrfx_saadc.h"
#include "nrf_drv_ppi.h"
#include "bsp.h"
#include "bsp_config.h"
#include "app_error.h"
#include "nrf_soc.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "nrf_sdh_soc.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_dfu.h"
#include "ble_cts_c.h"
#include "ble_db_discovery.h"
#include "ble_dis.h"
#include "ble_bas.h"
#include "ble_racp.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "app_timer.h"
#include "app_scheduler.h"
#include "app_gpiote.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_power.h"
#include "nrf_drv_clock.h"
#include "uicr_config.h"
#include "sht3.h"
#include "kx022.h"
#include "our_service.h"
#include "our_service_db.h"
#include "nrf_calendar.h"
#include "led_indication.h"
#include "compiler_abstraction.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_bootloader_info.h"

#include "fds.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"

#if defined( __GNUC__ ) && (__LINT__ == 0)
    // This is required if one wants to use floating-point values in 'printf'
    // (by default this feature is not linked together with newlib-nano).
    // Please note, however, that this adds about 13 kB code footprint...
    __ASM(".global _printf_float");
#endif

// Opt in/out services for power consumption measurement
#undef  GOTO_SYSTEM_OFF
#define USE_PWR_MANAGEMENT_INIT
#define USE_LOG_INIT
#define USE_DCDCEN
#define USE_LFCLK_APP_TIMER
#define USE_BSP
#undef  USE_NFC                 // not yet implemented!
#define USE_TWI
#define USE_SENSOR
#define USE_SENSORINIT_SHT3
#define USE_SENSORINIT_KX022
#define USE_SENSOR_SHT3
#define USE_SENSOR_KX022
#define USE_SENSOR_SAADC
#define USE_APPTIMER
#define USE_OFFLINE_FUNCTION
#define USE_SCHEDULER
#define USE_GAP_GATT
#define USE_CONNPARAMS_PEERMGR
#define USE_CONN_ADV_INIT       // for startup; else: non-conn, non-scan adv
#define USE_ADVERTISING
#define USE_OUR_SERVICES
#define USE_CTS
#define USE_DIS
#define USE_DFU
#define USE_BUTTONLESS_DFU

// App Timer defines
APP_TIMER_DEF(m_repeated_timer_init);                       /**< Handler for repeated timer for init process (sensor, offline buffer, ...). */
APP_TIMER_DEF(m_repeated_timer_read_saadc);                 /**< Handler for repeated timer used to read battery level by SAADC. */
APP_TIMER_DEF(m_repeated_timer_read_sensor);                /**< Handler for repeated timer used to read TWI sensors. */
APP_TIMER_DEF(m_singleshot_timer_read_sensor_step);         /**< Handler for single shot timer for TWI sensor, steps. */
APP_TIMER_DEF(m_repeated_timer_update_offlinebuffer);       /**< Handler for repeated timer to update offline buffer. */
APP_TIMER_DEF(m_singleshot_timer_config_mode);              /**< Handler for single shot timer for config mode (after long press for X secs). */
APP_TIMER_DEF(m_singleshot_timer_delete_bonds);             /**< Handler for single shot timer for initiating delete bonds. */
#define APP_TIMER_TICKS_INIT                    APP_TIMER_TICKS(200)
#define APP_TIMER_TICKS_SAADC                   APP_TIMER_TICKS(60000)      // every 1 min
#define APP_TIMER_TICKS_SENSOR                  APP_TIMER_TICKS(15000)      // every 15 secs
#define APP_TIMER_TICKS_UPDATE_OFFLINEBUFFER    APP_TIMER_TICKS(300000)     // every 5 min, overall with interval every 15 min
#define OFFLINE_BUFFER_SAMPLE_INTERVAL          1                           // in multiples of APP_TIMER_TICKS_UPDATE_OFFLINEBUFFER
#define APP_TIMER_TICKS_CONFIG_MODE             APP_TIMER_TICKS(10000)      // inactivity timer for config mode
#define APP_TIMER_TICKS_WAIT_DELETE_BONDS       APP_TIMER_TICKS(3000)       // wait X secs

// SAADC defines
#define SAADC_CALIBRATION_INTERVAL  5       // SAADC calibration interval relative to NRF_DRV_SAADC_EVT_DONE event
#define SAADC_SAMPLES_IN_BUFFER     1       // Number of SAADC samples in RAM before returning a SAADC event
#define SAADC_BURST_MODE            0       // Set to 1 to enable BURST mode, otherwise set to 0.

// SAADC forward declaration
static void saadc_init(void);

// SAADC variables
static nrf_saadc_value_t    m_buffer_pool[2][SAADC_SAMPLES_IN_BUFFER];
static uint32_t             m_adc_evt_counter = 0;
static bool                 m_saadc_initialized = false;      

// SAADC reference and conversion data
#define ADC_REF_VOLTAGE_IN_MILLIVOLTS   600     /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION    6       /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS  0       /**< Typical forward voltage drop of the diode (270 mV), but no diode on this beacon. */
#define ADC_RES_10BIT                   1024    /**< Maximum digital value for 10-bit ADC conversion. */
#define ADC_RES_12BIT                   4096    /**< Maximum digital value for 12-bit ADC conversion. */
#define ADC_RES_14BIT                   16384   /**< Maximum digital value for 14-bit ADC conversion. */

/**@brief Macro to convert the result of ADC conversion in millivolts.
 *
 * @param[in]  ADC_VALUE   ADC result.
 *
 * @retval     Result converted to millivolts.
 */
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_12BIT) * ADC_PRE_SCALING_COMPENSATION)


// TWI defines and variables
#define TWI_INSTANCE_ID     0
static const nrf_drv_twi_t  m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

// Pin number for indicating communication with sensors
#ifdef BSP_LED_0
    #define READ_ALL_INDICATOR  BSP_BOARD_LED_0
#else
    #error "Please choose an output pin"
#endif

// Button mode
static bool m_button_config_mode = false;

// Sensor defines
#define BUFFER_SIZE             21  // read buffer from sensors: temp+hum (6=2*msb,lsb,crc) + xyz (6=3*lsb,msb) + INT_REL (5) + INS1 (4)
static uint8_t m_buffer[BUFFER_SIZE];

#if (BUFFER_SIZE < 21)
    #error Buffer too small.
#endif

// Offline Buffer
#ifdef USE_OFFLINE_FUNCTION
ret_code_t offline_buffer_update(uint8_t *buffer);
#endif // USE_OFFLINE_FUCTION

// BLE defines 
#define APP_BLE_OBSERVER_PRIO           3                                   /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                   /**< A tag identifying the SoftDevice BLE configuration. */
#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(1000, UNIT_0_625_MS)  /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */
#define CONNECTABLE_ADV_INTERVAL        MSEC_TO_UNITS(1000, UNIT_0_625_MS)  /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)    /**< Minimum acceptable connection interval (was: 0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)    /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                   /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)     /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(2000)               /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (was: 5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)              /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                   /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                   /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                   /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                   /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                   /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                   /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                   /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                  /**< Maximum encryption key size. */

// Initialise the BLE advertising data with some rememberable values, only used during advertising_init()
#define APP_COMPANY_IDENTIFIER  0x0059          /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
#define APP_BEACON_INFO_LENGTH  0x10            /**< Total length of information advertised by the Beacon. */
#define APP_MAJOR_VALUE         0x00, 0x07      /**< Major value used to identify Beacons. */
#define APP_MINOR_VALUE         0x00, 0xFF      /**< Minor value used to identify Beacons. -> caution: see UICR */
#define APP_DATA_TEMP           0xfd, 0xfd      /**< Temperature data. */
#define APP_DATA_HUM            0xfe, 0xde      /**< Humidity data. */
#define APP_DAT_X               0xaa, 0xaa      /**< Acceleration X data. */
#define APP_DAT_y               0xbb, 0xbb      /**< Acceleration Y data. */
#define APP_DAT_Z               0xcc, 0xcc      /**< Acceleration Z Temperature data. */
#define APP_DAT_BATTERY         0xFF, 0xFF      /**< Battery voltage data. */

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
#define MAJ_VAL_OFFSET_IN_MANUF_DATA    0           /**< Position of the MSB of the Major Value in m_beacon_info array. */
#define UICR_ADDRESS                    0x10001080  /**< Address of the UICR register  */
#endif

#define DEAD_BEEF                       0xDEADBEEF  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define PAYLOAD_OFFSET_IN_BEACON_INFO_ADV   11      /**< First position to write the payload to in enc advdata */
#define PAYLOAD_OFFSET_BATTERY_INFO     (PAYLOAD_OFFSET_IN_BEACON_INFO_ADV + 10)  /**< Position to write the battery voltage payload to */									

// BLE Services
uint8_t  m_device_name[16] = "BxFFFF";              /**< Name of device, temporary value if not set. */
static ble_uuid_t m_adv_uuids[] = 
{
    { BLE_UUID_OUR_SERVICE, BLE_UUID_TYPE_BLE  },   /**< 16-bit UUID for our service. */
};

// BLE Advertising Modes
typedef enum
{
    APP_ADV_NONE_SENSOR_NONE = 0,                   /**< The device will not advertise and not get sensor values. */
    APP_ADV_NONE,                                   /**< The device will not advertise at all, but get sensor values. */
    APP_ADV_NONSCAN_NONCONN,                        /**< The device will advertise in non-scannable, non-connectable mode. */
    APP_ADV_SCAN_CONN,                              /**< The device will advertise in scannable, connectable mode. */
    APP_ADV_MAXNUM
} app_advertising_mode_t;

// Peer Manager variables 
#ifdef USE_CONNPARAMS_PEERMGR
static pm_peer_id_t m_peer_id;                                                      /**< Device reference handle to the current bonded central. */
static pm_peer_id_t m_whitelist_peers[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];            /**< List of peers currently in the whitelist. */
static uint32_t     m_whitelist_peer_cnt;                                           /**< Number of peers currently in the whitelist. */
#endif // USE_CONNPARAMS_PEERMGR
static bool m_erase_bonds = false;
// Peer Manager forward declaration
static void peer_list_get(pm_peer_id_t * p_peers, uint32_t * p_size);
static void delete_bonds(void);

// Scheduler defines
#ifdef USE_SCHEDULER
#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE                   /**< Maximum size of scheduler events. Note that scheduler BLE stack events do not contain any data, as the events are being pulled from the stack in the event handler. */
#ifdef SVCALL_AS_NORMAL_FUNCTION
#define SCHED_QUEUE_SIZE                20                                          /**< Maximum number of events in the scheduler queue. More is needed in case of Serialization. */
#else
#define SCHED_QUEUE_SIZE                10                                          /**< Maximum number of events in the scheduler queue. */
#endif
#endif // USE_SCHEDULER

// Our Service defines
BLE_OS_DEF(m_our_service);

// DIS - Device Information Service defines
#ifdef USE_DIS
// Values are taken from ble_beacon_dis.inc and ble_beacon_dis_sw.inc
#include "ble_beacon_dis.inc"
#include "ble_beacon_dis_sw.inc"
#define DIS_MANUFACTURER_NAME           BLE_BEACON_DIS_MANUFACTURER_NAME            /**< Manufacturer. Will be passed to Device Information Service. */
#define DIS_MODEL_NUMBER                BLE_BEACON_DIS_MODEL_NUMBER
#define DIS_SERIAL_NUMBER               BLE_BEACON_DIS_SERIAL_NUMBER
#define DIS_HW_REV                      BLE_BEACON_DIS_HW_REV
#define DIS_SW_REV                      BLE_BEACON_DIS_SW_REV                       /**< Software, use "git describe --tags". */
#define DIS_FW_REV                      BLE_BEACON_DIS_FW_REV                       /**< Firmware, use version of SDK/SoftDevice. */
#endif // USE_DIS

// CTS - Current Time Service defiens
#ifdef USE_CTS
BLE_CTS_C_DEF(m_cts_c);                                         /**< Current Time service instance. */
BLE_DB_DISCOVERY_DEF(m_ble_db_discovery);                       /**< DB discovery module instance. */
#endif

// Battery Service 
BLE_BAS_DEF(m_bas);                                             /**< Structure used to identify the battery service. */

#ifdef USE_GAP_GATT
NRF_BLE_GATT_DEF(m_gatt);                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                         /**< Context for the Queued Write module.*/
#endif // USE_GAP_GATT

NRF_BLE_GQ_DEF(m_ble_gatt_queue, NRF_SDH_BLE_CENTRAL_LINK_COUNT, NRF_BLE_GQ_QUEUE_SIZE);

// BLE Advertising variables and structs
static ble_gap_adv_params_t m_adv_params;                       /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t  m_addl_adv_manuf_data[APP_BEACON_INFO_LENGTH];  /**< Value of the additional manufacturer specific data that will be placed in air (initialized to all zeros). */
static uint8_t  m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;  /**< Advertising handle used to identify an advertising set. */
static uint8_t  m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];   /**< Buffer for storing an encoded advertising set. */
static uint8_t  m_enc_srdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];    /**< Buffer for storing an encoded scan response set. */
static app_advertising_mode_t m_adv_mode = APP_ADV_NONE;        /**< Advertising mode used, e.g., non-scan/non-conn, scan/conn, none). */
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;        /**< Handle of the current connection. */
static bool     m_ble_adv_on_disconnect_disabled = false;       /**< Disable advertising after disconnect, used only for DFU update procedure. */

// BLE Advertising forward declaration
static void advertising_start(bool erase_bonds);
static void advertising_reconfig(app_advertising_mode_t adv_mode);

/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = m_enc_srdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    }
};

#ifdef USE_CTS
static char const * day_of_week[] =
{
    "Unknown",
    "Monday",
    "Tuesday",
    "Wednesday",
    "Thursday",
    "Friday",
    "Saturday",
    "Sunday"
};

static char const * month_of_year[] =
{
    "Unknown",
    "January",
    "February",
    "March",
    "April",
    "May",
    "June",
    "July",
    "August",
    "September",
    "October",
    "November",
    "December"
};
#endif

/**@brief Function for initializing logging. */
static void log_init()
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing power management.
 */
static void power_management_init()
{
    APP_ERROR_CHECK(nrf_pwr_mgmt_init());
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle()
{
    if (NRF_LOG_PROCESS() == false){
        nrf_pwr_mgmt_run();
    }
}

/**@brief Function for initializing clock source. */
static void lfclk_config()
{
    // Initialize the clock source specified in the nrf_drv_config.h file, i.e. the CLOCK_CONFIG_LF_SRC constant
    ret_code_t err_code = nrf_drv_clock_init();                  			
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL);
}

static void config_event_key_pressed()
{
    ret_code_t err_code;

    if(m_button_config_mode)
    {
        // restart config mode timer
        err_code = app_timer_stop(m_singleshot_timer_config_mode);
        APP_ERROR_CHECK(err_code);

        err_code = app_timer_start(m_singleshot_timer_config_mode, APP_TIMER_TICKS_CONFIG_MODE, NULL); 
        APP_ERROR_CHECK(err_code);

        m_adv_mode = (m_adv_mode + 1) % APP_ADV_MAXNUM;
        advertising_reconfig(m_adv_mode);
        NRF_LOG_DEBUG("set m_adv_scan_response %d", m_adv_mode);
    }
}

static void config_event_key_long_pressed()
{
    ret_code_t err_code;

    if(!m_button_config_mode)
    {
        m_button_config_mode = true;
            
        err_code = app_timer_start(m_singleshot_timer_config_mode, APP_TIMER_TICKS_CONFIG_MODE, NULL); 
        APP_ERROR_CHECK(err_code);

        led_indication_start(LED_INDICATION_6);
    } 
    else {
        err_code = app_timer_start(m_singleshot_timer_delete_bonds, APP_TIMER_TICKS_WAIT_DELETE_BONDS, NULL); 
        APP_ERROR_CHECK(err_code);
    }
}




/**@brief Function for handling BSP events.
 *
 * @details  This function will handle the BSP events as button press.
 */
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;
    static bool current_leds        = false;

    NRF_LOG_DEBUG("bsp_event_handler, button %d", event);
	
    switch (event)
    {
    case BSP_EVENT_KEY_0: // button on beacon pressed
        NRF_LOG_DEBUG("button BSP_EVENT_KEY_0");
        config_event_key_pressed();
        break;

    case BSP_EVENT_KEY_0_RELEASED: // button on beacon released
        NRF_LOG_DEBUG("button BSP_EVENT_KEY_0_RELEASED");
        break;
    
    case BSP_EVENT_KEY_0_LONG: // button on beacon long pressed
        NRF_LOG_DEBUG("button BSP_EVENT_KEY_0_LONG");
        config_event_key_long_pressed();
        break;

    case BSP_EVENT_KEY_1: // button on jig pressed
        NRF_LOG_DEBUG("button BSP_EVENT_KEY_1");
        config_event_key_pressed();
        break;
    
    case BSP_EVENT_KEY_1_RELEASED: // button on jig released
        NRF_LOG_DEBUG("button BSP_EVENT_KEY_1_RELEASED");
        break;
    
    case BSP_EVENT_KEY_1_LONG: // button on jig long pressed
        NRF_LOG_DEBUG("button BSP_EVENT_KEY_1_LONG"); 
        config_event_key_long_pressed();
        break;

    case BSP_EVENT_WHITELIST_OFF:
        NRF_LOG_DEBUG("BSP_EVENT_WHITELIST_OFF");    
#ifdef USE_CTS
// TODO
//        if (m_cts_c.conn_handle == BLE_CONN_HANDLE_INVALID)
//        {
//            err_code = ble_advertising_restart_without_whitelist(&m_advertising);
//            if (err_code != NRF_ERROR_INVALID_STATE)
//            {
//                APP_ERROR_CHECK(err_code);
//            }
//        }
#endif
        break;

    default:
        break;
    }
}


/**@brief Function for initializing bsp module.
 */
void bsp_configuration()
{
    uint32_t err_code;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing TWI.
 */
static void twi_config()
{
    nrf_drv_twi_config_t const config = {
        .scl                = ARDUINO_SCL_PIN,
        .sda                = ARDUINO_SDA_PIN,
        .frequency          = NRF_DRV_TWI_FREQ_100K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOWEST,
        .clear_bus_init     = false
    };

    APP_ERROR_CHECK(nrf_drv_twi_init(&m_twi, &config, NULL, NULL));	// blocking TWI
    nrf_drv_twi_enable(&m_twi);
}

/**@brief Function for performing battery measurement and updating the Battery Level characteristic
 *        in Battery Service.
 */
static void battery_level_update(uint8_t battery_level)
{
    ret_code_t err_code;

    err_code = ble_bas_battery_level_update(&m_bas, battery_level, BLE_CONN_HANDLE_ALL);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
}

/**@brief Function for handling the ADC interrupt.
 *
 * @details  This function will fetch the conversion result from the ADC, convert the value into
 *           percentage and send it to peer.
 */
void saadc_event_handler(nrfx_saadc_evt_t const * p_event)
{
    uint32_t            err_code;
    nrf_saadc_value_t   adc_result;
    uint8_t             percentage_batt_lvl;
    static uint16_t     m_battery_millivolts = 3333;    // default to some value, say 3333

//    NRF_LOG_DEBUG("saadc_event_handler");

    // regular SAADC sensor calibration 
    if (p_event->type == NRFX_SAADC_EVT_DONE){
        if((m_adc_evt_counter % SAADC_CALIBRATION_INTERVAL) == 0){
//            NRF_LOG_DEBUG("SAADC calibration starting...");
            NRF_SAADC->EVENTS_CALIBRATEDONE = 0; 
            nrf_saadc_task_trigger(NRF_SAADC_TASK_CALIBRATEOFFSET);
            while(!NRF_SAADC->EVENTS_CALIBRATEDONE);
            while(NRF_SAADC->STATUS == (SAADC_STATUS_STATUS_Busy << SAADC_STATUS_STATUS_Pos));
//            NRF_LOG_DEBUG("SAADC calibration complete ! \n");
        }

        m_adc_evt_counter++;
				
        err_code = nrfx_saadc_buffer_convert(p_event->data.done.p_buffer, SAADC_SAMPLES_IN_BUFFER);  //Set buffer so the SAADC can write to it again. This is either "buffer 1" or "buffer 2"
        APP_ERROR_CHECK(err_code);
			
        adc_result = p_event->data.done.p_buffer[0];

        m_battery_millivolts = ADC_RESULT_IN_MILLI_VOLTS(adc_result) +
                                    DIODE_FWD_VOLT_DROP_MILLIVOLTS;
			
        percentage_batt_lvl = battery_level_in_percent(m_battery_millivolts);

//        NRF_LOG_DEBUG("saadc_event_handler, done, perc %d, volts %d", percentage_batt_lvl, m_battery_millivolts);

        // update payload data in encoded advertising data
        uint8_t payload_idx = PAYLOAD_OFFSET_BATTERY_INFO;
        m_adv_data.adv_data.p_data[payload_idx++] = MSB_16(m_battery_millivolts);
        m_adv_data.adv_data.p_data[payload_idx++] = LSB_16(m_battery_millivolts);

        nrfx_saadc_uninit();                                                        // Unintialize SAADC to disable EasyDMA and save power
        NRF_SAADC->INTENCLR = (SAADC_INTENCLR_END_Clear << SAADC_INTENCLR_END_Pos); // Disable the SAADC interrupt
        NVIC_ClearPendingIRQ(SAADC_IRQn);                                           // Clear the SAADC interrupt if set
        m_saadc_initialized = false;                                                // Set SAADC as uninitialized

//        NRF_LOG_DEBUG("saadc_event_handler, m_battery_millivolts %d, percentage_batt_lvl %d", m_battery_millivolts,percentage_batt_lvl);
        battery_level_update(percentage_batt_lvl);
    }
}


/**@brief Function for configuring ADC to do battery level conversion.
 */
static void saadc_init()
{
    //Configure SAADC
    nrfx_saadc_config_t saadc_config = NRFX_SAADC_DEFAULT_CONFIG;
	
    // Initialize SAADC
    APP_ERROR_CHECK(nrfx_saadc_init(&saadc_config, saadc_event_handler));

    //Configure SAADC channel
    nrf_saadc_channel_config_t channel_config =
        NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);

    //Initialize SAADC channel
    APP_ERROR_CHECK(nrfx_saadc_channel_init(0, &channel_config));

		// Configure burst mode for channel 0
    if(SAADC_BURST_MODE){
        NRF_SAADC->CH[0].CONFIG |= 0x01000000; 
    }

    //Set SAADC buffer 1. The SAADC will start to write to this buffer
    APP_ERROR_CHECK(nrfx_saadc_buffer_convert(m_buffer_pool[0], SAADC_SAMPLES_IN_BUFFER));    

    //Set SAADC buffer 2. The SAADC will write to this buffer when buffer 1 is full. This will give the applicaiton time to process data in buffer 1.
    APP_ERROR_CHECK(nrfx_saadc_buffer_convert(m_buffer_pool[1],SAADC_SAMPLES_IN_BUFFER));   
}


/**@brief Function for initializing sensors.
 */
static void sensor_init()
{	
#ifdef USE_SENSORINIT_SHT3
    // SHT3
    static uint8_t config_sht3_0[2] = { (SHT3_SOFTRESET >> 8), (SHT3_SOFTRESET & 0xFF) };
    APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, SHT3_ADDR, config_sht3_0, 2, false));
#endif 

#ifdef USE_SENSORINIT_KX022
    // KX022 
    static uint8_t config_kx022_0[2] = {KX022_1020_REG_CNTL1, 0x00 };	// KX022_1020_STANDBY 
    APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, KX022_ADDR, config_kx022_0, 2, false));
#endif
}



/**@brief Function for processing all sensor data.
 *
 * @details  This function will calculate the mean values after reading from
 *           and prepare for adv packet
 */
void process_all_data()
{
#ifdef DEBUG
/*
        // calculate values from raw data, used only for debug. 
        // For adv package take already encoded data from sensor
        float   temp        = SHT3_GET_TEMPERATURE_VALUE(m_buffer[0], m_buffer[1]);
        float   humidity    = SHT3_GET_HUMIDITY_VALUE   (m_buffer[3], m_buffer[4]);
        int16_t x           = KX022_GET_ACC(m_buffer[ 6], m_buffer[ 7]);
        int16_t y           = KX022_GET_ACC(m_buffer[ 8], m_buffer[ 9]);
        int16_t z           = KX022_GET_ACC(m_buffer[10], m_buffer[11]);

        // Log example: Temp: 220.00 | Hum:340.00 | X: -257, Y: -129, Z: 16204 
        NRF_LOG_RAW_INFO("Temp: " NRF_LOG_FLOAT_MARKER " | Hum:" NRF_LOG_FLOAT_MARKER " | ", NRF_LOG_FLOAT(temp), NRF_LOG_FLOAT(humidity));
        NRF_LOG_RAW_INFO("X %6d, Y %6d, Z %6d \n", x, y, z);
        NRF_LOG_INFO("m_buffer[0..11]:");
        NRF_LOG_RAW_HEXDUMP_INFO(m_buffer, 12);
//        NRF_LOG_RAW_INFO("INS1 %d, INS2 %d, INS3 %d, STAT %d\n",
//            m_buffer[17], m_buffer[18], m_buffer[19], m_buffer[20]); 
*/
#endif // DEBUG

    // update payload data in encoded advertising data
    uint8_t payload_idx = PAYLOAD_OFFSET_IN_BEACON_INFO_ADV;

    m_adv_data.adv_data.p_data[payload_idx++] = m_buffer[ 0];
    m_adv_data.adv_data.p_data[payload_idx++] = m_buffer[ 1];
    m_adv_data.adv_data.p_data[payload_idx++] = m_buffer[ 3];
    m_adv_data.adv_data.p_data[payload_idx++] = m_buffer[ 4];
    m_adv_data.adv_data.p_data[payload_idx++] = m_buffer[ 6];
    m_adv_data.adv_data.p_data[payload_idx++] = m_buffer[ 7];
    m_adv_data.adv_data.p_data[payload_idx++] = m_buffer[ 8];
    m_adv_data.adv_data.p_data[payload_idx++] = m_buffer[ 9];
    m_adv_data.adv_data.p_data[payload_idx++] = m_buffer[10];
    m_adv_data.adv_data.p_data[payload_idx++] = m_buffer[11];

#ifdef DEBUG
//    NRF_LOG_INFO("m_advertising.enc_advdata[0..30]:");
//    NRF_LOG_RAW_HEXDUMP_INFO(m_advertising.enc_advdata, 31);
#endif // DEBUG
}

/**@brief Function for multi-step retrieval of sensor data
 *
 * @details  This function will request and fetch the results from the sonsors
 *           in an multi-step approach implemented with app_timer
 */
static void read_all_sensors(bool restart)
{
    ret_code_t err_code;

    // KX022 
    static uint8_t config_kx022_0[2] = {KX022_1020_REG_CNTL1,   0x00 }; // KX022_1020_STANDBY 
    static uint8_t config_kx022_1[2] = {KX022_1020_REG_CNTL1, 	0x40 };	// KX022_1020_STANDBY | KX022_1020_HIGH_RESOLUTION
    static uint8_t config_kx022_2[2] = {KX022_1020_REG_ODCNTL, 	0x04 };	// KX022_1020_OUTPUT_RATE_200_HZ
    static uint8_t config_kx022_3[2] = {KX022_1020_REG_CNTL1, 	0xC0 };	// KX022_1020_OPERATE | KX022_1020_HIGH_RESOLUTION

    // SHT3
    static uint8_t config_SHT3_0[2]  = {SHT3_MEAS_HIGHREP >> 8, SHT3_MEAS_HIGHREP & 0xFF};

    uint8_t         reg[2];
    static uint8_t  step = 0;
    uint32_t        counter_current = 0;
    static uint32_t counter_read_sht3;  // counter value then SHT3 is ready
		
    if(restart)
        step = 0;

    // nested approach
    // step0
    //   - start long running (15ms) SHT3 retrieval first
    //   - initialize KX022 (still standby)
    //   - wait KX022 read config
    // step1
    //   - initialize KX022 to operation
    //   - wait for KX022 data ready
    // step2
    //   - read KX022 data
    //   - wait for SH3 completed
    // step3
    //   - read SHT3 data
    //   - call function to further process the read data
    switch(step){
    case 0:
        // SHT3
#ifdef USE_SENSOR_SHT3
        APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, SHT3_ADDR, config_SHT3_0, 2, false));
#endif // USE_SENSOR_SHT3
#ifdef USE_SENSOR_KX022
        // KX022 
        APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, KX022_ADDR, config_kx022_0, 2, false));
        APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, KX022_ADDR, config_kx022_1, 2, false));
        APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, KX022_ADDR, config_kx022_2, 2, false));
#endif // USE_SENSOR_KX022   

        // timer for next step, 6 ms (>1.2/ODR)
        err_code = app_timer_start(m_singleshot_timer_read_sensor_step, APP_TIMER_TICKS(6), NULL); // (6)
        APP_ERROR_CHECK(err_code);
    
        step++;
        break;
    
    case 1:
#ifdef USE_SENSOR_KX022
        // KX022 
        APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, KX022_ADDR, config_kx022_3, 2, false));
#endif // USE_SENSOR_KX022   
    
        // timer for next step,6 ms (>1.2/ODR)
        err_code = app_timer_start(m_singleshot_timer_read_sensor_step, APP_TIMER_TICKS(6), NULL); // (6)
        APP_ERROR_CHECK(err_code);

        step++;
        break;
    
    case 2:
#ifdef USE_SENSOR_KX022
        // KX022 
        reg[0] = KX022_1020_REG_XOUTL;
        APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, KX022_ADDR, reg, 1, true));
        // read 6 bytes (x (lsb+msb), y (lsb+msb), z (lsb+msb)
        APP_ERROR_CHECK(nrf_drv_twi_rx(&m_twi, KX022_ADDR, &m_buffer[6], 6));
        APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, KX022_ADDR, config_kx022_0, 2, false));
#endif // USE_SENSOR_KX022   

        // timer fillup for SHT3 15 ms (15 -6 -6 -running time of functions = ~1 ms)
        err_code = app_timer_start(m_singleshot_timer_read_sensor_step, APP_TIMER_TICKS(1), NULL); // (1)
        APP_ERROR_CHECK(err_code);

        step++;
        break;  // time to go until SHT3 is ready
    
    case 3:
#ifdef USE_SENSOR_SHT3
        // read 6 bytes (temp (msb+lsb+crc) and hum (msb+lsb+crc)
        APP_ERROR_CHECK(nrf_drv_twi_rx(&m_twi, SHT3_ADDR, &m_buffer[0], 6));
#endif // USE_SENSOR_SHT3
            
        process_all_data();
        // TODO work on INS1 and INT_REL
        //  KX022_READ_XYZ(&m_buffer[6])        // read 6 bytes (x (lsb+msb), y (lsb+msb), z (lsb+msb)
        //  KX022_READ_INS1(&m_buffer[17])      // read 4 bytes 
        //  KX022_READ_INT_REL(&m_buffer[12])   // read 5 byte interrupt source information
        
        step = 0;   // reset for next sensor retrieval cycle
        break;
    default:
        NRF_LOG_ERROR("read_all: default -> should not happen");
        break;
    }
}

static void repeated_timer_handler_read_saadc()
{
    if(m_adv_mode == APP_ADV_NONE_SENSOR_NONE)
    {
        return;
    }

#ifdef USE_SENSOR_SAADC
    if(!m_saadc_initialized) {
        saadc_init();
    }
    m_saadc_initialized = true;

    // Trigger the SAADC SAMPLE task
    nrfx_saadc_sample();
#endif // USE_SENSOR_SAADC
}

static void repeated_timer_handler_read_sensors()
{
    if(m_adv_mode == APP_ADV_NONE_SENSOR_NONE)
    {
        return;
    }

    // Trigger the sensor retrieval task from the beginning
    read_all_sensors(true);	// init w/step0
}

static void singleshot_timer_handler_read_sensor_step()
{
    // Trigger the sensor retrieval task for subsequent steps
    read_all_sensors(false);	// subsequent steps
}

static void repeated_timer_handler_update_offlinebuffer()
{
    ret_code_t err_code;
    static uint8_t timer_intervall_counter = OFFLINE_BUFFER_SAMPLE_INTERVAL;    // always do first one...

    timer_intervall_counter++;
    if(timer_intervall_counter < OFFLINE_BUFFER_SAMPLE_INTERVAL){
        return;
    }
    timer_intervall_counter = 0;

    if(ble_os_db_num_free_entries_get() == 0)
    {
        err_code = ble_os_db_record_delete(0);
        APP_ERROR_CHECK(err_code);
        NRF_LOG_DEBUG("Buffer was full, deleted oldest entry");
    }

#ifdef USE_OFFLINE_FUNCTION
    // Update offline buffer with current measurement regularly
    err_code = offline_buffer_update(m_buffer);
    if(err_code == NRF_ERROR_NO_MEM)
    {
        NRF_LOG_ERROR("Offline Buffer full");
    } else {
        APP_ERROR_CHECK(err_code);
    }
#endif // USE_OFFLINE_FUNCTION
}

static void singleshot_timer_handler_config_mode()
{
    m_button_config_mode = false;
    led_indication_start(LED_INDICATION_7);
}


static void singleshot_timer_handler_delete_bonds()
{
    static uint8_t step = 0;

    ret_code_t err_code;

    switch(step) {
    case 0:
        advertising_reconfig(APP_ADV_NONE);
        step++;
        
        err_code = app_timer_start(m_singleshot_timer_delete_bonds, APP_TIMER_TICKS(2000), NULL); 
        APP_ERROR_CHECK(err_code);
        break;
    case 1:
        led_indication_start(LED_INDICATION_5);
        delete_bonds();
        step = 0;
        break;
    }
}

static void repeated_timer_handler_init()
{
    ret_code_t err_code;
    static uint8_t step = 0;
     
    switch (step)
    {
    case 0:
        // SAADC
        NRF_LOG_DEBUG("repeated_timer_handler_init: step 0, SAADC");
        
        err_code = app_timer_start(m_repeated_timer_read_saadc, APP_TIMER_TICKS_SAADC, NULL);
        APP_ERROR_CHECK(err_code);
        
        repeated_timer_handler_read_saadc();
        step++;

        break;
        
    case 1:
        // Sensor
        NRF_LOG_DEBUG("repeated_timer_handler_init: step 1, Sensor");
        
        err_code = app_timer_start(m_repeated_timer_read_sensor, APP_TIMER_TICKS_SENSOR, NULL);
        APP_ERROR_CHECK(err_code);

        repeated_timer_handler_read_sensors();
        step++;

        break;
        
    case 2:
        // Offline buffer update
        NRF_LOG_DEBUG("repeated_timer_handler_init: step 2, Offline buffer update");
        
        err_code = app_timer_start(m_repeated_timer_update_offlinebuffer, APP_TIMER_TICKS_UPDATE_OFFLINEBUFFER, NULL);
        APP_ERROR_CHECK(err_code);
        
        repeated_timer_handler_update_offlinebuffer();
        step++;

        break;
        
    case 3:
        // Start advertising
        NRF_LOG_DEBUG("repeated_timer_handler_init: step 3, stop init timer, start advertising");

        err_code = app_timer_stop(m_repeated_timer_init);
        APP_ERROR_CHECK(err_code);
#ifdef USE_ADVERTISING
        advertising_start(m_erase_bonds);
#endif // USE_ADVERTISING

        step++;

        break;
        
    default:
        NRF_LOG_ERROR("repeated_timer_handler_init: unknown step");
        break;
    }
}

static void timers_create()
{
    ret_code_t err_code;

    // Create timers
    err_code = app_timer_create(&m_repeated_timer_init,
                                APP_TIMER_MODE_REPEATED,
                                repeated_timer_handler_init);
    APP_ERROR_CHECK(err_code);
    
    err_code = app_timer_create(&m_repeated_timer_read_saadc,
                                APP_TIMER_MODE_REPEATED,
                                repeated_timer_handler_read_saadc);
    APP_ERROR_CHECK(err_code);
    
    err_code = app_timer_create(&m_repeated_timer_read_sensor,
                                APP_TIMER_MODE_REPEATED,
                                repeated_timer_handler_read_sensors);
    APP_ERROR_CHECK(err_code);
    
    err_code = app_timer_create(&m_singleshot_timer_read_sensor_step,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                singleshot_timer_handler_read_sensor_step);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_repeated_timer_update_offlinebuffer,
                                APP_TIMER_MODE_REPEATED,
                                repeated_timer_handler_update_offlinebuffer);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_singleshot_timer_config_mode,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                singleshot_timer_handler_config_mode);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_singleshot_timer_delete_bonds,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                singleshot_timer_handler_delete_bonds);
    APP_ERROR_CHECK(err_code);
}

static void timers_start()
{
    ret_code_t err_code;

    err_code = app_timer_start(m_repeated_timer_init, APP_TIMER_TICKS_INIT, NULL);
    APP_ERROR_CHECK(err_code);
}

#ifdef USE_OFFLINE_FUNCTION

/**@brief Function to update next entry in offline buffer.
 *
 * @details  This functions stores the next entry in the offline buffer, returns true on succes or false if buffer full.
 */
ret_code_t offline_buffer_update(uint8_t *buffer)
{   
    ret_code_t err_code;
    ble_os_rec_t rec;

    // NRF_LOG_DEBUG("size of ble_os_rec_t: %d", sizeof(rec)); // 12

    rec.meas.time_stamp = nrf_cal_get_time_long();
    memcpy(&rec.meas.temperature,   buffer,   2);
    memcpy(&rec.meas.humidity,      buffer+3, 2);   // skip CRC in temperature, thus +3

    err_code = ble_os_sensor_new_meas(&m_our_service, &rec);

    NRF_LOG_DEBUG("offline_buffer_update: seq %d, time %d, temp 0x%4X, humidity 0x%4X",
                    rec.meas.sequence_number, 
                    rec.meas.time_stamp, 
                    rec.meas.temperature, 
                    rec.meas.humidity);

    return err_code;
}
#endif // USE_OFFLINE_FUNCTION



/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            break;

        case BLE_ADV_EVT_SLOW:
            NRF_LOG_INFO("Slow advertising.");
            break;

        case BLE_ADV_EVT_IDLE:
            NRF_LOG_INFO("Idle advertising.");
            break;

#ifdef USE_CONNPARAMS_PEERMGR
        case BLE_ADV_EVT_FAST_WHITELIST:
            NRF_LOG_INFO("Fast advertising with WhiteList");
            break;

        case BLE_ADV_EVT_SLOW_WHITELIST:
            NRF_LOG_INFO("Slow advertising with WhiteList");
//            err_code = ble_advertising_restart_without_whitelist(&m_advertising);
//            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_WHITELIST_REQUEST:
        {
            ble_gap_addr_t whitelist_addrs[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            ble_gap_irk_t  whitelist_irks[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            uint32_t       addr_cnt = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
            uint32_t       irk_cnt  = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

            err_code = pm_whitelist_get(whitelist_addrs, &addr_cnt,
                                        whitelist_irks,  &irk_cnt);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_DEBUG("pm_whitelist_get returns %d addr in whitelist and %d irk whitelist",
                           addr_cnt,
                           irk_cnt);

            // Apply the whitelist.
//            err_code = ble_advertising_whitelist_reply(&m_advertising,
//                                                       whitelist_addrs,
//                                                       addr_cnt,
//                                                       whitelist_irks,
//                                                       irk_cnt);
//            APP_ERROR_CHECK(err_code);
        }
        break;
#endif // USE_CONNPARAMS_PEERMGR
        default:
            break;
    }
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;

#ifdef USE_CONNPARAMS_PEERMGR
    pm_handler_secure_on_connection(p_ble_evt);
#endif // USE_CONNPARAMS_PEERMGR
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
#ifdef USE_CTS
            if (p_ble_evt->evt.gap_evt.conn_handle == m_cts_c.conn_handle)
            {
                m_cts_c.conn_handle = BLE_CONN_HANDLE_INVALID;
            }
#endif
#ifdef USE_ADVERTISING
            if(!m_ble_adv_on_disconnect_disabled)
            {
                advertising_start(false);
            }
#endif // USE_ADVERTISING

            break;

        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
#ifdef USE_GAP_GATT
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
#endif // USE_GAP_GATT
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init()
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);

    // Call ble_our_service_on_ble_evt() to do housekeeping of ble connections related to our service and characteristics
//    NRF_SDH_BLE_OBSERVER(m_our_service_observer, APP_BLE_OBSERVER_PRIO, ble_os_on_ble_evt, (void*) &m_our_service);
}


#ifdef USE_SCHEDULER
/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}
#endif // USE_SCHEDULER


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)m_device_name,
                                          strlen(m_device_name));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

#ifdef USE_GAP_GATT
/**@brief GATT module event handler.
 */
static void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        NRF_LOG_INFO("GATT ATT MTU on connection 0x%x changed to %d.",
                     p_evt->conn_handle,
                     p_evt->params.att_mtu_effective);
    }

//    ble_os_on_gatt_evt(&m_our_service, p_evt);
}



/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);
}
#endif // USE_GAP_GATT


#ifdef USE_CTS
/**@brief Function for handling Database Discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective service instances.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    NRF_LOG_DEBUG("db_disc_handler handler called with event 0x%x", p_evt->evt_type);
    ble_cts_c_on_db_disc_evt(&m_cts_c, p_evt);
}

/**
 * @brief Database discovery collector initialization.
 */
static void db_discovery_init(void)
{
    ble_db_discovery_init_t db_init;

    memset(&db_init, 0, sizeof(ble_db_discovery_init_t));

    db_init.evt_handler  = db_disc_handler;
    db_init.p_gatt_queue = &m_ble_gatt_queue;

    ret_code_t err_code = ble_db_discovery_init(&db_init);
    APP_ERROR_CHECK(err_code);

}
#endif // USE_CTS


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

#ifdef USE_GAP_GATT
/**@brief Function for initializing the Queued Write Module.
 */
static void qwr_init(void)
{
    ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init_obj = {0};

    qwr_init_obj.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init_obj);
    APP_ERROR_CHECK(err_code);
}
#endif // USE_GAP_GATT

#ifdef USE_DIS
/**@brief Function for initializing Device Information Service.
 */
static void dis_init(void)
{
    ret_code_t          err_code;
    ble_dis_init_t      dis_init;
            
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str,  (char *)DIS_MANUFACTURER_NAME);
    ble_srv_ascii_to_utf8(&dis_init.serial_num_str,     (char *)DIS_SERIAL_NUMBER);
    ble_srv_ascii_to_utf8(&dis_init.model_num_str,      (char *)DIS_MODEL_NUMBER);
    ble_srv_ascii_to_utf8(&dis_init.hw_rev_str,         (char *)DIS_HW_REV);
    ble_srv_ascii_to_utf8(&dis_init.sw_rev_str,         (char *)DIS_SW_REV);
    ble_srv_ascii_to_utf8(&dis_init.fw_rev_str,         (char *)DIS_FW_REV);
    dis_init.dis_char_rd_sec = SEC_OPEN;

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}
#endif

#ifdef USE_CTS
/**@brief Function for handling the Current Time Service errors.
 *
 * @param[in]  nrf_error  Error code containing information about what went wrong.
 */
static void current_time_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the Current Time Service errors.
 *
 * @param[in] p_evt  Event received from the Current Time Service client.
 */
static void current_time_print(ble_cts_c_evt_t * p_evt)
{
    NRF_LOG_DEBUG("Current Time:");
    NRF_LOG_DEBUG("Date: %d.%d.%d", 
        p_evt->params.current_time.exact_time_256.day_date_time.date_time.day,
        p_evt->params.current_time.exact_time_256.day_date_time.date_time.month,
        p_evt->params.current_time.exact_time_256.day_date_time.date_time.year);
    NRF_LOG_DEBUG("Time: %d:%d:%d (%d/256)", 
        p_evt->params.current_time.exact_time_256.day_date_time.date_time.hours, 
        p_evt->params.current_time.exact_time_256.day_date_time.date_time.minutes, 
        p_evt->params.current_time.exact_time_256.day_date_time.date_time.seconds, 
        p_evt->params.current_time.exact_time_256.fractions256);
    NRF_LOG_FLUSH();
//    NRF_LOG_INFO("Adjust reason:");
//    NRF_LOG_INFO("Daylight savings %x", p_evt->params.current_time.adjust_reason.change_of_daylight_savings_time);
//    NRF_LOG_INFO("Time zone        %x", p_evt->params.current_time.adjust_reason.change_of_time_zone);
//    NRF_LOG_INFO("External update  %x", p_evt->params.current_time.adjust_reason.external_reference_time_update);
//    NRF_LOG_INFO("Manual update    %x", p_evt->params.current_time.adjust_reason.manual_time_update);
}


/**@brief Function for handling the Current Time Service event to update the nrf_calendar.
 *
 * @param[in] p_evt  Event received from the Current Time Service client.
 */
static void current_time_update_calendar(ble_cts_c_evt_t * p_evt)
{
    nrf_cal_set_time(
        p_evt->params.current_time.exact_time_256.day_date_time.date_time.year,
        p_evt->params.current_time.exact_time_256.day_date_time.date_time.month,
        p_evt->params.current_time.exact_time_256.day_date_time.date_time.day,
        p_evt->params.current_time.exact_time_256.day_date_time.date_time.hours,
        p_evt->params.current_time.exact_time_256.day_date_time.date_time.minutes,
        p_evt->params.current_time.exact_time_256.day_date_time.date_time.seconds);

}

/**@brief Function for handling the Current Time Service client events.
 *
 * @details This function will be called for all events in the Current Time Service client that
 *          are passed to the application.
 *
 * @param[in] p_evt Event received from the Current Time Service client.
 */
static void on_cts_c_evt(ble_cts_c_t * p_cts, ble_cts_c_evt_t * p_evt)
{
    ret_code_t err_code;
    time_t calendar_time_before_update, calendar_update_delta;
    bool do_db_backward_update = false;

    switch (p_evt->evt_type)
    {
        case BLE_CTS_C_EVT_DISCOVERY_COMPLETE:
            NRF_LOG_DEBUG("Current Time Service discovered on server.");
            err_code = ble_cts_c_handles_assign(&m_cts_c,
                                                p_evt->conn_handle,
                                                &p_evt->params.char_handles);
            APP_ERROR_CHECK(err_code);

#ifdef USE_CTS
            if (m_cts_c.conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_cts_c_current_time_read(&m_cts_c);
                if (err_code == NRF_ERROR_NOT_FOUND)
                {
                    NRF_LOG_DEBUG("Current Time Service is not discovered.");
                } else {
                    APP_ERROR_CHECK(err_code);
                }
            }
#endif

            break;

        case BLE_CTS_C_EVT_DISCOVERY_FAILED:
            NRF_LOG_WARNING("Current Time Service not found on server. ");
            // TODO currently do nothing...
            break;

        case BLE_CTS_C_EVT_DISCONN_COMPLETE:
            NRF_LOG_DEBUG("Disconnect Complete.");
            break;

        case BLE_CTS_C_EVT_CURRENT_TIME:
            NRF_LOG_DEBUG("Current Time received.");
            current_time_print(p_evt);
            if(!nrf_cal_get_initialized()){
                // we need to update the entries in the db with the current time which is available now.
                // So, take current calendar time before and after update, and the difference is the time to add to existing 
                // db entries, which gives the correct time stamps (but drift not corrected)
                do_db_backward_update = true;
                calendar_time_before_update = nrf_cal_get_time_long();
            }
            current_time_update_calendar(p_evt);

            if(do_db_backward_update)
            {
                calendar_update_delta = nrf_cal_get_time_long() - calendar_time_before_update;
                NRF_LOG_DEBUG("BLE_CTS_C_EVT_CURRENT_TIME: backward_update %d, time_before %d, time_after %d, time_delta %d",
                    (do_db_backward_update?1:0), calendar_time_before_update,  nrf_cal_get_time_long(), calendar_update_delta);
                ble_os_db_update_time_stamps(calendar_update_delta);
            }

            break;

        case BLE_CTS_C_EVT_INVALID_TIME:
            NRF_LOG_WARNING("Invalid Time received.");
            break;

        default:
            break;
    }
}



/**@brief Handler for shutdown preparation.
 *
 * @details During shutdown procedures, this function will be called at a 1 second interval
 *          untill the function returns true. When the function returns true, it means that the
 *          app is ready to reset to DFU mode.
 *
 * @param[in]   event   Power manager event.
 *
 * @retval  True if shutdown is allowed by this power manager handler, otherwise false.
 */
static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_DFU:
            NRF_LOG_INFO("Power management wants to reset to DFU mode.");
            // YOUR_JOB: Get ready to reset into DFU mode
            //
            // If you aren't finished with any ongoing tasks, return "false" to
            // signal to the system that reset is impossible at this stage.
            //
            // Here is an example using a variable to delay resetting the device.
            //
            // if (!m_ready_for_reset)
            // {
            //      return false;
            // }
            // else
            //{
            //
            //    // Device ready to enter
            //    uint32_t err_code;
            //    err_code = sd_softdevice_disable();
            //    APP_ERROR_CHECK(err_code);
            //    err_code = app_timer_stop_all();
            //    APP_ERROR_CHECK(err_code);
            //}
            break;

        default:
            // YOUR_JOB: Implement any of the other events available from the power management module:
            //      -NRF_PWR_MGMT_EVT_PREPARE_SYSOFF
            //      -NRF_PWR_MGMT_EVT_PREPARE_WAKEUP
            //      -NRF_PWR_MGMT_EVT_PREPARE_RESET
            return true;
    }

    NRF_LOG_INFO("Power management allowed to reset to DFU mode.");
    return true;
}


//lint -esym(528, m_app_shutdown_handler)
/**@brief Register application shutdown handler with priority 0.
 */
NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);


static void buttonless_dfu_sdh_state_observer(nrf_sdh_state_evt_t state, void * p_context)
{
    if (state == NRF_SDH_EVT_STATE_DISABLED)
    {
        // Softdevice was disabled before going into reset. Inform bootloader to skip CRC on next boot.
        nrf_power_gpregret2_set(BOOTLOADER_DFU_SKIP_CRC);

        //Go to system off.
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
    }
}

/* nrf_sdh state observer. */
NRF_SDH_STATE_OBSERVER(m_buttonless_dfu_state_obs, 0) =
{
    .handler = buttonless_dfu_sdh_state_observer,
};


static void advertising_config_get(ble_adv_modes_config_t * p_config)
{
    memset(p_config, 0, sizeof(ble_adv_modes_config_t));

// TODO
    // Set advertising modes and intervals
//    p_config->ble_adv_fast_enabled    = false;    // currently only use slow
//    p_config->ble_adv_fast_interval   = APP_FAST_ADV_INTERVAL;
//    p_config->ble_adv_fast_timeout    = APP_ADV_FAST_DURATION;
//    p_config->ble_adv_slow_enabled    = true;
//    p_config->ble_adv_slow_interval   = APP_SLOW_ADV_INTERVAL;
//    p_config->ble_adv_slow_timeout    = APP_ADV_SLOW_DURATION;
}


static void disconnect(uint16_t conn_handle, void * p_context)
{
    UNUSED_PARAMETER(p_context);

    ret_code_t err_code = sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("Failed to disconnect connection. Connection handle: %d Error: %d", conn_handle, err_code);
    }
    else
    {
        NRF_LOG_DEBUG("Disconnected connection handle %d", conn_handle);
    }
}



// YOUR_JOB: Update this code if you want to do anything given a DFU event (optional).
/**@brief Function for handling dfu events from the Buttonless Secure DFU service
 *
 * @param[in]   event   Event from the Buttonless Secure DFU service.
 */
static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event)
{
    switch (event)
    {
        case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
        {
            NRF_LOG_INFO("Device is preparing to enter bootloader mode.");

            // Prevent device from advertising on disconnect.
            m_ble_adv_on_disconnect_disabled = true;

            // Disconnect all other bonded devices that currently are connected.
            // This is required to receive a service changed indication
            // on bootup after a successful (or aborted) Device Firmware Update.
            uint32_t conn_count = ble_conn_state_for_each_connected(disconnect, NULL);
            NRF_LOG_INFO("Disconnected %d links.", conn_count);
            break;
        }

        case BLE_DFU_EVT_BOOTLOADER_ENTER:
            // YOUR_JOB: Write app-specific unwritten data to FLASH, control finalization of this
            //           by delaying reset by reporting false in app_shutdown_handler
            NRF_LOG_INFO("Device will enter bootloader mode.");
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
            NRF_LOG_ERROR("Request to enter bootloader mode failed asynchroneously.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            break;

        case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
            NRF_LOG_ERROR("Request to send a response to client failed.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            APP_ERROR_CHECK(false);
            break;

        default:
            NRF_LOG_ERROR("Unknown event from ble_dfu_buttonless.");
            break;
    }
}


/**@brief Function for initializing Buttonless DFU.
 */
static void dfu_init(void)
{
    ret_code_t       err_code;
    ble_dfu_buttonless_init_t dfus_init = {0};
    
    dfus_init.evt_handler = ble_dfu_evt_handler;

    err_code = ble_dfu_buttonless_init(&dfus_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing CTS.
 */
static void cts_init(void)
{
    ret_code_t       err_code;
    ble_cts_c_init_t   cts_init = {0};

    cts_init.evt_handler    = on_cts_c_evt;
    cts_init.p_gatt_queue   = &m_ble_gatt_queue;
    cts_init.error_handler  = current_time_error_handler;

    err_code               = ble_cts_c_init(&m_cts_c, &cts_init);
    APP_ERROR_CHECK(err_code);
}
#endif

/**@brief Function for initializing BAS.
 */
 static void bas_init(void)
{
    ret_code_t       err_code;

    ble_bas_init_t     bas_init;

    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    // Here the sec level for the Battery Service can be changed/increased.
    bas_init.bl_rd_sec        = SEC_OPEN;
    bas_init.bl_cccd_wr_sec   = SEC_OPEN;
    bas_init.bl_report_rd_sec = SEC_OPEN;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Service errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void os_init(void)
{
    ret_code_t      err_code;

    ble_os_init_t   os_init;

    // Initialize Our Service
    memset(&os_init, 0, sizeof(os_init));

    os_init.evt_handler             = NULL;
    os_init.error_handler           = service_error_handler;
    os_init.feature                 = 0;
//    os_init.feature                 |= BLE_OS_FEATURE_LOW_BATT;
    os_init.feature                 |= BLE_OS_FEATURE_TEMPERATURE;
    os_init.feature                 |= BLE_OS_FEATURE_HUMIDITY;
    os_init.annunciation            = 0;
    os_init.status_update.flags     = 0;

    // Here the sec level for the Our Service can be changed/increased.
    os_init.os_meas_cccd_wr_sec     = SEC_JUST_WORKS;
    os_init.os_feature_rd_sec       = SEC_JUST_WORKS;
    os_init.os_annunciation_rd_sec  = SEC_JUST_WORKS;
    os_init.os_annunciation_wr_sec  = SEC_JUST_WORKS;
    os_init.racp_cccd_wr_sec        = SEC_JUST_WORKS;
    os_init.racp_wr_sec             = SEC_JUST_WORKS;

    err_code = ble_os_init(&m_our_service, &os_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the YYY Service events.
 * YOUR_JOB implement a service handler function depending on the event the service you are using can generate
 *
 * @details This function will be called for all YY Service events which are passed to
 *          the application.
 *
 * @param[in]   p_yy_service   YY Service structure.
 * @param[in]   p_evt          Event received from the YY Service.
 *
 *
static void on_yys_evt(ble_yy_service_t     * p_yy_service,
                       ble_yy_service_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_YY_NAME_EVT_WRITE:
            APPL_LOG("[APPL]: charact written with value %s. ", p_evt->params.char_xx.value.p_str);
            break;

        default:
            // No implementation needed.
            break;
    }
}
*/

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t         err_code;
#ifdef USE_GAP_GATT
    qwr_init();
#endif
#ifdef USE_DIS
    dis_init();
#endif
#ifdef USE_DFU
    dfu_init();
#endif
    bas_init();

#ifdef USE_CTS
    cts_init();
#endif // USE_CTS
    
    // Initialize our own service 
    os_init();
}


/**@brief Function for initializing the connectable advertisement parameters.
 *
 * @details This function initializes the advertisement parameters to values that will put
 *          the application in connectable mode.
 *
 */
static void connectable_adv_init(void)
{
    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.properties.type    = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    m_adv_params.duration           = BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED;
    m_adv_params.p_peer_addr        = NULL;
    m_adv_params.filter_policy      = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval           = CONNECTABLE_ADV_INTERVAL;
    m_adv_params.primary_phy        = BLE_GAP_PHY_1MBPS;
}


/**@brief Function for initializing the non-connectable advertisement parameters.
 *
 * @details This function initializes the advertisement parameters to values that will put
 *          the application in non-connectable mode.
 *
 */
static void non_connectable_adv_init(void)
{
    uint32_t        err_code;

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.properties.type    = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
    m_adv_params.duration           = BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED;
    m_adv_params.p_peer_addr        = NULL;
    m_adv_params.filter_policy      = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval           = NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.primary_phy        = BLE_GAP_PHY_1MBPS;
}

static void include_scan_response_in_adv(bool include)
{
    ret_code_t                  err_code;
    ble_advdata_t               srdata;
    ble_advdata_manuf_data_t    manuf_data_response;

    if (!include){
        ASSERT(m_adv_params.properties.type  == BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED);
        
        // TODO
        //m_adv_mode = APP_ADV_NONSCAN_NONCONN;
        m_adv_data.scan_rsp_data.len = 0;
        m_adv_data.scan_rsp_data.p_data = NULL;        
    } else {
        ASSERT(m_adv_params.properties.type  == BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED);

        // TODO
        //m_adv_mode = APP_ADV_SCAN_CONN;
        m_adv_data.scan_rsp_data.p_data = m_enc_srdata;
        m_adv_data.scan_rsp_data.len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX;

        // Build and set scan response data and manufacturer specific data packet
        memset(&srdata, 0, sizeof(srdata));
        
        uint8_t data_response[]                 = "MyTherResp";
        manuf_data_response.company_identifier  = APP_COMPANY_IDENTIFIER;
        manuf_data_response.data.p_data         = data_response;
        manuf_data_response.data.size           = sizeof(data_response);
        srdata.name_type                        = BLE_ADVDATA_FULL_NAME;
        srdata.p_manuf_specific_data            = &manuf_data_response;
//        srdata.uuids_complete.uuid_cnt     = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
//        srdata.uuids_complete.p_uuids      = m_adv_uuids;

        err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init()
{
    ret_code_t               err_code;
    ble_advdata_t            advdata;
    ble_advdata_manuf_data_t manuf_data;
    uint8_t                  flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE; //BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    ble_gap_conn_sec_mode_t sec_mode;

//    APP_ERROR_CHECK_BOOL(sizeof(flags) == ADV_FLAGS_LEN);  // Assert that these two values of the same.

    uint8_t init_manuf_data[APP_BEACON_INFO_LENGTH] =      /**< Information advertised by the Beacon. */
    {
        APP_MAJOR_VALUE,        // Device major value
        APP_MINOR_VALUE,        // Device minor value
        APP_DATA_TEMP,          // temperature
        APP_DATA_HUM,           // humidity
        APP_DAT_X,              // accel x pos
        APP_DAT_y,              // accel y pos
        APP_DAT_Z,              // accel z pos
        APP_DAT_BATTERY         // battery voltage
    };

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
    // If USE_UICR_FOR_MAJ_MIN_VALUES is defined, the major and minor values will be read from the
    // UICR instead of using the default values. The major and minor values obtained from the UICR
    // are encoded into advertising data in big endian order (MSB First).
    // To set the UICR used by this example to a desired value, write to the address 0x10001080
    // using the nrfjprog tool. The command to be used is as follows.
    // nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val <your major/minor value>
    uint16_t major_value = ((*(uint32_t *)UICR_ADDRESS) & 0xFFFF0000) >> 16;
    uint16_t minor_value = ((*(uint32_t *)UICR_ADDRESS) & 0x0000FFFF);

    uint8_t index = MAJ_VAL_OFFSET_IN_MANUF_DATA;

    init_manuf_data[index++] = MSB_16(major_value);
    init_manuf_data[index++] = LSB_16(major_value);

    init_manuf_data[index++] = MSB_16(minor_value);
    init_manuf_data[index++] = LSB_16(minor_value);

    // Change the device name to the one determined using UICR values
    sprintf(m_device_name, "Bx%02X%02X", major_value, minor_value);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)m_device_name,
                                          strlen(m_device_name));
    APP_ERROR_CHECK(err_code);
#endif

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));

    //Set manufacturing data
    manuf_data.company_identifier   = APP_COMPANY_IDENTIFIER;
    manuf_data.data.size            = APP_BEACON_INFO_LENGTH; //APP_BEACON_INFO_LENGTH;  ADV_ADDL_MANUF_DATA_LEN
    manuf_data.data.p_data          = init_manuf_data; //m_addl_adv_manuf_data;    
    advdata.name_type               = BLE_ADVDATA_NO_NAME;
    advdata.flags                   = flags;
    advdata.p_manuf_specific_data   = &manuf_data;

    // Initialize advertising parameters (used when starting advertising).
#ifdef USE_CONN_ADV_INIT
    connectable_adv_init();  
    m_adv_mode = APP_ADV_SCAN_CONN;
#else 
    non_connectable_adv_init();
    m_adv_mode = APP_ADV_NONSCAN_NONCONN;
#endif

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

#ifdef USE_CONN_ADV_INIT
    include_scan_response_in_adv(true);
#else
    include_scan_response_in_adv(false);
#endif // USE_CONN_ADV_INIT

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event
    }
    else
    {
        ret_code_t err_code;
#ifdef USE_CONNPARAMS_PEERMGR
        memset(m_whitelist_peers, PM_PEER_ID_INVALID, sizeof(m_whitelist_peers));
        m_whitelist_peer_cnt = (sizeof(m_whitelist_peers) / sizeof(pm_peer_id_t));

        peer_list_get(m_whitelist_peers, &m_whitelist_peer_cnt);

        err_code = pm_whitelist_set(m_whitelist_peers, m_whitelist_peer_cnt);
        APP_ERROR_CHECK(err_code);

        // Setup the device identies list.
        // Some SoftDevices do not support this feature.
        err_code = pm_device_identities_list_set(m_whitelist_peers, m_whitelist_peer_cnt);
        if (err_code != NRF_ERROR_NOT_SUPPORTED)
        {
            APP_ERROR_CHECK(err_code);
        }
#endif // USE_CONNPARAMS_PEERMGR

        err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
        APP_ERROR_CHECK(err_code);
    }
}


static void advertising_reconfig(app_advertising_mode_t adv_mode)
{
    ret_code_t err_code;

    err_code = sd_ble_gap_adv_stop(m_adv_handle);
    if( (err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_INVALID_STATE) )
    {
        APP_ERROR_CHECK(err_code);
    }

    switch(adv_mode){
    case APP_ADV_NONE_SENSOR_NONE:
        led_indication_start(LED_INDICATION_1);
        break;
    case APP_ADV_NONE:
        led_indication_start(LED_INDICATION_2);
        break;
    case APP_ADV_NONSCAN_NONCONN:
        non_connectable_adv_init();
        include_scan_response_in_adv(false);
        led_indication_start(LED_INDICATION_3);
        break;
    case APP_ADV_SCAN_CONN:
        connectable_adv_init();  
        include_scan_response_in_adv(true);
        led_indication_start(LED_INDICATION_4);
        break;
    default:
        break;
    }

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
    APP_ERROR_CHECK(err_code);

    m_adv_mode = adv_mode;
    if( (adv_mode != APP_ADV_NONE) && (adv_mode != APP_ADV_NONE_SENSOR_NONE) )
    {
        advertising_start(false);
    }
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


#ifdef USE_CONNPARAMS_PEERMGR

/**@brief Fetch the list of peer manager peer IDs.
 *
 * @param[inout] p_peers   The buffer where to store the list of peer IDs.
 * @param[inout] p_size    In: The size of the @p p_peers buffer.
 *                         Out: The number of peers copied in the buffer.
 */
static void peer_list_get(pm_peer_id_t * p_peers, uint32_t * p_size)
{
    pm_peer_id_t peer_id;
    uint32_t     peers_to_copy;

    peers_to_copy = (*p_size < BLE_GAP_WHITELIST_ADDR_MAX_COUNT) ?
                     *p_size : BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

    peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
    *p_size = 0;

    while ((peer_id != PM_PEER_ID_INVALID) && (peers_to_copy--))
    {
        p_peers[(*p_size)++] = peer_id;
        peer_id = pm_next_peer_id_get(peer_id);
    }
}
#endif // USE_CONNPARAMS_PEERMGR

/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

     NRF_LOG_INFO("pm_evt_handler: evt_id = %d", p_evt->evt_id);
#ifdef USE_CONNPARAMS_PEERMGR
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);
#endif // USE_CONNPARAMS_PEERMGR
    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            NRF_LOG_INFO("Connected to a previously bonded device.");
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            NRF_LOG_INFO("Connection secured: role: %d, conn_handle: 0x%x, procedure: %d.",
                         ble_conn_state_role(p_evt->conn_handle),
                         p_evt->conn_handle,
                         p_evt->params.conn_sec_succeeded.procedure);
#ifdef USE_CONNPARAMS_PEERMGR
            m_peer_id = p_evt->peer_id;
#endif

#ifdef USE_CTS
            // Discover peer's services.
            err_code  = ble_db_discovery_start(&m_ble_db_discovery, p_evt->conn_handle);
            APP_ERROR_CHECK(err_code);
#endif
        } break;

        case PM_EVT_CONN_SEC_FAILED:
        {
            /* Often, when securing fails, it shouldn't be restarted, for security reasons.
             * Other times, it can be restarted directly.
             * Sometimes it can be restarted, but only after changing some Security Parameters.
             * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
             * Sometimes it is impossible, to secure the link, or the peer device does not support it.
             * How to handle this error is highly application dependent. */
        } break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = true};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
            NRF_LOG_INFO("pm_evt_handler: PM_EVT_PEERS_DELETE_SUCCEEDED");
        } break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;

#ifdef USE_CONNPARAMS_PEERMGR
        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        {
            bool already_added = false;

            // Note: You should check on what kind of white list policy your application should use.
            if (     p_evt->params.peer_data_update_succeeded.flash_changed
                 && (p_evt->params.peer_data_update_succeeded.data_id == PM_PEER_DATA_ID_BONDING))
            {
                NRF_LOG_DEBUG("New Bond, add the peer to the whitelist if possible");
                NRF_LOG_DEBUG("\tm_whitelist_peer_cnt %d, MAX_PEERS_WLIST %d",
                               m_whitelist_peer_cnt + 1,
                               BLE_GAP_WHITELIST_ADDR_MAX_COUNT);

                for (uint8_t i = 0; i<m_whitelist_peer_cnt; i++)
                {
                    if (m_whitelist_peers[i] == m_peer_id)
                    {
                        already_added= true; 
                        NRF_LOG_DEBUG("Peer is already in whitelist");
                        break;
                    }
                }
        
                if ((!already_added) && (m_whitelist_peer_cnt < BLE_GAP_WHITELIST_ADDR_MAX_COUNT) )
                {
                    // Bonded to a new peer, add it to the whitelist.
                    m_whitelist_peers[m_whitelist_peer_cnt++] = m_peer_id;

                    // The whitelist has been modified, update it in the Peer Manager.
                    err_code = pm_device_identities_list_set(m_whitelist_peers, m_whitelist_peer_cnt);
                    if (err_code != NRF_ERROR_NOT_SUPPORTED)
                    {
                        APP_ERROR_CHECK(err_code);
                    }

                    err_code = pm_whitelist_set(m_whitelist_peers, m_whitelist_peer_cnt);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break;
#endif // USE_CONNPARAMS_PEERMGR
        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
            // This can happen when the local DB has changed.
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}

#ifdef USE_NFC
/**@brief Function for initializing NFC BLE pairing module.
 */
static void nfc_pairing_init()
{
    ble_advertising_t * const p_advertising = ble_adv_instance_ptr_get();

    ret_code_t err_code = nfc_ble_pair_init(p_advertising, (nfc_pairing_mode_t)NFC_PAIRING_MODE);
    APP_ERROR_CHECK(err_code);
}
#endif // USE_NFC

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**
 * @brief Function for application main entry.
 */
int main()
{
    ret_code_t       err_code;

#ifdef GOTO_SYSTEM_OFF
    NRF_POWER->SYSTEMOFF = 1;
#endif

    // Initialization and configuration
#ifdef USE_LOG_INIT
    log_init();                 // Initialize logging
    NRF_LOG_INFO("ble_beacon - main started.");
#endif // USE_LOG_INIT
#ifdef USE_BUTTONLESS_DFU
    // Initialize the async SVCI interface to bootloader before any interrupts are enabled.
    err_code = ble_dfu_buttonless_async_svci_init();
    APP_ERROR_CHECK(err_code);
#endif // USE_BUTTONLESS_DFU

#ifdef USE_PWR_MANAGEMENT_INIT
    power_management_init();    // Initialize power management	
#endif // USE_PWR_MANAGEMENT_INIT

#ifdef USE_DCDCEN
    NRF_POWER->DCDCEN = 1;      // Enabling the DCDC converter for lower current consumption
#endif // USEDCDCEN

#ifdef USE_LFCLK_APP_TIMER
    lfclk_config();             // Configure low frequency 32kHz clock
    app_timer_init();           // Initialize app timer
#endif // USE_LFCLK_APP_TIMER
    
#ifdef USE_BSP
    bsp_configuration();        // Initialize BSP (leds and buttons)
    bsp_event_to_button_action_assign(0, BSP_BUTTON_ACTION_RELEASE,     BSP_EVENT_KEY_0_RELEASED);
    bsp_event_to_button_action_assign(0, BSP_BUTTON_ACTION_LONG_PUSH,   BSP_EVENT_KEY_0_LONG);
    bsp_event_to_button_action_assign(1, BSP_BUTTON_ACTION_RELEASE,     BSP_EVENT_KEY_1_RELEASED);
    bsp_event_to_button_action_assign(1, BSP_BUTTON_ACTION_LONG_PUSH,   BSP_EVENT_KEY_1_LONG);
    led_indication_init();
#endif // USE_BSP

#ifdef USE_TWI
    twi_config();               // Initialize TWI (with transaction manager) 
    nrf_delay_ms(10);           // sensor startup time: KX022 10 ms, SHT3 1 ms
#endif // USE_TWI

#ifdef USE_SENSOR
    sensor_init();              // Initialize sensors
#endif // USE_SENSOR

    nrf_cal_init();             // Initialize calender, but needs nrf_cal_time_set()

#ifdef USE_APPTIMER
    timers_create();
#endif // USE_APPTIMER

#ifdef USE_OFFLINE_FUNCTION
//    offline_buffer_init();      // Initialize offline buffer
#endif // USE_OFFLINE_FUNCTION
	
    ble_stack_init();           // Initialize the BLE stack

#ifdef USE_SCHEDULER
    scheduler_init();
#endif // USE_SCHEDULER

#ifdef USE_GAP_GATT
    gap_params_init();
    gatt_init();
#endif // USE_GAP_GATT

#ifdef USE_CTS
    db_discovery_init();
#endif // USE_CTS


#ifdef USE_OUR_SERVICES
    services_init();
#endif

#ifdef USE_ADVERTISING
    advertising_init();         // Initialize the advertising functionality
#endif

#ifdef USE_CONNPARAMS_PEERMGR
    conn_params_init();
    peer_manager_init();    
#endif // USE_CONNPARAMS_PEERMGR
		
#ifdef USE_NFC
#ifdef CONFIG_NFCT_PINS_AS_GPIOS
    #error CONFIG_NFCT_PINS_AS_GPIOS must not be set when using NFC.
#endif // CONFIG_NFCT_PINS_AS_GPIOS
    nfc_pairing_init();
#endif // USE_NFC

    // Start execution.
    NRF_LOG_INFO("Beacon started.");

#ifdef USE_APPTIMER
    // start (repeated) init timer (SAADC, sensor, offline buffer) 
    // and then stops init timer and starts advertising.
    timers_start();  
#endif // USE_APPTIMER

//sd_power_system_off();
    for (;;)
    {
        NRF_LOG_FLUSH();
        idle_state_handle();
    }
};