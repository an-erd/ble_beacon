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
#include "boards.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrfx_saadc.h"
#include "nrf_drv_ppi.h"
#include "bsp.h"
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
#include "nrf_calendar.h"
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
#define USE_TWI
#define USE_SENSOR
#define USE_SENSORINIT_SHT3
#define USE_SENSORINIT_KX022
#define USE_APPTIMER
#undef  OFFLINE_FUNCTION
#define USE_SCHEDULER
#define USE_GAP_GATT
#define USE_CONNPARAMS_PEERMGR
#undef  USE_CONN_ADV_INIT
#undef  USE_NONCONN_ADV_INIT
#define USE_ADVERTISING
#define USE_OUR_SERVICES
#define USE_CTS
#define USE_DIS

// App Timer defines
APP_TIMER_DEF(m_repeated_timer_read_saadc);                 /**< Handler for repeated timer used to read battery level by SAADC. */
APP_TIMER_DEF(m_repeated_timer_read_sensor);                /**< Handler for repeated timer used to read TWI sensors. */
APP_TIMER_DEF(m_singleshot_timer_read_sensor_step);         /**< Handler for repeated timer for TWI sensor, steps. */
#define APP_TIMER_TICKS_SAADC       APP_TIMER_TICKS(60000)
#define APP_TIMER_TICKS_SENSOR      APP_TIMER_TICKS(15000)

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

// Sensor defines
#define BUFFER_SIZE             21  // read buffer from sensors: temp+hum (6=2*msb,lsb,crc) + xyz (6=3*lsb,msb) + INT_REL (5) + INS1 (4)
static uint8_t m_buffer[BUFFER_SIZE];

#if (BUFFER_SIZE < 21)
    #error Buffer too small.
#endif

// DFU defines
#define INITIATE_DFU_TIMEOUT    15  // secs in which the code (long-long-long button press must be completed)

// Offline Buffer
// Time (4 byte), Temerature (2 byte), Humidity (2 byte) -> 8 x uint8_t byte/entry
// 20 KB = 20.000 Byte -> 2.500 entries a 8 byte
#ifdef OFFLINE_FUNCTION
#define OFFLINE_BUFFER_RESERVED_BYTE    20000   // 20 KB RAM reserved
#define OFFLINE_BUFFER_SIZE_PER_ENTRY   8       // uint8_t
#define OFFLINE_BUFFER_SIZE             (OFFLINE_BUFFER_RESERVED_BYTE / OFFLINE_BUFFER_SIZE_PER_ENTRY)
static int m_offlinebuffer_counter = 0;         // next entry in buffer
static uint8_t m_offlinebuffer[OFFLINE_BUFFER_SIZE][OFFLINE_BUFFER_SIZE_PER_ENTRY] = { 0xFF };
void offline_buffer_init();
bool offline_buffer_update(uint32_t counter, uint8_t *buffer);
//void test_data_send_array(int num_to_send, bool restart);
#endif // OFFLINE_FUCTION

// BLE defines 
#define APP_FAST_ADV_INTERVAL           50                                  /**< The advertising interval for fast advertisement. */
#define APP_SLOW_ADV_INTERVAL           MSEC_TO_UNITS(1000, UNIT_0_625_MS)  /**< The advertising interval for slow advertisement. */
#define APP_ADV_FAST_DURATION           18000                               /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define APP_ADV_SLOW_DURATION           0                                   /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define APP_BLE_OBSERVER_PRIO           3                                   /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                   /**< A tag identifying the SoftDevice BLE configuration. */
#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(1000, UNIT_0_625_MS)  /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)    /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)    /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                   /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)     /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)               /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
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
#define APP_BEACON_INFO_LENGTH  0x10            /**< Total length of information advertised by the Beacon. */
#define APP_COMPANY_IDENTIFIER  0x0059          /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
#define APP_MAJOR_VALUE         0x00, 0x07      /**< Major value used to identify Beacons. */
#define APP_MINOR_VALUE         0x00, 0xFF      /**< Minor value used to identify Beacons. -> caution: see UICR */
#define APP_DATA_TEMP           0xfd, 0xfd      /**< Temperature data. */
#define APP_DATA_HUM            0xfe, 0xde      /**< Humidity data. */
#define APP_DAT_X               0xaa, 0xaa      /**< Acceleration X data. */
#define APP_DAT_y               0xbb, 0xbb      /**< Acceleration Y data. */
#define APP_DAT_Z               0xcc, 0xcc      /**< Acceleration Z Temperature data. */
#define APP_DAT_BATTERY         0xFF, 0xFF      /**< Battery voltage data. */
#define APP_BEACON_PAD          0xFF            /**< Padding data (maybe used, maybe not.) */

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
#define MAJ_VAL_OFFSET_IN_MANUF_DATA    0                       /**< Position of the MSB of the Major Value in m_beacon_info array. */
#define UICR_ADDRESS                    0x10001080              /**< Address of the UICR register  */
#endif

#define DEAD_BEEF                       0xDEADBEEF  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define PAYLOAD_OFFSET_IN_BEACON_INFO_ADV   18  /**< First position to write the payload to in enc advdata */
#define PAYLOAD_OFFSET_BATTERY_INFO     (PAYLOAD_OFFSET_IN_BEACON_INFO_ADV + 10)  /**< Position to write the battery voltage payload to */									
#define PAYLOAD_OFFSET_IN_BEACON_INFO_ADV_OLD   11  /**< First position to write the payload to in enc advdata */
#define PAYLOAD_OFFSET_BATTERY_INFO_OLD     (PAYLOAD_OFFSET_IN_BEACON_INFO_ADV_OLD + 10)  /**< Position to write the battery voltage payload to */									

// BLE Services
#define DEVICE_NAME                     "Beac8"                 /**< Name of device. Will be included in the advertising data. */
static ble_uuid_t m_adv_uuids[] = 
{
    { BLE_UUID_OUR_SERVICE, BLE_UUID_TYPE_BLE  },               /**< 16-bit UUID for our service. */
};


// Peer Manager variables 
#ifdef USE_CONNPARAMS_PEERMGR
static pm_peer_id_t m_peer_id;                                                      /**< Device reference handle to the current bonded central. */
static pm_peer_id_t m_whitelist_peers[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];            /**< List of peers currently in the whitelist. */
static uint32_t     m_whitelist_peer_cnt;                                           /**< Number of peers currently in the whitelist. */
#endif // USE_CONNPARAMS_PEERMGR

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
ble_os_t m_our_service;



// DIS - Device Information Service defines
#ifdef USE_DIS
#define DIS_MANUFACTURER_NAME           "ansprechendeKunst"     /**< Manufacturer. Will be passed to Device Information Service. */
#define DIS_MODEL_NUMBER                "1"
#define DIS_SERIAL_NUMBER               "8" 
#define DIS_HW_REV                      "1.0"
#define DIS_SW_REV                      "0.9"
#define DIS_FW_REV                      "0.1a"
#endif // USE_DIS

// CTS - Current Time Service defiens
#ifdef USE_CTS
BLE_CTS_C_DEF(m_cts_c);                                         /**< Current Time service instance. */
BLE_DB_DISCOVERY_DEF(m_ble_db_discovery);                                           /**< DB discovery module instance. */
#endif

#ifdef USE_GAP_GATT
NRF_BLE_GATT_DEF(m_gatt);                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                         /**< Context for the Queued Write module.*/
#endif // USE_GAP_GATT
BLE_ADVERTISING_DEF(m_advertising);                             /**< Advertising module instance. */


// BLE Advertising variables and structs
static ble_gap_adv_params_t m_adv_params;                                   /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t              m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;  /**< Advertising handle used to identify an advertising set. */
static uint8_t              m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];   /**< Buffer for storing an encoded advertising set. */
static uint16_t             m_conn_handle = BLE_CONN_HANDLE_INVALID;        /**< Handle of the current connection. */

static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] =      /**< Information advertised by the Beacon. */
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
        .p_data = NULL,
        .len    = 0
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


/**@brief Function for handling BSP events.
 *
 * @details  This function will handle the BSP events as button press.
 */
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;
    static uint32_t counter_dfu     = 0;
    static uint8_t  countdown_dfu   = 3;
    static bool current_leds        = false;

    NRF_LOG_DEBUG("bsp_event_handler, button %d", event);
	
    switch (event)
    {
    case BSP_EVENT_KEY_0: // button on beacon pressed
        NRF_LOG_INFO("button BSP_EVENT_KEY_0");
        if(current_leds){
            current_leds = false;
            bsp_board_leds_off();
        } else {
            current_leds = true;
            bsp_board_leds_on();
        }
        
        break;

    case BSP_EVENT_KEY_0_RELEASED: // button on beacon released
        NRF_LOG_INFO("button BSP_EVENT_KEY_0_RELEASED");
        break;
    
    case BSP_EVENT_KEY_0_LONG: // button on beacon long pressed
        NRF_LOG_INFO("button BSP_EVENT_KEY_0_LONG");
        if(app_timer_cnt_diff_compute(app_timer_cnt_get(), counter_dfu) > APP_TIMER_TICKS(INITIATE_DFU_TIMEOUT*1000)){
            // reset time and countdown to initiate DFU
            NRF_LOG_INFO("reset time and countdown to initiate DFU");
            counter_dfu = app_timer_cnt_get();
            countdown_dfu = 3;
        }

        switch(countdown_dfu){
        case 3:
        case 2:
            countdown_dfu--;
            break;
        case 1:
            NRF_LOG_INFO("Initiated DFU now");
            NRF_LOG_FLUSH();
            APP_ERROR_CHECK(sd_power_gpregret_set(0, BOOTLOADER_DFU_START));
            sd_nvic_SystemReset();   
            break;
        default:
            break;
        }
        break;

    case BSP_EVENT_KEY_1: // button on jig pressed
        NRF_LOG_INFO("button BSP_EVENT_KEY_1");
#ifdef USE_CTS
        if (m_cts_c.conn_handle != BLE_CONN_HANDLE_INVALID)
        {
            err_code = ble_cts_c_current_time_read(&m_cts_c);
            if (err_code == NRF_ERROR_NOT_FOUND)
            {
                NRF_LOG_INFO("Current Time Service is not discovered.");
            }
        }
#endif
        break;
    
    case BSP_EVENT_KEY_1_RELEASED: // button on jig released
        NRF_LOG_INFO("button BSP_EVENT_KEY_1_RELEASED");
        break;
    
    case BSP_EVENT_KEY_1_LONG: // button on jig long pressed
        NRF_LOG_INFO("button BSP_EVENT_KEY_1_LONG"); 
//        NRF_LOG_INFO("   app_timer_cnt_get() %d, counter_dfu %d, APP_TIMER_TICKS() %d, app_timer_cnt_diff_compute %d, countdown_dfu %d",
//            app_timer_cnt_get(), counter_dfu, APP_TIMER_TICKS(INITIATE_DFU_TIMEOUT*1000),
//            app_timer_cnt_diff_compute(app_timer_cnt_get(), counter_dfu) > APP_TIMER_TICKS(INITIATE_DFU_TIMEOUT*1000),
//            countdown_dfu);
        if(app_timer_cnt_diff_compute(app_timer_cnt_get(), counter_dfu) > APP_TIMER_TICKS(INITIATE_DFU_TIMEOUT*1000)){
            // reset time and countdown to initiate DFU
            NRF_LOG_INFO("reset time and countdown to initiate DFU");
            counter_dfu = app_timer_cnt_get();
            countdown_dfu = 3;
//            NRF_LOG_INFO("   counter_dfu %d, countdown_dfu %d", counter_dfu, countdown_dfu);
        }

        switch(countdown_dfu){
        case 3:
        case 2:
            countdown_dfu--;
//            NRF_LOG_INFO("   countdown_dfu %d", countdown_dfu);
            break;
        case 1:
            NRF_LOG_INFO("Initiated DFU now");
            NRF_LOG_FLUSH();
            APP_ERROR_CHECK(sd_power_gpregret_set(0, BOOTLOADER_DFU_START));
            sd_nvic_SystemReset();   
            break;
        default:
            break;
        }
        break;

    case BSP_EVENT_WHITELIST_OFF:
        NRF_LOG_INFO("BSP_EVENT_WHITELIST_OFF");    
#ifdef USE_CTS

        if (m_cts_c.conn_handle == BLE_CONN_HANDLE_INVALID)
        {
            err_code = ble_advertising_restart_without_whitelist(&m_advertising);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
        }
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
        m_advertising.enc_advdata[payload_idx++] = MSB_16(m_battery_millivolts);
        m_advertising.enc_advdata[payload_idx++] = LSB_16(m_battery_millivolts);

        nrfx_saadc_uninit();                                                        // Unintialize SAADC to disable EasyDMA and save power
        NRF_SAADC->INTENCLR = (SAADC_INTENCLR_END_Clear << SAADC_INTENCLR_END_Pos); // Disable the SAADC interrupt
        NVIC_ClearPendingIRQ(SAADC_IRQn);                                           // Clear the SAADC interrupt if set
        m_saadc_initialized = false;                                                // Set SAADC as uninitialized
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
//    NRF_LOG_DEBUG("process_all_data()");
	
//    if(!m_advertising.initialized) {
//        NRF_LOG_DEBUG("m_advertising not initialized -> exiting process_all_data()");
//        return;
//    } 

#ifdef DEBUG
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
#endif // DEBUG

    // update payload data in encoded advertising data
    uint8_t payload_idx = PAYLOAD_OFFSET_IN_BEACON_INFO_ADV;

    m_advertising.enc_advdata[payload_idx++] = m_buffer[ 0];
    m_advertising.enc_advdata[payload_idx++] = m_buffer[ 1];
    m_advertising.enc_advdata[payload_idx++] = m_buffer[ 3];
    m_advertising.enc_advdata[payload_idx++] = m_buffer[ 4];
    m_advertising.enc_advdata[payload_idx++] = m_buffer[ 6];
    m_advertising.enc_advdata[payload_idx++] = m_buffer[ 7];
    m_advertising.enc_advdata[payload_idx++] = m_buffer[ 8];
    m_advertising.enc_advdata[payload_idx++] = m_buffer[ 9];
    m_advertising.enc_advdata[payload_idx++] = m_buffer[10];
    m_advertising.enc_advdata[payload_idx++] = m_buffer[11];

#ifdef DEBUG
    NRF_LOG_INFO("m_advertising.enc_advdata[0..30]:");
    NRF_LOG_RAW_HEXDUMP_INFO(m_advertising.enc_advdata, 31);
#endif // DEBUG

#ifdef OFFLINE_FUNCTION
    // update offline buffer
    // TODO
    offline_buffer_update(...);
#endif // OFFLINE_FUNCTION
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
        APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, SHT3_ADDR, config_SHT3_0, 2, false));
        
        // KX022 
        APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, KX022_ADDR, config_kx022_0, 2, false));
        APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, KX022_ADDR, config_kx022_1, 2, false));
        APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, KX022_ADDR, config_kx022_2, 2, false));
   
        // timer for next step, 6 ms (>1.2/ODR)
        err_code = app_timer_start(m_singleshot_timer_read_sensor_step, APP_TIMER_TICKS(6), NULL);
        APP_ERROR_CHECK(err_code);
    
        step++;
        break;
    
    case 1:
        // KX022 
        APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, KX022_ADDR, config_kx022_3, 2, false));
    
        // timer for next step,6 ms (>1.2/ODR)
        err_code = app_timer_start(m_singleshot_timer_read_sensor_step, APP_TIMER_TICKS(6), NULL);
        APP_ERROR_CHECK(err_code);

        step++;
        break;
    
    case 2:
        // KX022 
        reg[0] = KX022_1020_REG_XOUTL;
        APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, KX022_ADDR, reg, 1, true));
        // read 6 bytes (x (lsb+msb), y (lsb+msb), z (lsb+msb)
        APP_ERROR_CHECK(nrf_drv_twi_rx(&m_twi, KX022_ADDR, &m_buffer[6], 6));
        APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, KX022_ADDR, config_kx022_0, 2, false));

        // timer fillup for SHT3 15 ms (15 -6 -6 -running time of functions = ~1 ms)
        err_code = app_timer_start(m_singleshot_timer_read_sensor_step, APP_TIMER_TICKS(1), NULL);
        APP_ERROR_CHECK(err_code);

        step++;
        break;  // time to go until SHT3 is ready
    
    case 3:
        // read 6 bytes (temp (msb+lsb+crc) and hum (msb+lsb+crc)
        APP_ERROR_CHECK(nrf_drv_twi_rx(&m_twi, SHT3_ADDR, &m_buffer[0], 6));
            
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
    if(!m_saadc_initialized) {
        saadc_init();
    }
    m_saadc_initialized = true;

    // Trigger the SAADC SAMPLE task
    nrfx_saadc_sample();
}

static void repeated_timer_handler_read_sensors()
{
    // Trigger the sensor retrieval task from the beginning
    read_all_sensors(true);	// init w/step0
}

static void singleshot_timer_handler_read_sensor_step()
{
    // Trigger the sensor retrieval task for subsequent steps
    read_all_sensors(false);	// subsequent steps
}

static void timers_create()
{
    ret_code_t err_code;

    // Create timers
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
}

static void timers_start()
{
    ret_code_t err_code;

    err_code = app_timer_start(m_repeated_timer_read_saadc, APP_TIMER_TICKS_SAADC, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_repeated_timer_read_sensor, APP_TIMER_TICKS_SENSOR, NULL);
    APP_ERROR_CHECK(err_code);
    

}

#ifdef OFFLINE_FUNCTION
/**@brief Function to initialize the offline buffer.
 *
 * @details  This function initializes the counter for current entry and the buffer array.
 */
void offline_buffer_init()
{
    m_offlinebuffer_counter = 0;
    memset(m_offlinebuffer, 0xFF, sizeof(m_offlinebuffer));
}

/**@brief Function to update next entry in offline buffer.
 *
 * @details  This functions stores the next entry in the offline buffer, returns true on succes or false if buffer full.
 */
bool offline_buffer_update(uint32_t counter, uint8_t *buffer)
{
    if (m_offlinebuffer_counter == OFFLINE_BUFFER_SIZE)
        return false;

    m_offlinebuffer[m_offlinebuffer_counter][0];
    memcpy(&m_offlinebuffer[m_offlinebuffer_counter][0], (uint8_t *)&counter, 4);
    memcpy(&m_offlinebuffer[m_offlinebuffer_counter][4], buffer, 4);
    NRF_LOG_INFO("counter %d", counter);
    NRF_LOG_HEXDUMP_INFO((uint8_t *)&counter, 4);
    NRF_LOG_HEXDUMP_INFO(&m_offlinebuffer[m_offlinebuffer_counter][0], 8);
    m_offlinebuffer_counter++;

    return true;
}
#endif // OFFLINE_FUNCTION



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
            err_code = ble_advertising_restart_without_whitelist(&m_advertising);
            APP_ERROR_CHECK(err_code);
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
            err_code = ble_advertising_whitelist_reply(&m_advertising,
                                                       whitelist_addrs,
                                                       addr_cnt,
                                                       whitelist_irks,
                                                       irk_cnt);
            APP_ERROR_CHECK(err_code);
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
    NRF_SDH_BLE_OBSERVER(m_our_service_observer, APP_BLE_OBSERVER_PRIO, ble_our_service_on_ble_evt, (void*) &m_our_service);
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
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
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
/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
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
    ble_cts_c_on_db_disc_evt(&m_cts_c, p_evt);
}

/**
 * @brief Database discovery collector initialization.
 */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
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


#ifdef USE_DIS
/**@brief Function for initializing Device Information Service.
 */
static void dis_init(void)
{
    ret_code_t       err_code;
    ble_dis_init_t     dis_init;
            
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
    NRF_LOG_INFO("\r\nCurrent Time:");
    NRF_LOG_INFO("\r\nDate:");

    NRF_LOG_INFO("\tDay of week   %s", (uint32_t)day_of_week[p_evt->
                                                         params.
                                                         current_time.
                                                         exact_time_256.
                                                         day_date_time.
                                                         day_of_week]);

    if (p_evt->params.current_time.exact_time_256.day_date_time.date_time.day == 0)
    {
        NRF_LOG_INFO("\tDay of month  Unknown");
    }
    else
    {
        NRF_LOG_INFO("\tDay of month  %i",
                       p_evt->params.current_time.exact_time_256.day_date_time.date_time.day);
    }

    NRF_LOG_INFO("\tMonth of year %s",
    (uint32_t)month_of_year[p_evt->params.current_time.exact_time_256.day_date_time.date_time.month]);
    if (p_evt->params.current_time.exact_time_256.day_date_time.date_time.year == 0)
    {
        NRF_LOG_INFO("\tYear          Unknown");
    }
    else
    {
        NRF_LOG_INFO("\tYear          %i",
                       p_evt->params.current_time.exact_time_256.day_date_time.date_time.year);
    }
    NRF_LOG_INFO("\r\nTime:");
    NRF_LOG_INFO("\tHours     %i",
                   p_evt->params.current_time.exact_time_256.day_date_time.date_time.hours);
    NRF_LOG_INFO("\tMinutes   %i",
                   p_evt->params.current_time.exact_time_256.day_date_time.date_time.minutes);
    NRF_LOG_INFO("\tSeconds   %i",
                   p_evt->params.current_time.exact_time_256.day_date_time.date_time.seconds);
    NRF_LOG_INFO("\tFractions %i/256 of a second",
                   p_evt->params.current_time.exact_time_256.fractions256);

    NRF_LOG_INFO("\r\nAdjust reason:\r");
    NRF_LOG_INFO("\tDaylight savings %x",
                   p_evt->params.current_time.adjust_reason.change_of_daylight_savings_time);
    NRF_LOG_INFO("\tTime zone        %x",
                   p_evt->params.current_time.adjust_reason.change_of_time_zone);
    NRF_LOG_INFO("\tExternal update  %x",
                   p_evt->params.current_time.adjust_reason.external_reference_time_update);
    NRF_LOG_INFO("\tManual update    %x",
                   p_evt->params.current_time.adjust_reason.manual_time_update);
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

    switch (p_evt->evt_type)
    {
        case BLE_CTS_C_EVT_DISCOVERY_COMPLETE:
            NRF_LOG_INFO("Current Time Service discovered on server.");
            err_code = ble_cts_c_handles_assign(&m_cts_c,
                                                p_evt->conn_handle,
                                                &p_evt->params.char_handles);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_CTS_C_EVT_DISCOVERY_FAILED:
            NRF_LOG_INFO("Current Time Service not found on server. ");
            // CTS not found in this case we just disconnect. There is no reason to stay
            // in the connection for this simple app since it all wants is to interact with CT
            if (p_evt->conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                err_code = sd_ble_gap_disconnect(p_evt->conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BLE_CTS_C_EVT_DISCONN_COMPLETE:
            NRF_LOG_INFO("Disconnect Complete.");
            break;

        case BLE_CTS_C_EVT_CURRENT_TIME:
            NRF_LOG_INFO("Current Time received.");
            current_time_print(p_evt);
            break;

        case BLE_CTS_C_EVT_INVALID_TIME:
            NRF_LOG_INFO("Invalid Time received.");
            break;

        default:
            break;
    }
}

/**@brief Function for initializing CTS.
 */
static void cts_init(void)
{
    ret_code_t       err_code;    // Initialize CTS.
    ble_cts_c_init_t   cts_init = {0};

    cts_init.evt_handler   = on_cts_c_evt;
    cts_init.error_handler = current_time_error_handler;

    err_code               = ble_cts_c_init(&m_cts_c, &cts_init);
    APP_ERROR_CHECK(err_code);
}
#endif

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

static void print_current_time()
{
    printf("Uncalibrated time:\t%s\r\n", nrf_cal_get_time_string(false));
    printf("Calibrated time:\t%s\r\n", nrf_cal_get_time_string(true));
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t         err_code;

    qwr_init();
    dis_init();

#ifdef USE_CTS
    cts_init();
#endif // USE_CTS
    
    // Initialize our own service 
    our_service_init(&m_our_service);
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

    m_adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    m_adv_params.duration        = APP_ADV_SLOW_DURATION; // was: APP_ADV_DURATION;

    m_adv_params.p_peer_addr   = NULL;
    m_adv_params.filter_policy = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval      = APP_SLOW_ADV_INTERVAL; // was: CONNECTABLE_ADV_INTERVAL;
}


/**@brief Function for initializing the non-connectable advertisement parameters.
 *
 * @details This function initializes the advertisement parameters to values that will put
 *          the application in non-connectable mode.
 *
 */
static void non_connectable_adv_init(void)
{
    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED; // BLE_GAP_ADV_TYPE_CONNECTABLE_NONSCANNABLE_DIRECTED;
    m_adv_params.duration        = APP_ADV_SLOW_DURATION; // was: APP_ADV_DURATION;
    m_adv_params.p_peer_addr     = NULL;
    m_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval        = APP_SLOW_ADV_INTERVAL; // was: NON_CONNECTABLE_ADV_INTERVAL;
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init()
{
    uint32_t        err_code;

    ble_advertising_init_t init;
    memset(&init, 0, sizeof(init));


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
#endif

    //Set manufacturing data
    ble_advdata_manuf_data_t                manuf_specific_data;
    manuf_specific_data.company_identifier  = APP_COMPANY_IDENTIFIER;
    manuf_specific_data.data.p_data         = (uint8_t *) init_manuf_data;
    manuf_specific_data.data.size           = APP_BEACON_INFO_LENGTH;

    // Build and set advertising data.
    init.advdata.name_type                  = BLE_ADVDATA_NO_NAME;  // BLE_ADVDATA_FULL_NAME
    init.advdata.flags                      = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.p_manuf_specific_data      = &manuf_specific_data;
    init.advdata.uuids_complete.uuid_cnt    = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids     = m_adv_uuids;

    int8_t tx_power                         = 0; // set TX power for advertising
    init.advdata.p_tx_power_level           = &tx_power;

    // TODO work on scan response. Necessary?
     // Build and set scan response data and manufacturer specific data packet
//    ble_advdata_manuf_data_t                manuf_data_response;
//    uint8_t data_response[]                 = "Many_bytes_of_data";
//    manuf_data_response.company_identifier  = 0x0059;
//    manuf_data_response.data.p_data         = data_response;
//    manuf_data_response.data.size           = sizeof(data_response);
//    init.srdata.name_type                   = BLE_ADVDATA_NO_NAME; // BLE_ADVDATA_NO_NAME; BLE_ADVDATA_FULL_NAME;
//    init.srdata.p_manuf_specific_data       = &manuf_data_response;
//    init.srdata.uuids_complete.uuid_cnt     = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
//    init.srdata.uuids_complete.p_uuids      = m_adv_uuids;

    // Set advertising modes and intervals
    init.config.ble_adv_fast_enabled    = false;    // currently only use slow
    init.config.ble_adv_fast_interval   = APP_FAST_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout    = APP_ADV_FAST_DURATION;
    init.config.ble_adv_slow_enabled    = true;
    init.config.ble_adv_slow_interval   = APP_SLOW_ADV_INTERVAL;
    init.config.ble_adv_slow_timeout    = APP_ADV_SLOW_DURATION;

    // Set event handler that will be called upon advertising events
    init.evt_handler = on_adv_evt;

    // ble_app
    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETED_SUCEEDED event
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

        err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_SLOW);
        APP_ERROR_CHECK(err_code);
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
            m_peer_id = p_evt->peer_id;

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
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
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
            advertising_start(false);
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
            // Note: You should check on what kind of white list policy your application should use.
            if (     p_evt->params.peer_data_update_succeeded.flash_changed
                 && (p_evt->params.peer_data_update_succeeded.data_id == PM_PEER_DATA_ID_BONDING))
            {
                NRF_LOG_DEBUG("New Bond, add the peer to the whitelist if possible");
                NRF_LOG_DEBUG("\tm_whitelist_peer_cnt %d, MAX_PEERS_WLIST %d",
                               m_whitelist_peer_cnt + 1,
                               BLE_GAP_WHITELIST_ADDR_MAX_COUNT);

                if (m_whitelist_peer_cnt < BLE_GAP_WHITELIST_ADDR_MAX_COUNT)
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
    bool erase_bonds = false;

#ifdef GOTO_SYSTEM_OFF
    NRF_POWER->SYSTEMOFF = 1;
#endif

    // Initialization and configuration
#ifdef USE_LOG_INIT
    log_init();                 // Initialize logging
#endif // USE_LOG_INIT

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
    bsp_event_to_button_action_assign(0, BSP_BUTTON_ACTION_RELEASE,  BSP_EVENT_KEY_0_RELEASED);
    bsp_event_to_button_action_assign(0, BSP_BUTTON_ACTION_LONG_PUSH,  BSP_EVENT_KEY_0_LONG);
    bsp_event_to_button_action_assign(1, BSP_BUTTON_ACTION_RELEASE,  BSP_EVENT_KEY_1_RELEASED);
    bsp_event_to_button_action_assign(1, BSP_BUTTON_ACTION_LONG_PUSH,  BSP_EVENT_KEY_1_LONG);    
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
    timers_start();
#endif // USE_APPTIMER

#ifdef OFFLINE_FUNCTION
    offline_buffer_init();      // Initialize offline buffer
#endif // OFFLINE_FUNCTION
	
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
#ifdef USE_CONN_ADV_INIT
    connectable_adv_init();
#endif 
#ifdef USE_NONCONN_ADV_INIT
    non_connectable_adv_init();
#endif 
    advertising_init();         // Initialize the advertising functionality
#endif

#ifdef USE_CONNPARAMS_PEERMGR
    conn_params_init();
    peer_manager_init();    
#endif // USE_CONNPARAMS_PEERMGR
		
    // Start execution.
    NRF_LOG_INFO("Beacon started.");

#ifdef USE_ADVERTISING
    non_connectable_adv_init();
    advertising_start(erase_bonds);
#endif
//sd_power_system_off();
    for (;;)
    {
        NRF_LOG_FLUSH();
        idle_state_handle();
    }
};
