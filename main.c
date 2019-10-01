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
 * @defgroup ble_sdk_app_beacon_main_ae main.c
 * @{
 * @ingroup ble_sdk_app_beacon
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
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nordic_common.h"
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
#include "ble_dis.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "app_timer.h"
#include "app_gpiote.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_power.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_timer.h"
#include "nrfx_rtc.h"
#include "nrf_drv_rtc.h"
#include "uicr_config.h"
#include "sht3.h"
#include "kx022.h"
#include "our_service.h"
#include "compiler_abstraction.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_bootloader_info.h"

#include "fds.h"
#include "peer_manager.h"
#include "bsp_btn_ble.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"



#if defined( __GNUC__ ) && (__LINT__ == 0)
    // This is required if one wants to use floating-point values in 'printf'
    // (by default this feature is not linked together with newlib-nano).
    // Please note, however, that this adds about 13 kB code footprint...
    __ASM(".global _printf_float");
#endif


// Event scheduled
// TODO check, sensor sample 10 or 15 sec
// - BLE adv         1 sec
// - SADC sample    60 sec
// - Sensor sample  15 sec

// RTC defines
#define RTC_CC_VALUE                8       // prescale 256 Hz, RTC_CC_VALUE=8 => 1/32 sec
#define RTC_SADC_UPDATE             1875
#define RTC_SENSOR_UPDATE           480

// SAADC defines
#define SAADC_CALIBRATION_INTERVAL  5       // SAADC calibration interval relative to NRF_DRV_SAADC_EVT_DONE event
#define SAADC_SAMPLES_IN_BUFFER     1       // Number of SAADC samples in RAM before returning a SAADC event
#define SAADC_BURST_MODE            0       // Set to 1 to enable BURST mode, otherwise set to 0.

// SAADC forward declaration and variables
static void saadc_init(void);
const  nrf_drv_rtc_t        rtc = NRF_DRV_RTC_INSTANCE(2);
static nrf_saadc_value_t    m_buffer_pool[2][SAADC_SAMPLES_IN_BUFFER];
static uint32_t             m_adc_evt_counter = 0;
static bool                 m_saadc_initialized = false;      

// TWI defines
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

// DFU defines
#define INITIATE_DFU_TIMEOUT    15  // secs in which the code (long-long-long button press must be completed)

typedef struct
{
    float   temp;
    float   humidity;
    int16_t x;
    int16_t y;
    int16_t z;
} sample_t;
static sample_t     m_sample = { 0, 0, 0, 0, 0 };
static uint16_t     m_battery_millivolts = 3333;    // default to some value, say 3333

#if (BUFFER_SIZE < 21)
    #error Buffer too small.
#endif

/**@brief Macro to convert the result of ADC conversion in millivolts.
 *
 * @param[in]  ADC_VALUE   ADC result.
 *
 * @retval     Result converted to millivolts.
 */
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_12BIT) * ADC_PRE_SCALING_COMPENSATION)

// BLE defines and structs
#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(1000, UNIT_0_625_MS)  /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */
#define APP_FAST_ADV_INTERVAL           50
#define APP_SLOW_ADV_INTERVAL           MSEC_TO_UNITS(1000, UNIT_0_625_MS)

// beacon data
#define APP_BEACON_INFO_LENGTH  0x10            /**< Total length of information advertised by the Beacon. */
//#define APP_ADV_DATA_LENGTH     0x15            /**< Length of manufacturer specific data in the advertisement. */
#define APP_DEVICE_TYPE         0x02            /**< 0x02 refers to Beacon. */
#define APP_COMPANY_IDENTIFIER  0x0059          /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
#define APP_BEACON_UUID_SHORT   0x01, 0x12, 0x23, 0x34  /**< Proprietary UUID for Beacon. */
#define APP_MAJOR_VALUE         0x00, 0x07      /**< Major value used to identify Beacons. */
#define APP_MINOR_VALUE         0x00, 0x08      /**< Minor value used to identify Beacons. -> caution: see UICR*/
#define APP_MEASURED_RSSI       0xC3            /**< The Beacon's measured RSSI at 1 meter distance in dBm. */
#define APP_DATA_TEMP           0xfe, 0xfe      /**< Temperature data. */
#define APP_DATA_HUM            0xfd, 0xfd      /**< Humidity data. */
#define APP_DAT_X               0xaa, 0xaa      /**< Acceleration X data. */
#define APP_DAT_y               0xbb, 0xbb      /**< Acceleration Y data. */
#define APP_DAT_Z               0xcc, 0xcc      /**< Acceleration Z Temperature data. */
#define APP_DAT_BATTERY         0x0B, 0xB8      /**< Battery voltage data. */
#define APP_BEACON_PAD          0x99            /**< Padding data (maybe used, maybe not. */

#define PAYLOAD_OFFSET_IN_BEACON_INFO   18      /**< First position to write the payload to */
#define PAYLOAD_OFFSET_IN_BEACON_INFO_ADV   18      /**< NEW: First position to write the payload to with new adv code // TODO */
#define PAYLOAD_OFFSET_BATTERY_INFO     (PAYLOAD_OFFSET_IN_BEACON_INFO+10)  /**< Position to write the battery voltage payload to */									
#define ADC_REF_VOLTAGE_IN_MILLIVOLTS   600     /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION    6       /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS  0       /**< Typical forward voltage drop of the diode (270 mV), but no diode on this beacon. */
#define ADC_RES_10BIT                   1024    /**< Maximum digital value for 10-bit ADC conversion. */
#define ADC_RES_12BIT                   4096    /**< Maximum digital value for 12-bit ADC conversion. */
#define ADC_RES_14BIT                   16384   /**< Maximum digital value for 14-bit ADC conversion. */

// BLE Services + Device information service (DIS) defines
#define DEVICE_NAME                     "Beac8"                                 /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "NordicSemiconductor"                   /**< Manufacturer. Will be passed to Device Information Service. */
#define SERIAL_NUMBER                   "0000000000000001" 

//#define APP_ADV_INTERVAL                50                                      /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define APP_ADV_DURATION                18000                                   /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< Maximum encryption key size. */




#define DEAD_BEEF                       0xDEADBEEF  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
#define MAJ_VAL_OFFSET_IN_BEACON_INFO   6       /**< Position of the MSB of the Major Value in m_beacon_info array. */
#define UICR_ADDRESS            0x10001080      /**< Address of the UICR register  */
#endif

NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                             /**< Advertising module instance. */


static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

ble_os_t m_our_service;

static ble_uuid_t m_adv_uuids[] = 
{
    { BLE_UUID_OUR_SERVICE,                 BLE_UUID_TYPE_BLE  },    // only short 16bit UUID in advdata
//    { BLE_UUID_DEVICE_INFORMATION_SERVICE,  BLE_UUID_TYPE_BLE}
};


static void advertising_start(bool erase_bonds);

/**@brief Function for processing all sensor data.
 *
 * @details  This function will calculate the mean values after reading from
 *           and prepare for adv packet
 */
void process_all_data()
{
//m_advertising.enc_advdata[PAYLOAD_OFFSET_IN_BEACON_INFO_ADV] = 0xFF;
//return;    // TODO
    NRF_LOG_DEBUG("process_all_data()");
	
    if(!m_advertising.initialized) {
        NRF_LOG_DEBUG("m_advertising not initialized -> exiting process_all_data()");
        return;
    } else {
        NRF_LOG_INFO("process_all_data");
    }

    uint8_t payload_idx = PAYLOAD_OFFSET_IN_BEACON_INFO_ADV; // TODO PAYLOAD_OFFSET_IN_BEACON_INFO;
    static uint8_t counter_show_val = 0;

    // calculate values from raw data, but for adv package take already encoded data
    m_sample.temp      = SHT3_GET_TEMPERATURE_VALUE(m_buffer[0], m_buffer[1]);
    m_sample.humidity  = SHT3_GET_HUMIDITY_VALUE   (m_buffer[3], m_buffer[4]);
    m_sample.x         = KX022_GET_ACC(m_buffer[ 6], m_buffer[ 7]);
    m_sample.y         = KX022_GET_ACC(m_buffer[ 8], m_buffer[ 9]);
    m_sample.z         = KX022_GET_ACC(m_buffer[10], m_buffer[11]);

//    NRF_LOG_HEXDUMP_DEBUG(m_adv_data.adv_data.p_data, 29);
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
    if(counter_show_val%10){
        // Log example: Temp: 220.00 | Hum:340.00 | X: -257, Y: -129, Z: 16204 
        NRF_LOG_RAW_INFO("Temp: " NRF_LOG_FLOAT_MARKER " | Hum:" NRF_LOG_FLOAT_MARKER " | ", 
        m_sample.temp,
        m_sample.humidity);
        NRF_LOG_RAW_INFO("X %6d, Y %6d, Z %6d |", 
            (int16_t) m_sample.x,
            (int16_t) m_sample.y,
            (int16_t) m_sample.z);
//        NRF_LOG_RAW_INFO("INS1 %d, INS2 %d, INS3 %d, STAT %d\n",
//            m_buffer[17], m_buffer[18], m_buffer[19], m_buffer[20]); 
    }

    // TODO  
    int32_t temperature = ((m_buffer[1]<< 8) | m_buffer[0]);   
    NRF_LOG_INFO("to update %d %d %d", m_buffer[1], m_buffer[0], temperature);
//    sd_temp_get(&temperature);
    our_service_characteristic_update(&m_our_service, &temperature);
 
}

/**@brief Function for multi-step retrieval of sensor data
 *
 * @details  This function will request and fetch the results from the sonsors
 *           in an multi-step approach to implement low-power with RTC
 */
static void read_all_sensors(bool restart)
{
    // KX022 
    static uint8_t config_kx022_0[2] = {KX022_1020_REG_CNTL1, 				0x00 };	// KX022_1020_STANDBY 
    static uint8_t config_kx022_1[2] = {KX022_1020_REG_CNTL1, 				0x40 };	// KX022_1020_STANDBY | KX022_1020_HIGH_RESOLUTION
    static uint8_t config_kx022_2[2] = {KX022_1020_REG_ODCNTL, 			 	0x04 };	// KX022_1020_OUTPUT_RATE_200_HZ
    static uint8_t config_kx022_3[2] = {KX022_1020_REG_CNTL1, 				0xC0 };	// KX022_1020_OPERATE | KX022_1020_HIGH_RESOLUTION

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
        NRF_LOG_DEBUG("read_all step0");
    
        // SHT3
        APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, SHT3_ADDR, config_SHT3_0, 2, false));
        counter_current = nrfx_rtc_counter_get(&rtc);
        counter_read_sht3 = counter_current + 
            NRFX_RTC_US_TO_TICKS(15000, NRFX_RTC_DEFAULT_CONFIG_FREQUENCY) + 1; // 4 = 4/256s = 0,015625 > max duration 15ms
        NRF_LOG_DEBUG("read_all step0 counter current %d, counter read sht3 ready %d",
            counter_current, counter_read_sht3);
    
        // KX022 
        APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, KX022_ADDR, config_kx022_0, 2, false));
        APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, KX022_ADDR, config_kx022_1, 2, false));
        APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, KX022_ADDR, config_kx022_2, 2, false));
    
        counter_current = nrfx_rtc_counter_get(&rtc);
        APP_ERROR_CHECK(nrf_drv_rtc_cc_set(&rtc, 2, counter_current+
            NRFX_RTC_US_TO_TICKS(4000, NRFX_RTC_DEFAULT_CONFIG_FREQUENCY) + 1, true));	// 1 = 1/256s = 0,0039 =~4ms >1.2/ODR
        step++;
        break;
    
    case 1:
        NRF_LOG_DEBUG("read_all step1");
    
        // KX022 
        APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, KX022_ADDR, config_kx022_3, 2, false));
    
        counter_current = nrfx_rtc_counter_get(&rtc);
        APP_ERROR_CHECK(nrf_drv_rtc_cc_set(&rtc, 2, counter_current+
            NRFX_RTC_US_TO_TICKS(4000, NRFX_RTC_DEFAULT_CONFIG_FREQUENCY) + 1, true));	// 1 = 1/256s = 0,0039 =~4ms >1.2/ODR
        step++;
        break;
    
    case 2:
        NRF_LOG_DEBUG("read_all step2");
    
        // KX022 
        reg[0] = KX022_1020_REG_XOUTL;
        APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, KX022_ADDR, reg, 1, true));
        // read 6 bytes (x (lsb+msb), y (lsb+msb), z (lsb+msb)
        APP_ERROR_CHECK(nrf_drv_twi_rx(&m_twi, KX022_ADDR, &m_buffer[6], 6));
        APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, KX022_ADDR, config_kx022_0, 2, false));

        counter_current = nrfx_rtc_counter_get(&rtc);
        NRF_LOG_DEBUG("read_all step2 counter current %d, counter read sht3 ready %d",
            counter_current, counter_read_sht3);
        if(counter_current < counter_read_sht3){
            APP_ERROR_CHECK(nrf_drv_rtc_cc_set(&rtc, 2, counter_read_sht3, true));
            step++;
            break;  // time to go until SHT3 is ready
        } else {
            step++;
            // just continue w/step3, SHT3 is ready
            NRF_LOG_DEBUG("just continue w/step3");
        }
    
    case 3:
        NRF_LOG_DEBUG("read_all step3");
    
        // read 6 bytes (temp (msb+lsb+crc) and hum (msb+lsb+crc)
        APP_ERROR_CHECK(nrf_drv_twi_rx(&m_twi, SHT3_ADDR, &m_buffer[0], 6));
            
        process_all_data();
    
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

/**@brief Function for handling the ADC interrupt.
 *
 * @details  This function will fetch the conversion result from the ADC, convert the value into
 *           percentage and send it to peer.
 */
void saadc_event_handler(nrf_drv_saadc_evt_t const * p_event)
{
    uint32_t            err_code;
    nrf_saadc_value_t   adc_result;
    uint8_t             percentage_batt_lvl;

    NRF_LOG_DEBUG("saadc_event_handler");

    // regular SAADC sensor calibration 
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE){
        if((m_adc_evt_counter % SAADC_CALIBRATION_INTERVAL) == 0){
            NRF_LOG_DEBUG("SAADC calibration starting...");
            NRF_SAADC->EVENTS_CALIBRATEDONE = 0; 
            nrf_saadc_task_trigger(NRF_SAADC_TASK_CALIBRATEOFFSET);
            while(!NRF_SAADC->EVENTS_CALIBRATEDONE);
            while(NRF_SAADC->STATUS == (SAADC_STATUS_STATUS_Busy << SAADC_STATUS_STATUS_Pos));
            NRF_LOG_DEBUG("SAADC calibration complete ! \n");
        }

        m_adc_evt_counter++;
				
        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAADC_SAMPLES_IN_BUFFER);  //Set buffer so the SAADC can write to it again. This is either "buffer 1" or "buffer 2"
        APP_ERROR_CHECK(err_code);
			
        adc_result = p_event->data.done.p_buffer[0];

        m_battery_millivolts = ADC_RESULT_IN_MILLI_VOLTS(adc_result) +
                                    DIODE_FWD_VOLT_DROP_MILLIVOLTS;
			
        percentage_batt_lvl = battery_level_in_percent(m_battery_millivolts);

        NRF_LOG_DEBUG("saadc_event_handler, done, perc %d, volts %d", percentage_batt_lvl, m_battery_millivolts);

        uint8_t payload_idx = PAYLOAD_OFFSET_BATTERY_INFO;
        m_advertising.enc_advdata[payload_idx++] = MSB_16(m_battery_millivolts);
        m_advertising.enc_advdata[payload_idx++] = LSB_16(m_battery_millivolts);
				
        nrf_drv_saadc_uninit();                                                                   //Unintialize SAADC to disable EasyDMA and save power
        NRF_SAADC->INTENCLR = (SAADC_INTENCLR_END_Clear << SAADC_INTENCLR_END_Pos);               //Disable the SAADC interrupt
        NVIC_ClearPendingIRQ(SAADC_IRQn);                                                         //Clear the SAADC interrupt if set
        m_saadc_initialized = false;                                                              //Set SAADC as uninitialized
    }
}

/**@brief Function for handling the RTC interrupt.
 *
 * @details  This function will handle the RTC interrupt and trigger subsequent
 *           actions.
 */
static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
    uint32_t counter_current;

    NRF_LOG_DEBUG("rtc_handler");

    // SAADC (battery voltage)
    if (int_type == NRF_DRV_RTC_INT_COMPARE0){
        NRF_LOG_DEBUG("rtc_handler COMPARE0");

        if(!m_saadc_initialized) {
            saadc_init();
        }
        m_saadc_initialized = true;

        // Trigger the SAADC SAMPLE task
        nrf_drv_saadc_sample();
		
        // Set counter for next sample
        counter_current = nrfx_rtc_counter_get(&rtc);
        APP_ERROR_CHECK(nrf_drv_rtc_cc_set(&rtc, 0, 
            counter_current + RTC_CC_VALUE * RTC_SADC_UPDATE, true));
    }
		
    // read sensors
    if (int_type == NRF_DRV_RTC_INT_COMPARE1){
        NRF_LOG_DEBUG("rtc_handler COMPARE1");

        // Trigger the sensor retrieval task
        read_all_sensors(true);	// init w/step0
			
        // Set counter for next sample
        counter_current = nrfx_rtc_counter_get(&rtc);
        APP_ERROR_CHECK(nrf_drv_rtc_cc_set(&rtc, 1, 
            counter_current + RTC_CC_VALUE * RTC_SENSOR_UPDATE, true));
    }
		
    // delay timeout during sensor retrieval function
    if (int_type == NRF_DRV_RTC_INT_COMPARE2){
        // Trigger the sensor retrieval task for subsequent steps
        read_all_sensors(false);
    }
}

/**@brief Function for handling BSP events.
 *
 * @details  This function will handle the BSP events as button press.
 */
static void bsp_event_handler(bsp_event_t event)
{
    static uint32_t counter_dfu = 0;
    static uint8_t  countdown_dfu = 3;
    static bool current_leds = false;

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
        if(nrfx_rtc_counter_get(&rtc) > counter_dfu){
            // reset time and countdown to initiate DFU
            NRF_LOG_INFO("reset time and countdown to initiate DFU");
            counter_dfu = nrfx_rtc_counter_get(&rtc) +
                INITIATE_DFU_TIMEOUT * NRFX_RTC_DEFAULT_CONFIG_FREQUENCY;
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
        break;

    case BSP_EVENT_KEY_1_RELEASED: // button on jig released
        NRF_LOG_INFO("button BSP_EVENT_KEY_1_RELEASED");
        break;
    
    case BSP_EVENT_KEY_1_LONG: // button on jig long pressed
        NRF_LOG_INFO("button BSP_EVENT_KEY_1_LONG");    

        if(nrfx_rtc_counter_get(&rtc) > counter_dfu){
            // reset time and countdown to initiate DFU
            NRF_LOG_INFO("reset time and countdown to initiate DFU");
            counter_dfu = nrfx_rtc_counter_get(&rtc) +
                INITIATE_DFU_TIMEOUT * NRFX_RTC_DEFAULT_CONFIG_FREQUENCY;
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
    default:
        break;
    }
}

static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] =      /**< Information advertised by the Beacon. */
{
//    APP_DEVICE_TYPE,        // Manufacturer specific information.
//    APP_ADV_DATA_LENGTH,    // Manufacturer specific information. Length of the manufacturer specific data 
//    APP_BEACON_UUID_SHORT,  // short UUID value.
    APP_MAJOR_VALUE,        // Device major value
    APP_MINOR_VALUE,        // Device minor value
//    APP_MEASURED_RSSI,      // Beacon's measured TX power 
    APP_DATA_TEMP,          // temperature
    APP_DATA_HUM,           // humidity
    APP_DAT_X,              // accel x pos
    APP_DAT_y,              // accel y pos
    APP_DAT_Z,              // accel z pos
    APP_DAT_BATTERY         // battery voltage
};


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


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
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
        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_SLOW); // TODO was BLE_ADV_MODE_FAST

        APP_ERROR_CHECK(err_code);
    }
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

/**@brief Function for initializing bsp module.
 */
void bsp_configuration()
{
    uint32_t err_code;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing logging. */
static void log_init()
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

static void lfclk_config()
{
    // Initialize the clock source specified in the nrf_drv_config.h file, i.e. the CLOCK_CONFIG_LF_SRC constant
    ret_code_t err_code = nrf_drv_clock_init();                  			
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL);
}

static void rtc_config()
{
    // Initialize RTC instance
    nrf_drv_rtc_config_t rtc_configuration = NRF_DRV_RTC_DEFAULT_CONFIG;		
	
    NRF_LOG_DEBUG("rtc_config: prescaler %d, freq %d, rtc input freq %d", 
        rtc_configuration.prescaler, NRFX_RTC_DEFAULT_CONFIG_FREQUENCY, RTC_INPUT_FREQ);
	
    // Initialize RTC with callback handler
    APP_ERROR_CHECK(nrf_drv_rtc_init(&rtc, &rtc_configuration, rtc_handler));

    //Set RTC compare0 value to trigger first interrupt 
    APP_ERROR_CHECK(nrf_drv_rtc_cc_set(&rtc, 0, RTC_CC_VALUE*16, true));

    //Set RTC compare1 value to trigger first interrupt 
    APP_ERROR_CHECK(nrf_drv_rtc_cc_set(&rtc, 1, RTC_CC_VALUE*32, true));

    //Enable RTC instance
    nrf_drv_rtc_enable(&rtc);
}

/**@brief Function for configuring ADC to do battery level conversion.
 */
static void saadc_init()
{
    //Configure SAADC
    nrf_drv_saadc_config_t saadc_config = NRFX_SAADC_DEFAULT_CONFIG;
	
    // Initialize SAADC
    APP_ERROR_CHECK(nrf_drv_saadc_init(&saadc_config, saadc_event_handler));

    //Configure SAADC channel
    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);

    //Initialize SAADC channel
    APP_ERROR_CHECK(nrf_drv_saadc_channel_init(0, &channel_config));

		// Configure burst mode for channel 0
    if(SAADC_BURST_MODE){
        NRF_SAADC->CH[0].CONFIG |= 0x01000000; 
    }

    //Set SAADC buffer 1. The SAADC will start to write to this buffer
    APP_ERROR_CHECK(nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAADC_SAMPLES_IN_BUFFER));    

    //Set SAADC buffer 2. The SAADC will write to this buffer when buffer 1 is full. This will give the applicaiton time to process data in buffer 1.
    APP_ERROR_CHECK(nrf_drv_saadc_buffer_convert(m_buffer_pool[1],SAADC_SAMPLES_IN_BUFFER));   
}


/**@brief Function for initializing power management.
 */
static void power_management_init()
{
    APP_ERROR_CHECK(nrf_pwr_mgmt_init());
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

    NRF_LOG_DEBUG("twi_config");

    APP_ERROR_CHECK(nrf_drv_twi_init(&m_twi, &config, NULL, NULL));	// blocking TWI
    nrf_drv_twi_enable(&m_twi);
}

/**@brief Function for initializing sensors.
 */
static void sensor_init()
{	
    // SHT3
    static uint8_t config_sht3_0[2] = { (SHT3_SOFTRESET >> 8), (SHT3_SOFTRESET & 0xFF) };
    APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, SHT3_ADDR, config_sht3_0, 2, false));
		
    // KX022 
    static uint8_t config_kx022_0[2] = {KX022_1020_REG_CNTL1, 0x00 };	// KX022_1020_STANDBY 
    APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, KX022_ADDR, config_kx022_0, 2, false));
}

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
//            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
//            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_SLOW:
            NRF_LOG_INFO("Slow advertising.");
//            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
//            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            NRF_LOG_INFO("Idle advertising.");
//            sleep_mode_enter();   // TODO
            break;

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

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            // LED indication will be changed when advertising starts.
            break;

        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
//            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
//            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
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

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

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

        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
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


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
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

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init = {0};
    ble_dis_init_t     dis_init;

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize our own service 
    our_service_init (&m_our_service);

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);
    ble_srv_ascii_to_utf8(&dis_init.serial_num_str, (char *)SERIAL_NUMBER);

    dis_init.dis_char_rd_sec = SEC_OPEN;

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
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
/*
#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
    // If USE_UICR_FOR_MAJ_MIN_VALUES is defined, the major and minor values will be read from the
    // UICR instead of using the default values. The major and minor values obtained from the UICR
    // are encoded into advertising data in big endian order (MSB First).
    // To set the UICR used by this example to a desired value, write to the address 0x10001080
    // using the nrfjprog tool. The command to be used is as follows.
    // nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val <your major/minor value>
    // For example, for a major value and minor value of 0xabcd and 0x0102 respectively, the
    // the following command should be used.
    // nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val 0xabcd0102
    uint16_t major_value = ((*(uint32_t *)UICR_ADDRESS) & 0xFFFF0000) >> 16;
    uint16_t minor_value = ((*(uint32_t *)UICR_ADDRESS) & 0x0000FFFF);

    uint8_t index = MAJ_VAL_OFFSET_IN_BEACON_INFO;

    m_beacon_info[index++] = MSB_16(major_value);
    m_beacon_info[index++] = LSB_16(major_value);

    m_beacon_info[index++] = MSB_16(minor_value);
    m_beacon_info[index++] = LSB_16(minor_value);
#endif
*/
    //Set manufacturing data
    ble_advdata_manuf_data_t                manuf_specific_data;
    manuf_specific_data.company_identifier  = APP_COMPANY_IDENTIFIER;
    manuf_specific_data.data.p_data         = (uint8_t *) m_beacon_info;
    manuf_specific_data.data.size           = APP_BEACON_INFO_LENGTH;

    // Build and set advertising data.
    init.advdata.name_type                  = BLE_ADVDATA_NO_NAME;  // BLE_ADVDATA_FULL_NAME
    init.advdata.flags                      = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.p_manuf_specific_data      = &manuf_specific_data;
    init.advdata.uuids_complete.uuid_cnt    = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids     = m_adv_uuids;

    int8_t tx_power                         = 0; // set TX power for advertising
    init.advdata.p_tx_power_level           = &tx_power;

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

    // Initialize advertising parameters (used when starting advertising).
    // TODO
//    memset(&m_adv_params, 0, sizeof(m_adv_params));
//
//    m_adv_params.properties.type    = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
//    m_adv_params.p_peer_addr        = NULL;     // Undirected advertisement.
//    m_adv_params.filter_policy      = BLE_GAP_ADV_FP_ANY;
//    m_adv_params.interval           = NON_CONNECTABLE_ADV_INTERVAL;
//    m_adv_params.duration           = 0;        // Never time out.

    // Set advertising modes and intervals
    init.config.ble_adv_fast_enabled    = true;
    init.config.ble_adv_fast_interval   = APP_FAST_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout    = APP_ADV_DURATION;
    init.config.ble_adv_slow_enabled    = true;
    init.config.ble_adv_slow_interval   = APP_SLOW_ADV_INTERVAL;
    init.config.ble_adv_slow_timeout    = 0;

    // Set event handler that will be called upon advertising events
    init.evt_handler = on_adv_evt;

    // ble_app
    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

/**
 * @brief Function for application main entry.
 */
int main()
{
    bool erase_bonds = false;

    // Initialization and configuration
    log_init();                 // Initialize logging
    power_management_init();    // Initialize power management	
    NRF_POWER->DCDCEN = 1;      // Enabling the DCDC converter for lower current consumption
    lfclk_config();             // Configure low frequency 32kHz clock
    app_timer_init();           // Initialize app timer
    
    bsp_configuration();        // Initialize BSP (leds and buttons)
    bsp_event_to_button_action_assign(0, BSP_BUTTON_ACTION_RELEASE,  BSP_EVENT_KEY_0_RELEASED);
    bsp_event_to_button_action_assign(0, BSP_BUTTON_ACTION_LONG_PUSH,  BSP_EVENT_KEY_0_LONG);
    bsp_event_to_button_action_assign(1, BSP_BUTTON_ACTION_RELEASE,  BSP_EVENT_KEY_1_RELEASED);
    bsp_event_to_button_action_assign(1, BSP_BUTTON_ACTION_LONG_PUSH,  BSP_EVENT_KEY_1_LONG);    
    rtc_config();               // Configure RTC
    twi_config();               // Initialize TWI (with transaction manager) 

    nrf_delay_ms(10);           // sensor startup time: KX022 10 ms, SHT3 1 ms
    sensor_init();              // Initialize sensors
	
    ble_stack_init();           // Initialize the BLE stack
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();         // Initialize the advertising functionality
    conn_params_init();
    peer_manager_init();    
		
    // Start execution.
    NRF_LOG_INFO("Beacon started.");

    advertising_start(erase_bonds);

    for (;;)
    {
        NRF_LOG_FLUSH();
        idle_state_handle();
    }
}
