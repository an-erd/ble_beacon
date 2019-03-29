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
#include "boards.h"
#include "nrf.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nordic_common.h"
#include "bsp.h"
#include "app_error.h"
#include "nrf_soc.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_delay.h"
#include "ble_advdata.h"
#include "app_util_platform.h"
#include "app_timer.h"
//#include "app_button.h"
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
#include "compiler_abstraction.h"

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
// - BLE adv         1 sec
// - SADC sample    60 sec
// - Sensor sample  15 sec

// RTC defines
#define RTC_CC_VALUE 				8       // prescale 256 Hz, RTC_CC_VALUE=8 => 1/32 sec
#define RTC_SADC_UPDATE             1875
#define RTC_SENSOR_UPDATE           480

// SAADC defines
#define SAADC_CALIBRATION_INTERVAL  5       // SAADC calibration interval relative to NRF_DRV_SAADC_EVT_DONE event
#define SAADC_SAMPLES_IN_BUFFER     1       // Number of SAADC samples in RAM before returning a SAADC event
#define SAADC_BURST_MODE            0       // Set to 1 to enable BURST mode, otherwise set to 0.

// SAADC forward declaration and variables
void saadc_init(void);
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
#define BUFFER_SIZE         21  // read buffer from sensors: temp+hum (6=2*msb,lsb,crc) + xyz (6=3*lsb,msb) + INT_REL (5) + INS1 (4)
static uint8_t m_buffer[BUFFER_SIZE];

// Data structures needed for averaging of data read from sensors.
#define NUMBER_OF_SAMPLES   5

typedef struct
{
    int16_t temp;
    int16_t humidity;
    int32_t x;
    int32_t y;
    int32_t z;
} sum_t;
static sum_t m_sum = { 0, 0, 0, 0, 0 };

typedef struct
{
    int16_t temp;
    int16_t humidity;
    int16_t x;
    int16_t y;
    int16_t z;
} sample_t;
static sample_t     m_samples[NUMBER_OF_SAMPLES] = { { 0, 0, 0, 0, 0 } };
static uint16_t 	m_battery_millivolts = 3333;    // default to some value, say 3333
static uint8_t      m_sample_idx = 0;

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
        ((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION)

// BLE defines and structs
#define APP_BLE_CONN_CFG_TAG            1                                   /**< A tag identifying the SoftDevice BLE configuration. */
#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(1000, UNIT_0_625_MS)  /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

// beacon data
#define APP_BEACON_INFO_LENGTH  0x17            /**< Total length of information advertised by the Beacon. */
#define APP_ADV_DATA_LENGTH     0x15            /**< Length of manufacturer specific data in the advertisement. */
#define APP_DEVICE_TYPE         0x02            /**< 0x02 refers to Beacon. */
#define APP_COMPANY_IDENTIFIER  0x0059          /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
#define APP_BEACON_UUID_SHORT   0x01, 0x12, 0x23, 0x34  /**< Proprietary UUID for Beacon. */
#define APP_MAJOR_VALUE         0x00, 0x07      /**< Major value used to identify Beacons. */
#define APP_MINOR_VALUE         0x00, 0x01      /**< Minor value used to identify Beacons. */
#define APP_MEASURED_RSSI       0xC3            /**< The Beacon's measured RSSI at 1 meter distance in dBm. */
#define APP_DATA_TEMP           0xfe, 0xfe      /**< Temperature data. */
#define APP_DATA_HUM            0xfd, 0xfd      /**< Humidity data. */
#define APP_DAT_X               0xaa, 0xaa      /**< Acceleration X data. */
#define APP_DAT_y               0xbb, 0xbb      /**< Acceleration Y data. */
#define APP_DAT_Z               0xcc, 0xcc      /**< Acceleration Z Temperature data. */
#define APP_DAT_BATTERY         0x0B, 0xB8      /**< Battery voltage data. */
#define APP_BEACON_PAD          0x99            /**< Padding data (maybe used, maybe not. */

#define PAYLOAD_OFFSET_IN_BEACON_INFO   18      /**< First position to write the payload to */
#define PAYLOAD_OFFSET_BATTERY_INFO     (PAYLOAD_OFFSET_IN_BEACON_INFO+10)  /**< Position to write the battery voltage payload to */									
#define ADC_REF_VOLTAGE_IN_MILLIVOLTS   600     /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION    6       /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS  0       /**< Typical forward voltage drop of the diode (270 mV), but no diode on this beacon. */
#define ADC_RES_10BIT                   1024    /**< Maximum digital value for 10-bit ADC conversion. */
#define ADC_RES_12BIT                   4096    /**< Maximum digital value for 12-bit ADC conversion. */
									
#define DEAD_BEEF                       0xDEADBEEF  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
#define MAJ_VAL_OFFSET_IN_BEACON_INFO   6       /**< Position of the MSB of the Major Value in m_beacon_info array. */
#define UICR_ADDRESS            0x10001080      /**< Address of the UICR register  */
#endif

static ble_gap_adv_params_t m_adv_params;       /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t              m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;  /**< Advertising handle used to identify an advertising set. */
static uint8_t              m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];   /**< Buffer for storing an encoded advertising set. */
static volatile bool        g_setAdvData = false;                           /**< one-shot flag for setting adv data. */

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

/**@brief Function for processing all sensor data.
 *
 * @details  This function will calculate the mean values after reading from
 *           and prepare for adv packet
 */
void process_all_data()
{
    NRF_LOG_DEBUG("process_all_data()");
	
    uint8_t payload_idx = PAYLOAD_OFFSET_IN_BEACON_INFO;

    sample_t * p_sample = &m_samples[m_sample_idx];

    m_sum.temp          -= p_sample->temp;
    m_sum.humidity      -= p_sample->humidity;
    m_sum.x             -= p_sample->x;
    m_sum.y             -= p_sample->y;
    m_sum.z             -= p_sample->z;

    p_sample->temp      = SHT3_GET_TEMPERATURE_VALUE(m_buffer[0], m_buffer[1]);
    p_sample->humidity  = SHT3_GET_HUMIDITY_VALUE   (m_buffer[3], m_buffer[4]);
    p_sample->x         = KX022_GET_ACC(m_buffer[ 6], m_buffer[ 7]);
    p_sample->y         = KX022_GET_ACC(m_buffer[ 8], m_buffer[ 9]);
    p_sample->z         = KX022_GET_ACC(m_buffer[10], m_buffer[11]);

    m_sum.temp          += p_sample->temp;
    m_sum.humidity      += p_sample->humidity;
    m_sum.x             += p_sample->x;
    m_sum.y             += p_sample->y;
    m_sum.z             += p_sample->z;

    ++m_sample_idx;
    m_sample_idx %= NUMBER_OF_SAMPLES;

//    NRF_LOG_HEXDUMP_DEBUG(m_adv_data.adv_data.p_data, 29);
    m_adv_data.adv_data.p_data[payload_idx++] = MSB_16(m_sum.temp       *10/NUMBER_OF_SAMPLES);
    m_adv_data.adv_data.p_data[payload_idx++] = LSB_16(m_sum.temp       *10/NUMBER_OF_SAMPLES);
    m_adv_data.adv_data.p_data[payload_idx++] = MSB_16(m_sum.humidity   /NUMBER_OF_SAMPLES);
    m_adv_data.adv_data.p_data[payload_idx++] = LSB_16(m_sum.humidity   /NUMBER_OF_SAMPLES);
    m_adv_data.adv_data.p_data[payload_idx++] = MSB_16(m_sum.x          /NUMBER_OF_SAMPLES);
    m_adv_data.adv_data.p_data[payload_idx++] = LSB_16(m_sum.x          /NUMBER_OF_SAMPLES);
    m_adv_data.adv_data.p_data[payload_idx++] = MSB_16(m_sum.y          /NUMBER_OF_SAMPLES);
    m_adv_data.adv_data.p_data[payload_idx++] = LSB_16(m_sum.y          /NUMBER_OF_SAMPLES);
    m_adv_data.adv_data.p_data[payload_idx++] = MSB_16(m_sum.z          /NUMBER_OF_SAMPLES);
    m_adv_data.adv_data.p_data[payload_idx++] = LSB_16(m_sum.z          /NUMBER_OF_SAMPLES);
    if((m_sample_idx%10) == 0 ){
        // Log example: Temp: 220.00 | Hum:340.00 | X: -257, Y: -129, Z: 16204 
        NRF_LOG_RAW_INFO("Temp: " NRF_LOG_FLOAT_MARKER " | Hum:" NRF_LOG_FLOAT_MARKER " | ", 
        NRF_LOG_FLOAT((float)((m_sum.temp*10) / NUMBER_OF_SAMPLES)),
        NRF_LOG_FLOAT((float)((m_sum.humidity*10) / NUMBER_OF_SAMPLES)));
        NRF_LOG_RAW_INFO("X %6d, Y %6d, Z %6d |", 
            (int16_t) p_sample->x,
            (int16_t) p_sample->y,
            (int16_t) p_sample->z);
        NRF_LOG_RAW_INFO("sum X %6d, Y %6d, Z %6d |", 
            (int16_t) (m_sum.x / NUMBER_OF_SAMPLES),
            (int16_t) (m_sum.y / NUMBER_OF_SAMPLES),
            (int16_t) (m_sum.z/ NUMBER_OF_SAMPLES));
        NRF_LOG_RAW_INFO("INS1 %d, INS2 %d, INS3 %d, STAT %d\n",
            m_buffer[17], m_buffer[18], m_buffer[19], m_buffer[20]); 
    }
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
        m_adv_data.adv_data.p_data[payload_idx++] = MSB_16(m_battery_millivolts);
        m_adv_data.adv_data.p_data[payload_idx++] = LSB_16(m_battery_millivolts);
				
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
    NRF_LOG_DEBUG("bsp_event_handler, button %d", event);
	
    switch (event)
    {
    case BSP_EVENT_KEY_0: // button on beacon pressed
        NRF_LOG_INFO("button BSP_EVENT_KEY_0");
        break;

    case BSP_EVENT_KEY_0_RELEASED: // button on beacon released
        NRF_LOG_INFO("button BSP_EVENT_KEY_0_RELEASED");
        break;
    
    case BSP_EVENT_KEY_0_LONG: // button on beacon long pressed
        NRF_LOG_INFO("button BSP_EVENT_KEY_0_LONG");
        break;
    
    case BSP_EVENT_KEY_1: // button on jig pressed
        NRF_LOG_INFO("button BSP_EVENT_KEY_1");
        break;

    case BSP_EVENT_KEY_1_RELEASED: // button on jig released
        NRF_LOG_INFO("button BSP_EVENT_KEY_1_RELEASED");
        break;
    
    case BSP_EVENT_KEY_1_LONG: // button on jig long pressed
        NRF_LOG_INFO("button BSP_EVENT_KEY_1_LONG");
        break;


    default:
        break;
    }
}

static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] =      /**< Information advertised by the Beacon. */
{
    APP_DEVICE_TYPE,        // Manufacturer specific information.
    APP_ADV_DATA_LENGTH,    // Manufacturer specific information. Length of the manufacturer specific data 
    APP_BEACON_UUID_SHORT,  // short UUID value.
    APP_MAJOR_VALUE,        // Device major value
    APP_MINOR_VALUE,        // Device minor value
    APP_MEASURED_RSSI,      // Beacon's measured TX power 
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

/**@brief Function for starting advertising.
 */
static void advertising_start()
{
    ret_code_t err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

//  err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
//  APP_ERROR_CHECK(err_code);
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
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init()
{
    uint32_t        err_code;
    ble_advdata_t   advdata;
    uint8_t         flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

    ble_advdata_manuf_data_t manuf_specific_data;

    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;

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

    manuf_specific_data.data.p_data = (uint8_t *) m_beacon_info;
    manuf_specific_data.data.size   = APP_BEACON_INFO_LENGTH;

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_NO_NAME;
    advdata.flags                   = flags;
    advdata.p_manuf_specific_data   = &manuf_specific_data;

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.properties.type    = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
    m_adv_params.p_peer_addr        = NULL;     // Undirected advertisement.
    m_adv_params.filter_policy      = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval           = NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.duration           = 0;        // Never time out.

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
    APP_ERROR_CHECK(err_code);

    // SET TX POWER FOR ADVERTISING
    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_adv_handle, 0);
    APP_ERROR_CHECK(err_code); 
}

/**
 * @brief Function for application main entry.
 */
int main()
{
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

    nrf_delay_ms(10);           // sensor tartup time: KX022 10 ms, SHT3 1 ms
    sensor_init();              // Initialize sensors
	
    ble_stack_init();           // Initialize the BLE stack
    advertising_init();         // Initialize the advertising functionality
		
    // Start execution.
    NRF_LOG_INFO("Beacon started.");

    advertising_start();

    for (;;)
    {
        NRF_LOG_FLUSH();
        idle_state_handle();
    }
}
