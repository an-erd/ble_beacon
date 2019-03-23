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
#include "ble_advdata.h"
#include "app_util_platform.h"
#include "app_timer.h"
#include "app_button.h"
#include "app_gpiote.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_power.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"
#include "nrfx_rtc.h"
#include "nrf_twi_mngr.h"
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

// RTC defines
#define RTC_CC_VALUE 4                  //Determines the RTC interrupt frequency and thereby the SAADC sampling frequency
// prescaler is 32 Hz, so RTC_CC_VALUE=4 => 1/8 sec
#define RTC_SADC_UPDATE		80						// =every 10 sec (multiply by RTC_CC_VALUE/ = *1/8 sec)
#define RTC_SENSOR_UPDATE	40						// =every  5 sec

// SAADC defines
#define SAADC_CALIBRATION_INTERVAL 5    //Determines how often the SAADC should be calibrated relative to NRF_DRV_SAADC_EVT_DONE event. E.g. value 5 will make the SAADC calibrate every fifth time the NRF_DRV_SAADC_EVT_DONE is received.
#define SAADC_SAMPLES_IN_BUFFER 1       //Number of SAADC samples in RAM before returning a SAADC event. For low power SAADC set this constant to 1. Otherwise the EasyDMA will be enabled for an extended time which consumes high current.
#define SAADC_OVERSAMPLE NRF_SAADC_OVERSAMPLE_DISABLED  //Oversampling setting for the SAADC. Setting oversample to 4x This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times. Enable BURST mode to make the SAADC sample 4 times when triggering SAMPLE task once.
#define SAADC_BURST_MODE 0              //Set to 1 to enable BURST mode, otherwise set to 0.

// SAADC forward declaration and variables
void saadc_init(void);
const  nrf_drv_rtc_t           rtc = NRF_DRV_RTC_INSTANCE(2); /**< Declaring an instance of nrf_drv_rtc for RTC2. */
static nrf_saadc_value_t       m_buffer_pool[2][SAADC_SAMPLES_IN_BUFFER];
static uint32_t                m_adc_evt_counter = 0;
static bool                    m_saadc_initialized = false;      

// TWI defines
#define TWI_INSTANCE_ID     				0
#define MAX_PENDING_TRANSACTIONS    20
NRF_TWI_MNGR_DEF(m_nrf_twi_mngr, MAX_PENDING_TRANSACTIONS, TWI_INSTANCE_ID);

// Pin number for indicating communication with sensors.
#ifdef BSP_LED_0
    #define READ_ALL_INDICATOR  BSP_BOARD_LED_0
#else
    #error "Please choose an output pin"
#endif

// Sensor defines
#define BUFFER_SIZE  21		// Buffer for data read from sensors: temp+hum (6=2*msb,lsb,crc) + xyz (6=3*lsb,msb) + INT_REL (5) + INS1 (4)
static uint8_t m_buffer[BUFFER_SIZE];

// Data structures needed for averaging of data read from sensors.
#define NUMBER_OF_SAMPLES  5
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
static sample_t 	m_samples[NUMBER_OF_SAMPLES] = { { 0, 0, 0, 0, 0 } };

static uint16_t 	battery_millivolts = 3111;	// default to some value, say 3111
static uint8_t 		m_sample_idx = 0;

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


// BLE defines and structs--------------------------------------
#define APP_BLE_CONN_CFG_TAG            1                                  	/**< A tag identifying the SoftDevice BLE configuration. */
#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(1000, UNIT_0_625_MS)  	/**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */
// #define BATTERY_LEVEL_MEAS_INTERVAL     APP_TIMER_TICKS(2000)            		/**< Battery level measurement interval (ticks). */

// beacon data
#define APP_BEACON_INFO_LENGTH          0x17	                            	/**< Total length of information advertised by the Beacon. */
#define APP_ADV_DATA_LENGTH             0x15                               	/**< Length of manufacturer specific data in the advertisement. */
#define APP_DEVICE_TYPE                 0x02                               	/**< 0x02 refers to Beacon. */
#define APP_COMPANY_IDENTIFIER          0x0059                             	/**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
#define APP_BEACON_UUID_SHORT           0x01, 0x12, 0x23, 0x34           		/**< Proprietary UUID for Beacon. */
#define APP_MAJOR_VALUE                 0x00, 0x07                         	/**< Major value used to identify Beacons. */
#define APP_MINOR_VALUE                 0x00, 0x01                         	/**< Minor value used to identify Beacons. */
#define APP_MEASURED_RSSI               0xC3                               	/**< The Beacon's measured RSSI at 1 meter distance in dBm. */
#define APP_DATA_TEMP										0xfe, 0xfe													/**< Temperature data. */
#define APP_DATA_HUM										0xfd, 0xfd													/**< Humidity data. */
#define APP_DAT_X												0xaa, 0xaa													/**< Acceleration X data. */
#define APP_DAT_y												0xbb, 0xbb													/**< Acceleration Y data. */
#define APP_DAT_Z												0xcc, 0xcc													/**< Acceleration Z Temperature data. */
#define APP_DAT_BATTERY									0x0B, 0xB8													/**< Battery voltage data. */
#define APP_BEACON_PAD									0x99																/**< Padding data (maybe used, maybe not. */

#define PAYLOAD_OFFSET_IN_BEACON_INFO		18																	/**< First position to write the payload to */
#define PAYLOAD_OFFSET_BATTERY_INFO			(PAYLOAD_OFFSET_IN_BEACON_INFO+10)	/**< Position to write the battery voltage payload to */									
#define ADC_REF_VOLTAGE_IN_MILLIVOLTS   600                                 /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION    6                                   /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS  0				// was: 270                 /**< Typical forward voltage drop of the diode (270 mV), but no diode on this beacon. */
#define ADC_RES_10BIT                   1024                                /**< Maximum digital value for 10-bit ADC conversion. */
#define ADC_RES_12BIT                   4096                                /**< Maximum digital value for 12-bit ADC conversion. */
									
#define DEAD_BEEF                       0xDEADBEEF                       	  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
#define MAJ_VAL_OFFSET_IN_BEACON_INFO   18                                 	/**< Position of the MSB of the Major Value in m_beacon_info array. */
#define UICR_ADDRESS                    0x10001080                         	/**< Address of the UICR register used by this example. The major and minor versions to be encoded into the advertising data will be picked up from this location. */
#endif

static ble_gap_adv_params_t m_adv_params;                                  	/**< Parameters to be passed to the stack when starting advertising. */
static uint8_t              m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET; 	/**< Advertising handle used to identify an advertising set. */
static uint8_t              m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];  	/**< Buffer for storing an encoded advertising set. */
static volatile bool 				g_setAdvData = false; 													/**< one-shot flag for setting adv data. */

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

void read_all_cb(ret_code_t result, void * p_user_data)
{
	NRF_LOG_INFO("read_all_cb");

	if (result != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("read_all_cb - error: %d", (int)result);
        return;
    }

		uint8_t payload_idx = PAYLOAD_OFFSET_IN_BEACON_INFO;

    sample_t * p_sample = &m_samples[m_sample_idx];
//		NRF_LOG_RAW_INFO("Idx %6d p_sample %6d m_sum %6d ", m_sample_idx, p_sample->x, m_sum.x);

    m_sum.temp 					-= p_sample->temp;
		m_sum.humidity 			-= p_sample->humidity;
    m_sum.x    					-= p_sample->x;
    m_sum.y    					-= p_sample->y;
    m_sum.z    					-= p_sample->z;

    p_sample->temp 			= SHT3_GET_TEMPERATURE_VALUE(m_buffer[0], m_buffer[1]);
    p_sample->humidity 	= SHT3_GET_HUMIDITY_VALUE   (m_buffer[3], m_buffer[4]);
		p_sample->x					= KX022_GET_ACC(m_buffer[ 6], m_buffer[ 7]);
		p_sample->y					= KX022_GET_ACC(m_buffer[ 8], m_buffer[ 9]);
		p_sample->z					= KX022_GET_ACC(m_buffer[10], m_buffer[11]);

    m_sum.temp 					+= p_sample->temp;
    m_sum.humidity 			+= p_sample->humidity;
    m_sum.x    					+= p_sample->x;
    m_sum.y    					+= p_sample->y;
    m_sum.z    					+= p_sample->z;

//		NRF_LOG_RAW_INFO("new p_sample %6d new m_sum %6d \n",p_sample->x, m_sum.x);

    ++m_sample_idx;
    if (m_sample_idx >= NUMBER_OF_SAMPLES)
    {
        m_sample_idx = 0;
    }

//		NRF_LOG_HEXDUMP_INFO(m_adv_data.adv_data.p_data, 29);
		m_adv_data.adv_data.p_data[payload_idx++] = MSB_16(m_sum.temp * 10/NUMBER_OF_SAMPLES);
		m_adv_data.adv_data.p_data[payload_idx++] = LSB_16(m_sum.temp	* 10/NUMBER_OF_SAMPLES);
		m_adv_data.adv_data.p_data[payload_idx++] = MSB_16(m_sum.humidity	/NUMBER_OF_SAMPLES);
		m_adv_data.adv_data.p_data[payload_idx++] = LSB_16(m_sum.humidity	/NUMBER_OF_SAMPLES);
		m_adv_data.adv_data.p_data[payload_idx++] = MSB_16(m_sum.x				/NUMBER_OF_SAMPLES);
		m_adv_data.adv_data.p_data[payload_idx++] = LSB_16(m_sum.x				/NUMBER_OF_SAMPLES);
		m_adv_data.adv_data.p_data[payload_idx++] = MSB_16(m_sum.y				/NUMBER_OF_SAMPLES);
		m_adv_data.adv_data.p_data[payload_idx++] = LSB_16(m_sum.y				/NUMBER_OF_SAMPLES);
		m_adv_data.adv_data.p_data[payload_idx++] = MSB_16(m_sum.z				/NUMBER_OF_SAMPLES);
		m_adv_data.adv_data.p_data[payload_idx++] = LSB_16(m_sum.z				/NUMBER_OF_SAMPLES);
		
//		m_adv_data.adv_data.p_data[payload_idx++] = MSB_16(battery_millivolts);
//		m_adv_data.adv_data.p_data[payload_idx++] = LSB_16(battery_millivolts);
		
    if ( (m_sample_idx%10) == 0 ){
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

static void read_all()
{
    static nrf_twi_mngr_transfer_t const transfers[] =
    {
				// read 6 bytes (temp (msb+lsb+crc) and hum (msb+lsb+crc)
        SHT3_READ_TEMP(&m_buffer[0]) 
//        ,
//        // read 6 bytes (x (lsb+msb), y (lsb+msb), z (lsb+msb)
//				KX022_READ_XYZ(&m_buffer[6])
//				,
//				// read 4 bytes 
//				KX022_READ_INS1(&m_buffer[17])
//				,
//				// read 5 byte interrupt source information
//				KX022_READ_INT_REL(&m_buffer[12])
    };
    static nrf_twi_mngr_transaction_t NRF_TWI_MNGR_BUFFER_LOC_IND transaction =
    {
        .callback            = read_all_cb,
        .p_user_data         = NULL,
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };

    APP_ERROR_CHECK(nrf_twi_mngr_schedule(&m_nrf_twi_mngr, &transaction));
}

/**@brief Function for handling the ADC interrupt.
 *
 * @details  This function will fetch the conversion result from the ADC, convert the value into
 *           percentage and send it to peer.
 */
void saadc_event_handler(nrf_drv_saadc_evt_t const * p_event)
{
		uint32_t          err_code;
    nrf_saadc_value_t adc_result;
    uint16_t          batt_lvl_in_milli_volts;
    uint8_t           percentage_batt_lvl;

		NRF_LOG_INFO("saadc_event_handler");

    if (p_event->type == NRF_DRV_SAADC_EVT_DONE){
        if((m_adc_evt_counter % SAADC_CALIBRATION_INTERVAL) == 0){
            NRF_LOG_INFO("SAADC calibration starting...");
            NRF_SAADC->EVENTS_CALIBRATEDONE = 0; 
            nrf_saadc_task_trigger(NRF_SAADC_TASK_CALIBRATEOFFSET);
            while(!NRF_SAADC->EVENTS_CALIBRATEDONE);
            while(NRF_SAADC->STATUS == (SAADC_STATUS_STATUS_Busy << SAADC_STATUS_STATUS_Pos));
            NRF_LOG_INFO("SAADC calibration complete ! \n");
        }

				m_adc_evt_counter++;
				
        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAADC_SAMPLES_IN_BUFFER);  //Set buffer so the SAADC can write to it again. This is either "buffer 1" or "buffer 2"
        APP_ERROR_CHECK(err_code);
			
        adc_result = p_event->data.done.p_buffer[0];

        batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result) +
                                  DIODE_FWD_VOLT_DROP_MILLIVOLTS;
			
				battery_millivolts = batt_lvl_in_milli_volts;		// use for beacon adv
        percentage_batt_lvl = battery_level_in_percent(batt_lvl_in_milli_volts);

				NRF_LOG_INFO("saadc_event_handler, done, perc %d, volts %d", percentage_batt_lvl, batt_lvl_in_milli_volts);

				uint8_t payload_idx = PAYLOAD_OFFSET_BATTERY_INFO;
				m_adv_data.adv_data.p_data[payload_idx++] = MSB_16(battery_millivolts);
				m_adv_data.adv_data.p_data[payload_idx++] = LSB_16(battery_millivolts);

				
				nrf_drv_saadc_uninit();                                                                   //Unintialize SAADC to disable EasyDMA and save power
        NRF_SAADC->INTENCLR = (SAADC_INTENCLR_END_Clear << SAADC_INTENCLR_END_Pos);               //Disable the SAADC interrupt
        NVIC_ClearPendingIRQ(SAADC_IRQn);                                                         //Clear the SAADC interrupt if set
        m_saadc_initialized = false;                                                              //Set SAADC as uninitialized
		}
}

static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
//		NRF_LOG_INFO("rtc_handler");

		// SAADC (battery voltage) every 10 secs
    if (int_type == NRF_DRV_RTC_INT_COMPARE0)
    {
				NRF_LOG_INFO("rtc_handler COMPARE0");

				if(!m_saadc_initialized) {
            saadc_init();
        }
        m_saadc_initialized = true;                                    //Set SAADC as initialized

				// Trigger the SAADC SAMPLE task
        nrf_drv_saadc_sample();
		
				// Add a time portion to rtc counter compare	and re-enable
				rtc.p_reg->CC[0] += RTC_CC_VALUE * RTC_SADC_UPDATE;
				nrf_drv_rtc_int_enable(&rtc, NRF_RTC_INT_COMPARE0_MASK);
    }
		
		// read sensors every 1/8 secs
		if (int_type == NRF_DRV_RTC_INT_COMPARE1)
    {
				NRF_LOG_INFO("rtc_handler COMPARE1");

				// Trigger the sensor retrieval task
				read_all();
			
				// Add a time portion to rtc counter compare	and re-enable
				rtc.p_reg->CC[1] += RTC_CC_VALUE * RTC_SENSOR_UPDATE;
				nrf_drv_rtc_int_enable(&rtc, NRF_RTC_INT_COMPARE1_MASK);
    }

}

////////////////////////////////////////////////////////////////////////////////
// Buttons handling (by means of BSP).
//
static void bsp_event_handler(bsp_event_t event)
{

		NRF_LOG_INFO("bsp_event_handler, button %d", event);
	
    switch (event)
    {
    case BSP_EVENT_KEY_0: // button on beacon pressed
				NRF_LOG_INFO("button BSP_EVENT_KEY_0");
        break;

    case BSP_EVENT_KEY_1: // button on programming board pressed
				NRF_LOG_INFO("button BSP_EVENT_KEY_1");
        break;

    default:
        break;
    }
}


static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] =  	/**< Information advertised by the Beacon. */
{
    APP_DEVICE_TYPE,     		// Manufacturer specific information.
    APP_ADV_DATA_LENGTH, 		// Manufacturer specific information. Length of the manufacturer specific data 
	  APP_BEACON_UUID_SHORT,  // short UUID value.
    APP_MAJOR_VALUE,     		// Device major value
    APP_MINOR_VALUE,     		// Device minor value
    APP_MEASURED_RSSI,   		// Beacon's measured TX power 
		APP_DATA_TEMP,					// temperature
		APP_DATA_HUM,						// humidity
		APP_DAT_X,							// accel x pos
		APP_DAT_y,							// accel y pos
		APP_DAT_Z,							// accel z pos
		APP_DAT_BATTERY					// battery voltage
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

//		err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
//		APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle()
{
    if (NRF_LOG_PROCESS() == false)
    {
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
    uint32_t err_code;

		// Initialize RTC instance
    nrf_drv_rtc_config_t rtc_configuration = NRF_DRV_RTC_DEFAULT_CONFIG;		
	
		NRF_LOG_INFO("rtc_config: prescaler %d, freq %d, rtc input freq %d", 
				rtc_configuration.prescaler, NRFX_RTC_DEFAULT_CONFIG_FREQUENCY, RTC_INPUT_FREQ);

	
		// Initialize RTC with callback handler
    err_code = nrf_drv_rtc_init(&rtc, &rtc_configuration, rtc_handler);
    APP_ERROR_CHECK(err_code);

		//Set RTC compare0 value to trigger first interrupt 
    err_code = nrf_drv_rtc_cc_set(&rtc, 0, RTC_CC_VALUE*4, true);
    APP_ERROR_CHECK(err_code);

		//Set RTC compare1 value to trigger first interrupt 
    err_code = nrf_drv_rtc_cc_set(&rtc, 1, RTC_CC_VALUE*4, true);
    APP_ERROR_CHECK(err_code);

    //Enable RTC instance
    nrf_drv_rtc_enable(&rtc);
}


/**@brief Function for configuring ADC to do battery level conversion.
 */
static void saadc_init()
{
		ret_code_t err_code;
	
		//Configure SAADC
		nrf_drv_saadc_config_t saadc_config = 
				NRFX_SAADC_DEFAULT_CONFIG;
	
		// Initialize SAADC
    err_code = nrf_drv_saadc_init(&saadc_config, saadc_event_handler);
    APP_ERROR_CHECK(err_code);

    //Configure SAADC channel
    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);

    //Initialize SAADC channel
		err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

		// Configure burst mode for channel 0
    if(SAADC_BURST_MODE)
    {
        NRF_SAADC->CH[0].CONFIG |= 0x01000000; 
    }

		//Set SAADC buffer 1. The SAADC will start to write to this buffer
    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0],SAADC_SAMPLES_IN_BUFFER);    
    APP_ERROR_CHECK(err_code);
		
		//Set SAADC buffer 2. The SAADC will write to this buffer when buffer 1 is full. This will give the applicaiton time to process data in buffer 1.
		err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1],SAADC_SAMPLES_IN_BUFFER);    
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing power management.
 */
static void power_management_init()
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


// TWI (with transaction manager) initialization.
static void twi_config()
{
    uint32_t err_code;

    nrf_drv_twi_config_t const config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_LOWEST,
       .clear_bus_init     = false
    };

    err_code = nrf_twi_mngr_init(&m_nrf_twi_mngr, &config);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing sensors.
 */
static void sensor_init()
{	
    APP_ERROR_CHECK(nrf_twi_mngr_perform(&m_nrf_twi_mngr, NULL, sht3_init_transfers,
        SHT3_INIT_TRANSFER_COUNT, NULL));
//    APP_ERROR_CHECK(nrf_twi_mngr_perform(&m_nrf_twi_mngr, NULL, kx022_init_transfers,
//        KX022_INIT_TRANSFER_COUNT, NULL));
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
    uint32_t      err_code;
    ble_advdata_t advdata;
    uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

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

    advdata.name_type             = BLE_ADVDATA_NO_NAME;
    advdata.flags                 = flags;
    advdata.p_manuf_specific_data = &manuf_specific_data;

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
    m_adv_params.p_peer_addr     = NULL;    // Undirected advertisement.
    m_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval        = NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.duration        = 0;       // Never time out.

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
    APP_ERROR_CHECK(err_code);
		

    // NORDIC: SET HIGH TX POWER FOR ADVERTISING
    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_adv_handle, 0); 	// was: 4 
    APP_ERROR_CHECK(err_code); 
}


// TEST CHECK FOR POWER CONSUMPTION
/*
 * Handler to be called when button is pushed.
 * param[in]   pin_no             The pin number where the event is genereated
 * param[in]   button_action     Is the button pushed or released
 */
static void button_handler(uint8_t pin_no, uint8_t button_action)
{
    if(button_action == APP_BUTTON_PUSH)
    {
			    NRF_LOG_INFO("button_handler, push %d", pin_no);
    }
		
    if(button_action == APP_BUTTON_RELEASE)
    {
			    NRF_LOG_INFO("button_handler, release %d", pin_no);
    }
}

void button_init()
{
	uint32_t err_code;

    // Button configuration structure.
    static app_button_cfg_t p_button[] = { {25, APP_BUTTON_ACTIVE_LOW, NRF_GPIO_PIN_PULLUP, button_handler}};

    // Macro for initializing the GPIOTE module.
    // It will handle dimensioning and allocation of the memory buffer required by the module, making sure that the buffer is correctly aligned.
    APP_GPIOTE_INIT(1);

    // Initializing the buttons.
    err_code = app_button_init(p_button, sizeof(p_button) / sizeof(p_button[0]), 50);
    APP_ERROR_CHECK(err_code);
                                            
    // Enabling the buttons.                                        
    err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);
}

// END TEST

/**
 * @brief Function for application main entry.
 */
int main()
{
		// Initialization and configuration
    log_init();										// Initialize logging
    power_management_init();			// Initialize power management	

		bsp_board_init(BSP_INIT_LEDS);
		bsp_board_led_off(0);
//		button_init();
	
	  NRF_POWER->DCDCEN = 1; 				// Enabling the DCDC converter for lower current consumption
	
	  lfclk_config();               // Configure low frequency 32kHz clock
    rtc_config();                 // Configure RTC

		twi_config();									// Initialize TWI (with transaction manager) 
		sensor_init();								// Initialize sensors
	
    ble_stack_init();							// Initialize the BLE stack
		advertising_init();						// Initialize the advertising functionality
		
    // Start execution.
    NRF_LOG_INFO("Beacon started.");

		advertising_start();

    // Enter main loop.
    for (;;)
    {
        NRF_LOG_FLUSH();
        idle_state_handle();
    }
}
