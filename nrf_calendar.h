/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 * Changed and added functionality by Andreas Erdmann, 2019.
 *
 */

#ifndef __NRF_CALENDAR_H__
#define __NRF_CALENDAR_H__

#include <stdint.h>
#include <stdbool.h>
#include "time.h"

// Initializes the calendar library. Run this before calling any other functions. 
void nrf_cal_init(void);

// Update the internal time by calling this function regularly every "calendar_increment" seconds.
void nrf_cal_increment(void);

// Returns true if internal nrf_calendar time structures had been initialized with nrf_cal_set_time()
bool nrf_cal_get_initialized(void);

// Sets the date and time stored in the calendar library. 
// When this function is called a second time calibration data will be automatically generated based on the error in time since the
// last call to the set time function. To ensure good calibration this function should not be called too often 
// (depending on the accuracy of the 32 kHz clock it should be sufficient to call it between once a week and once a month). 
void nrf_cal_set_time(uint32_t year, uint32_t month, uint32_t day, uint32_t hour, uint32_t minute, uint32_t second);

// Returns the uncalibrated time as a tm struct. For more information about the tm struct and the time.h library in general please refer to:
// http://www.tutorialspoint.com/c_standard_library/time_h.htm
struct tm *nrf_cal_get_time(void);

// Returns the calibrated time as a tm struct. If no calibration data is available it will return the uncalibrated time.
struct tm *nrf_cal_get_time_calibrated(void);

// Returns the calibrated time (calibration factor != 0) or the uncalibrated time, or time since boot, resp., as time_t
time_t nrf_cat_get_time_long(void);

// Returns a string for printing the date and time. Turn the calibration on/off by setting the calibrate parameter. 
char *nrf_cal_get_time_string(bool calibrated);

// Print the calibrated and uncalibrated time
void nrf_cal_print_current_time(void);

#endif
