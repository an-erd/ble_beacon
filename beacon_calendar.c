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
 */

#include "nrf.h"
#include "nrf_log.h"
#include "nrf_drv_rtc.h"
#include "beacon_calendar.h"

extern const nrf_drv_rtc_t rtc;
static struct tm time_struct, m_tm_return_time; 

// store last updated time and corresponding RTC counter
static time_t m_time;
static uint32_t m_time_counter;

// store last calibrate time and corresponding RTC counter
static time_t m_last_calibrate_time = 0;
static uint32_t m_last_calibrate_time_counter = 0;

static float m_calibrate_factor = 0.0f;
 
void nrf_cal_init()
{
    // Prerequisites
    //  - LFCLK is running (e.g., explicitly started or with SoftDevice)
    //  - useage is to just call nrf_cal_get_time[_calibrated] which will update the timer variables
}

void nrf_cal_set_time(uint32_t year, uint32_t month, uint32_t day, uint32_t hour, uint32_t minute, uint32_t second)
{
    static time_t uncal_difftime, difftime, newtime;
    
    uint32_t current_rtc_counter = nrfx_rtc_counter_get(&rtc);

    time_struct.tm_year = year - 1900;
    time_struct.tm_mon = month;
    time_struct.tm_mday = day;
    time_struct.tm_hour = hour;
    time_struct.tm_min = minute;
    time_struct.tm_sec = second;   
    newtime = mktime(&time_struct);

    // Calculate the calibration offset 
    if(m_last_calibrate_time != 0)
    {
        difftime = newtime - m_last_calibrate_time;
        uncal_difftime = m_time - m_last_calibrate_time;
        m_calibrate_factor = (float)difftime / (float)uncal_difftime;
        NRF_LOG_INFO("m_calibrate_factor %d", m_calibrate_factor);
    }

    // Assign the new time to the local time variables and store corresponding RTC counter
    m_time = m_last_calibrate_time = newtime;
    m_time_counter = m_last_calibrate_time_counter = current_rtc_counter;
}    

struct tm *nrf_cal_get_time(void)
{
    uint32_t current_rtc_counter = nrfx_rtc_counter_get(&rtc);
    time_t return_time;
    return_time = m_time + (current_rtc_counter - m_time_counter)/(8*32);   // TODO config values, no constants
    m_tm_return_time = *localtime(&return_time);
    return &m_tm_return_time;
}

struct tm *nrf_cal_get_time_calibrated(void)
{
    uint32_t current_rtc_counter = nrfx_rtc_counter_get(&rtc);
    time_t uncalibrated_time, calibrated_time;
    if(m_calibrate_factor != 0.0f)
    {
        uncalibrated_time = m_time + (current_rtc_counter - m_time_counter)/(8*32);   // TODO config values, no constants
        calibrated_time = m_last_calibrate_time + (time_t)((float)(uncalibrated_time - m_last_calibrate_time) * m_calibrate_factor + 0.5f);
        m_tm_return_time = *localtime(&calibrated_time);
        return &m_tm_return_time;
    }
    else return nrf_cal_get_time();
}

char *nrf_cal_get_time_string(bool calibrated)
{
    static char cal_string[80];
    strftime(cal_string, 80, "%x - %H:%M:%S", (calibrated ? nrf_cal_get_time_calibrated() : nrf_cal_get_time()));
    return cal_string;
}
 