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
#include "app_timer.h"
#include "nrf_drv_clock.h"
#include "nrf_calendar.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

// App Timer defines
APP_TIMER_DEF(m_repeated_timer_update_calendar);    /**< Handler for repeated timer used to update calendar. */
#define CALENDAR_UPDATE_SECONDS     60
#define APP_TIMER_TICKS_CALENDAR    APP_TIMER_TICKS(CALENDAR_UPDATE_SECONDS*1000)


// ticks = (MS) * (uint64_t)APP_TIMER_CLOCK_FREQ
//         / 1000 * (APP_TIMER_CONFIG_RTC_FREQUENCY + 1)))
// MS =    ticks * (APP_TIMER_CONFIG_RTC_FREQUENCY + 1)
//         / (uint64_t)APP_TIMER_CLOCK_FREQ

#define APP_TIMER_SEC(TICKS)                                        \
            ((uint32_t)ROUNDED_DIV(                                 \
            (TICKS) * (APP_TIMER_CONFIG_RTC_FREQUENCY + 1),         \
            (uint64_t)APP_TIMER_CLOCK_FREQ))

static struct tm time_struct, m_tm_return_time; 
static time_t m_time, m_last_calibrate_time = 0;
static float m_calibrate_factor = 0.0f;
static uint32_t m_calendar_increment = CALENDAR_UPDATE_SECONDS;
static uint32_t m_app_timer_cnt_last_increment = 0;
static bool m_timer_running = false;

static void repeated_timer_handler_update_calendar()
{
    m_time += m_calendar_increment;
    m_app_timer_cnt_last_increment = app_timer_cnt_get();
}

void nrf_cal_init(void)
{
    // Prerequisites
    //  - LFCLK is running (e.g., explicitly started or with SoftDevice)
    //  - useage: 
    //      - set time at least once
    //      - just call nrf_cal_get_time[_calibrated] which will return time

    ret_code_t err_code;

    err_code = app_timer_create(&m_repeated_timer_update_calendar,
                                APP_TIMER_MODE_REPEATED,
                                repeated_timer_handler_update_calendar);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_repeated_timer_update_calendar, APP_TIMER_TICKS_CALENDAR, NULL);
    APP_ERROR_CHECK(err_code);  
    m_timer_running = true;

}

// Returns true if internal nrf_calendar time structures had been initialized with nrf_cal_set_time()
bool nrf_cal_get_initialized()
{
    return (m_last_calibrate_time != 0);
}

void nrf_cal_set_time(uint32_t year, uint32_t month, uint32_t day, uint32_t hour, uint32_t minute, uint32_t second)
{
    ret_code_t err_code;
    static time_t uncal_difftime, difftime, newtime;
    
    time_struct.tm_year = year - 1900;
    time_struct.tm_mon = month;
    time_struct.tm_mday = day;
    time_struct.tm_hour = hour;
    time_struct.tm_min = minute;
    time_struct.tm_sec = second;   
    newtime = mktime(&time_struct);

    if(m_timer_running)
    {
        err_code = app_timer_stop(m_repeated_timer_update_calendar);
        APP_ERROR_CHECK(err_code);
        m_timer_running = false;
    }
        
    // Calculate the calibration offset 
    if(m_last_calibrate_time != 0)
    {
        difftime = newtime - m_last_calibrate_time;
        uncal_difftime = 
            m_time + APP_TIMER_SEC(app_timer_cnt_diff_compute(app_timer_cnt_get(), m_app_timer_cnt_last_increment))
            - m_last_calibrate_time;
        m_calibrate_factor = (float)difftime / (float)uncal_difftime;
        NRF_LOG_INFO("difftime %d, uncal_difftime %d, m_calibrate_factor: " NRF_LOG_FLOAT_MARKER " ", 
            difftime, uncal_difftime, NRF_LOG_FLOAT(m_calibrate_factor));
    }
    
    // Assign the new time to the local time variables
    m_time = m_last_calibrate_time = newtime;
    m_app_timer_cnt_last_increment = app_timer_cnt_get();
    
    err_code = app_timer_start(m_repeated_timer_update_calendar, APP_TIMER_TICKS_CALENDAR, NULL);
    APP_ERROR_CHECK(err_code);  
    m_timer_running = true;

    // TODO
    /*
    static char cal_string[80];
    strftime(cal_string, 80, "%d.%m.%y / %H:%M:%S", localtime(&m_time));
    NRF_LOG_INFO("nrf_cal_set_time, m_time just set to: %s", cal_string);
    NRF_LOG_FLUSH();
    */
}    

struct tm *nrf_cal_get_time(void)
{
    time_t return_time;
    return_time = m_time + APP_TIMER_SEC(app_timer_cnt_diff_compute(app_timer_cnt_get(), m_app_timer_cnt_last_increment));

    m_tm_return_time = *localtime(&return_time);
    return &m_tm_return_time;
}

struct tm *nrf_cal_get_time_calibrated(void)
{
    time_t uncalibrated_time, calibrated_time;
    if(m_calibrate_factor != 0.0f)
    {
        uncalibrated_time = m_time + APP_TIMER_SEC(app_timer_cnt_diff_compute(app_timer_cnt_get(), m_app_timer_cnt_last_increment));
        calibrated_time = m_last_calibrate_time + (time_t)((float)(uncalibrated_time - m_last_calibrate_time) * m_calibrate_factor + 0.5f);
        m_tm_return_time = *localtime(&calibrated_time);
        return &m_tm_return_time;
    }
    else return nrf_cal_get_time();
}

time_t nrf_cal_get_time_long(void)
{
    time_t uncalibrated_time, calibrated_time;
    uncalibrated_time = m_time + APP_TIMER_SEC(app_timer_cnt_diff_compute(app_timer_cnt_get(), m_app_timer_cnt_last_increment));
    if(m_calibrate_factor != 0.0f){
        calibrated_time = m_last_calibrate_time + (time_t)((float)(uncalibrated_time - m_last_calibrate_time) * m_calibrate_factor + 0.5f);
        return calibrated_time;
    } else {
        return uncalibrated_time;
    }
}

char *nrf_cal_get_time_string(bool calibrated)
{
    static char cal_string[80];
//    strftime(cal_string, 80, "%x - %H:%M:%S", (calibrated ? nrf_cal_get_time_calibrated() : nrf_cal_get_time()));
    strftime(cal_string, 80, "%d.%m.%y / %H:%M:%S", (calibrated ? nrf_cal_get_time_calibrated() : nrf_cal_get_time()));
    return cal_string;
}
 
 void nrf_cal_print_current_time()
{
    printf("Uncalibrated time:\t%s\r\n", nrf_cal_get_time_string(false));
    printf("Calibrated time:\t%s\r\n", nrf_cal_get_time_string(true));
}

