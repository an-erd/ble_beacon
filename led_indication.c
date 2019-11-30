#include "nrf.h"
#include "app_timer.h"
#include "nrf_drv_clock.h"
#include "led_indication.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"



// App Timer defines
APP_TIMER_DEF(m_singleshot_timer_led_indication);    /**< Handler for single shot timer used for led indication. */

static uint8_t  m_led_indication_type   = LED_NO_INDICATION;
static uint8_t  m_led_indication_step   = 0;
static uint8_t  m_led_current_status    = 0;
static uint16_t m_led_indication_prog[LED_NUM_INDICATIONS][LED_INDCATION_MAX_STEPS] = 
    // Steps: time in ms for (0) OFF - (1) ON - OFF - ON - OFF - ON - OFF - (7) ON
    {
        {500, 100, 200, 400,   0,   0,   0,   0,   0,   0},  // LED_INDICATION_1 = .-
        {500, 100, 200, 100, 200, 400,   0,   0,   0,   0},  // LED_INDICATION_2 = ..-
        {500, 400,   0,   0,   0,   0,   0,   0,   0,   0},  // LED_INDICATION_3 = -
        {500, 400, 200, 400, 200, 400,   0,   0,   0,   0}   // LED_INDICATION_4 = ---
    };

 // APP_TIMER_TICKS(MS)



static void singleshot_timer_led_indication_handler()
{
    ret_code_t err_code;
    uint16_t duration;

    m_led_indication_step++;
    duration = m_led_indication_prog[m_led_indication_type][m_led_indication_step];

    if(duration == 0)
    {
        bsp_board_led_off(BSP_LED_INDICATE);
        m_led_indication_type   = LED_NO_INDICATION;
        m_led_indication_step   = 0;
        m_led_current_status    = 0;
        
        return;
    }

    bsp_board_led_invert(BSP_LED_INDICATE);
    m_led_current_status = (m_led_current_status + 1) % 2;

    err_code = app_timer_start(m_singleshot_timer_led_indication, APP_TIMER_TICKS(duration), NULL);
    APP_ERROR_CHECK(err_code);  
}


void led_indication_init(void)
{
    ret_code_t err_code;

    bsp_board_led_off(BSP_LED_INDICATE);
    m_led_indication_type   = LED_NO_INDICATION;
    m_led_indication_step   = 0;
    m_led_current_status    = 0;

    err_code = app_timer_create(&m_singleshot_timer_led_indication,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                singleshot_timer_led_indication_handler);
    APP_ERROR_CHECK(err_code);
}

// Indication with type 
void led_indication_start(uint8_t indication_type)
{
    ret_code_t err_code;
    uint16_t duration;

    m_led_indication_type = indication_type;
    duration = m_led_indication_prog[m_led_indication_type][m_led_indication_step];

    ASSERT(duration != 0);

    bsp_board_led_off(BSP_LED_INDICATE);
    m_led_current_status = 0;

    err_code = app_timer_start(m_singleshot_timer_led_indication, APP_TIMER_TICKS(duration), NULL);
    APP_ERROR_CHECK(err_code);  
}

