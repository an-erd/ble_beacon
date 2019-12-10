#ifndef __LED_INDICATION_H__
#define __LED_INDICATION_H__

#include <stdint.h>
#include <stdbool.h>
#include "time.h"
#include "bsp.h"


// BSP 
#define BSP_LED_INDICATE            BSP_BOARD_LED_0

// OWN LED INDICATION
#define LED_NO_INDICATION           255

#define LED_INDICATION_1            0   // no sensor, no advertising
#define LED_INDICATION_2            1   // sensor, no advertising but offline buffer
#define LED_INDICATION_3            2   // sensor, non-scan/non-conn advertising
#define LED_INDICATION_4            3   // sensor, scan/conn advertising
#define LED_INDICATION_5            4   // delete bonds, go to standard mode (defined by USE_CONN_ADV_INIT)
#define LED_INDICATION_6            5   // start config mode
#define LED_INDICATION_7            6   // end config mode
#define LED_NUM_INDICATIONS         7

#define LED_INDCATION_MAX_STEPS     12

// Initializes the led indication library. Run this before calling any other functions. 
void led_indication_init(void);

// Indication with type 
void led_indication_start(uint8_t indication_type);

#endif // __LED_INDICATION_H__
