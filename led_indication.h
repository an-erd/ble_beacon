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
#define LED_INDICATION_1            0
#define LED_INDICATION_2            1
#define LED_INDICATION_3            2
#define LED_NUM_INDICATIONS         3
#define LED_INDCATION_MAX_STEPS     10

// Initializes the led indication library. Run this before calling any other functions. 
void led_indication_init(void);

// Indication with type 
void led_indication_start(uint8_t indication_type);

#endif // __LED_INDICATION_H__
