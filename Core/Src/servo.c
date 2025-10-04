/*
 * servo.c
 *
 *  Created on: Sep 30, 2025
 *      Author: Shamanthi Rajagopal
 */

#include "servo.h"
#include <string.h>
#include <math.h>
#include "tim.h"

#ifndef SERVO_TIMEOUT_MS
#define SERVO_TIMEOUT_MS 500  // for servo fail safe func
#endif

// NEED TO CHANGE SERVO paramemters, check main...
static ServoCfg g_servos[NUM_SERVOS] = { 									// numbers 5,6,7,8
    { .htim=&htim1, .channel=TIM_CHANNEL_1, //which timer and PWM channel
      .us_min=1000,  		// servo pulse bound min
	  .us_max=2000, 		// servo pulse bound max
	  .us_neutral=1500, 	//neutral bound like avg?
	  .period_us=20000,		// PWM period in micro secs
      .actuator_id=5, 		//DroneCan ID					=> double check if this is correct
	  .last_unitless=0.0f, 	//status for tracking
	  .last_update_ms=0
    }
};

static uint32_t ticks_from_us(const ServoCfg *s, uint16_t us) { // helper function to convert micro seconds to timer ticks
    const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(s->htim);
    const double   tpu = ((double)(arr + 1U)) / (double)s->period_us;
    uint32_t ticks = (uint32_t)llround(us * tpu);

    if (ticks > arr) { // counter ticks from 0 to ARR)
    	ticks = arr;
    }

    return ticks; //return microsecond value
}

static inline uint16_t clamp_us(const ServoCfg *s, int value) { // helper function to keep servo in range

	if (value < s->us_min) {
		return s->us_min;
	}

    if (value > s->us_max) {
    	return s->us_max;
    }

    return (uint16_t)value;
}

void servo_init_and_start(void) { // function to initialize and start PWM

	for (int i = 0; i < NUM_SERVOS; i++) { // starts PWM on each configured channel
        HAL_TIM_PWM_Start(g_servos[i].htim, g_servos[i].channel);

        __HAL_TIM_SET_COMPARE(g_servos[i].htim, g_servos[i].channel,ticks_from_us(&g_servos[i], g_servos[i].us_neutral));	//sets to neutral

        //records values
        g_servos[i].last_unitless = 0.0f;
        g_servos[i].last_update_ms = HAL_GetTick();
    }
}

int servo_index_from_id(uint8_t actuator_id) { //Map droneCan id to servo index

	for (int i = 0; i < NUM_SERVOS; i++) {
        if (g_servos[i].actuator_id == actuator_id) {
        	return i;
        }
    }

    return -1; // servo figures out which timer/channel to use
}

void servo_write_unitless_idx(int index, float x) { // write unit less command, with x as input
    if (index < 0 || index >= NUM_SERVOS) return; // x is 0 so neutral which is default

    if (x >  1.0f) x =  1.0f;

    if (x < -1.0f) x = -1.0f;

    const ServoCfg *s = &g_servos[index]; // get index of servo
    const float half = (float)(s->us_max - s->us_min) * 0.5f; // caluclate half of range
    const float target_us = (float)s->us_neutral + x * half;
    const uint16_t us = clamp_us(s, (int)lroundf(target_us));

    __HAL_TIM_SET_COMPARE(s->htim, s->channel, ticks_from_us(s, us));
    ((ServoCfg*)s)->last_unitless = x;
    ((ServoCfg*)s)->last_update_ms = HAL_GetTick();
}

void servo_write_pwm_us_idx(int index, uint16_t us) { // function to write microseconds raw
    if (index < 0 || index >= NUM_SERVOS) return; // error check, error with index

    const ServoCfg *s = &g_servos[index]; // get index of servo

    us = clamp_us(s, us);
    __HAL_TIM_SET_COMPARE(s->htim, s->channel, ticks_from_us(s, us));
    // derive unitless

    float x = ((int)us - (int)s->us_neutral) / (float)(s->us_max - s->us_min) * 2.0f;

    if (x > 1.f) x = 1.f; if (x < -1.f) x = -1.f;

    ((ServoCfg*)s)->last_unitless = x;
    ((ServoCfg*)s)->last_update_ms = HAL_GetTick();
}

bool servo_status_get_by_index(uint8_t index, uint8_t *actuator_id, float *unitless) {

	//iterate through servo index and get the droneCan id and position to encode message
	if (index >= NUM_SERVOS) {
		return false; // when no more channels to run
	}

	if (actuator_id) {
		*actuator_id = g_servos[index].actuator_id;
	}

	if (unitless) {
		*unitless    = g_servos[index].last_unitless;
	}

	return true;
}

void servo_tick_fail_safe(uint32_t now_ms) { // sets servo to neutral position if no command in 0.5 s

    for (int i = 0; i < NUM_SERVOS; i++) {

    	if ((now_ms - g_servos[i].last_update_ms) > SERVO_TIMEOUT_MS) {

            const uint16_t us = g_servos[i].us_neutral;
            __HAL_TIM_SET_COMPARE(g_servos[i].htim, g_servos[i].channel, ticks_from_us(&g_servos[i], us));
            g_servos[i].last_unitless = 0.0f;
            g_servos[i].last_update_ms = 0;
        }
    }
}

 
 
