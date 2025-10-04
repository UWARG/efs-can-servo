/*
 * servo.h
 *
 *  Created on: Sep 30, 2025
 *      Author: Shamanthi Rajagopal
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include <stdint.h>
#include <stdbool.h>
#include <tim.h>

#endif /* INC_SERVO_H_ */

#define NUM_SERVOS 4 // i think 4? Not sure

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;			// TIM_CHANNEL_!
    uint16_t us_min;			// 1000
    uint16_t us_max;			// 2000
    uint16_t us_neutral;		//1500
    uint16_t period_us;			//20000 (50 Hz)
    uint8_t  actuator_id;		// for DroneCAN
    float    last_unitless;		// -1 to 1
    uint32_t last_update_ms;	// HAL_GetTick() time
} ServoCfg;


void servo_init_and_start(void); 					//fills default values and starts pwm at neutral
int servo_index_from_id(uint8_t actuator_id); 		//Map actuator ID
void servo_write_unitless_idx(int idx, float x);	// Writes -1...+1
void servo_write_pwn_us_idx(int idx, uint16_us);	//write with microseconds

static inline void servo_write_unitless_id(uint8_t id, float x) {
    int i = servo_index_from_id(id);
    if (i >= 0) {servo_write_unitless_idx(i, x);}
}

static inline void servo_write_pwm_us_id(uint8_t id, uint16_t us) {
    int i = servo_index_from_id(id);
    if (i >= 0) {servo_write_pwm_us_idx(i, us);}
}

bool servo_status_get_by_index(uint8_t index, /*out*/uint8_t *actuator_id, /*out*/float *unitless); // status helper function used by send_ServoStatus
