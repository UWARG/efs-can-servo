/*
 * single_servo_lighting_controller.cpp
 * 
 * This file handles all of the higher level logic / interfacing to the single servo board.
 *
 * It will create 3 instances of the WS2812 LED's for each LED on the board
 *
 * It handles the final output buffer that is sent to DMA, as well as DMA transfer half complete callback
 *
 *  Created on: Feb 10, 2025
 *      Author: Ethan
 * 		
 */

#include <cstring>
#include <stdint.h>
 
#include "tim.h"
 
#include "single_servo_lighting_controller.hpp"
#include "ws2812.h"
#include "conversions.h"

SingleServoLightingController::SingleServoLightingController(uint8_t *dma_output_buffer, 
        uint8_t *bank_output_buffer,WS2812 *leds) {
    this->dma_buffer = dma_output_buffer;
	this->bank_buffer = bank_output_buffer;
	this->leds = leds;
	initialize_bank_buffer_on();
	initialize_dma_buffer();

    // Initialize all of the internal LEDs as off
	for (int i = 0; i < NUM_LEDS; ++i) {
		this->leds[i].initialize_led_off(
				bank_output_buffer + NUM_LEDS_PADDING * 24 + 24 * i);
	}
}

void SingleServoLightingController::start_lighting_control() {
	HAL_TIMEx_PWMN_Start_DMA(&htim1, TIM_CHANNEL_2,
		(uint32_t*) this->dma_buffer, DMA_OUTPUT_BUFFER_SIZE);
}

void SingleServoLightingController::set_led_on(uint8_t index) {
	this->leds[index].set_brightness(100);
}

void SingleServoLightingController::set_led_off(uint8_t index) {
	this->leds[index].set_brightness(0);
}

void SingleServoLightingController::blink_led_once(uint8_t index) {
	set_led_on(index);
	HAL_Delay(10);
	set_led_off(index);
}

void SingleServoLightingController::recolour_led(RGB_colour_t desired_colour, uint8_t index) {
	this->leds[index].set_led_colour(desired_colour,0);
}

/////////////////
// Private fn
/////////////////
void SingleServoLightingController::initialize_bank_buffer_off() {
	for (int i = 0; i < BANK_OUTPUT_BUFFER_SIZE; ++i) {
		// Check if the bit is a padding bit or value bit
		if (i < 24 * NUM_LEDS_PADDING
				|| i >= (BANK_OUTPUT_BUFFER_SIZE - 24 * NUM_LEDS_PADDING)) {
			this->bank_buffer[i] = 0;
		} else {
			this->bank_buffer[i] = PWM_LO;
		}
	}
}

void SingleServoLightingController::initialize_bank_buffer_on() {
	for (int i = 0; i < BANK_OUTPUT_BUFFER_SIZE; ++i) {
		// Check if the bit is a padding bit or value bit
		if (i < 24 * NUM_LEDS_PADDING
				|| i >= (BANK_OUTPUT_BUFFER_SIZE - 24 * NUM_LEDS_PADDING)) {
			this->bank_buffer[i] = 0;
		} else {
			if ((i % 8) > 4) {
				this->bank_buffer[i] = PWM_HI;
			} else {
				this->bank_buffer[i] = PWM_LO;
			}
		}
	}
}

void SingleServoLightingController::initialize_dma_buffer() {
	// memcpy first bank
	std::memcpy(this->dma_buffer, this->bank_buffer, BANK_OUTPUT_BUFFER_SIZE);
	// memcpy the second bank
	std::memcpy(this->dma_buffer + BANK_OUTPUT_BUFFER_SIZE, this->bank_buffer,
			BANK_OUTPUT_BUFFER_SIZE);
}
