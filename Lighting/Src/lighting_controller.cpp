/*
 * lighting_controller.cpp
 *
 * GOAL: Repurpose to control the Neopixel LEDs on the CAN Single Servo Board
 * 
 * This file handles all of the higher level logic / interfacing to the can board.
 *
 * It will create 3 instances of the WS2812 LED's for each LED on the single servo board
 *
 * It handles the final output buffer that is sent to DMA, as well as DMA transfer half complete callback
 *
 *  Created on: Feb 2, 2025
 *      Author: Anni and Fola
 * 		Code Ported From: efs-can-lighting/Lighting/Src/lighting_controller.cpp
 */

#include <cstring>
#include <stdint.h>

#include "tim.h"

#include "lighting_controller.hpp"
#include "ws2812.hpp"
#include "conversions.hpp"

//#define STARTUP_SEQUENCE_1 // very basic selftest

extern TIM_HandleTypeDef htim7;

static constexpr uint8_t NUM_LEDS = 3;
static constexpr uint8_t NUM_LEDS_PADDING = 6;
static constexpr uint16_t DMA_OUTPUT_BUFFER_SIZE = (NUM_LEDS
		+ NUM_LEDS_PADDING * 2) * 24 * 2;		// TODO: remove magic num
static constexpr uint16_t BANK_OUTPUT_BUFFER_SIZE = (NUM_LEDS
		+ NUM_LEDS_PADDING * 2) * 24 * 2;	// TODO: remove magic num

uint8_t dma_output_buffer[DMA_OUTPUT_BUFFER_SIZE];
uint8_t bank_output_buffer[BANK_OUTPUT_BUFFER_SIZE];

WS2812 leds[NUM_LEDS]; // TODO: make this work

// Initial setup call
LightingController rev3(dma_output_buffer, bank_output_buffer, leds); // TODO: Once we have custom functions registered as callbacks.....

extern TIM_HandleTypeDef htim;

void run_lighting_board() {

	// Call to start lighting control
	rev3.start_lighting_control();

	// RGB Colour & Brightness control demo setup

	RGB_colour_t my_colour;
	my_colour.red = 65;
	my_colour.green = 0;
	my_colour.blue = 127;

	int red_direction = 1;   // 1 for increasing, -1 for decreasing
	int green_direction = 1; // 1 for increasing, -1 for decreasing
	int blue_direction = 1;  // 1 for increasing, -1 for decreasing

	uint8_t brightness_slow = 0;
	int brightness_direction = 1;
	uint8_t brightness = 0;

	// DOMAIN SETUP
	// TODO: move Control Domain building to special functions

	RGB_colour_t WHITE = { 255, 255, 255 };
	RGB_colour_t RED = { 255, 0, 0 };
	uint8_t BRIGHTNESS_MAX = 100;

	// build beacon domain
	rev3.set_domain_colour(CD_BEACON, RED, BRIGHTNESS_MAX);
	rev3.add_led_to_cd(1, CD_BEACON);
	rev3.add_led_to_cd(4, CD_BEACON);

	// build strobe domain
	rev3.set_domain_colour(CD_STROBE, WHITE, BRIGHTNESS_MAX);
	rev3.add_led_to_cd(0, CD_STROBE);
	rev3.add_led_to_cd(2, CD_STROBE);
	rev3.add_led_to_cd(3, CD_STROBE);
	rev3.add_led_to_cd(5, CD_STROBE);

	while (true) {
		// Demo program to update LED colors & show control domain functionality

		brightness_slow = brightness_slow + 1;
		if (brightness_slow >= 20) {
			brightness += brightness_direction;
			brightness_slow = 0;
		}
		if (brightness >= 50) {
			brightness = 50;
			brightness_direction = -1;
		} else if (brightness <= 0) {
			brightness = 0;
			brightness_direction = 1;
		}

		// Update red value
		my_colour.red += red_direction;
		if (my_colour.red >= 255) {
			my_colour.red = 255;
			red_direction = -1; // Start decreasing
		} else if (my_colour.red <= 0) {
			my_colour.red = 0;
			red_direction = 1; // Start increasing
		}

		// Update green value
		my_colour.green += green_direction;
		if (my_colour.green >= 255) {
			my_colour.green = 255;
			green_direction = -1; // Start decreasing
		} else if (my_colour.green <= 0) {
			my_colour.green = 0;
			green_direction = 1; // Start increasing
		}

		// Update blue value
		my_colour.blue += blue_direction;
		if (my_colour.blue >= 255) {
			my_colour.blue = 255;
			blue_direction = -1; // Start decreasing
		} else if (my_colour.blue <= 0) {
			my_colour.blue = 0;
			blue_direction = 1; // Start increasing
		}

		// Add a small delay for smooth transitions
		HAL_Delay(10); // Adjust this value for faster/slower fading

		rev3.set_domain_colour(CD_MAIN, my_colour, brightness);
		rev3.enable_control_domain(CD_MAIN);
	}
}

LightingController::LightingController(uint8_t *dma_output_buffer,
		uint8_t *bank_output_buffer, WS2812 *leds) {
	this->dma_buffer = dma_output_buffer;
	this->bank_buffer = bank_output_buffer;
	this->leds = leds;
	initialize_bank_buffer_on();
	initialize_dma_buffer();

	// Initialize all of the internal LED's as well
	for (int i = 0; i < NUM_LEDS; ++i) {
		this->leds[i].initialize_led_on(
				bank_output_buffer + NUM_LEDS_PADDING * 24 + 24 * i);
		this->domain_leds[CD_MAIN] |= 1 << i;	// this led index is now enabled
	}
}

void LightingController::start_lighting_control() {
	HAL_TIMEx_PWMN_Start_DMA(&htim1, TIM_CHANNEL_2,
			(uint32_t*) dma_output_buffer, DMA_OUTPUT_BUFFER_SIZE);

#ifdef STARTUP_SEQUENCE_1
	HAL_Delay(1000);
	// Average startup selftest moment
	for (int i = 0; i < NUM_LEDS; ++i) {
		this->leds[i].initialize_led_off();
		HAL_Delay(1000);
	}

	for (int i = 0; i < NUM_LEDS; ++i) {
		RGB_colour_t my_colour;
		my_colour.red = (i % 3 == 0) ? 80 : 0;
		my_colour.green = (i % 3 == 1) ? 80 : 0;
		my_colour.blue = (i % 3 == 2) ? 80 : 0;
		this->leds[i].set_led_colour(my_colour);
		HAL_Delay(1000);
	}
#endif
}

void LightingController::recolour_all(RGB_colour_t desired_colour) {
	for (int i = 0; i < NUM_LEDS; ++i) {
		this->leds[i].set_led_colour(desired_colour);
	}
}

void LightingController::recolour_all(RGB_colour_t desired_colour,
		uint8_t brightness) {
	for (int i = 0; i < NUM_LEDS; ++i) {
		this->leds[i].set_led_colour(desired_colour, brightness);
	}
}

void LightingController::recolour_by_index(uint8_t index,
		RGB_colour_t desired_colour) {
	this->leds[index].set_led_colour(desired_colour);
}

void LightingController::recolour_by_index(uint8_t index,
		RGB_colour_t desired_colour, uint8_t brightness) {
	this->leds[index].set_led_colour(desired_colour, brightness);
}

////////////////////
// CONTROL DOMAIN FN
////////////////////

void LightingController::add_led_to_cd(uint8_t index, ControlDomain domain) {
	// set the index in the domain
	this->domain_leds[domain] |= 1 << index;
}

void LightingController::remove_led_from_cd(uint8_t index,
		ControlDomain domain) {
	// clear the index from the domain
	this->domain_leds[domain] &= ~(1 << index);
}

void LightingController::set_domain_colour(ControlDomain domain,
		RGB_colour_t colour, uint8_t brightness) {
	this->domain_colours[domain] = colour;
	this->domain_brightness[domain] = brightness;
}

void LightingController::enable_control_domain(ControlDomain domain) {
	this->domain_state[domain] = true;	// TODO: have active/disabled booleans
	for (int i = 0; i < CD_LENGTH; ++i) {
		if (this->domain_state[i]) { // IF THIS DOMAIN IS ACTIVE
			for (int j = 0; j < NUM_LEDS; ++j) {
				if (this->domain_leds[i] & (1 << j)) {
					this->leds[j].set_led_colour(domain_colours[i],
							domain_brightness[i]);
				}
			}
		}
	}
}

void LightingController::disable_control_domain(ControlDomain domain) {
	this->domain_state[domain] = false;
	for (int i = 0; i < CD_LENGTH; ++i) {
		if (this->domain_state[domain]) { // IF THIS DOMAIN IS ACTIVE
			for (int j = 0; j < NUM_LEDS; ++j) {
				if (this->domain_leds[i] & (1 << j)) {
					this->leds[i].set_led_colour(domain_colours[i],
							domain_brightness[i]);
				}
			}
		}
	}
}

/////////////////
// Private fn
/////////////////
void LightingController::initialize_bank_buffer_off() {
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

void LightingController::initialize_bank_buffer_on() {
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

void LightingController::initialize_dma_buffer() {
	// memcpy first bank
	std::memcpy(this->dma_buffer, bank_output_buffer, BANK_OUTPUT_BUFFER_SIZE);
	// memcpy the second bank
	std::memcpy(this->dma_buffer + BANK_OUTPUT_BUFFER_SIZE, bank_output_buffer,
			BANK_OUTPUT_BUFFER_SIZE);
}

//////////////////////////////////////////
// CALLBACKS
//////////////////////////////////////////
// TODO: Register custom callbacks for the timers
// https://community.st.com/t5/stm32-mcus/how-to-use-register-callbacks-in-stm32/ta-p/580499
// We should be able to register _class_ functions as callbacks?
void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim) {
	// | BANK 1 | BANK 2 |
	//          ^ Current location
	// So update BANK 1
	std::memcpy(dma_output_buffer, bank_output_buffer, BANK_OUTPUT_BUFFER_SIZE);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	// | BANK 1 | BANK 2 |
	//                   ^ Current location
	// So update BANK 2
	std::memcpy(dma_output_buffer + BANK_OUTPUT_BUFFER_SIZE, bank_output_buffer,
			BANK_OUTPUT_BUFFER_SIZE);
}

void TIM6_OneSecondCallback(TIM_HandleTypeDef *htim) {
	HAL_TIM_Base_Start_IT(&htim7);
}

void TIM7_100msCallback(TIM_HandleTypeDef *htim7) {
	static uint8_t stage = 0;
	if (stage == 0) { 			// STROBE ON
		rev3.enable_control_domain(CD_STROBE);
		stage = 1;
	} else if (stage == 1) { 	// STROBE OFF
		rev3.disable_control_domain(CD_STROBE);
		stage = 2;
	} else if (stage == 2) {	// STROBE ON
		rev3.enable_control_domain(CD_STROBE);
		stage = 3;
	} else if (stage == 3) {	// STROBE OFF
		rev3.disable_control_domain(CD_STROBE);
		stage = 4;
	} else if (stage == 4) {	// OFF
		stage = 5;
	} else if (stage == 5) {	// OFF
		stage = 6;
	} else if (stage == 6) {	// BCN ON
		rev3.enable_control_domain(CD_BEACON);
		stage = 7;
	} else if (stage == 7) {	// BCN OFF
		rev3.disable_control_domain(CD_BEACON);
		stage = 0;
		HAL_TIM_Base_Stop_IT(htim7);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim->Instance == TIM6) {
		TIM6_OneSecondCallback(htim);
	} else if (htim->Instance == TIM7) {
		TIM7_100msCallback(htim);
	}
}
