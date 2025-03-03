/*
 * single_servo_lighting_controller.hpp
 *
 *  Created on: Feb 8, 2025
 *      Author: Ethan
 */

#include <cstdint>

#include "conversions.hpp"
#include "ws2812.hpp"

class SingleServoLightingController {
public:
	/**
	 * Constructor for lighting controller
	 * 
	 * TODO: Initialize lighting controller with a reference to the led bank output
	 */
	SingleServoLightingController(uint8_t *dma_output_buffer, uint8_t *bank_output_buffer,
		WS2812 *leds);

	/**
	 * Start sending lighting control data to neopixels.
	 * Send values corresponding to neopixel brightness to capture compare register (CCR).
	 */
	void start_lighting_control();

	/**
	 * Sets specified LED on.
	 *
	 * @param desired_colour : RGB_colour_t that you want set.
	 * 
	 * @param index : The LED on the lighting board that you want to set.
	 * 
	 */
	void set_led_on(RGB_colour_t desired_colour, uint8_t index);

	/**
	 * Recolours specified LED to the desired color.
	 * 
	 * @param desired_colour RGB_colour_t that you want set.
	 *
	 * @param index The LED on the lighting board that you want to send.
	 */
	void recolour_led(RGB_colour_t desired_colour, uint8_t index);

	/**
	 * Sets specified LED off.
	 * 
	 * @param index : The LED on the lighting board that you want to set.
	 * 
	 */
	void set_led_off(uint8_t index);

	/**
	 * Blink an LED once for 1ms.
	 * 
	 * @param index : The LED on the lighting board that you want to set.
	 * 
	 */
	void blink_led_once(uint8_t index);

private:
	static constexpr uint8_t NUM_LEDS = 3;
    uint8_t *dma_buffer;
	uint8_t *bank_buffer;
	WS2812 *leds;

	void initialize_bank_buffer_off();
	void initialize_bank_buffer_on();
	void initialize_dma_buffer();
};