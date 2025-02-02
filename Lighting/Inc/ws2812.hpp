/*
 * ws2812.hpp
 *
 *  Created on: Nov 21, 2024
 *      Author: Anni
 */

#ifndef INC_WS2812_HPP_
#define INC_WS2812_HPP_

#include <stdint.h>

#include "conversions.hpp"

static constexpr uint8_t PWM_LO = 19;
static constexpr uint8_t PWM_HI = 38;

/**
 * @class WS2812
 * @brief A class representing a single WS2812 LED
 *
 * This class manages the state of a single WS2812 LED by directly manipulating
 * the memory buffer associated with it's output data. When you set an LED,
 * it will store state (RGB), and immediately set LED colour.
 */
class WS2812 {
public:

	/**
	 * Default Constructor
	 *
	 * You must then call `initialize_<>()` with a pointer to the output buffer
	 */
	WS2812();
	/**
	 * Constructs a WS2812 object
	 *
	 * @param output_buffer : Pointer to the start of the output buffer
	 */
	WS2812(uint8_t *output_buffer);

	/**
	 * Initializes the LED to be full brightness on
	 *
	 * TODO: Add overloading to allow "on" to have a specified brightness
	 */
	void initialize_led_on();
	void initialize_led_on(uint8_t *led_output_buffer);

	/**
	 * Initializes the LED to be full off
	 *
	 * Assumes that output buffer has already been set
	 */
	void initialize_led_off();

	/**
	 * Initializes the LED to be full off
	 *
	 * @param led_output_buffer : Pointer to the start of the output buffer for this LED
	 */
	void initialize_led_off(uint8_t *led_output_buffer);

	/**
	 * Copies the data in the output buffer into the specified memory location
	 */
	void get_led_output_buffer(uint8_t *led_output_buffer);

	/**
	 * Sets the LED colour immediately.
	 *
	 * Since the LED itself works off of 8 bit R/G/B values, this is the function
	 * that all HSL/HEX functions will call!
	 *
	 * @param rgb_colour_value : RGB_colour_t that you want set
	 */
	void set_led_colour(RGB_colour_t rgb_colour_value);	// TODO: proper error codes/return types

	/**
	 * Sets the LED colour immediately based on the brightness
	 *
	 * @param rgb_colour_value : RGB_colour_t that you want set
	 *
	 * @param colour_brightness : RGB_colour_t brightness
	 */
	void set_led_colour(RGB_colour_t rgb_colour_value,
			uint8_t colour_brightness);

	// TODO: set various methods for setting LED colour using custom HEX_COLOUR or HSL
	// Maybe take advantage of Unions, otherwise conversion.cpp is there for us!

	/**
	 * Gets the current colour of the LED (in case it needs to be reset?)
	 * 
	 * TODO: return current LED colour
	 */
	RGB_colour_t get_led_colour();

	/**
	 * Sets the LED colour brightness
	 *
	 * @param colour_brightness : RGB_colour_t brightness
	 */

	void set_brightness(uint8_t colour_brightness);

private:
	static constexpr uint8_t BITS_PER_COLOUR = 8;
	static constexpr uint8_t BITS_PER_LED = BITS_PER_COLOUR * 3; // R, G, B

	uint8_t *buffer; // Each LED keeps track of the start of it's buffer within a bank
	uint8_t *g_offset;	// Green is first
	uint8_t *r_offset;	// Then Red
	uint8_t *b_offset;	// Lastly Blue

	RGB_colour_t colour;	//LED colour
	uint8_t brightness;	//Sets the colour brightness as a percentage (0 is 0% and 100 is 100%)

	/**
	 * Writes colour data from this->colour to the buffer
	 *
	 * @param colour : RGB_colour_t "colour" to write to the buffer
	 *
	 */
	void push_colour_to_output_buffer(RGB_colour_t colour);

};

#endif /* INC_WS2812_HPP_ */