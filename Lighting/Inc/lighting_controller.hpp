/*
 * lighting_controller.hpp
 *
 *  Created on: Nov 21, 2024
 *      Author: Anni and Fola
 */

#ifndef INC_LIGHTING_CONTROLLER_HPP_
#define INC_LIGHTING_CONTROLLER_HPP_

#include <cstdint>

#include "conversions.hpp"
#include "ws2812.hpp"

/**
 * @enum ControlDomain
 * Represents the different command mode that an LED might be responsive to.
 *
 * CD_MAIN - default mode for LED's
 * CD_BEACON - beacon lights (red pulse 1x per second), indicates LV ON
 * CD_STROBE - strobe lights (Double white flash 1x per second), indicates HV ON
 *
 * Add other modes as necessary
 */
enum ControlDomain {
	CD_MAIN = 0,
	CD_BEACON = 1,
	CD_STROBE = 2,
	CD_LENGTH = 3 // increment this as necessary
};

// TODO: Make these public
void run_lighting_board();

class LightingController {
public:
	/**
	 * TODO: Initialize lighting controller with a reference to the led bank output
	 */
	LightingController(uint8_t *dma_output_buffer, uint8_t *bank_output_buffer,
			WS2812 *leds);

	/**
	 * Start sending lighting control data to neopixels.
	 * Send values corresponding to neopixel brightness to capture compare register (CCR).
	 */

	void start_lighting_control();

	/**
	 * Sets all LEDs under controller's domain to desired colors.
	 *
	 * @param desired_colour : RGB_colour_t that you want set
	 */
	void recolour_all(RGB_colour_t desired_colour);

	/**
	 * Sets all LEDs under controller's domain to desired colors.
	 *
	 * @param desired_colour : RGB_colour_t that you want set
	 *
	 * @param brightness : sets the same brightness for each LEDs
	 */
	void recolour_all(RGB_colour_t desired_colour, uint8_t brightness);

	/**
	 * Sets all LEDs under controller's domain to desired colors.
	 *
	 * @param index : The LED on the lighting board that you want to send.
	 *
	 * @param desired_colour: RGB_colour_t that you want set.
	 */
	void recolour_by_index(uint8_t index, RGB_colour_t desired_colour);

	/**
	 * Sets all LEDs under controller's domain to desired colors.
	 *
	 * @param index : The LED on the lighting board that you want to send.
	 *
	 * @param desired_colour: RGB_colour_t that you want set.
	 *
	 * @param brightness: the brightness of a specific LED on the LED board.
	 */
	void recolour_by_index(uint8_t index, RGB_colour_t desired_colour,
			uint8_t brightness);

	/**
	 * Adds an LED to a Control Domain
	 *
	 * @param index : LED index on the board
	 * @param domain : Control Domain to add the LED to.
	 */
	void add_led_to_cd(uint8_t index, ControlDomain domain);

	/**
	 * Removes an LED from a Control Domain
	 *
	 * @param index : LED index on the board
	 * @param domain : Control Domain to remove the LED from
	 */
	void remove_led_from_cd(uint8_t index, ControlDomain domain);

	/**
	 * Enable (turn on) a Control Domain
	 *
	 * This will assign all LED's that are not superceded by this domain the brightness
	 * and colour of the specified domain. It is necesasry to call `set_domain_colour` in
	 * order to update the colour of the domain ATM.
	 *
	 * @param domain : domain to be enabled.
	 */
	void enable_control_domain(ControlDomain domain);

	/**
	 * Disables (turn off) a Control Domain
	 *
	 * Any LED that is part of another control domain will revert to that control domains
	 * colour & brightness setting.
	 *
	 * @param domain : domain to be disabled
	 */
	void disable_control_domain(ControlDomain domain);

	/**
	 * Sets the colour & brightness of a Control Domain
	 *
	 * TODO: future functions for only brightness/colour.
	 *
	 * @param domain : domain to be modified
	 * @param colour : desired colour
	 * @param brightness : desired brightness
	 */
	void set_domain_colour(ControlDomain domain, RGB_colour_t colour,
			uint8_t brightness);

	// TODO: add function to "recolour domain" (enables & sets colour)
	// Use this instead of set + enable?

private:
	static constexpr uint8_t NUM_LEDS = 6;
	uint8_t *dma_buffer;
	uint8_t *bank_buffer;
	WS2812 *leds;
	bool domain_state[CD_LENGTH];				// true when domain is active
	uint8_t domain_leds[CD_LENGTH];	// Bitmask of LED's which are active in each domain
	RGB_colour_t domain_colours[CD_LENGTH];		// Domain colour
	uint8_t domain_brightness[CD_LENGTH];		// Domain brightness

	void initialize_bank_buffer_off();
	void initialize_bank_buffer_on();
	void initialize_dma_buffer();
};
#endif /* INC_LIGHTING_CONTROLLER_HPP_ */
