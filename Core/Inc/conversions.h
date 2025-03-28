/*
 * conversions.hpp
 *
 *  Created on: Nov 21, 2024
 *      Author: Anni
 */

#ifndef INC_CONVERSIONS_HPP_
#define INC_CONVERSIONS_HPP_

static constexpr uint8_t NUM_LEDS = 3;
static constexpr uint8_t NUM_LEDS_PADDING = 3;
static constexpr uint16_t DMA_OUTPUT_BUFFER_SIZE = (NUM_LEDS
    + NUM_LEDS_PADDING * 2) * 24 * 2;		// TODO: remove magic num
static constexpr uint16_t BANK_OUTPUT_BUFFER_SIZE = (NUM_LEDS
    + NUM_LEDS_PADDING * 2) * 24 * 2;	// TODO: remove magic num

typedef struct {
	uint8_t red;
	uint8_t green;
	uint8_t blue;
} RGB_colour_t;

typedef struct {
	uint16_t hue;		// TODO: limit this to 0-360 (degrees)
	uint8_t saturation;	// TODO: limit this to 0-100 (percentage)
	uint8_t lightness; 	// TODO: limit this to 0-100 (percentage)
} HSL_colour_t;


#endif /* INC_CONVERSIONS_HPP_ */
