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

private:
    uint8_t *dma_buffer;
	uint8_t *bank_buffer;
	WS2812 *leds;
};