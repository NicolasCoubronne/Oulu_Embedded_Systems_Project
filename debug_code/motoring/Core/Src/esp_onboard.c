/*
 * esp_onboard.c
 *
 *  Created on: May 17, 2023
 *      Author: tim
 */

#include "main.h"
#include "stm32f4xx_hal.h"

#include "esp_onboard.h"

//Led status (for interrupts)
ledMode OPERATIONAL_LED_MODE = LED_OFF;
ledMode ERROR_LED_MODE = LED_OFF;
// Current measure
double AVERAGE_CURRENT = 0.0;
unsigned int CURRENT_MEASURE_COUNT = 0;

void set_operation_led(ledMode led_mode)
{
	OPERATIONAL_LED_MODE = led_mode;
	switch (led_mode) {
	case LED_OFF:
		HAL_GPIO_WritePin(OPERATION_LED_PORT, OPERATION_LED_PIN, GPIO_PIN_RESET);
		break;

	case LED_ON:
		HAL_GPIO_WritePin(OPERATION_LED_PORT, OPERATION_LED_PIN, GPIO_PIN_SET);
		break;

	default:
		break;
	}
}

void set_error_led(ledMode led_mode)
{
	ERROR_LED_MODE = led_mode;
	switch (led_mode) {
	case LED_OFF:
		HAL_GPIO_WritePin(ERROR_LED_PORT, ERROR_LED_PIN, GPIO_PIN_RESET);
		break;

	case LED_ON:
		HAL_GPIO_WritePin(ERROR_LED_PORT, ERROR_LED_PIN, GPIO_PIN_SET);
		break;

	default:
		break;
	}
}

double get_current_measure()
{
	//TODO: proper implementation when we have PCB
	static double dummy = 0.0;
	dummy += 0.1;
	return dummy;
}
