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
	//TODO: Make use of PCB leds
	//TODO: right now uses the extra nucleo green led
	switch (led_mode) {
	case LED_OFF:
		HAL_GPIO_WritePin(LD2_GREEN_GPIO_Port, LD2_GREEN_Pin, GPIO_PIN_RESET);
		break;

	case LED_ON:
		HAL_GPIO_WritePin(LD2_GREEN_GPIO_Port, LD2_GREEN_Pin, GPIO_PIN_SET);
		break;

	default:
		break;
	}
}

void set_error_led(ledMode led_mode)
{
	ERROR_LED_MODE = led_mode;
	//TODO: Make use of PCB leds
}

double get_current_measure()
{
	//TODO: proper implementation when we have PCB
	static double dummy = 0.0;
	dummy += 0.1;
	return dummy;
}
