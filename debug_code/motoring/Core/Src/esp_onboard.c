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
uint32_t AVERAGE_CURRENT = 0;
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
	double adc_val;
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	//adc_val = (HAL_ADC_GetValue(&hadc1) / 4096.0 - 1.65) * 10.0;
	adc_val = ((double)HAL_ADC_GetValue(&hadc1) / 4096.0 * 3.282);// - (3.282/2.0);
	return adc_val;

	return HAL_ADC_Stop(&hadc1);
}
