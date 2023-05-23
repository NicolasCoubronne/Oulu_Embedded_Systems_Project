/*
 * esp_onboard.c
 *
 *  Created on: May 17, 2023
 *      Author: tim
 *
 *      Interface for the onboard leds & current sensor
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

/* Set the mode of the operation led
 * Part of the blinking is implemented in timer interrupt
 *
 * led_mode: enum of the led mode
 */
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

/* Set the mode of the error led
 *
 * led_mode: enum of the led mode
 */
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

/* Get current measurement from the current sensor
 * This is not actually working correctly, the range of the sensor is too high for our implementation
 * and the precision of the ADC is not enough to get any meaningful data from the sensor (it's basically noise)
 * Therefore we didn't even do the voltage->current conversion of the input value
 *
 * returns: the raw voltage from the sensor which would correspond to some amperage
 */
double get_current_measure()
{
	double adc_val;
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	//adc_val = (HAL_ADC_GetValue(&hadc1) / 4096.0 - 1.65) * 10.0;
	adc_val = ((double)HAL_ADC_GetValue(&hadc1) / 4096.0 * 3.282);// - (3.282/2.0);
	HAL_ADC_Stop(&hadc1);
	return adc_val;
}
