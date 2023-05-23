/*
 * esp_onboard.h
 *
 *  Created on: May 17, 2023
 *      Author: tim
 *
 *      Interface for the onboard leds & current sensor (header)
 *      More documentation in implementation file
 */

#ifndef INC_ESP_ONBOARD_H_
#define INC_ESP_ONBOARD_H_

#define OPERATION_LED_PORT LED_EXT_1_GPIO_Port
#define OPERATION_LED_PIN LED_EXT_1_Pin
#define ERROR_LED_PORT LED_EXT_2_GPIO_Port
#define ERROR_LED_PIN LED_EXT_2_Pin

typedef enum {
	LED_OFF,
	LED_ON,
	LED_BLINK
} ledMode;

extern ledMode OPERATIONAL_LED_MODE;
extern ledMode ERROR_LED_MODE;
extern uint32_t AVERAGE_CURRENT;
extern unsigned int CURRENT_MEASURE_COUNT;

void set_operation_led(ledMode led_mode);
void set_error_led(ledMode led_mode);
double get_current_measure();

#endif /* INC_ESP_ONBOARD_H_ */
