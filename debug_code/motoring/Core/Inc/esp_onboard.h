/*
 * esp_onboard.h
 *
 *  Created on: May 17, 2023
 *      Author: tim
 */

#ifndef INC_ESP_ONBOARD_H_
#define INC_ESP_ONBOARD_H_

typedef enum {
	LED_OFF,
	LED_ON,
	LED_BLINK
} ledMode;

extern ledMode OPERATIONAL_LED_MODE;
extern ledMode ERROR_LED_MODE;
extern double AVERAGE_CURRENT;
extern unsigned int CURRENT_MEASURE_COUNT;

void set_operation_led(ledMode led_mode);
void set_error_led(ledMode led_mode);
double get_current_measure();

#endif /* INC_ESP_ONBOARD_H_ */
