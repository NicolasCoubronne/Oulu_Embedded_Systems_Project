/*
 * esp_fsm.h
 *
 *  Created on: May 17, 2023
 *      Author: tim
 */

#ifndef INC_ESP_FSM_H_
#define INC_ESP_FSM_H_

typedef enum {
	ARM_INIT,
	ARM_MOVE_TO_IDLE,
	ARM_SEEK_CCW,
	ARM_SEEK_EDGE_CCW,
	ARM_SEEK_CW,
	ARM_SEEK_EDGE_CW,
	ARM_MOVE_TO_GRAB,
	ARM_GRAB_CLAW,
	ARM_MOVE_TO_DUMP,
	ARM_ERROR_STATE
} armState;

typedef enum {
	LED_OFF,
	LED_ON,
	LED_BLINK
} ledMode;

extern ledMode OPERATIONAL_LED_MODE;
extern ledMode ERROR_LED_MODE;
extern double AVERAGE_CURRENT;
extern unsigned int CURRENT_MEASURE_COUNT;

double get_current_measure();

#endif /* INC_ESP_FSM_H_ */
