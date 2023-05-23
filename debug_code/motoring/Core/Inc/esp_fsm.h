/*
 * esp_fsm.h
 *
 *  Created on: May 17, 2023
 *      Author: tim
 *
 *      Robot arm state machine (header)
 *      More documentation in implementation file
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

void arm_start_sm();

#endif /* INC_ESP_FSM_H_ */
