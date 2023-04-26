/*
 * esp_fsm.c
 *
 *  Created on: Apr 25, 2023
 *      Author: Jussi Virtanen
 *
 *  Robot arm state machine proto
 */

#include "esp_ax12a.h"
#include "tof.h"
#include "main.h"
#include "stm32f4xx_hal.h"

typedef enum {
	ARM_INIT,
	ARM_MOVE_TO_IDLE,
	ARM_SEEK_CCW,
	ARM_SEEK_EDGE_CCW,
	ARM_SEEK_CW,
	ARM_SEEK_EDGE_CW,
	ARM_MOVE_TO_GRAB,
	ARM_GRAB_CLAW,
	ARM_MOVE_TO_DUMP
} armState;

typedef enum {
	SEEK_CW = 0,
	SEEK_CCW = 1
} seekDir;

unsigned int get_dist() {
	// TODO: preoper implementation
	return 0;
}

void arm_start_sm()
{
	unsigned int angle_ccw_edge, angle_cw_edge; // Angles where object edge were found
	unsigned int middle_rot;
	armState arm_state;
	unsigned int distance_threshold_mm; // max distance where arm will consider an object to exist
	unsigned int ccw_seek_angle_limit, cw_seek_angle_limit; // Seek area limits
	unsigned int dump_angle;

	// Init limits and start values, these are just examples. TODO: Set correct values
	distance_threshold_mm = 400;
	ccw_seek_angle_limit = 358; // ~45 degrees ccw
	cw_seek_angle_limit = 666; // ~45 degrees cw
	dump_angle = 1000; // Near max limit cw
	angle_ccw_edge = angle_cw_edge = 0; // Use 0 as the magic number where angle has not yet been found
	arm_state = ARM_INIT;

	//TOF Init
	uint32_t refSpadCount;
	uint8_t isApertureSpads;
	uint8_t VhvSettings;
	uint8_t PhaseCal;

	VL53L0X_RangingMeasurementData_t RangingData;
	static VL53L0X_Error myStatus = VL53L0X_ERROR_NONE;
	unsigned int sensor_distance_mm;

	myStatus=TOF1_VL53L0X_Init_Single(&refSpadCount,&isApertureSpads,&VhvSettings,&PhaseCal,0);
	if (myStatus!=VL53L0X_ERROR_NONE) {
		printf("TOF1 Initialization Error : %i\r\n", myStatus);
	}

	TOFX_VL53L0X_LongRangeSettings(TOF1);
	VL53L0X_StartMeasurement (TOF1);

	//TEST OF TOF
/*
while (1){
	myStatus = VL53L0X_PerformSingleRangingMeasurement(TOF1, &RangingData);
			if (myStatus!=VL53L0X_ERROR_NONE) {
				Error_Handler();
			}
			if(RangingData.RangeStatus == 0)
			{
				sensor_distance_mm= RangingData.RangeMilliMeter;
				printf("sensor value : %d \r\n", sensor_distance_mm);
			}
}
*/

	switch(arm_state) {

	case ARM_INIT:
		//TODO: Initialize huart interface
		//TODO: Initialize servo settings (max speed, torque etc)
		//ARM_STATE = ARM_MOVE_TO_IDLE;
		break;

	case ARM_MOVE_TO_IDLE:
		//TODO: Move to good idle position (blocked move)
		//TODO: Begin seek mode (from bluetooth input?)
		arm_state = ARM_SEEK_CCW;
		break;

	case ARM_SEEK_CCW:
		// Set goal to ccw edge if not yet set
		if (ax_get_goal_raw(4) != ccw_seek_angle_limit) {
			ax_set_goal_raw(4, ccw_seek_angle_limit);
		}
		// If at edge, invert seek direction
		if (ax_get_current_position(4) <= ccw_seek_angle_limit) {
			arm_state = ARM_SEEK_CW;
		}
		// cw edge found
		if (angle_cw_edge == 0 && get_dist() < distance_threshold_mm) {
			arm_state = ARM_SEEK_EDGE_CCW;
		}
		break;

	case ARM_SEEK_EDGE_CCW:
		angle_cw_edge = ax_get_current_position(4);
		// ccw edge found with cw edge found
		if (get_dist() > distance_threshold_mm) {
			angle_ccw_edge = ax_get_current_position(4);
			ax_stop(4);
			arm_state = ARM_MOVE_TO_GRAB;
		}

	case ARM_SEEK_CW:
		// Set goal to cw edge if not yet set
		if (ax_get_goal_raw(4) != cw_seek_angle_limit) {
			ax_set_goal_raw(4, cw_seek_angle_limit);
		}
		// If at edge, invert seek direction
		if (ax_get_current_position(4) >= cw_seek_angle_limit) {
			arm_state = ARM_SEEK_CCW;
		}
		// ccw edge found
		if (angle_ccw_edge == 0 && get_dist() < distance_threshold_mm) {
			arm_state = ARM_SEEK_EDGE_CW;
		}
		break;

	case ARM_SEEK_EDGE_CW:
		angle_ccw_edge = ax_get_current_position(4);
		// ccw edge found with cw edge found
		if (get_dist() > distance_threshold_mm) {
			angle_cw_edge = ax_get_current_position(4);
			ax_stop(4);
			arm_state = ARM_MOVE_TO_GRAB;
		}

	case ARM_MOVE_TO_GRAB:
		middle_rot = (angle_ccw_edge + angle_cw_edge) / 2;
		ax_move_blocked(4, middle_rot, 2);
		//TODO: find rotatoin angles for correct distance, move inblocked mode
		arm_state = ARM_GRAB_CLAW;
		break;

	case ARM_GRAB_CLAW:
		/* TODO: grab in blocked mode (wait for timeout)
		 * IF grab angle at limit, object wasnt grabbed (goto start)
		 * Otherwise assume we have object
		 */
		arm_state = ARM_MOVE_TO_DUMP;
		break;

	case ARM_MOVE_TO_DUMP:
		/* TODO: move to dump pos
		 * open claw
		 */
		arm_state = ARM_MOVE_TO_IDLE;
		break;

	default:
		break;
	}
}
