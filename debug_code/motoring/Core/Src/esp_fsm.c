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
#include <math.h>
#include <stdlib.h>
#include "esp_arm.h"

extern UART_HandleTypeDef huart1;

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
	SEEK_CW = 0,
	SEEK_CCW = 1
} seekDir;

//Bluetooth variables
extern uint8_t buf_RX[Size];
extern uint8_t buf_TX[Size];
uint8_t buf_len;

//TOF Variables
uint32_t refSpadCount;
uint8_t isApertureSpads;
uint8_t VhvSettings;
uint8_t PhaseCal;
VL53L0X_RangingMeasurementData_t RangingData;
static VL53L0X_Error myStatus = VL53L0X_ERROR_NONE;
unsigned int sensor_distance_mm;


unsigned int get_dist() {
	unsigned int sensor_distance_mm= 0;

	for (int i=0; i<5; i++){
		myStatus = VL53L0X_PerformSingleRangingMeasurement(TOF1, &RangingData);
		if (myStatus!=VL53L0X_ERROR_NONE) {
			Error_Handler();
		}
		//if (RangingData.RangeMilliMeter < sensor_distance_mm) sensor_distance_mm = RangingData.RangeMilliMeter;
		sensor_distance_mm += RangingData.RangeMilliMeter;
		HAL_Delay(25);
	}

	sensor_distance_mm = sensor_distance_mm/5;

	/*
	myStatus = VL53L0X_PerformSingleRangingMeasurement(TOF1, &RangingData);
	if (myStatus!=VL53L0X_ERROR_NONE) {
		Error_Handler();
	}
	*/

	//while(RangingData.RangeStatus != 0) {}

	//sensor_distance_mm = RangingData.RangeMilliMeter;
	buf_len = sprintf(buf_RX, "TOF value : %d\r\n", sensor_distance_mm);
	HAL_UART_Transmit(&huart1, (uint8_t*)buf_RX,buf_len, HAL_MAX_DELAY);


	if(sensor_distance_mm < 100) return 1000;

	return sensor_distance_mm;
}

void arm_start_sm()
{

	//Motors variables
	unsigned int angle_ccw_edge, angle_cw_edge; // Angles where object edge were found
	unsigned int middle_rot;
	armState arm_state;
	unsigned int distance_threshold_mm; // max distance where arm will consider an object to exist
	unsigned int object_offset_distance;
	unsigned int ccw_seek_angle_limit, cw_seek_angle_limit; // Seek area limits
	unsigned int dump_angle;

	unsigned int angle_base_joint;
	unsigned int angle_middle_joint;
	unsigned int angle_claw_joint;
	unsigned int claw_angle_limit_open;
	unsigned int claw_angle_limit_closed;

	// Init limits and start values, these are just examples. TODO: Set correct values
	distance_threshold_mm = 300;
	object_offset_distance = -80;
	ccw_seek_angle_limit = 205; // ~90 degrees ccw
	cw_seek_angle_limit = 819; // ~90 degrees cw
	claw_angle_limit_closed = 600; // fully closed
	claw_angle_limit_open = 320; // fully open
	dump_angle = 900; // Near max limit cw
	angle_ccw_edge = angle_cw_edge = 0; // Use 0 as the magic number where angle has not yet been found
	arm_state = ARM_MOVE_TO_IDLE;

	//TOF Init
	myStatus=TOF1_VL53L0X_Init_Single(&refSpadCount,&isApertureSpads,&VhvSettings,&PhaseCal,0);
	if (myStatus!=VL53L0X_ERROR_NONE) {
		printf("TOF1 Initialization Error : %i\r\n", myStatus);
	}


	TOFX_VL53L0X_LongRangeSettings(TOF1);
	VL53L0X_StartMeasurement (TOF1);

	//MOTOR INIT

	uint8_t ids[] = {9, 3, 2, 8, 4};
	uint16_t init_angles[] = {600, 390, 950, claw_angle_limit_open, 512};
	uint16_t init_speed[] = {70, 60, 70, 50, 20};
	float to = 1;

	for (int i=0; i < sizeof(ids)/sizeof(ids[0]); i++) {
		ax_set_move_speed(ids[i], init_speed[i]);
		HAL_Delay(1000);
	}
	for (int i=0; i < sizeof(ids)/sizeof(ids[0]); i++) {
		ax_move_blocked(ids[i], init_angles[i], to);
		HAL_Delay(1000);
	}

	ax_set_max_torque(8, 200);


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
			//printf("sensor value : %d \r\n", sensor_distance_mm);
			buf_len = sprintf(buf_RX, "TOF value : %d\r\n", sensor_distance_mm);
			HAL_UART_Transmit(&huart1, (uint8_t*)buf_RX,buf_len, HAL_MAX_DELAY);
		}
	}
	*/

	while(1) {
		switch(arm_state) {

		case ARM_MOVE_TO_IDLE:
			HAL_UART_Transmit(&huart1, (uint8_t*)"Entering state : ARM_MOVE_TO_IDLE\r\n", sizeof("Entering state : ARM_MOVE_TO_IDLE\r\n"), HAL_MAX_DELAY);
			// Reset object edges
			angle_ccw_edge = angle_cw_edge = 0;
			for (int i=0; i < sizeof(ids)/sizeof(ids[0]); i++) {
				ax_stop(ids[i]);
			}
			//Move to good idle position (blocked move)
			for (int i=0; i < sizeof(ids)/sizeof(ids[0]); i++) {
				ax_move_blocked(ids[i], init_angles[i], to);
				HAL_Delay(200);
			}
			//TODO: Begin seek mode (from bluetooth input?)
			arm_state = ARM_SEEK_CCW;
			break;

		case ARM_SEEK_CCW:
			HAL_UART_Transmit(&huart1, (uint8_t*)"Entering state : ARM_SEEK_CCW\r\n", sizeof("Entering state : ARM_SEEK_CCW\r\n"), HAL_MAX_DELAY);
			angle_ccw_edge = angle_cw_edge = 0;
			// Set goal to ccw edge if not yet set
			ax_set_goal_raw(4, ccw_seek_angle_limit);

			while(1){
				// If at edge, invert seek direction
				if (ax_diff_from_goal(4) <= 10) {
					HAL_Delay(1000);
					arm_state = ARM_SEEK_CW;
					break;
				}
				// cw edge found
				if (get_dist() < distance_threshold_mm) {
					arm_state = ARM_SEEK_EDGE_CCW;
					break;
				}

				HAL_Delay(50);
			}

			break;

		case ARM_SEEK_EDGE_CCW:
			HAL_UART_Transmit(&huart1, (uint8_t*)"Entering state : ARM_SEEK_EDGE_CCW\r\n", sizeof("Entering state : ARM_SEEK_EDGE_CCW\r\n"), HAL_MAX_DELAY);
			angle_cw_edge = ax_get_current_position(4);
			while(1){
				// If at edge, go to beginning (error)
				if (ax_diff_from_goal(4) <= 10) {
					HAL_Delay(1000);
					arm_state = ARM_MOVE_TO_IDLE;
					break;
				}
				// ccw edge found with cw edge found
				if (get_dist() > distance_threshold_mm) {
					angle_ccw_edge = ax_get_current_position(4);
					ax_stop(4);
					arm_state = ARM_MOVE_TO_GRAB;

					break;
				}
				HAL_Delay(50);
			}
			break;

		case ARM_SEEK_CW:
			HAL_UART_Transmit(&huart1, (uint8_t*)"Entering state : ARM_SEEK_CW\r\n", sizeof("Entering state : ARM_SEEK_CW\r\n"), HAL_MAX_DELAY);
			angle_ccw_edge = angle_cw_edge = 0;
			// Set goal to cw edge if not yet set
			ax_set_goal_raw(4, cw_seek_angle_limit);

			while(1){
				// If at edge, invert seek direction
				if (ax_diff_from_goal(4) <= 10) {
					HAL_Delay(1000);
					arm_state = ARM_SEEK_CCW;
					break;
				}
				// ccw edge found
				if ( get_dist() < distance_threshold_mm) {
					arm_state = ARM_SEEK_EDGE_CW;
					break;
				}

				HAL_Delay(50);
			}

			break;

		case ARM_SEEK_EDGE_CW:
			HAL_UART_Transmit(&huart1, (uint8_t*)"Entering state : ARM_SEEK_EDGE_CW\r\n", sizeof("Entering state : ARM_SEEK_EDGE_CW\r\n"), HAL_MAX_DELAY);
			angle_ccw_edge = ax_get_current_position(4);
			while(1){
				// If at edge, go to beginning (error)
				if (ax_diff_from_goal(4) <= 10) {
					HAL_Delay(1000);
					arm_state = ARM_MOVE_TO_IDLE;
					break;
				}
				// ccw edge found with cw edge found
				if (get_dist() > distance_threshold_mm) {
					angle_cw_edge = ax_get_current_position(4);
					ax_stop(4);
					arm_state = ARM_MOVE_TO_GRAB;

					break;
				}
				HAL_Delay(50);
			}
			break;

		case ARM_MOVE_TO_GRAB:
			HAL_UART_Transmit(&huart1, (uint8_t*)"Entering state : ARM_MOVE_TO_GRAB\r\n", sizeof("Entering state : ARM_MOVE_TO_GRAB\r\n"), HAL_MAX_DELAY);
			middle_rot = (angle_ccw_edge + angle_cw_edge) / 2;
			ax_move_blocked(4, middle_rot, 2);
			HAL_Delay(1000);
			//find rotation angles for correct distance, move inblocked mode
			unsigned int distance = get_dist();

			while(distance > distance_threshold_mm) distance = get_dist();
			ax_move_blocked(4, middle_rot - offset_base_angle(distance+object_offset_distance), 2);
			HAL_Delay(1000);

			if (arm_angles_from_dist(distance+object_offset_distance, &angle_base_joint,  &angle_middle_joint, &angle_claw_joint) == 0) {
				ax_move_blocked(9, angle_claw_joint, 2);
				HAL_Delay(500);
				ax_move_blocked(2, angle_middle_joint, 2);
				HAL_Delay(500);
				ax_move_blocked(3, angle_base_joint, 2);
				HAL_Delay(1000);
			}

			arm_state = ARM_GRAB_CLAW;
			break;

		case ARM_GRAB_CLAW:
			HAL_UART_Transmit(&huart1, (uint8_t*)"Entering state : ARM_GRAB_CLAW\r\n", sizeof("Entering state : ARM_GRAB_CLAW\r\n"), HAL_MAX_DELAY);
			/* TODO: grab in blocked mode (wait for timeout)
			 * IF grab angle at limit, object wasnt grabbed (goto start)
			 * Otherwise assume we have object
			 */
			ax_move_blocked(8, claw_angle_limit_closed, 2);

			HAL_Delay(200);

			if (abs( (int ) (ax_get_current_position(8) - claw_angle_limit_closed)) > 15) {
				arm_state = ARM_MOVE_TO_DUMP;
			}

			else{
				arm_state = ARM_ERROR_STATE;
			}
			HAL_Delay(1000);

			break;

		case ARM_MOVE_TO_DUMP:
			HAL_UART_Transmit(&huart1, (uint8_t*)"Entering state : ARM_MOVE_TO_DUMP\r\n", sizeof("Entering state : ARM_MOVE_TO_DUMP\r\n"), HAL_MAX_DELAY);
			ax_move_blocked(3, 512, 2);
			HAL_Delay(1000);
			ax_move_blocked(2, 819, 2);
			HAL_Delay(1000);
			ax_move_blocked(9, 512, 2);
			HAL_Delay(1000);
			ax_move_blocked(4, 100, 2);
			HAL_Delay(1000);
			ax_move_blocked(8, claw_angle_limit_open, 2);
			HAL_Delay(1000);
			arm_state = ARM_MOVE_TO_IDLE;
			break;

		case ARM_ERROR_STATE:
			HAL_UART_Transmit(&huart1, (uint8_t*)"Entering state : ARM_ERROR_STATE\r\n", sizeof("Entering state : ARM_ERROR_STATE\r\n"), HAL_MAX_DELAY);
			//while(1){}

			arm_state = ARM_MOVE_TO_IDLE;
			break;

		default:
			break;
		}
	}
}
