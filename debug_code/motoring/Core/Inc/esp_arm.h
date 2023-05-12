/*
 * esp_arm.h
 *
 *  Created on: Apr 26, 2023
 *      Author: tim
 */

#ifndef INC_ESP_ARM_H_
#define INC_ESP_ARM_H_

int arm_angles_from_dist(unsigned int distance,
		unsigned int *angle_base_joint, unsigned int *angle_middle_joint, unsigned int *angle_claw_joint);
unsigned int offset_base_angle(unsigned int distance);

#endif /* INC_ESP_ARM_H_ */
