/*
 * esp_arm.c
 *
 *  Created on: Apr 26, 2023
 *      Author: Jussi Virtanen
 *
 *  High-level functions for arm control
 */

#include "esp_ax12a.h"

unsigned int rad_to_ax(double angle)
{
	double ax_360 = 1023.0 / 300.0 * 360.0;
	return (unsigned int)(ax_360 * (angle/(2.0*M_PI)));
}

void arm_angles_from_dist(unsigned int distance,
		unsigned int *angle_arm1, unsigned int *angle_arm2, unsigned int *angle_arm3)
{
	double dist = (double)distance;
	double arm_len_from_base = 154;
	double arm_len_from_joint = 140;
	angle_base = acos(pow(arm_len_from_base, 2) + pow(distance, 2) - pow(arm_len_from_joint, 2)) / (2.0 * arm_len_from_base * distance);
	angle_joint = acos(pow(arm_len_from_base, 2) + pow(arm_len_from_joint, 2) - pow(distance, 2)) / (2.0 * arm_len_from_base * arm_len_from_joint);
	angle_claw = acos(pow(arm_len_from_joint, 2) + pow(distance, 2) - pow(arm_len_from_base, 2)) / (2.0 * arm_len_from_joint * distance);

	angle_base_raw = 512 + rad_to_ax(M_PI/2.0 - angle_base);
	angle_base_joint_raw = 512 + rad_to_ax(M_PI/2.0 - angle_base);
	angle_base_joint_raw = 512 + rad_to_ax(M_PI/2.0 - angle_base);
}
