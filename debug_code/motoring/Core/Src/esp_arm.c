/*
 * esp_arm.c
 *
 *  Created on: Apr 26, 2023
 *      Author: Jussi Virtanen
 *
 *  High-level functions for arm control
 */

#include "math.h"

unsigned int rad_to_ax(double angle)
{
	double ax_360 = 1023.0 / 300.0 * 360.0;
	return (unsigned int)(ax_360 * (angle/(2.0*M_PI)));
}

/* Get arm rotations from object distance (pythagoras + law of cosines)
 * https://en.wikipedia.org/wiki/Law_of_cosines
 *
 * angle_base_joint: base join rotation
 * angle_middle_joint: middle joint rotation
 * angle_claw_joint: claw joint rotation
 */
void arm_angles_from_dist(unsigned int distance,
		unsigned int *angle_base_joint, unsigned int *angle_middle_joint, unsigned int *angle_claw_joint)
{
	// Everything in mm
	double angle_base_joint_rad, angle_middle_joint_rad, angle_claw_joint_rad;
	double height_offset;
	double base_offset_angle, claw_offset_angle;
	double arm_len_from_base = 154; // a in law of cosines
	double arm_len_from_joint = 140; // b in law of cosines
	/* Need some height offset, basically a good estimation could be:
	 * base height - claw height + object height
	 */
	height_offset = 50.0;

	double dist = sqrt(pow((double)distance, 2) - pow(height_offset, 2)); // c in law of cosines

	// beta in law of cosines
	angle_base_joint_rad = acos((pow(arm_len_from_base, 2) + pow(dist, 2) - pow(arm_len_from_joint, 2)) / (2.0 * arm_len_from_base * dist));
	// gamma in law of cosines
	angle_middle_joint_rad = acos((pow(arm_len_from_base, 2) + pow(arm_len_from_joint, 2) - pow(dist, 2)) / (2.0 * arm_len_from_base * arm_len_from_joint));
	// alpha in law of cosines
	angle_claw_joint_rad = M_PI - angle_base_rad - angle_joint_rad;

	// Servo angles will need to be offset due to height offset
	base_offset_angle = asin(height_offset / distance);
	claw_offset_angle = acos(height_offset / distance);

	*angle_base_joint = 512 + rad_to_ax(M_PI/2.0 - angle_base_joint_rad - base_offset_angle);
	*angle_middle_joint = 512 + rad_to_ax(M_PI - angle_middle_joint_rad);
	*angle_claw_joint = 512 - rad_to_ax(M_PI - angle_claw_joint_rad - claw_offset_angle);
}
