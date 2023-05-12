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
int arm_angles_from_dist(unsigned int distance,
		unsigned int *angle_base_joint, unsigned int *angle_middle_joint, unsigned int *angle_claw_joint)
{
	/* Calculations in rads and millimeters,
	 * return values in raw servo angles
	 */
	double angle_base_joint_rad, angle_middle_joint_rad, angle_claw_joint_rad;
	double height_offset;
	double base_offset_angle, claw_offset_angle;
	double arm_len_from_base = 154.0; // a in law of cosines
	double arm_len_from_joint = 140.0; // b in law of cosines
	double max_distance;
	/* Need some height offset, basically a good estimation could be:
	 * base height - claw height + object height
	 */
	height_offset = 0.0;

	max_distance = sqrt(pow(arm_len_from_base +arm_len_from_joint,2) -pow(height_offset,2));
	if(distance >= max_distance) {
		return -1;
	}

	double dist = sqrt(pow((double)distance, 2.0) + pow(height_offset, 2.0)); // c in law of cosines

	// beta in law of cosines
	angle_base_joint_rad = acos((pow(arm_len_from_base, 2.0) + pow(dist, 2.0) - pow(arm_len_from_joint, 2.0)) / (2.0 * arm_len_from_base * dist));
	// gamma in law of cosines
	angle_middle_joint_rad = acos((pow(arm_len_from_base, 2.0) + pow(arm_len_from_joint, 2.0) - pow(dist, 2.0)) / (2.0 * arm_len_from_base * arm_len_from_joint));
	// alpha in law of cosines
	angle_claw_joint_rad = M_PI - angle_base_joint_rad - angle_middle_joint_rad;

	// Servo angles will need to be offset due to height offset
	if (height_offset > 0.0) {
		base_offset_angle = asin(height_offset / distance);
		claw_offset_angle = acos(height_offset / distance);
	} else {
		base_offset_angle = claw_offset_angle = 0.0;
	}

	*angle_base_joint = 512 + rad_to_ax(M_PI/2.0 - angle_base_joint_rad - base_offset_angle);
	*angle_middle_joint = 512 + rad_to_ax(M_PI - angle_middle_joint_rad);
	*angle_claw_joint = 512 + rad_to_ax(angle_claw_joint_rad - claw_offset_angle);

	return 0;
}

unsigned int offset_base_angle(unsigned int distance)
{
	double claw_offset = 15.0; //mm
	double offset_angle;

	offset_angle = atan(claw_offset/distance);
	return rad_to_ax(offset_angle);
}
