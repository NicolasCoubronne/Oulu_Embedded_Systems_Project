/*
 * demos.c
 *
 *  Created on: Apr 21, 2023
 *      Author: tim
 */

#include "main.h"

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#include "demos.h"
#include "esp_ax12a.h"
#include "util.h"

void demo_init()
{
	uint8_t ids[] = {3, 2, 9, 8, 4};
	uint16_t init_angles[] = {512, 700, 300, 512, 512};
	uint16_t init_speed[] = {75, 75, 75, 50, 75};
	float to = 1;

	for (int i=0; i < sizeof(ids)/sizeof(ids[0]); i++) {
	  ax_set_move_speed(ids[i], init_speed[i]);
	  HAL_Delay(1000);
	}
	for (int i=0; i < sizeof(ids)/sizeof(ids[0]); i++) {
	  ax_move_blocked(ids[i], init_angles[i], to);
	  HAL_Delay(1000);
	}
}

void demo1()
{
	uint8_t ids[] = {3, 2, 9, 8, 4};
	uint16_t init_angles[] = {512, 700, 300, 512, 512};
	uint16_t init_speed[] = {50, 50, 50, 50, 25};
	uint8_t id;
	uint16_t cpos, gpos;

	for (int i=0; i < sizeof(ids)/sizeof(ids[0]); i++) {
	  ax_set_move_speed(ids[i], init_speed[i]);
	  HAL_Delay(1000);
	  ax_get_move_speed(ids[i]);
	  //printf("move speed %u\n", temp);
	  HAL_Delay(1000);
	  ax_set_goal_raw(ids[i], init_angles[i]);
	  HAL_Delay(5000);
	}
	id = 4;
	ax_set_goal_raw(id, 100);
	HAL_Delay(15000);
	cpos = ax_get_current_position(id);
	gpos = ax_get_goal_raw(id);
	printf("current: %u, goal: %u\n", cpos, gpos);
	HAL_Delay(1000);

	id = 4;
	ax_set_goal_raw(id, 900);
	HAL_Delay(15000);
	cpos = ax_get_current_position(id);
	gpos = ax_get_goal_raw(id);
	printf("current: %u, goal: %u\n", cpos, gpos);
	HAL_Delay(1000);

	id = 3;
	ax_set_goal_raw(id, 700);
	HAL_Delay(5000);
	cpos = ax_get_current_position(id);
	gpos = ax_get_goal_raw(id);
	printf("current: %u, goal: %u\n", cpos, gpos);
	HAL_Delay(1000);

	id = 2;
	ax_set_goal_raw(id, 600);
	HAL_Delay(5000);
	cpos = ax_get_current_position(id);
	gpos = ax_get_goal_raw(id);
	printf("current: %u, goal: %u\n", cpos, gpos);
	HAL_Delay(1000);

	id = 9;
	ax_set_goal_raw(id, 200);
	HAL_Delay(5000);
	cpos = ax_get_current_position(id);
	gpos = ax_get_goal_raw(id);
	printf("current: %u, goal: %u\n", cpos, gpos);
	HAL_Delay(1000);

	id = 8;
	ax_set_goal_raw(id, 400);
	HAL_Delay(5000);
	cpos = ax_get_current_position(id);
	gpos = ax_get_goal_raw(id);
	printf("current: %u, goal: %u\n", cpos, gpos);
	HAL_Delay(1000);

	id = 8;
	ax_set_goal_raw(id, 600);
	HAL_Delay(5000);
	cpos = ax_get_current_position(id);
	gpos = ax_get_goal_raw(id);
	printf("current: %u, goal: %u\n", cpos, gpos);
	HAL_Delay(1000);
}

void demo2()
{
	uint8_t ids[] = {3, 2, 9, 8, 4};
	uint16_t init_angles[] = {512, 700, 300, 512, 512};
	uint16_t init_speed[] = {50, 50, 50, 50, 40};
	uint8_t id;
	float to = 5;

	for (int i=0; i < sizeof(ids)/sizeof(ids[0]); i++) {
	  ax_set_move_speed(ids[i], init_speed[i]);
	  HAL_Delay(200);
	}
	for (int i=0; i < sizeof(ids)/sizeof(ids[0]); i++) {
	  ax_move_blocked(ids[i], init_angles[i], to);
	  HAL_Delay(200);
	}

	id = 4;
	ax_move_blocked(id, 300, to);
	HAL_Delay(200);
	ax_move_blocked(id, 700, to);
	HAL_Delay(200);
	ax_move_blocked(id, 512, to);
	HAL_Delay(200);
	id = 3;
	ax_move_blocked(id, 700, to);
	HAL_Delay(200);
	id = 2;
	ax_move_blocked(id, 600, to);
	HAL_Delay(200);
	id = 9;
	ax_move_blocked(id, 200, to);
	HAL_Delay(200);
	id = 8;
	ax_move_blocked(id, 400, to);
	HAL_Delay(200);
	ax_move_blocked(id, 600, to);
	HAL_Delay(200);
}

void demo_load()
{
	//uint8_t ids[] = {3, 2, 9, 8, 4};
	float to = 1;
	uint8_t id;

	demo_init();

	id = 8;
	ax_set_max_torque(8, 600);
	while(1) {
		ax_move_blocked(id, 400, to);
		HAL_Delay(1000);
		ax_set_goal_raw(id, 600);
		for (int i = 0; i < 100; i++) {
			printf("load: %u\n", ax_get_current_load(id));
			HAL_Delay(50);
		}
	}
}

// Print angles of servos (move them manually)
void print_angles()
{
	uint8_t ids[] = {3, 2, 9, 8, 4};
	unsigned int positions[5];

	while(1) {
		for (int i=0; i < 5; i++) {
		  positions[i] = ax_get_current_position(ids[i]);
		}
		printf("ID:\t\t");
		for (int i=0; i < 5; i++) {
		  printf("%u\t", ids[i]);
		}
		printf("\nAngle:\t");
		for (int i=0; i < 5; i++) {
		  printf("%u\t", positions[i]);
		}
		printf("\n");
		HAL_Delay(500);
	}
}
