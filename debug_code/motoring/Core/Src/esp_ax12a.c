/*
 * esp_ax12a.c
 *
 *  Created on: Apr 6, 2023
 *      Author: Jussi Virtanen
 *
 * Interface for AX-12A servos for ESP
 */
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "stm32f4xx_hal.h"

#include "esp_ax12a.h"
#include "util.h"

// UART send and receive buffers
uint8_t *ax_send_buffer;
uint8_t *ax_recv_buffer;
// UART interfaces for base (rotation) and rest of the arm, set at init
UART_HandleTypeDef *base_uart;
UART_HandleTypeDef *arm_uart;

/*
 * Initialize servos and UART buffers
 */
void ax_init(UART_HandleTypeDef *base_uart, UART_HandleTypeDef *arm_uart)
{
	// some initialization code here (set max movement speed, max torque, initialize position etc.)
	base_uart = base_uart;
	arm_uart = arm_uart;
	ax_send_buffer = (uint8_t*)calloc(SEND_BUF_SIZE, sizeof(uint8_t));
	ax_recv_buffer = (uint8_t*)calloc(RECV_BUF_SIZE, sizeof(uint8_t));
}

/*
 * Deinitialize servos and UART buffers
 */
void ax_deinit()
{
	// Some deinit code (what is even needed? Will our program ever end, or do we just cut the power?)
	free(ax_send_buffer);
	free(ax_recv_buffer);
}

/*
 * "Map" of servo id-> uart interface
 */
UART_HandleTypeDef* get_huart(uint8_t servo_id)
{
	// TODO
	return base_uart;
}

/*
 * Send and receive uart data to servo
 */
void send_recv_uart(uint8_t servo_id, dmp_inst instruction, uint8_t *param_list, size_t param_len, size_t return_len)
{
	int packet_size;
	UART_HandleTypeDef *huart;
	HAL_StatusTypeDef uart_status_send;
	HAL_StatusTypeDef uart_status_recv;

	packet_size = 6 + param_len;

	ax_send_buffer[0] = 0xFF;
	ax_send_buffer[1] = 0xFF;
	ax_send_buffer[2] = servo_id;
	ax_send_buffer[3] = param_len + 2;
	ax_send_buffer[4] = instruction;
	for (int i = 0; i < param_len; i++) {
		ax_send_buffer[5+i] = param_list[i];
	}
	ax_send_buffer[packet_size-1] = dmp_chksm(ax_send_buffer);

	huart = get_huart(servo_id);
	/* Dont do anything except enable receiver between send and receive,
	 * otherwise Nucleo might not be fast enough to catch return packet
	 */
	HAL_HalfDuplex_EnableTransmitter(huart);
	uart_status_send = HAL_UART_Transmit(huart, ax_send_buffer, packet_size, UART_SEND_TIMEOUT);
	HAL_HalfDuplex_EnableReceiver(huart);
	uart_status_recv = HAL_UART_Receive(huart, ax_recv_buffer, return_len, UART_RECV_TIMEOUT);

#if DEBUG_SEND || DEBUG_RECV
	char temp[50];
#endif

#if DEBUG_SEND
	array8_to_hex(ax_send_buffer, packet_size, temp);
	if (uart_status_send != HAL_OK) {
		printf("Failed to send packet %s via UART\n", temp);
	} else {
		printf("Sent packet %s via UART\n", temp);
	}
#endif

#if DEBUG_RECV
	array8_to_hex(ax_recv_buffer, return_len, temp);
	if (uart_status_recv != HAL_OK) {
		printf("Failed to receive status packet\n");
	} else {
		printf("Received status packet %s via UART\n", temp);
	}
#endif
}

/*
 * Main AX-12A interface begins
 */
void ax_ping(uint8_t id)
{
	send_recv_uart(id, DMP_PING, NULL, 0, 6);
}

void ax_reset(uint8_t id)
{
	send_recv_uart(id, DMP_RESET, NULL, 0, 6);
}

void ax_set_id(uint8_t old_id, uint8_t new_id)
{
	uint8_t params[] = {3, new_id}; // ID Address = 3
	send_recv_uart(old_id, DMP_WRITE, params, 2, 6);
}

void ax_set_angle_limit(uint8_t id, uint16_t angle, bool ccw)
{
	uint8_t address;
	uint8_t b0 = (uint8_t)(angle >> 8);
	uint8_t b1 = (uint8_t)angle;
	if (ccw) {
		address = 8;
	} else {
		address = 6;
	}  // Angle limit address = 6 for CW, 8 for CCW
	uint8_t params[] = {address, b0, b1};
	send_recv_uart(id, DMP_WRITE, params, 3, 6);
}

uint16_t ax_get_angle_limit(uint8_t id, bool ccw)
{
	uint8_t address;
	uint16_t angle = 0;
	if (ccw) {
		address = 8;
	} else {
		address = 6;
	}
	uint8_t params[] = {address, 2};
	send_recv_uart(id, DMP_READ, params, 2, 8);
	angle |= ax_recv_buffer[5];
	angle <<= 8;
	angle |= ax_recv_buffer[6];
	return angle;
}

void ax_set_max_torque(uint8_t id, uint16_t torque)
{
	uint8_t b0 = (uint8_t)(torque >> 8);
	uint8_t b1 = (uint8_t)torque;
	uint8_t params[] = {14, b0, b1}; // Torque address = 14
	send_recv_uart(id, DMP_WRITE, params, 3, 6);
}

uint16_t ax_get_max_torque(uint8_t id)
{
	uint16_t max_torque = 0;
	uint8_t params[] = {14, 2};
	send_recv_uart(id, DMP_READ, params, 2, 8);
	max_torque |= ax_recv_buffer[5];
	max_torque <<= 8;
	max_torque |= ax_recv_buffer[6];
	return max_torque;
}

void ax_set_led(uint8_t id, bool mode)
{
	uint8_t params[] = {25, mode}; // Led address = 25
	send_recv_uart(id, DMP_WRITE, params, 2, 6);
}

bool ax_get_led(uint8_t id)
{
	uint8_t params[] = {25, 1};
	send_recv_uart(id, DMP_READ, params, 2, 7);
	return (bool)ax_recv_buffer[5];
}

void ax_toggle_led(uint8_t id)
{
	bool mode = !ax_get_led(id);
	uint8_t params[] = {25, mode};
	send_recv_uart(id, DMP_WRITE, params, 2, 6);
}

void ax_set_goal_raw(uint8_t id, uint16_t angle)
{
	uint8_t b0 = (uint8_t)(angle >> 8);
	uint8_t b1 = (uint8_t)angle;
	uint8_t params[] = {30, b0, b1}; // Goal address = 30
	send_recv_uart(id, DMP_WRITE, params, 3, 6);
}

uint16_t ax_get_goal_raw(uint8_t id)
{
	uint16_t angle = 0;
	uint8_t params[] = {30, 2};
	send_recv_uart(id, DMP_READ, params, 2, 8);
	angle |= ax_recv_buffer[5];
	angle <<= 8;
	angle |= ax_recv_buffer[6];
	return angle;
}

void ax_set_goal_deg(uint8_t id, float angle)
{
	uint16_t raw_angle = (uint16_t)round(angle / 300.0f * 1023.0f);
	ax_set_goal_raw(id, raw_angle);
}

void ax_set_move_speed(uint8_t id, uint16_t speed)
{
	uint8_t b0 = (uint8_t)(speed >> 8);
	uint8_t b1 = (uint8_t)speed;
	uint8_t params[] = {32, b0, b1}; // Speed address = 32
	send_recv_uart(id, DMP_WRITE, params, 3, 6);
}

uint16_t ax_get_move_speed(uint8_t id)
{
	uint16_t speed = 0;
	uint8_t params[] = {32, 2};
	send_recv_uart(id, DMP_READ, params, 2, 8);
	speed |= ax_recv_buffer[5];
	speed <<= 8;
	speed |= ax_recv_buffer[6];
	return speed;
}

uint16_t ax_get_current_position(uint8_t id)
{
	uint16_t pos = 0;
	uint8_t params[] = {36, 2}; // Current position address = 36
	send_recv_uart(id, DMP_READ, params, 2, 8);
	pos |= ax_recv_buffer[5];
	pos <<= 8;
	pos |= ax_recv_buffer[6];
	return pos;
}

uint16_t ax_get_current_speed(uint8_t id)
{
	uint16_t speed = 0;
	uint8_t params[] = {38, 2}; // Current speed address = 38
	send_recv_uart(id, DMP_READ, params, 2, 8);
	speed |= ax_recv_buffer[5];
	speed <<= 8;
	speed |= ax_recv_buffer[6];
	return speed;
}

uint16_t ax_get_current_load(uint8_t id)
{
	uint16_t load = 0;
	uint8_t params[] = {40, 2}; // Current load address = 40
	send_recv_uart(id, DMP_READ, params, 2, 8);
	load |= ax_recv_buffer[5];
	load <<= 8;
	load |= ax_recv_buffer[6];
	return load;
}
