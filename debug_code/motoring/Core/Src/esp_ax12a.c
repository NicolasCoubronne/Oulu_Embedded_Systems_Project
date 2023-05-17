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
uint8_t ax_send_buffer[SEND_BUF_SIZE];
uint8_t ax_recv_buffer[RECV_BUF_SIZE];
// UART interfaces for base (rotation) and rest of the arm, set at init
UART_HandleTypeDef *base_uart;
UART_HandleTypeDef *arm_uart;

/*
 * Initialize servos and UART buffers
 *
 * base_uart: pointer to UART interface to which the base (rotation) servo is connected
 * arm_uart: pointer to UART interface to which the arm servos are connected
 */
void ax_init(UART_HandleTypeDef *base_uart_p, UART_HandleTypeDef *arm_uart_p)
{
	// some initialization code here (set max movement speed, max torque, initialize position etc.)
	base_uart = base_uart_p;
	arm_uart = arm_uart_p;
}

/*
 * Deinitialize servos and UART buffers
 */
void ax_deinit()
{
	// Some deinit code (what is even needed? Will our program ever end, or do we just cut the power?)
}

/*
 * "Map" of servo id-> uart interface.
 * The UART interfaces of each servo wont change dynamically,
 * so we can hard-code this.
 *
 * servo_id: servo ID
 *
 * returns: pointer to UART interface where this servo is located
 */
UART_HandleTypeDef* get_huart(uint8_t servo_id)
{
	// TODO
	return base_uart;
}

/*
 * Send and receive uart data to servo
 * Saves the send packet to ax_send_buffer
 * Saves the return packet to ax_recv_buffer
 *
 * servo_id: servo ID
 * instruction: instruction type as per dynamixel protocol 1.0
 * param_list: pointer to a list of parameters
 * param_len: length of parameter list
 * return_len: length of return status packet
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

	for(int i = 0; i<5;i++){
	HAL_HalfDuplex_EnableTransmitter(huart);
	uart_status_send = HAL_UART_Transmit(huart, ax_send_buffer, packet_size, UART_SEND_TIMEOUT);
	HAL_HalfDuplex_EnableReceiver(huart);
	uart_status_recv = HAL_UART_Receive(huart, ax_recv_buffer, return_len, UART_RECV_TIMEOUT);

	if(uart_status_send == HAL_OK && uart_status_recv == HAL_OK) break;
	}

#if DEBUG_SEND || DEBUG_RECV
	char temp[50];
#endif

#if DEBUG_SEND
	array8_to_hex(ax_send_buffer, packet_size, temp);
	if (uart_status_send != HAL_OK) {
		printf("Failed to send packet %s via UART\n", temp);
		while(1){}
	} else {
		printf("Sent packet %s via UART\n", temp);
	}
#endif

#if DEBUG_RECV
	array8_to_hex(ax_recv_buffer, return_len, temp);
	if (uart_status_recv != HAL_OK) {
		printf("Failed to receive status packet\n");
		while(1){}
	} else {
		printf("Received status packet %s via UART\n", temp);
	}
#endif
}

/*
 * Main AX-12A interface begins
 */


/*
 * Send ping command to servo
 *
 * id: servo ID
 */
void ax_ping(uint8_t id)
{
	send_recv_uart(id, DMP_PING, NULL, 0, 6);
}

/*
 * Factory reset a servo
 * Will not change ID
 *
 * id: servo ID
 */
void ax_factory_reset(uint8_t id)
{
	send_recv_uart(id, DMP_RESET, NULL, 0, 6);
}

/*
 * Change the ID of a servo
 *
 * old_id: current servo ID
 * new_id: new servo ID
 */
void ax_set_id(uint8_t old_id, uint8_t new_id)
{
	uint8_t params[] = {3, new_id}; // ID Address = 3
	send_recv_uart(old_id, DMP_WRITE, params, 2, 6);
}

/*
 * Set the angle limit of a servo
 *
 * id: servo id
 * angle: new angle limit (range: 0..1023, corresponds to 0..300 degrees)
 * ccw: whether to set the clockwise (0) or counterclockwise (1) limit
 */
void ax_set_angle_limit(uint8_t id, unsigned int angle, bool ccw)
{
	uint8_t address;
	uint8_t b0 = (uint8_t)(angle >> 8);
	uint8_t b1 = (uint8_t)angle;
	if (ccw) {
		address = 8;
	} else {
		address = 6;
	}  // Angle limit address = 6 for CW, 8 for CCW
	uint8_t params[] = {address, b1, b0};
	send_recv_uart(id, DMP_WRITE, params, 3, 6);
}

/*
 * Set the angle limit of a servo
 *
 * id: servo id
 * ccw: whether to get the clockwise (0) or counterclockwise (1) limit
 *
 * returns: angle limit (0..1023)
 */
unsigned int ax_get_angle_limit(uint8_t id, bool ccw)
{
	uint8_t address;
	unsigned int angle = 0;
	if (ccw) {
		address = 8;
	} else {
		address = 6;
	}
	uint8_t params[] = {address, 2};
	send_recv_uart(id, DMP_READ, params, 2, 8);
	angle |= ax_recv_buffer[6];
	angle <<= 8;
	angle |= ax_recv_buffer[5];
	return angle;
}

/*
 * Set the max torque of servo
 *
 * id: servo id
 * torque: new max torque, range (0..1023), one unit corresponds to ~0.1% torque
 */
void ax_set_max_torque(uint8_t id, unsigned int torque)
{
	uint8_t b0 = (uint8_t)(torque >> 8);
	uint8_t b1 = (uint8_t)torque;
	uint8_t params[] = {34, b1, b0}; // Torque address = 14
	send_recv_uart(id, DMP_WRITE, params, 3, 6);
}

/*
 * Get the max torque of servo
 *
 * id: servo id
 *
 * returns: max torque, range (0..1023), one unit corresponds to ~0.1% torque
 */
unsigned int ax_get_max_torque(uint8_t id)
{
	unsigned int max_torque = 0;
	uint8_t params[] = {14, 2};
	send_recv_uart(id, DMP_READ, params, 2, 8);
	max_torque |= ax_recv_buffer[6];
	max_torque <<= 8;
	max_torque |= ax_recv_buffer[5];
	return max_torque;
}

/*
 * Set servo led
 *
 * id: servo id
 * mode: off (0) or on (1)
 */
void ax_set_led(uint8_t id, bool mode)
{
	uint8_t params[] = {25, mode}; // Led address = 25
	send_recv_uart(id, DMP_WRITE, params, 2, 6);
}

/*
 * Get servo led status
 *
 * id: servo id
 *
 * returns: status of led, off (0) or on (1)
 */
bool ax_get_led(uint8_t id)
{
	uint8_t params[] = {25, 1};
	send_recv_uart(id, DMP_READ, params, 2, 7);
	return (bool)ax_recv_buffer[5];
}

/*
 * Toggle led
 *
 * id: servo id
 */
void ax_toggle_led(uint8_t id)
{
	bool mode = !ax_get_led(id);
	uint8_t params[] = {25, mode};
	send_recv_uart(id, DMP_WRITE, params, 2, 6);
}

/*
 * Set new servo goal position
 *
 * id: servo id
 * angle: new goal position, range is (0..1023) (internal units, corresponds to 0..300 degrees)
 */
void ax_set_goal_raw(uint8_t id, unsigned int angle)
{
	uint8_t b0 = (uint8_t)(angle >> 8);
	uint8_t b1 = (uint8_t)angle;
	uint8_t params[] = {30, b1, b0}; // Goal address = 30
	send_recv_uart(id, DMP_WRITE, params, 3, 6);
}

/*
 * Get servo goal position
 * This is NOT the current position
 *
 * id: servo id
 *
 * returns: goal position, range is (0..1023)
 */
unsigned int ax_get_goal_raw(uint8_t id)
{
	unsigned int angle = 0;
	uint8_t params[] = {30, 2};
	send_recv_uart(id, DMP_READ, params, 2, 8);
	angle |= ax_recv_buffer[6];
	angle <<= 8;
	angle |= ax_recv_buffer[5];
	return angle;
}

/*
 * Set new servo goal position
 *
 * id: servo id
 * angle: new goal position, range is (0..300) degrees
 */
void ax_set_goal_deg(uint8_t id, float angle)
{
	unsigned int raw_angle = (unsigned int)round(angle / 300.0f * 1023.0f);
	ax_set_goal_raw(id, raw_angle);
}

/*
 * Set servo move speed
 *
 * id: servo id
 * speed: move speed, range is (0..1023), corresponds to ~0.1% of max speed
 */
void ax_set_move_speed(uint8_t id, unsigned int speed)
{
	uint8_t b0 = (uint8_t)(speed >> 8);
	uint8_t b1 = (uint8_t)speed;
	uint8_t params[] = {32, b1, b0}; // Speed address = 32
	send_recv_uart(id, DMP_WRITE, params, 3, 6);
}

/*
 * Get servo move speed
 * This is NOT the current move speed
 *
 * id: servo id
 *
 * returns: move speed, range is (0..1023), corresponds to ~0.1% of max speed
 */
unsigned int ax_get_move_speed(uint8_t id)
{
	unsigned int speed = 0;
	uint8_t params[] = {32, 2};
	send_recv_uart(id, DMP_READ, params, 2, 8);
	speed |= ax_recv_buffer[6];
	speed <<= 8;
	speed |= ax_recv_buffer[5];
	return speed;
}

/*
 * Get current servo position
 *
 * id: servo id
 *
 * returns: current position, range is (0..1023)
 */
unsigned int ax_get_current_position(uint8_t id)
{
	unsigned int pos = 0;
	uint8_t params[] = {36, 2}; // Current position address = 36
	send_recv_uart(id, DMP_READ, params, 2, 8);
	pos |= ax_recv_buffer[6];
	pos <<= 8;
	pos |= ax_recv_buffer[5];
	return pos;
}

/*
 * Get current servo move speed
 *
 * id: servo id
 *
 * returns: current move speed, range is (0..1023), corresponds to ~0.1% of max speed
 */
unsigned int ax_get_current_speed(uint8_t id)
{
	unsigned int speed = 0;
	uint8_t params[] = {38, 2}; // Current speed address = 38
	send_recv_uart(id, DMP_READ, params, 2, 8);
	speed |= ax_recv_buffer[6];
	speed <<= 8;
	speed |= ax_recv_buffer[5];
	return speed;
}

unsigned int ax_get_current_load(uint8_t id)
{
	unsigned int load = 0;
	uint8_t params[] = {40, 2}; // Current load address = 40
	send_recv_uart(id, DMP_READ, params, 2, 8);
	load |= ax_recv_buffer[6];
	load <<= 8;
	load |= ax_recv_buffer[5];
	if (load < 1024) {
		load *= -1;
	} else {
		load -= 1024;
	}
	return load;
}

/*
 * Stop servo (=set goal position to current position)
 *
 * id: servo id
 */
void ax_stop(uint8_t id)
{
	ax_set_goal_raw(id, ax_get_current_position(id));
}

/*
 * Set servo goal, blocks until pos near goal OR timeout is reached while position is not changing much
 *
 * id: servo id
 * angle: goal angle
 * timeout: when to stop blocking even if goal isn't reached (seconds)
 */
void ax_move_blocked(uint8_t id, unsigned int angle, unsigned int timeout)
{
	/* Block until servo is threshold distance close to goal, otherwise wait until timeout
	 */
	double threshold = 10;
	int poll_period_ms = 200;
	unsigned int waited = 0;
	unsigned int wait_after = 1000;

	unsigned int diff, prev_diff;

	ax_set_goal_raw(id, angle);
	do {
		prev_diff = ax_diff_from_goal(id);
		HAL_Delay(poll_period_ms);
		diff = ax_diff_from_goal(id);
		if (abs(prev_diff - diff) < threshold) {
			waited += poll_period_ms;
		}
	} while (diff > threshold && waited < timeout);
	if (waited < wait_after) {
		HAL_Delay(wait_after - waited);
	}
}

/*
 * Get current distance from goal distance (absolute value, servo angle units)
 *
 * id: servo id
 */
unsigned int ax_diff_from_goal(uint8_t id)
{
	int diff = abs((int)ax_get_goal_raw(id) - (int)ax_get_current_position(id));
	return (unsigned int)diff;
}
