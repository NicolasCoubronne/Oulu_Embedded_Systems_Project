/*
 * esp_ax12a.c
 *
 *  Created on: Apr 6, 2023
 *      Author: Jussi Virtanen
 *
 * Interface for AX-12A servos for ESP
 */
#include <stdio.h>
#include <stdlib.h>

#include "stm32f4xx_hal.h"

#include "esp_ax12a.h"
#include "util.h"

// Nucleo might not be fast enough to receive if you do anything (like printing) before receive!
#define DEBUG_SEND 0
#define DEBUG_RECV 1
#define UART_TIMEOUT 500

/*
 * Return uart packet checksum as described by Dynamixel protocol 1.0
 */
uint8_t bio_chksm(uint8_t* buffer)
{
	uint32_t checksum = 0;
	for (int i = 0; i < buffer[3] + 1; i++) {
		checksum += buffer[2 + i];
	}
	return ~((uint8_t)checksum);
}

/*
 * Send and receive uart data to servo
 */
void send_recv_uart(UART_HandleTypeDef *huart, uint8_t servo_id, uint8_t instruction, uint8_t *param_list, size_t param_len) {
	// Buffer length in bytes: header (2) + id + len + instruction type + param_amount + checksum
	int send_buffer_len = 2 + 1 + 1 + 1 + param_len + 1;
	uint8_t *send_buffer = (uint8_t*)calloc(send_buffer_len, sizeof(uint8_t));
	uint8_t *recv_buffer = (uint8_t*)calloc(6, sizeof(uint8_t));

	send_buffer[0] = 0xFF;
	send_buffer[1] = 0xFF;
	send_buffer[2] = servo_id;
	send_buffer[3] = param_len + 2;
	send_buffer[4] = instruction;
	for (int i = 0; i < param_len; i++) {
		send_buffer[5+i] = param_list[i];
	}
	send_buffer[5+param_len] = bio_chksm(send_buffer);

	HAL_HalfDuplex_EnableTransmitter(huart);
	if (HAL_UART_Transmit(huart, send_buffer, send_buffer_len, UART_TIMEOUT) != HAL_OK) {
#if DEBUG_SEND // Probably need to lower baud rate and/or receive timing from servos to debug print without failing to read uart data back
		printf("Failed to send packet ");
		aprint(send_buffer, send_buffer_len);
		printf("via UART\n");
	} else {
		printf("Sent packet ");
		aprint(send_buffer, send_buffer_len);
		printf("via UART\n");
#endif
	}

	HAL_HalfDuplex_EnableReceiver(huart);
	if (HAL_UART_Receive(huart, recv_buffer, 6, UART_TIMEOUT) != HAL_OK) {
#if DEBUG_RECV
		printf("Failed to receive status packet\n");
	} else {
		printf("Received status packet ");
		aprint(recv_buffer, 6);
		printf("via UART\n");
#endif
	}

	free(send_buffer);
	free(recv_buffer);
}

void set_led(UART_HandleTypeDef *huart, uint8_t id, uint8_t mode)
{
	uint8_t params[] = {0x19, mode};
	send_recv_uart(huart, id, 3, params, 2);
}

void ping(UART_HandleTypeDef *huart, uint8_t id)
{
	send_recv_uart(huart, id, 1, NULL, 0);
}
