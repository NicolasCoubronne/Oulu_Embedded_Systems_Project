/*
 * esp_ax12a.c
 *
 *  Created on: Apr 6, 2023
 *      Author: Jussi Virtanen
 */

#ifndef INC_ESP_AX12A_H_
#define INC_ESP_AX12A_H_

#include <stdbool.h>

#define DEBUG_SEND 1
#define DEBUG_RECV 1

#define UART_SEND_TIMEOUT 500
#define UART_RECV_TIMEOUT 500
#define SEND_BUF_SIZE 20
#define RECV_BUF_SIZE 10

typedef enum {
	DMP_PING = 1,
	DMP_READ = 2,
	DMP_WRITE = 3,
	DMP_RESET = 6
} dmp_inst;

void ax_init(UART_HandleTypeDef *base_uart_p, UART_HandleTypeDef *arm_uart_p);
void ax_deinit();
void ax_ping(uint8_t id);
void ax_factory_reset(uint8_t id);
void ax_set_id(uint8_t old_id, uint8_t new_id);
void ax_set_angle_limit(uint8_t id, unsigned int angle, bool ccw);
unsigned int ax_get_angle_limit(uint8_t id, bool ccw);
void ax_set_max_torque(uint8_t id, unsigned int torque);
unsigned int ax_get_max_torque(uint8_t id);
void ax_set_led(uint8_t id, bool mode);
bool ax_get_led(uint8_t id);
void ax_toggle_led(uint8_t id);
void ax_set_goal_raw(uint8_t id, unsigned int angle);
unsigned int ax_get_goal_raw(uint8_t id);
void ax_set_goal_deg(uint8_t id, float angle);
void ax_set_move_speed(uint8_t id, unsigned int speed);
unsigned int ax_get_move_speed(uint8_t id);
unsigned int ax_get_current_position(uint8_t id);
unsigned int ax_get_current_speed(uint8_t id);
unsigned int ax_get_current_load(uint8_t id);
void ax_stop(uint8_t id);
void ax_move_blocked(uint8_t id, unsigned int angle, float timeout);


#endif /* INC_ESP_AX12A_H_ */
