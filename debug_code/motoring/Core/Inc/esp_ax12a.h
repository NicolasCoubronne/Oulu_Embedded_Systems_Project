/*
 * esp_ax12a.c
 *
 *  Created on: Apr 6, 2023
 *      Author: Jussi Virtanen
 */

#ifndef INC_ESP_AX12A_H_
#define INC_ESP_AX12A_H_

void ax_init(UART_HandleTypeDef *base_uart, UART_HandleTypeDef *arm_uart);
void ax_deinit();
void ax_ping(uint8_t id);
void ax_reset(uint8_t id);
void ax_set_id(uint8_t old_id, uint8_t new_id);
void ax_set_angle_limit(uint8_t id, uint16_t angle, bool ccw);
uint16_t ax_get_cw_limit(uint8_t id, uint16_t angle, bool ccw);
void ax_set_led(uint8_t id, uint8_t mode);
bool ax_get_led(uint8_t id);
void ax_toggle_led(uint8_t id);
void ax_set_goal_raw(uint8_t id, uint16_t angle);
void ax_set_goal_deg(uint8_t id, float angle);


#endif /* INC_ESP_AX12A_H_ */
