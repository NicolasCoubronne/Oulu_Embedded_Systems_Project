/*
 * esp_ax12a.c
 *
 *  Created on: Apr 6, 2023
 *      Author: Jussi Virtanen
 */

#ifndef INC_ESP_AX12A_H_
#define INC_ESP_AX12A_H_


uint8_t bio_chksm(uint8_t* buffer);
void send_recv_uart(UART_HandleTypeDef *huart, uint8_t servo_id, uint8_t instruction, uint8_t *param_list, size_t param_len);
void set_led(UART_HandleTypeDef *huart, uint8_t id, uint8_t mode);
void ping(UART_HandleTypeDef *huart, uint8_t id);


#endif /* INC_ESP_AX12A_H_ */
