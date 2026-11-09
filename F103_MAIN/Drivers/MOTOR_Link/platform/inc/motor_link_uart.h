/*
 * motor_link_uart.h
 *
 *  Created on: Oct 29, 2026
 *      Author: pk
 */

#ifndef MOTOR_LINK_PLATFORM_INC_MOTOR_LINK_UART_H_
#define MOTOR_LINK_PLATFORM_INC_MOTOR_LINK_UART_H_

#include <stdint.h>
#include <stdio.h>
#include <stm32f1xx_hal.h>
#include "MOTOR_Link_Types.h"


#define ML_TX_BUFFER_SIZE 16
#define ML_RX_BUFFER_SIZE 32

typedef struct {
	UART_HandleTypeDef *huart;
	uint8_t tx_flag;
	uint8_t rx_buffer[ML_RX_BUFFER_SIZE];
}ML_UARTInterface;

ML_Status ML_UARTWrite(uint8_t *pData, size_t data_len);
ML_Status ML_UARTRead(uint8_t *pData, size_t data_len);

#endif /* MOTOR_LINK_PLATFORM_INC_MOTOR_LINK_UART_H_ */
