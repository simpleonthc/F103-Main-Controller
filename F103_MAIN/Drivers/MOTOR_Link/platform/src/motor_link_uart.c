/*
 * motor_link.c
 *
 *  Created on: Oct 26, 2026
 *      Author: pk
 */

#include "motor_link_uart.h"


extern ML_UARTInterface ML_Interface;

ML_Status ML_UARTWrite(uint8_t *pData, size_t data_len) {
	if (ML_Interface.tx_flag == 1) {
		HAL_UART_Transmit_DMA(ML_Interface.huart, pData, data_len);
		ML_Interface.tx_flag = 0;
	}

	return ML_OK;
}

ML_Status ML_UARTRead(uint8_t *pData, size_t data_len) {
	HAL_UART_Receive_DMA(ML_Interface.huart, pData, data_len);
	return ML_OK;
}

