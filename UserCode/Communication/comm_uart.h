#ifndef __COMM_UART_H__
#define __COMM_UART_H__ 

#include "stm32f1xx_hal.h"
#include "stdio.h"

void comm_uart_send(const uint8_t *data, uint16_t len);
HAL_StatusTypeDef comm_uart_send_async(const uint8_t *data, uint16_t len);
uint8_t comm_uart_tx_is_idle(void);
void comm_uart_recv(uint8_t *data, uint16_t len);
void comm_vofa_send1(const float data);
void comm_vofa_send2(const float data, const float data2);
void comm_vofa_send3(const float data, const float data2, const float data3);
HAL_StatusTypeDef comm_vofa_send3_async(const float data, const float data2, const float data3);
void comm_vofa_send4(const float data, const float data2, const float data3, const float data4);

#endif
