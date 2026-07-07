#ifndef __BSP_USART_H__
#define __BSP_USART_H__ 

#include "stm32f1xx_hal.h"
#include "usart.h"

void bsp_usart_send(const uint8_t *data, uint16_t size);
HAL_StatusTypeDef bsp_usart_send_async(const uint8_t *data, uint16_t size);
uint8_t bsp_usart_tx_is_idle(void);
void bsp_usart_tx_complete(UART_HandleTypeDef *huart);
void bsp_usart_recv(uint8_t *data, uint16_t size);

#endif
