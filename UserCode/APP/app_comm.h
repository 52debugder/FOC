#ifndef __APP_COMM_H__
#define __APP_COMM_H__ 

#include "stm32f1xx_hal.h"
#include "foc.h"

void app_uart_send(const uint8_t *p_data, uint16_t length);
void app_uart_recv(uint8_t *p_data, uint16_t length);
void app_iic_transmit(const uint8_t *p_data, uint16_t length);
void app_iic_receive(uint8_t *p_data, uint16_t length);
void app_debug_sample_from_isr(void);
void app_debug_print(void);

#endif
