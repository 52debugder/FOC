#ifndef __COMM_IIC_H__
#define __COMM_IIC_H__

#include "stm32f1xx_hal.h"

void comm_iic_transmit(const uint8_t *data, uint16_t len);
void comm_iic_receive(uint8_t *data, uint16_t len);

#endif
