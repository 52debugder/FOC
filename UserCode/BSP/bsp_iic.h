#ifndef __BSP_IIC_H__
#define __BSP_IIC_H__

#include "stm32f1xx_hal.h"
#include "i2c.h"

void bsp_iic_transmit(const uint8_t *pdata, uint16_t len);
void bsp_iic_receive(uint8_t *pdata, uint16_t len);
HAL_StatusTypeDef bsp_iic_mem_write(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *pdata, uint16_t len);
HAL_StatusTypeDef bsp_iic_mem_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *pdata, uint16_t len);
HAL_StatusTypeDef bsp_iic_is_ready(uint8_t dev_addr);

#endif
