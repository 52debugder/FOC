#include "bsp_iic.h"

#define BSP_IIC_HANDLE       &hi2c1
#define BSP_IIC_DEFAULT_ADDR 0x36U
#define BSP_IIC_TIMEOUT_MS   1000U
#define BSP_IIC_ADDR(addr)   ((uint16_t)((addr) << 1U))

void bsp_iic_transmit(const uint8_t *pdata, uint16_t len)
{
    HAL_I2C_Master_Transmit(BSP_IIC_HANDLE, BSP_IIC_ADDR(BSP_IIC_DEFAULT_ADDR), (uint8_t *)pdata, len, BSP_IIC_TIMEOUT_MS);
}

void bsp_iic_receive(uint8_t *pdata, uint16_t len)
{
    HAL_I2C_Master_Receive(BSP_IIC_HANDLE, BSP_IIC_ADDR(BSP_IIC_DEFAULT_ADDR), pdata, len, BSP_IIC_TIMEOUT_MS);
}

HAL_StatusTypeDef bsp_iic_mem_write(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *pdata, uint16_t len)
{
    return HAL_I2C_Mem_Write(BSP_IIC_HANDLE, BSP_IIC_ADDR(dev_addr), reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t *)pdata, len, BSP_IIC_TIMEOUT_MS);
}

HAL_StatusTypeDef bsp_iic_mem_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *pdata, uint16_t len)
{
    return HAL_I2C_Mem_Read(BSP_IIC_HANDLE, BSP_IIC_ADDR(dev_addr), reg_addr, I2C_MEMADD_SIZE_8BIT, pdata, len, BSP_IIC_TIMEOUT_MS);
}

HAL_StatusTypeDef bsp_iic_is_ready(uint8_t dev_addr)
{
    return HAL_I2C_IsDeviceReady(BSP_IIC_HANDLE, BSP_IIC_ADDR(dev_addr), 3U, BSP_IIC_TIMEOUT_MS);
}
