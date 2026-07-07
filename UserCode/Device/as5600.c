#include "as5600.h"
#include "bsp_iic.h"

#define AS5600_REG_ZMCO       0x00U
#define AS5600_REG_ZPOS_H     0x01U
#define AS5600_REG_MPOS_H     0x03U
#define AS5600_REG_MANG_H     0x05U
#define AS5600_REG_CONF_H     0x07U
#define AS5600_REG_STATUS     0x0BU
#define AS5600_REG_RAW_ANGLE  0x0CU
#define AS5600_REG_ANGLE      0x0EU
#define AS5600_REG_AGC        0x1AU
#define AS5600_REG_MAGNITUDE  0x1BU

#define AS5600_STATUS_MH      0x08U
#define AS5600_STATUS_ML      0x10U
#define AS5600_STATUS_MD      0x20U
#define AS5600_STATUS_MASK    0x38U

static HAL_StatusTypeDef AS5600_ReadReg(uint8_t reg, uint8_t *data, uint16_t len)
{
    if (data == 0 || len == 0U)
        return HAL_ERROR;

    return bsp_iic_mem_read(AS5600_I2C_ADDR, reg, data, len);
}

static HAL_StatusTypeDef AS5600_WriteReg(uint8_t reg, const uint8_t *data, uint16_t len)
{
    if (data == 0 || len == 0U)
        return HAL_ERROR;

    return bsp_iic_mem_write(AS5600_I2C_ADDR, reg, data, len);
}

static HAL_StatusTypeDef AS5600_Read12(uint8_t reg, uint16_t *value)
{
    uint8_t buf[2];
    HAL_StatusTypeDef status;

    if (value == 0)
        return HAL_ERROR;

    status = AS5600_ReadReg(reg, buf, sizeof(buf));
    if (status != HAL_OK)
        return status;

    *value = (uint16_t)((((uint16_t)buf[0] & 0x0FU) << 8U) | buf[1]);
    return HAL_OK;
}

static HAL_StatusTypeDef AS5600_Write12(uint8_t reg, uint16_t value)
{
    uint8_t buf[2];

    value &= AS5600_RAW_MAX;
    buf[0] = (uint8_t)((value >> 8U) & 0x0FU);
    buf[1] = (uint8_t)(value & 0xFFU);

    return AS5600_WriteReg(reg, buf, sizeof(buf));
}

HAL_StatusTypeDef AS5600_Init(void)
{
    return AS5600_IsConnected();
}

HAL_StatusTypeDef AS5600_IsConnected(void)
{
    return bsp_iic_is_ready(AS5600_I2C_ADDR);
}

HAL_StatusTypeDef AS5600_ReadRawAngle(uint16_t *raw_angle)
{
    return AS5600_Read12(AS5600_REG_RAW_ANGLE, raw_angle);
}

HAL_StatusTypeDef AS5600_ReadAngle(uint16_t *angle)
{
    return AS5600_Read12(AS5600_REG_ANGLE, angle);
}

HAL_StatusTypeDef AS5600_GetMechanicalAngle(float *angle_rad)
{
    uint16_t raw_angle;
    HAL_StatusTypeDef status;

    if (angle_rad == 0)
        return HAL_ERROR;

    status = AS5600_ReadRawAngle(&raw_angle);
    if (status != HAL_OK)
        return status;

    *angle_rad = AS5600_RawToRad(raw_angle);
    return HAL_OK;
}

HAL_StatusTypeDef AS5600_GetAngleRad(float *angle_rad)
{
    uint16_t angle;
    HAL_StatusTypeDef status;

    if (angle_rad == 0)
        return HAL_ERROR;

    status = AS5600_ReadAngle(&angle);
    if (status != HAL_OK)
        return status;

    *angle_rad = AS5600_RawToRad(angle);
    return HAL_OK;
}

HAL_StatusTypeDef AS5600_GetAngleDeg(float *angle_deg)
{
    uint16_t angle;
    HAL_StatusTypeDef status;

    if (angle_deg == 0)
        return HAL_ERROR;

    status = AS5600_ReadAngle(&angle);
    if (status != HAL_OK)
        return status;

    *angle_deg = AS5600_RawToDeg(angle);
    return HAL_OK;
}

HAL_StatusTypeDef AS5600_ReadStatus(uint8_t *status_reg)
{
    HAL_StatusTypeDef status;

    if (status_reg == 0)
        return HAL_ERROR;

    status = AS5600_ReadReg(AS5600_REG_STATUS, status_reg, 1U);
    if (status != HAL_OK)
        return status;

    *status_reg &= AS5600_STATUS_MASK;
    return HAL_OK;
}

HAL_StatusTypeDef AS5600_ReadMagnetState(as5600_magnet_state_t *state)
{
    uint8_t status_reg;
    HAL_StatusTypeDef status;

    if (state == 0)
        return HAL_ERROR;

    status = AS5600_ReadStatus(&status_reg);
    if (status != HAL_OK)
        return status;

    if ((status_reg & AS5600_STATUS_MD) == 0U)
        *state = AS5600_MAGNET_NOT_DETECTED;
    else if ((status_reg & AS5600_STATUS_ML) != 0U)
        *state = AS5600_MAGNET_TOO_WEAK;
    else if ((status_reg & AS5600_STATUS_MH) != 0U)
        *state = AS5600_MAGNET_TOO_STRONG;
    else
        *state = AS5600_MAGNET_OK;

    return HAL_OK;
}

HAL_StatusTypeDef AS5600_ReadAGC(uint8_t *agc)
{
    return AS5600_ReadReg(AS5600_REG_AGC, agc, 1U);
}

HAL_StatusTypeDef AS5600_ReadMagnitude(uint16_t *magnitude)
{
    return AS5600_Read12(AS5600_REG_MAGNITUDE, magnitude);
}

HAL_StatusTypeDef AS5600_ReadData(as5600_data_t *data)
{
    HAL_StatusTypeDef status;

    if (data == 0)
        return HAL_ERROR;

    status = AS5600_ReadRawAngle(&data->raw_angle);
    if (status != HAL_OK)
        return status;

    status = AS5600_ReadAngle(&data->angle);
    if (status != HAL_OK)
        return status;

    status = AS5600_ReadMagnitude(&data->magnitude);
    if (status != HAL_OK)
        return status;

    status = AS5600_ReadAGC(&data->agc);
    if (status != HAL_OK)
        return status;

    status = AS5600_ReadStatus(&data->status);
    if (status != HAL_OK)
        return status;

    if ((data->status & AS5600_STATUS_MD) == 0U)
        data->magnet_state = AS5600_MAGNET_NOT_DETECTED;
    else if ((data->status & AS5600_STATUS_ML) != 0U)
        data->magnet_state = AS5600_MAGNET_TOO_WEAK;
    else if ((data->status & AS5600_STATUS_MH) != 0U)
        data->magnet_state = AS5600_MAGNET_TOO_STRONG;
    else
        data->magnet_state = AS5600_MAGNET_OK;

    return HAL_OK;
}

HAL_StatusTypeDef AS5600_SetZeroPosition(uint16_t raw_angle)
{
    return AS5600_Write12(AS5600_REG_ZPOS_H, raw_angle);
}

HAL_StatusTypeDef AS5600_ReadZeroPosition(uint16_t *raw_angle)
{
    return AS5600_Read12(AS5600_REG_ZPOS_H, raw_angle);
}

float AS5600_RawToRad(uint16_t raw_angle)
{
    return (float)(raw_angle & AS5600_RAW_MAX) * (AS5600_TWO_PI / (float)AS5600_CPR);
}

float AS5600_RawToDeg(uint16_t raw_angle)
{
    return (float)(raw_angle & AS5600_RAW_MAX) * (360.0f / (float)AS5600_CPR);
}
