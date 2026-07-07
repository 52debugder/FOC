#ifndef __AS5600_H__
#define __AS5600_H__

#include "stm32f1xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AS5600_I2C_ADDR          0x36U
#define AS5600_CPR               4096U
#define AS5600_RAW_MAX           0x0FFFU
#define AS5600_TWO_PI            6.28318530718f

typedef enum {
    AS5600_MAGNET_NOT_DETECTED = 0,
    AS5600_MAGNET_OK,
    AS5600_MAGNET_TOO_WEAK,
    AS5600_MAGNET_TOO_STRONG
} as5600_magnet_state_t;

typedef struct {
    uint16_t raw_angle;
    uint16_t angle;
    uint16_t magnitude;
    uint8_t agc;
    uint8_t status;
    as5600_magnet_state_t magnet_state;
} as5600_data_t;

HAL_StatusTypeDef AS5600_Init(void);
HAL_StatusTypeDef AS5600_IsConnected(void);
HAL_StatusTypeDef AS5600_ReadRawAngle(uint16_t *raw_angle);
HAL_StatusTypeDef AS5600_ReadAngle(uint16_t *angle);
HAL_StatusTypeDef AS5600_GetMechanicalAngle(float *angle_rad);
HAL_StatusTypeDef AS5600_GetAngleRad(float *angle_rad);
HAL_StatusTypeDef AS5600_GetAngleDeg(float *angle_deg);
HAL_StatusTypeDef AS5600_ReadStatus(uint8_t *status);
HAL_StatusTypeDef AS5600_ReadMagnetState(as5600_magnet_state_t *state);
HAL_StatusTypeDef AS5600_ReadAGC(uint8_t *agc);
HAL_StatusTypeDef AS5600_ReadMagnitude(uint16_t *magnitude);
HAL_StatusTypeDef AS5600_ReadData(as5600_data_t *data);
HAL_StatusTypeDef AS5600_SetZeroPosition(uint16_t raw_angle);
HAL_StatusTypeDef AS5600_ReadZeroPosition(uint16_t *raw_angle);
float AS5600_RawToRad(uint16_t raw_angle);
float AS5600_RawToDeg(uint16_t raw_angle);

#ifdef __cplusplus
}
#endif

#endif
