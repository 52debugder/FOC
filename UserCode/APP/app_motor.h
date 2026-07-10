#ifndef __APP_MOTOR_H__
#define __APP_MOTOR_H__ 

#include "stm32f1xx_hal.h"
#include "foc.h"
#include "as5600.h"

typedef struct
{
    float target_speed_rpm;
    float measured_speed_rpm;
    float mechanical_angle_rad;
    float sensor_valid;
    uint32_t sample_time_us;
    uint8_t as5600_status;
    uint8_t as5600_agc;
    uint16_t as5600_magnitude;
    as5600_magnet_state_t magnet_state;
} app_motor_telemetry_t;

void app_foc_mainloop(void);
void app_motor_get_telemetry(uint8_t motor_num, app_motor_telemetry_t *telemetry);
uint32_t App_GetMicroseconds(void);

#endif
