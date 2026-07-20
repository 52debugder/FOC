/**
 * @file app_motor.c
 * @author 銘
 * @brief 
 * @version 0.1
 * @date 2026-06-11
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "app_motor.h"
#include "app_comm.h"

#define APP_MOTOR_SPEED_LPF_ALPHA 0.05f

static as5600_magnet_state_t app_motor_magnet_state;
static float app_motor_last_angle_rad;
static float app_motor_last_speed_rpm;
static uint8_t app_motor_as5600_status;
static uint8_t app_motor_as5600_agc;
static uint16_t app_motor_as5600_magnitude;
static uint32_t app_motor_last_tick_us;

static float app_motor_wrap_delta_pm_pi(float delta)
{
    while (delta > PI)
        delta -= FOC_TWO_PI_F;
    while (delta < -PI)
        delta += FOC_TWO_PI_F;

    return delta;
}

void app_foc_mainloop(void)
{
    static uint32_t last_as5600_tick = 0;
    uint32_t now = App_GetMicroseconds();
    uint32_t elapsed_us = now - last_as5600_tick;

    app_motor_last_tick_us = now;

    if(elapsed_us >= 1000U)
    {
        float dt = (float)elapsed_us * 1e-6f;
        last_as5600_tick = now;
        foc_handle_t *foc_motor = Foc_GetStruct(1);
        if(foc_motor->init_done)
        {
            as5600_data_t as5600_data;
            float angle_rad;

            if (AS5600_ReadData(&as5600_data) != HAL_OK)
            {
                __disable_irq();
                foc_motor->sensor_mech.speed = 0.0f;
                foc_motor->sensor_mech.vaild = 0.0f;
                __enable_irq();
                return;
            }

            angle_rad = AS5600_RawToRad(as5600_data.raw_angle);

            if ((as5600_data.magnet_state == AS5600_MAGNET_NOT_DETECTED) ||
                (as5600_data.magnet_state == AS5600_MAGNET_TOO_STRONG))
            {
                __disable_irq();
                app_motor_as5600_status = as5600_data.status;
                app_motor_as5600_agc = as5600_data.agc;
                app_motor_as5600_magnitude = as5600_data.magnitude;
                app_motor_magnet_state = as5600_data.magnet_state;
                foc_motor->sensor_mech.speed = 0.0f;
                foc_motor->sensor_mech.vaild = 0.0f;
                __enable_irq();
                return;
            }

            __disable_irq();
            app_motor_as5600_status = as5600_data.status;
            app_motor_as5600_agc = as5600_data.agc;
            app_motor_as5600_magnitude = as5600_data.magnitude;
            app_motor_magnet_state = as5600_data.magnet_state;
            app_motor_last_angle_rad = angle_rad;

            if (foc_motor->sensor_mech.vaild != 0.0f)
            {
                float delta = app_motor_wrap_delta_pm_pi(angle_rad - foc_motor->sensor_mech.angle);
                float speed_raw = -FOC_MechRadPerSecToRpm(delta / dt);
                app_motor_last_speed_rpm = foc_motor->sensor_mech.speed + APP_MOTOR_SPEED_LPF_ALPHA * (speed_raw - foc_motor->sensor_mech.speed);
                foc_motor->position_raw += delta;
            }
            else
            {
                app_motor_last_speed_rpm = 0.0f;
            }

            foc_motor->sensor_mech.angle = angle_rad;
            foc_motor->sensor_mech.speed = app_motor_last_speed_rpm;
            foc_motor->position = foc_motor->position_raw - foc_motor->position_offset;
            foc_motor->sensor_mech.sample_seq++;
            foc_motor->sensor_mech.vaild = 1.0f;
            __enable_irq();

            Foc_Update_SpeedLoop(1, dt);
        }
    }

}

void app_motor_get_telemetry(uint8_t motor_num, app_motor_telemetry_t *telemetry)
{
    foc_handle_t *motor = Foc_GetStruct(motor_num);

    if (telemetry == NULL)
        return;

    __disable_irq();
    telemetry->target_speed_rpm = motor->speed_ramp_target;
    telemetry->measured_speed_rpm = motor->sensor_mech.speed;
    telemetry->mechanical_angle_rad = app_motor_last_angle_rad;
    telemetry->sensor_valid = motor->sensor_mech.vaild;
    telemetry->sample_time_us = app_motor_last_tick_us;
    telemetry->as5600_status = app_motor_as5600_status;
    telemetry->as5600_agc = app_motor_as5600_agc;
    telemetry->as5600_magnitude = app_motor_as5600_magnitude;
    telemetry->magnet_state = app_motor_magnet_state;
    __enable_irq();
}

uint32_t App_GetMicroseconds(void)
{
   uint32_t ms = HAL_GetTick();
   uint32_t ticks = SysTick->LOAD + 1 - SysTick->VAL;
   return (ms * 1000 + (ticks * 1000) / SysTick->LOAD);
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1)
    {
        foc_handle_t *foc_motor = Foc_GetStruct(1);

        GPIOB->BSRR = GPIO_PIN_0;

        if (foc_motor->init_done == 1U)
            foc_motor->hal.adc_get_value(1, &foc_motor->i_adc_u, &foc_motor->i_adc_v, &foc_motor->i_adc_w);

        __HAL_ADC_ENABLE_IT(hadc, ADC_IT_JEOC); // HAL在JEOC回调后自动关闭中断使能，样本锁存后立即重新打开

        foc_state_t foc_state = Foc_Loop(1);
        if(foc_state == FOC_ERR_NOT_INIT)
        {
            GPIOB->BSRR = GPIO_PIN_1;
        }
        else
        {
            app_debug_sample_from_isr();
        }

        GPIOB->BSRR = GPIO_PIN_0 << 16;
    }
}




