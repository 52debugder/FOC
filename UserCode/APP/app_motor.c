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

#define APP_MOTOR_SAMPLE_PERIOD_US      1000U   // AS5600角度采样周期，1ms更新一次速度环
#define APP_MOTOR_STATUS_PERIOD_US      20000U  // AS5600磁场/AGC/幅值状态刷新周期，避免每拍多次I2C读取
#define APP_MOTOR_LOW_SPEED_RPM         250.0f  // 低于该速度时使用累积窗口测速，降低编码器量化抖动
#define APP_MOTOR_LOW_SPEED_MIN_DT_S    0.004f  // 低速测速最小累积时间，4ms内的角度差一起计算速度
#define APP_MOTOR_SPEED_LPF_TAU_S       0.020f  // 速度一阶低通时间常数，越大越平滑但响应越慢

static as5600_magnet_state_t app_motor_magnet_state; // 最近一次AS5600磁场状态
static float app_motor_last_angle_rad;               // 最近一次机械角度，单位rad
static float app_motor_last_speed_rpm;               // 滤波后的机械速度反馈，单位rpm
static float app_motor_speed_accum_delta_rad;        // 低速窗口累计机械角度变化，单位rad
static float app_motor_speed_accum_dt_s;             // 低速窗口累计采样时间，单位s
static uint8_t app_motor_as5600_status;              // 最近一次AS5600状态寄存器
static uint8_t app_motor_as5600_agc;                 // 最近一次AS5600自动增益值
static uint16_t app_motor_as5600_magnitude;          // 最近一次AS5600磁场幅值
static uint32_t app_motor_last_tick_us;              // 最近一次进入电机主循环的时间戳，单位us
static uint32_t app_motor_last_status_tick_us;       // 最近一次刷新AS5600状态信息的时间戳，单位us

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

    if(elapsed_us >= APP_MOTOR_SAMPLE_PERIOD_US)
    {
        float dt = (float)elapsed_us * 1e-6f;
        uint16_t raw_angle;
        last_as5600_tick = now;
        foc_handle_t *foc_motor = Foc_GetStruct(1);
        if(foc_motor->init_done)
        {
            float angle_rad;

            if (AS5600_ReadRawAngle(&raw_angle) != HAL_OK)
            {
                __disable_irq();
                foc_motor->sensor_mech.speed = 0.0f;
                foc_motor->sensor_mech.vaild = 0.0f;
                app_motor_speed_accum_delta_rad = 0.0f;
                app_motor_speed_accum_dt_s = 0.0f;
                __enable_irq();
                return;
            }

            if ((uint32_t)(now - app_motor_last_status_tick_us) >= APP_MOTOR_STATUS_PERIOD_US)
            {
                as5600_data_t as5600_data;
                if (AS5600_ReadData(&as5600_data) != HAL_OK)
                {
                    __disable_irq();
                    foc_motor->sensor_mech.speed = 0.0f;
                    foc_motor->sensor_mech.vaild = 0.0f;
                    app_motor_speed_accum_delta_rad = 0.0f;
                    app_motor_speed_accum_dt_s = 0.0f;
                    __enable_irq();
                    return;
                }

                app_motor_as5600_status = as5600_data.status;
                app_motor_as5600_agc = as5600_data.agc;
                app_motor_as5600_magnitude = as5600_data.magnitude;
                app_motor_magnet_state = as5600_data.magnet_state;
                app_motor_last_status_tick_us = now;

                if ((as5600_data.magnet_state == AS5600_MAGNET_NOT_DETECTED) ||
                    (as5600_data.magnet_state == AS5600_MAGNET_TOO_STRONG))
                {
                    __disable_irq();
                    foc_motor->sensor_mech.speed = 0.0f;
                    foc_motor->sensor_mech.vaild = 0.0f;
                    app_motor_speed_accum_delta_rad = 0.0f;
                    app_motor_speed_accum_dt_s = 0.0f;
                    __enable_irq();
                    return;
                }
            }

            angle_rad = AS5600_RawToRad(raw_angle);

            __disable_irq();
            app_motor_last_angle_rad = angle_rad;

            if (foc_motor->sensor_mech.vaild != 0.0f)
            {
                float delta = app_motor_wrap_delta_pm_pi(angle_rad - foc_motor->sensor_mech.angle);
                float speed_abs = (app_motor_last_speed_rpm >= 0.0f) ? app_motor_last_speed_rpm : -app_motor_last_speed_rpm;
                foc_motor->position_raw += delta;
                app_motor_speed_accum_delta_rad += delta;
                app_motor_speed_accum_dt_s += dt;

                if (speed_abs >= APP_MOTOR_LOW_SPEED_RPM ||
                    app_motor_speed_accum_dt_s >= APP_MOTOR_LOW_SPEED_MIN_DT_S)
                {
                    float speed_raw = -FOC_MechRadPerSecToRpm(app_motor_speed_accum_delta_rad / app_motor_speed_accum_dt_s);
                    float alpha = app_motor_speed_accum_dt_s / (APP_MOTOR_SPEED_LPF_TAU_S + app_motor_speed_accum_dt_s);
                    app_motor_last_speed_rpm += alpha * (speed_raw - app_motor_last_speed_rpm);
                    app_motor_speed_accum_delta_rad = 0.0f;
                    app_motor_speed_accum_dt_s = 0.0f;
                }
            }
            else
            {
                app_motor_last_speed_rpm = 0.0f;
                app_motor_speed_accum_delta_rad = 0.0f;
                app_motor_speed_accum_dt_s = 0.0f;
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
   uint32_t ms1;
   uint32_t ms2;
   uint32_t val;
   uint32_t load = SysTick->LOAD + 1U;

   do
   {
       ms1 = HAL_GetTick();
       val = SysTick->VAL;
       ms2 = HAL_GetTick();
   } while (ms1 != ms2);

   return (ms1 * 1000U) + (((load - val) * 1000U) / load);
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




