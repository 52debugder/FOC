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

#define APP_MOTOR_PI_F 3.14159265358979323846f
#define APP_MOTOR_TWO_PI_F 6.28318530717958647692f
#define APP_MOTOR_SPEED_LPF_ALPHA 0.05f

as5600_magnet_state_t magnet_state;
float angle;
float speed;
uint8_t as5600_status;
uint8_t as5600_agc;
uint16_t as5600_magnitude;

uint32_t count;

static float app_motor_wrap_delta_pm_pi(float delta)
{
    while (delta > APP_MOTOR_PI_F)
        delta -= APP_MOTOR_TWO_PI_F;
    while (delta < -APP_MOTOR_PI_F)
        delta += APP_MOTOR_TWO_PI_F;

    return delta;
}

void app_foc_init(uint8_t motor_num, const foc_hal_t *hal_interface)
{
    foc_handle_t *FOC_Motor = Foc_GetStruct(motor_num);
    FOC_Motor->num = motor_num;
    Foc_ParamInit(FOC_Motor, hal_interface); // FOC初始化

    FOC_Motor->hal.pwm_start(motor_num);
    FOC_Motor_Cali_Offset(FOC_Motor);        // 电机零点校准（上电静止时执行）
    FOC_Motor->hal.drv_enable(motor_num);            // 驱动使能
    FOC_Motor->mode = MOTOR_STATE_IDLE;
    FOC_Motor->state_timer = 0;
    FOC_Motor->init_done = 1;
    Foc_SetStruct(FOC_Motor, motor_num);
}

void app_foc_mainloop(void)
{
    static uint32_t last_as5600_tick = 0;
    uint32_t now = App_GetMicroseconds();
    count = now;
    uint32_t elapsed_us = now - last_as5600_tick;

    if(elapsed_us >= 1000U)
    {
        last_as5600_tick = now;
        foc_handle_t *foc_motor =  Foc_GetStruct(1);
        if(foc_motor->init_done)
        {
            as5600_data_t as5600_data;

            if (AS5600_ReadData(&as5600_data) != HAL_OK)
            {
                __disable_irq();
                foc_motor->sensor_mech.speed = 0.0f;
                foc_motor->sensor_mech.vaild = 0.0f;
                __enable_irq();
                return;
            }

            angle = AS5600_RawToRad(as5600_data.raw_angle);

            __disable_irq();
            as5600_status = as5600_data.status;
            as5600_agc = as5600_data.agc;
            as5600_magnitude = as5600_data.magnitude;
            magnet_state = as5600_data.magnet_state;
            __enable_irq();

            if ((magnet_state == AS5600_MAGNET_NOT_DETECTED) ||
                (magnet_state == AS5600_MAGNET_TOO_STRONG))
            {
                __disable_irq();
                foc_motor->sensor_mech.speed = 0.0f;
                foc_motor->sensor_mech.vaild = 0.0f;
                __enable_irq();
                return;
            }

            // if (foc_motor->sensor_mech.vaild != 0.0f)
            // {
            //     float delta = app_motor_wrap_delta_pm_pi(angle - foc_motor->sensor_mech.angle);
            //     float dt = (float)elapsed_us * 0.001f;
            //     float speed_raw = delta * (60.0f / (APP_MOTOR_TWO_PI_F * dt));
            //     speed = foc_motor->sensor_mech.speed + APP_MOTOR_SPEED_LPF_ALPHA * (speed_raw - foc_motor->sensor_mech.speed);
            // }

            float speed_new = 0.0f;
            if (foc_motor->sensor_mech.vaild != 0.0f)
            {
                float delta = app_motor_wrap_delta_pm_pi(angle - foc_motor->sensor_mech.angle);
                float dt = (float)elapsed_us * 1e-6f;
                float speed_raw = -delta * (60.0f / (APP_MOTOR_TWO_PI_F * dt));
                speed_new = foc_motor->sensor_mech.speed + APP_MOTOR_SPEED_LPF_ALPHA * (speed_raw - foc_motor->sensor_mech.speed);
            }

            speed = speed_new;

            __disable_irq();
            foc_motor->sensor_mech.angle = angle;
            foc_motor->sensor_mech.speed = speed;
            foc_motor->sensor_mech.sample_seq++;
            foc_motor->sensor_mech.vaild = 1.0f;
            __enable_irq();

            Foc_Update_SpeedLoop(1, (float)elapsed_us * 1e-6f);
        }
    }

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
        // static uint16_t isr_meas_cnt = 0;
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
            // app_debug_sample_from_isr();
        }

        GPIOB->BSRR = GPIO_PIN_0 << 16;

        // ISR频率测量：每200次Foc_Loop翻转PB1
        // 示波器测PB1频率 → ISR频率 = PB1频率 × 400
        // isr_meas_cnt++;
        // if (isr_meas_cnt >= 200) {
        //     isr_meas_cnt = 0;
        //     HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
        // }
    }
}




