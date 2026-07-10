/**
 * @file foc_utils.h
 * @author MING
 * @brief 存放派生参数，调用者非必要无需更改
 * @version 0.1
 * @date 2026-03-21
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef __FOC_UTILS_H__
#define __FOC_UTILS_H__ 

#include "foc_config.h"

#ifdef __cplusplus
extern "C" {
#endif


/*-------------------------------------派生参数 不需要更改-----------------------------------*/
#define FOC_TWO_PI_F        6.28318530717958647692f
#define OPEN_MECH_SPEED     (OPEN_LOOP_SPEED_RPM * FOC_TWO_PI_F / 60.0f)
#define OPEN_ELEC_SPEED     (OPEN_MECH_SPEED * POLE_PAIRS)
#define CURRENT_SCALE       (PWM_SCALE / (INA240_GAIN * SAMPLE_RESISTOR * ADC_RESOLUTION))  // 从ad数值转换到实际电流值的转换系数
#define TS_MOTORL           (TS/MOTOR_L)
#define DEAD_COMP_V         (PWM_VBUS * BTN7960_DEAD_TIME_S / TS)
#define PI_LIMIT            (PWM_VBUS * 0.95f / SQRT_3)                                     // 最大不失真电压 
#define SVPWM_K             (SQRT_3 * PWM_ARR / PWM_VBUS)
#define FOC_CURRENT_LIMIT_PU 1.0f
#define FOC_VOLTAGE_LIMIT_PU 1.0f
#define CURRENT_TARGET_LIMIT_PU (CURRENT_TARGET_LIMIT / FOC_CURRENT_BASE_A)
#define FW_ID_MAX_PU        (FW_ID_MAX / FOC_CURRENT_BASE_A)
#define HFI_INJECTION_VOLTAGE_PU (HFI_INJECTION_VOLTAGE / FOC_VOLTAGE_BASE_V)
#define HFI_IQ_HF_LIMIT_PU  (HFI_IQ_HF_LIMIT / FOC_CURRENT_BASE_A)
#define HFI_MIN_RESPONSE_PU (HFI_MIN_RESPONSE / FOC_CURRENT_BASE_A)
#define PI_KP_D_PU          (PI_KP_D * FOC_CURRENT_BASE_A / FOC_VOLTAGE_BASE_V)
#define PI_KI_D_PU          (PI_KI_D * FOC_CURRENT_BASE_A / FOC_VOLTAGE_BASE_V)
#define PI_KP_Q_PU          (PI_KP_Q * FOC_CURRENT_BASE_A / FOC_VOLTAGE_BASE_V)
#define PI_KI_Q_PU          (PI_KI_Q * FOC_CURRENT_BASE_A / FOC_VOLTAGE_BASE_V)

static inline float FOC_CurrentToPu(float current_a)
{
    return current_a / FOC_CURRENT_BASE_A;
}

static inline float FOC_CurrentFromPu(float current_pu)
{
    return current_pu * FOC_CURRENT_BASE_A;
}

static inline float FOC_VoltageToPu(float voltage_v)
{
    return voltage_v / FOC_VOLTAGE_BASE_V;
}

static inline float FOC_VoltageFromPu(float voltage_pu)
{
    return voltage_pu * FOC_VOLTAGE_BASE_V;
}

/**
 * @brief 机械速度（rpm）转电角速度（rad/s）
 * 
 * @param speed_rpm 机械速度（rpm）
 * @return float 电角速度（rad/s）
 */
static inline float FOC_MechRpmToElecRadPerSec(float speed_rpm)
{
    return speed_rpm * FOC_TWO_PI_F * POLE_PAIRS / 60.0f;
}

/**
 * @brief 电角速度（rad/s）转机械速度（rpm）
 *
 * @param omega_elec 电角速度（rad/s）
 * @return float 机械转速（rpm）
 */
static inline float FOC_ElecRadPerSecToMechRpm(float omega_elec)
{
    return omega_elec * 60.0f / (FOC_TWO_PI_F * POLE_PAIRS);
}

/**
 * @brief 角速度（rad/s）转机械速度（rpm）
 * 
 * @param omega_mech 角速度（rad/s）
 * @return float 机械速度（rpm）
 */
static inline float FOC_MechRadPerSecToRpm(float omega_mech)
{
    return omega_mech * 60.0f / FOC_TWO_PI_F;
}

/**
 * @brief 先对电角速度（rad/s）取绝对值再转换为机械速度（rpm）
 * 
 * @param omega_elec 电角速度（rad/s）
 * @return float 机械速度（rpm）
 */
static inline float FOC_AbsElecRadPerSecToMechRpm(float omega_elec)
{
    float omega_abs = (omega_elec >= 0.0f) ? omega_elec : -omega_elec;
    return FOC_ElecRadPerSecToMechRpm(omega_abs);
}

/**
 * @brief 机械角度（rad）转电角度（rad）
 * 
 * @param mech_angle_rad 机械角度（rad）
 * @return float 电角角度（rad）
 */

static inline float FOC_MechAngleToElecAngle(float mech_angle_rad)
{
    return mech_angle_rad * POLE_PAIRS - FOC_ELECTRICAL_ANGLE_OFFSET;
}


#ifdef __cplusplus
}
#endif

#endif

