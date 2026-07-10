/**
 * @file foc.c
 * @author MING
 * @brief foc调用库
 * @version 1.0
 * @date 2026-07-10
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "foc.h"

static foc_handle_t FOC_Motor[MAX_MOTOR_NUM + 1] = {0};
#if defined(FOC_OPEN_I_DEBUG_EN) || defined(FOC_CLOSE_I_DEBUG_EN)
static uint32_t vofa_cnt = 0;
#endif

#define FOC_PI_F 3.14159265358979323846f
#define FOC_ALIGN_STABLE_DELTA_RAD 0.01f
#define FOC_ALIGN_STABLE_COUNT 200U

static float foc_wrap_angle_0_2pi(float angle)
{
    if (angle >= FOC_TWO_PI_F)
        angle -= FOC_TWO_PI_F;
    else if (angle < 0.0f)
        angle += FOC_TWO_PI_F;

    return angle;
}

static float foc_wrap_delta_pm_pi(float delta)
{
    if (delta > FOC_PI_F)
        delta -= FOC_TWO_PI_F;
    else if (delta < -FOC_PI_F)
        delta += FOC_TWO_PI_F;

    return delta;
}

static foc_state_t Foc_Preprocess_CurrentSample(foc_handle_t *motor)
{
    foc_state_t foc_state = FOC_OK;

    motor->i_uvw.u = (float)((int32_t)motor->i_adc_u - motor->i_cali_uvw.u) * CURRENT_SCALE;
    motor->i_uvw.v = 0.0f;
    motor->i_uvw.w = (float)((int32_t)motor->i_adc_w - motor->i_cali_uvw.w) * CURRENT_SCALE;

    if (motor->i_uvw.u >= CURRENT_LIMIT)
    {
        motor->i_uvw.u = CURRENT_LIMIT;
        foc_state = FOC_ERR_OVERCURRENT;
    }
    else if (motor->i_uvw.u <= -CURRENT_LIMIT)
    {
        motor->i_uvw.u = -CURRENT_LIMIT;
        foc_state = FOC_ERR_OVERCURRENT;
    }

    if (motor->i_uvw.w >= CURRENT_LIMIT)
    {
        motor->i_uvw.w = CURRENT_LIMIT;
        foc_state = FOC_ERR_OVERCURRENT;
    }
    else if (motor->i_uvw.w <= -CURRENT_LIMIT)
    {
        motor->i_uvw.w = -CURRENT_LIMIT;
        foc_state = FOC_ERR_OVERCURRENT;
    }

    return foc_state;
}

static void Foc_SetElectricalAngle(foc_handle_t *motor, float theta)
{
    motor->theta = foc_wrap_angle_0_2pi(theta);
}

static void Foc_RefreshTrigCache(foc_handle_t *motor)
{
    FOC_Updata_Trig(motor);
}

static void Foc_AdvanceOpenLoopAngle(foc_handle_t *motor, float dt)
{
    float theta = motor->theta;

    if (motor->target_speed > 0.0f)
        theta += OPEN_ELEC_SPEED * dt;
    else if (motor->target_speed < 0.0f)
        theta -= OPEN_ELEC_SPEED * dt;

    Foc_SetElectricalAngle(motor, theta);
}

static uint8_t Foc_LoadClosedLoopSensorAngle(foc_handle_t *motor)
{
    if ((motor->trig_sample_valid == 0U) || (motor->sensor_mech.sample_seq != motor->trig_sample_seq))
    {
        float mech_angle = foc_wrap_angle_0_2pi(motor->sensor_mech.angle - motor->sensor_mech.zero_offset);

        Foc_SetElectricalAngle(motor, FOC_MechAngleToElecAngle(mech_angle));
        motor->speed = motor->sensor_mech.speed;
        motor->trig_sample_seq = motor->sensor_mech.sample_seq;
        motor->trig_sample_valid = 1U;
        return 1U;
    }

    return 0U;
}

/**
 * @brief foc初始化
 * 
 * @param motor 
 * @return foc_state_t 
 */
foc_state_t Foc_Init(uint8_t motor_num, const foc_hal_t *hal_interface)
{
    foc_handle_t *motor = &FOC_Motor[motor_num];
    motor->num = motor_num;
    Foc_ParamInit(motor, hal_interface); // FOC初始化

    motor->hal.pwm_start(motor_num);
    FOC_Motor_Cali_Offset(motor);        // 电机零点校准（上电静止时执行）
    motor->hal.drv_enable(motor_num);            // 驱动使能
    motor->mode = MOTOR_STATE_IDLE;
    motor->state_timer = 0;
    motor->init_done = 1;

    return FOC_OK;
}


/**
 * @brief foc参数初始化
 * 
 * @param motor 
 * @return foc_state_t 
 */
foc_state_t Foc_ParamInit(foc_handle_t *motor, const foc_hal_t *hal_interface)
{
    // 绑定定时器
    // motor->htim = htim;
    motor->hal = *hal_interface; // 绑定HAL
    motor->hal.init(motor->num);
    // d轴电流环参数初始化
    motor->pi_d = (foc_pid_t){
        .kp = PI_KP_D,
        .ki = PI_KI_D,
        .limit = PI_LIMIT,
        .target = 0.0f};
        
    // q轴电流环参数初始化
    motor->pi_q = (foc_pid_t){
        .kp = PI_KP_Q,
        .ki = PI_KI_Q,
        .limit = PI_LIMIT,
        .target = 0.0f};

    // 速度PI参数初始化
    motor->pi_speed = (foc_pid_t){
        .kp = PI_KP_SPEED,
        .ki = PI_KI_SPEED,
        .limit = PI_LIMIT_SPEED,
        .target = 0,
        .integral = 0.0f};

    // 位置PI参数初始化
    motor->pi_position = (foc_pid_t){
        .kp = PI_KP_POSITION,
        .ki = PI_KI_POSITION,
        .limit = PI_LIMIT_POSITION_RPM,
        .target = 0.0f,
        .integral = 0.0f};

    // 初始占空比
    motor->pwm = (foc_pwm_t){
        .duty_u = PWM_ARR / 2,
        .duty_v = PWM_ARR / 2,
        .duty_w = PWM_ARR / 2,};

    // 上一次ab轴电流
    motor->i_ab_pre = (foc_ab_t){
        .alpha = 0.0f,
        .beta = 0.0f};

    motor->e_ab = (foc_ab_t){
        .alpha = 0.0f,
        .beta = 0.0f};

    motor->i_cali_uvw = (foc_uvw_t){
        .u = 0.0f,
        .v = 0.0f,
        .w = 0.0f,
    };

    motor->sensor_mech = (foc_sensor_mech_t){
        .angle = 0.0f,
        .speed = 0.0f,
        .vaild = 0.0f,
        .dir = 0U,
        .zero_offset = 0.0f,
        .sample_seq = 0U,
        .align_prev_sample_seq = 0U,
        .align_prev_angle = 0.0f,
        .align_stable_count = 0U,
        .align_has_prev = 0U,
        .zero_offset_locked = 0U,
    };

    motor->pi_pll.integral = 0.0f;
    motor->speed_observer = 0.0f;
    motor->theta_Observer = 0.0f;
    motor->theta_obs_prev = 0.0f;

    motor->theta = 0.0f;
    motor->sin_theta = 0.0f;
    motor->cos_theta = 1.0f;
    motor->trig_sample_seq = 0U;
    motor->trig_sample_valid = 0U;

    motor->target_speed = 0.0f;
    motor->speed_ramp_target = 0.0f;
    motor->speed_sign = 1.0;
    motor->id_fw = 0.0f;
    motor->fw_active = 0.0f;
    motor->fw_voltage = 0.0f;

    motor->control_mode = FOC_CONTROL_SPEED;

    motor->target_position = 0.0f;
    motor->position_raw = 0.0f;
    motor->position = 0.0f;
    motor->position_offset = 0.0f;
    motor->position_dir = 0.0f;
#ifdef HFI_ENABLE
    HFI_Init(motor);
#endif
    motor->state_timer = 0;
    motor->state = FOC_OK;
    return FOC_OK;
}

/**
 * @brief FOC失能初始化
 * 
 * @param motor 
 * @return foc_state_t 
 */
foc_state_t Foc_Deinit(foc_handle_t *motor)
{
    return FOC_OK;
}

// float speed_diff;
/**
 * @brief foc主循环，用于在定时器中调用，采用了分段式处理，开环强拉到闭环状态
 * 
 * @param motor 
 * @return foc_state_t 
 */
foc_state_t Foc_Loop(uint8_t motor_num)
{
    foc_handle_t *motor = &FOC_Motor[motor_num];

    if (motor->init_done != 1)
        return FOC_ERR_NOT_INIT; // 初始化未完成，直接返回

    switch (motor->mode)
    {
    case MOTOR_STATE_IDLE: // 空状态
        // 每个控制周期都刷新为零电压，防止PWM寄存器残留
        motor->hal.pwm_set_duty(motor->num,
            PWM_ARR / 2, PWM_ARR / 2, PWM_ARR / 2);

        if (!Foc_Safe_Protect(motor->target_speed))
        {
            motor->theta             = 0.0f;
            motor->pi_pll.integral   = 0.0f;
            motor->pi_d.integral        = 0.0f;
            motor->pi_q.integral        = 0.0f;
            motor->pi_speed.integral    = 0.0f;
            motor->pi_position.integral = 0.0f;
            motor->theta_obs_prev       = 0.0f;
            motor->id_fw                = 0.0f;
            motor->sensor_mech.speed = 0.0f;
            motor->sensor_mech.zero_offset = 0.0f;
            motor->sensor_mech.align_prev_sample_seq = motor->sensor_mech.sample_seq;
            motor->sensor_mech.align_prev_angle = 0.0f;
            motor->sensor_mech.align_stable_count = 0U;
            motor->sensor_mech.align_has_prev = 0U;
            motor->sensor_mech.zero_offset_locked = 0U;
            motor->trig_sample_valid = 0U;

            if (motor->control_mode == FOC_CONTROL_POSITION)
            {
                motor->position_raw = 0.0f;
                motor->position_offset = 0.0f;
                motor->position = 0.0f;
                motor->position_dir = 0.0f;
            }
            motor->state_timer          = 0;
            motor->hal.drv_enable(motor_num);
            motor->mode = MOTOR_STATE_ALIGN;
        }
        break;

    case MOTOR_STATE_ALIGN:
        Foc_Align_Loop(motor, TS);

        // ALIGN 期间若遥控器归零 → 回 IDLE
        if (Foc_Safe_Protect(motor->target_speed))
        {
            motor->state_timer = 0;
            motor->mode = MOTOR_STATE_IDLE;
            break;
        }

        motor->state_timer++;

#ifdef FOC_SENSOR_EN
        if (motor->state_timer > 4000 && motor->sensor_mech.vaild != 0.0f && motor->sensor_mech.zero_offset_locked != 0U)
#else
        if(motor->state_timer > 4000)
#endif

        {
            motor->state_timer = 0;

#ifdef HFI_ENABLE // 使能高频注入
            if (motor->hfi_enable && motor->control_mode == FOC_CONTROL_POSITION)
            {
                motor->theta = 0.0f;
                motor->theta_Observer = 0.0f;
                motor->theta_obs_prev = 0.0f;
                motor->position_raw = 0.0f;
                motor->position_offset = 0.0f;
                motor->position = 0.0f;
                motor->speed_ramp_target = 0.0f;
                motor->pi_speed.integral = 0.0f;
                motor->pi_position.integral = 0.0f;
                HFI_Reset_Angle(motor, 0.0f);
                motor->PI_Speed_cnt = 0;
                motor->close_cnt = 0;
                motor->trig_sample_valid = 0U;
                motor->mode = MOTOR_STATE_CLOSE;
                break;
            }
#endif
            motor->pi_pll.integral = motor->target_speed > 0 ? OPEN_ELEC_SPEED : -OPEN_ELEC_SPEED;

#ifdef FOC_SENSOR_EN
#ifdef FOC_CLOSE_LOOP_EN
            motor->mode = MOTOR_STATE_CLOSE;
#endif
#else
            motor->mode = MOTOR_STATE_OPEN;
#endif
        }
        break;

    case MOTOR_STATE_OPEN: // 开环启动：按设定转速匀速旋转
        Foc_Open_Loop(motor, TS);

        // ALIGN 期间若遥控器归零 → 回 IDLE
        if (Foc_Safe_Protect(motor->target_speed))
        {
            motor->hal.pwm_set_duty(motor->num,
                PWM_ARR / 2, PWM_ARR / 2, PWM_ARR / 2);
            motor->e_ab.alpha     = 0.0f;
            motor->e_ab.beta      = 0.0f;
            motor->i_ab_hat.alpha = 0.0f;
            motor->i_ab_hat.beta  = 0.0f;
            motor->speed_observer = 0.0f;
            motor->speed_sign     = 1.0f;
            motor->state_timer    = 0;
            motor->mode = MOTOR_STATE_IDLE;
            break;
        }

        motor->state_timer++;

#ifdef FOC_CLOSE_LOOP_EN // 闭环使能
        {
            // 观测速度与开环速度接近才切换
            float speed_rpm = FOC_ElecRadPerSecToMechRpm(motor->speed_observer);
            float speed_diff = fabsf(fabsf(speed_rpm) - fabsf(OPEN_LOOP_SPEED_RPM));
            if (motor->state_timer > 8500 && speed_diff < OPEN_LOOP_SPEED_RPM * 0.1f && fabs(angle_error) < 0.1f)
            {
                motor->pi_pll.integral = motor->target_speed > 0 ? fabsf(motor->speed_observer) : -fabsf(motor->speed_observer);
                motor->pi_d.integral = 0.0f;
                motor->pi_d.output = 0.0f;

                motor->pi_speed.integral = 0.0f; // 初始驱动力
                motor->pi_speed.output = 6.7f;

                motor->pi_q.integral = PWM_VBUS * 0.5f; // 给个初始积分，约2.4V
                motor->pi_q.output = PWM_VBUS * 0.55f;

                if(motor->target_speed > 0)
                    motor->speed_ramp_target = FOC_AbsElecRadPerSecToMechRpm(motor->speed_observer) + 50.0f;
                else
                    motor->speed_ramp_target = -FOC_AbsElecRadPerSecToMechRpm(motor->speed_observer) + 50.0f;

                motor->theta_Observer = motor->theta;
                motor->theta_obs_prev = motor->theta;
                motor->pi_position.integral = 0.0f;
#ifdef HFI_ENABLE
                if (motor->hfi_enable)
                    HFI_Reset_Angle(motor, motor->theta);
#endif
                motor->PI_Speed_cnt = 0;
                motor->close_cnt = 0;
                motor->trig_sample_valid = 0U;

                motor->mode = MOTOR_STATE_CLOSE;
            }
        }
#endif

        break;

    case MOTOR_STATE_CLOSE:
        // 3. 闭环运行
#ifndef FOC_SENSOR_EN
        // 闭环期间若遥控器归零 → 回 IDLE
        if (motor->control_mode == FOC_CONTROL_SPEED && Foc_Safe_Protect(motor->target_speed))
        {
            // 先归零PWM再切状态
            motor->hal.pwm_set_duty(motor->num,
                PWM_ARR / 2, PWM_ARR / 2, PWM_ARR / 2);
            // 同时清空SMO残留状态，防止下次重启时观测器从错误状态收敛
            motor->e_ab.alpha     = 0.0f;
            motor->e_ab.beta      = 0.0f;
            motor->i_ab_hat.alpha = 0.0f;
            motor->i_ab_hat.beta  = 0.0f;
            motor->speed_observer = 0.0f;
            motor->speed_sign     = 1.0f;
            motor->mode = MOTOR_STATE_IDLE;
            break;
        }
#endif

        // 判断速度模式下，目标速度与运行速度的符号一致
        if (motor->control_mode == FOC_CONTROL_SPEED)
        {
            float run_sign    = (motor->speed_ramp_target >= 0.0f) ? 1.0f : -1.0f;
            float target_sign = (motor->target_speed      >= 0.0f) ? 1.0f : -1.0f;
            if (run_sign != target_sign)
            {
                // 关闭PWM输出，等惯性停下
                motor->hal.pwm_set_duty(motor->num, PWM_ARR/2, PWM_ARR/2, PWM_ARR/2);
                motor->mode = MOTOR_STATE_IDLE;
                break;
            }
        }
        Foc_Close_Loop(motor, TS);
        break;
    }
    return FOC_OK;
}

uint8_t Foc_Safe_Protect(float speed)
{
    uint8_t safe_flag = 0; // 安全保护标志,0:正常 1:异常

#ifdef FOC_SENSOR_EN
    safe_flag = 0;
#else
    safe_flag = fabsf(speed) >= SPEED_START_THRESHOLD;
#endif
    return safe_flag;
}

/**
 * @brief 轴对齐
 * 
 * @param motor 
 * @param dt 
 * @return foc_state_t 
 */
foc_state_t Foc_Align_Loop(foc_handle_t *motor, float dt)
{
    // 1. 定位阶段：给 D 轴施加固定电压，Q 轴为 0，强制转子对齐到 0 度
    motor->u_dq.d = 2.0f; // 2V 左右，根据电机阻抗微调
    motor->u_dq.q = 0.0f;
    Foc_SetElectricalAngle(motor, 0.0f);
    motor->pi_pll.integral = 0;
    motor->theta_Observer = 0.0f;
    motor->theta_obs_prev = 0.0f;

    if (motor->sensor_mech.vaild != 0.0f)
    {
        if (motor->sensor_mech.sample_seq != motor->sensor_mech.align_prev_sample_seq)
        {
            float angle = motor->sensor_mech.angle;
            motor->sensor_mech.align_prev_sample_seq = motor->sensor_mech.sample_seq;

            if (motor->sensor_mech.align_has_prev == 0U)
            {
                motor->sensor_mech.align_prev_angle = angle;
                motor->sensor_mech.align_stable_count = 0U;
                motor->sensor_mech.align_has_prev = 1U;
            }
            else
            {
                float delta = foc_wrap_delta_pm_pi(angle - motor->sensor_mech.align_prev_angle);
                motor->sensor_mech.align_prev_angle = angle;

                if (fabsf(delta) <= FOC_ALIGN_STABLE_DELTA_RAD)
                {
                    if (motor->sensor_mech.align_stable_count < FOC_ALIGN_STABLE_COUNT)
                        motor->sensor_mech.align_stable_count++;
                }
                else
                {
                    motor->sensor_mech.align_stable_count = 0U;
                }
            }

            if ((motor->sensor_mech.zero_offset_locked == 0U) &&
                (motor->sensor_mech.align_stable_count >= FOC_ALIGN_STABLE_COUNT))
            {
                motor->sensor_mech.zero_offset = angle;
                motor->sensor_mech.zero_offset_locked = 1U;
            }
        }
    }
    else
    {
        motor->sensor_mech.align_stable_count = 0U;
        motor->sensor_mech.align_has_prev = 0U;
    }
    
    Foc_RefreshTrigCache(motor);
    FOC_InvPark_Transform(motor);
    FOC_SVPWM_Generate(motor);
    return FOC_OK;
}

/**
 * @brief 开环控制
 * 
 * @param motor 
 * @param dt 
 * @return foc_state_t 
 */
foc_state_t Foc_Open_Loop(foc_handle_t *motor, float dt)
{
    foc_state_t foc_state = Foc_Preprocess_CurrentSample(motor);

    // 3. Clark变换，
    FOC_Clark_Transform(motor);

#ifdef FOC_SMO_EN // 观测器使能
    // 4. smo观测器推算转子位置，得到电角度和转速
    SMO_Observer(motor, dt, MOTOR_STATE_OPEN);
#endif

#ifdef FOC_OPEN_I_DEBUG_EN // 开环电流环调试代码使能
    // 固定角度为0，不依赖观测器
    // Park变换
    motor->i_dq.d = motor->i_ab.alpha;
    motor->i_dq.q = motor->i_ab.beta;

    // d轴电流阶跃信号
    vofa_cnt++;
    if (vofa_cnt >= 34000)
        vofa_cnt = 0;

    if (vofa_cnt >= 17000)
        motor->pi_d.target = CURRENT_LOOP_STEP_HIGH_A;
    else
        motor->pi_d.target = CURRENT_LOOP_STEP_LOW_A;
    motor->pi_q.target = 0.0f;

    const float current_target_limit_sq = CURRENT_TARGET_LIMIT * CURRENT_TARGET_LIMIT;
    float id_target = motor->pi_d.target;
    float iq_target_limit;

    if (id_target > CURRENT_TARGET_LIMIT)
        id_target = CURRENT_TARGET_LIMIT;
    else if (id_target < -CURRENT_TARGET_LIMIT)
        id_target = -CURRENT_TARGET_LIMIT;

    if (id_target == 0.0f)
    {
        iq_target_limit = CURRENT_TARGET_LIMIT;
    }
    else
    {
        float iq_limit_sq = current_target_limit_sq - id_target * id_target;
        if (iq_limit_sq < 0.0f)
            iq_limit_sq = 0.0f;
        iq_target_limit = sqrtf(iq_limit_sq);
    }

    motor->pi_d.target = id_target;
    if (motor->pi_q.target > iq_target_limit)
        motor->pi_q.target = iq_target_limit;
    else if (motor->pi_q.target < -iq_target_limit)
        motor->pi_q.target = -iq_target_limit;

    motor->pi_d.feedback = motor->i_dq.d;
    motor->pi_q.feedback = motor->i_dq.q;
    motor->pi_d.limit = PI_LIMIT;
    motor->pi_q.limit = PI_LIMIT;

    FOC_PI_Regulator(&motor->pi_d, dt); // pid计算
    FOC_PI_Regulator(&motor->pi_q, dt); // pid计算
    motor->u_dq.d = motor->pi_d.output;
    motor->u_dq.q = motor->pi_q.output;

    Foc_SetElectricalAngle(motor, 0.0f);
    Foc_RefreshTrigCache(motor);
#else
    Foc_RefreshTrigCache(motor);
    FOC_Park_Transform(motor);

    motor->u_dq.d = 0.0f;             // 通常d轴电流设为0以获得最大转矩效率
    motor->u_dq.q = PWM_VBUS * 0.35f; // q轴电压与期望转矩相关, 电压范围为母线电压的30%~50%

    Foc_AdvanceOpenLoopAngle(motor, dt);
#endif

    // 3. 反Park变换
    FOC_InvPark_Transform(motor);
    // 4. SVPWM生成并输出
    FOC_SVPWM_Generate(motor);
    return foc_state;
}

/**
 * @brief 闭环控制
 * 
 * @param motor 
 * @param dt 
 * @return foc_state_t 
 */
void Foc_Update_SpeedLoop(uint8_t motor_num, float dt)
{
    foc_handle_t *motor = &FOC_Motor[motor_num];

    if (motor->init_done != 1U || motor->mode != MOTOR_STATE_CLOSE)
        return;

#ifdef FOC_SPEED_PI_EN
    float speed_loop_target = motor->speed_ramp_target;

#ifdef FOC_POSITION_PI_EN
    if (motor->control_mode == FOC_CONTROL_POSITION)
    {
        float position_error = motor->target_position - motor->position;
        if (fabsf(position_error) < POSITION_DEADBAND_RAD)
        {
            motor->pi_position.integral = 0.0f;
            motor->pi_position.output = 0.0f;
            speed_loop_target = 0.0f;
        }
        else
        {
            motor->pi_position.target = motor->target_position;
            motor->pi_position.feedback = motor->position;
            motor->pi_position.limit = PI_LIMIT_POSITION_RPM;
            FOC_PI_Regulator(&motor->pi_position, dt);
            speed_loop_target = motor->pi_position.output;
#ifdef HFI_ENABLE
            if (!motor->hfi_enable && motor->position_dir != 0.0f && speed_loop_target * motor->position_dir < 0.0f)
                speed_loop_target = 0.0f;
#else
            if (motor->position_dir != 0.0f && speed_loop_target * motor->position_dir < 0.0f)
                speed_loop_target = 0.0f;
#endif
        }
        motor->pi_speed.target = speed_loop_target;
        motor->speed_ramp_target = speed_loop_target;
    }
    else
#endif
    {
        if (motor->target_speed > motor->speed_ramp_target)
        {
            motor->speed_ramp_target += SPEED_RAMP_RATE * dt;
            if (motor->speed_ramp_target > motor->pi_speed.target)
                motor->speed_ramp_target = motor->pi_speed.target;
        }
        else if (motor->target_speed < motor->speed_ramp_target)
        {
            motor->speed_ramp_target -= SPEED_RAMP_RATE * dt;
            if (motor->speed_ramp_target < motor->pi_speed.target)
                motor->speed_ramp_target = motor->pi_speed.target;
        }
        speed_loop_target = motor->speed_ramp_target;
    }

    {
        float spd_err = speed_loop_target - motor->speed;
        float iq_cmd;

        motor->pi_speed.integral += PI_KI_SPEED * spd_err * dt;

        if (motor->pi_speed.integral > PI_LIMIT_SPEED)
            motor->pi_speed.integral = PI_LIMIT_SPEED;
        else if (motor->pi_speed.integral < -PI_LIMIT_SPEED)
            motor->pi_speed.integral = -PI_LIMIT_SPEED;

        iq_cmd = PI_KP_SPEED * spd_err + motor->pi_speed.integral;

        if (iq_cmd > PI_LIMIT_SPEED)
            iq_cmd = PI_LIMIT_SPEED;
        else if (iq_cmd < -PI_LIMIT_SPEED)
            iq_cmd = -PI_LIMIT_SPEED;

        motor->pi_speed.output = iq_cmd;
        motor->pi_q.target = iq_cmd;
    }
#endif
}

foc_state_t Foc_Close_Loop(foc_handle_t *motor, float dt)
{
    foc_state_t foc_state = FOC_OK;
#if defined(FW_ENABLE) || defined(FOC_CLOSE_I_DEBUG_EN)
    const float current_target_limit_sq = CURRENT_TARGET_LIMIT * CURRENT_TARGET_LIMIT;
#endif
    // pi输出限幅缓启动
    float pi_limit;
    float pi_limit_sq;
    float voltage_mag;
    float voltage_scale;
    
    if(motor->close_cnt < 500)           // 缩短到500拍（0.04秒）
    {
        motor->close_cnt++;
        pi_limit = 3.0f + motor->close_cnt * 0.007f;  // 3V→6.5V，0.04秒到位
    }
    else
        pi_limit = PI_LIMIT;

    pi_limit_sq = pi_limit * pi_limit;

    // 电流前处理：减零偏、转安培、限幅
    foc_state = Foc_Preprocess_CurrentSample(motor);

    // Clark变换，
    FOC_Clark_Transform(motor);

    uint8_t trig_dirty = 0U;

#ifdef FOC_SMO_EN // 观测器使能
    // MRAS观测器推算转子位置，得到电角度和转速
    SMO_Observer(motor, dt, MOTOR_STATE_CLOSE);
    trig_dirty = 1U;
#else
    trig_dirty = Foc_LoadClosedLoopSensorAngle(motor);
#endif

#ifdef HFI_ENABLE // HFI使能
    if (motor->hfi_enable || trig_dirty != 0U)
    {
        HFI_Select_Angle(motor, dt);
        trig_dirty = 1U;
    }
#endif

    if (trig_dirty != 0U)
        Foc_RefreshTrigCache(motor);

    FOC_Park_Transform(motor);
#ifdef HFI_ENABLE // HFI使能
    HFI_Process_Current(motor, dt);
#endif

    if (motor->control_mode == FOC_CONTROL_POSITION)
    {
        float position_error = motor->target_position - motor->position; // 位置误差
        uint8_t position_crossed = (motor->position_dir != 0.0f && position_error * motor->position_dir <= POSITION_DEADBAND_RAD);
#ifdef HFI_ENABLE
        if (motor->hfi_enable)
            position_crossed = 0;
#endif
        if (fabsf(motor->speed) > POSITION_OVERSPEED_RPM || position_crossed)
        {
            motor->target_speed = 0.0f;
            motor->speed_ramp_target = 0.0f;
            motor->pi_speed.target = 0.0f;
            motor->pi_speed.integral = 0.0f;
            motor->pi_speed.output = 0.0f;
            motor->pi_position.integral = 0.0f;
            motor->pi_position.output = 0.0f;
            motor->pi_q.target = 0.0f;
            motor->pi_q.integral = 0.0f;
            motor->position_dir = 0.0f;
            motor->control_mode = FOC_CONTROL_SPEED;
            motor->hal.pwm_set_duty(motor->num, PWM_ARR / 2, PWM_ARR / 2, PWM_ARR / 2);
            motor->mode = MOTOR_STATE_IDLE;
            return foc_state;
        }
    }

// 6. 速度环已搬到前台主循环，ISR 内只保留电流环
    motor->pi_d.target = 0.0f;

#ifdef FW_ENABLE
    FOC_FieldWeakening(motor, TS);
#endif

#ifdef FOC_CLOSE_I_DEBUG_EN
    vofa_cnt++;
    if(vofa_cnt >= 17000 && vofa_cnt < 34000)
    {
        motor->pi_q.target = CURRENT_LOOP_STEP_HIGH_A; // 速度环输出→iq目标
        // motor->pi_q.target = 0.2f; // 速度环输出→iq目标
    }
    else if(vofa_cnt > 0 && vofa_cnt < 17000)
        motor->pi_q.target = CURRENT_LOOP_STEP_LOW_A; // 速度环输出→iq目标
        // motor->pi_q.target = 0.2f; // 速度环输出→iq目标
    else
        vofa_cnt = 0;
    motor->pi_d.target = 0.0f;                   // id始终为0
#elif defined(FW_ENABLE)
    float id_target = motor->pi_d.target;
    float iq_target_limit;

    if (id_target > CURRENT_TARGET_LIMIT)
        id_target = CURRENT_TARGET_LIMIT;
    else if (id_target < -CURRENT_TARGET_LIMIT)
        id_target = -CURRENT_TARGET_LIMIT;

    if (id_target == 0.0f)
    {
        iq_target_limit = CURRENT_TARGET_LIMIT;
    }
    else
    {
        float iq_limit_sq = current_target_limit_sq - id_target * id_target;
        if (iq_limit_sq < 0.0f)
            iq_limit_sq = 0.0f;
        iq_target_limit = sqrtf(iq_limit_sq);
    }

    motor->pi_d.target = id_target;
    if (motor->pi_q.target > iq_target_limit)
        motor->pi_q.target = iq_target_limit;
    else if (motor->pi_q.target < -iq_target_limit)
        motor->pi_q.target = -iq_target_limit;
#else
    motor->pi_d.target = 0.0f;
#endif

    // 6. 电流环PI调节
    motor->pi_d.feedback = motor->i_dq.d;
    motor->pi_q.feedback = motor->i_dq.q;
    motor->pi_d.limit = pi_limit;
    motor->pi_q.limit = pi_limit;

    {
        float error = motor->pi_d.target - motor->i_dq.d;
        float integral = motor->pi_d.integral + motor->pi_d.ki * error * dt;

        if (integral > pi_limit)
            integral = pi_limit;
        else if (integral < -pi_limit)
            integral = -pi_limit;

        float output = motor->pi_d.kp * error + integral;
        float limited_output = output;
        if (limited_output > pi_limit)
            limited_output = pi_limit;
        else if (limited_output < -pi_limit)
            limited_output = -pi_limit;

        if (output == limited_output || (output > pi_limit && error < 0.0f) || (output < -pi_limit && error > 0.0f))
            motor->pi_d.integral = integral;

        motor->pi_d.output = limited_output;
    }

    {
        float error = motor->pi_q.target - motor->i_dq.q;
        float integral = motor->pi_q.integral + motor->pi_q.ki * error * dt;

        if (integral > pi_limit)
            integral = pi_limit;
        else if (integral < -pi_limit)
            integral = -pi_limit;

        float output = motor->pi_q.kp * error + integral;
        float limited_output = output;
        if (limited_output > pi_limit)
            limited_output = pi_limit;
        else if (limited_output < -pi_limit)
            limited_output = -pi_limit;

        if (output == limited_output || (output > pi_limit && error < 0.0f) || (output < -pi_limit && error > 0.0f))
            motor->pi_q.integral = integral;

        motor->pi_q.output = limited_output;
    }

    motor->u_dq.d = motor->pi_d.output;
    motor->u_dq.q = motor->pi_q.output;

    voltage_mag = motor->u_dq.d * motor->u_dq.d + motor->u_dq.q * motor->u_dq.q;
    if (voltage_mag > pi_limit_sq)
    {
        voltage_mag = FOC_FastNorm(motor->u_dq.d, motor->u_dq.q);
        voltage_scale = pi_limit / voltage_mag;
        motor->u_dq.d *= voltage_scale;
        motor->u_dq.q *= voltage_scale;
        motor->pi_d.output = motor->u_dq.d;
        motor->pi_q.output = motor->u_dq.q;
    }
#ifdef HFI_ENABLE
    HFI_Add_Voltage(motor, dt);
#endif

    // 7. 反Park变换
    FOC_InvPark_Transform(motor);

    // 8. SVPWM生成并输出
    FOC_SVPWM_Generate(motor);
    return foc_state;
}

/**
 * @brief foc失能
 * 
 * @param motor 
 * @return foc_state_t 
 */
foc_state_t Foc_Stop(uint8_t motor_num)
{
    foc_handle_t *motor = &FOC_Motor[motor_num];
    motor->target_speed = 0.0f;
    motor->pi_speed.target = 0.0f;
    motor->speed_ramp_target = 0.0f;
    motor->id_fw = 0.0f;
    motor->fw_active = 0.0f;
    motor->fw_voltage = 0.0f;
    motor->control_mode = FOC_CONTROL_SPEED;
    motor->pi_position.integral = 0.0f;
    motor->pi_position.output = 0.0f;
    motor->position_dir = 0.0f;
#ifdef HFI_ENABLE
    HFI_Disable(motor);
#endif
    motor->hal.drv_disable(motor_num); // 驱动失能
    return FOC_OK;
}

/**
 * @brief 设置目标速度
 * 
 * @param motor 
 * @param speed 
 * @return foc_state_t 
 */
foc_state_t Foc_Set_Speed(uint8_t motor_num, float speed)
{
    foc_handle_t *motor = &FOC_Motor[motor_num];
    if (motor->control_mode != FOC_CONTROL_SPEED)
    {
        // 说明还在位置环
        motor->pi_position.integral = 0.0f;
        motor->pi_position.output = 0.0f;
        motor->pi_speed.integral = 0.0f;
    }
    motor->control_mode = FOC_CONTROL_SPEED;
    if (fabsf(motor->target_speed - speed) > 1.0f)
    {
        motor->id_fw = 0.0f;
        motor->fw_active = 0.0f;
        motor->fw_voltage = 0.0f;
        }
    motor->target_speed = speed;
    motor->pi_speed.target = speed;
    return FOC_OK;
}

foc_state_t Foc_Set_Position(uint8_t motor_num, float position_rad)
{
    foc_handle_t *motor = &FOC_Motor[motor_num];
    if (motor->mode != MOTOR_STATE_CLOSE)
    {
#ifdef HFI_ENABLE
        if (motor->hfi_enable)
        {
            float position_error = position_rad - motor->position;
            motor->control_mode = FOC_CONTROL_POSITION;
            motor->target_position = position_rad;
            motor->position_dir = fabsf(position_error) > POSITION_DEADBAND_RAD ? (position_error > 0.0f ? 1.0f : -1.0f) : 1.0f;
            motor->target_speed = motor->position_dir * OPEN_LOOP_SPEED_RPM;
            motor->pi_speed.target = motor->target_speed;
            return FOC_OK;
        }
#endif
        motor->target_position = position_rad;
        return FOC_ERR_LOOP;
    }

    if (motor->control_mode != FOC_CONTROL_POSITION || fabsf(position_rad - motor->target_position) > POSITION_DEADBAND_RAD)
    {
        float position_error = position_rad - motor->position;
        motor->pi_position.integral = 0.0f;
        motor->pi_position.output = 0.0f;
        motor->pi_speed.integral = 0.0f;
        motor->position_dir = fabsf(position_error) > POSITION_DEADBAND_RAD ? (position_error > 0.0f ? 1.0f : -1.0f) : 0.0f;
    }

    motor->control_mode = FOC_CONTROL_POSITION;
    motor->target_position = position_rad;
    motor->target_speed = motor->position_dir * PI_LIMIT_POSITION_RPM;
    motor->pi_speed.target = motor->target_speed;

    return FOC_OK;
}

foc_state_t Foc_Set_Control_Mode(uint8_t motor_num, foc_control_mode_t mode)
{
    if (mode != FOC_CONTROL_SPEED && mode != FOC_CONTROL_POSITION)
        return FOC_ERR_INVALID_PARAM;

    foc_handle_t *motor = &FOC_Motor[motor_num];
    if (motor->control_mode == mode)
        return FOC_OK;

    motor->control_mode = mode;
    motor->pi_position.integral = 0.0f;
    motor->pi_position.output = 0.0f;
    motor->pi_speed.integral = 0.0f;
    motor->position_dir = 0.0f;

    if (mode == FOC_CONTROL_POSITION)
        motor->target_position = motor->position;

    return FOC_OK;
}

foc_state_t Foc_Zero_Position(uint8_t motor_num)
{
    foc_handle_t *motor = &FOC_Motor[motor_num];
    motor->position_offset = motor->position_raw;
    motor->position = 0.0f;
    motor->target_position = 0.0f;
    motor->pi_position.integral = 0.0f;
    motor->pi_position.output = 0.0f;
    motor->position_dir = 0.0f;
    return FOC_OK;
}

float Foc_Get_Position(uint8_t motor_num)
{
    return FOC_Motor[motor_num].position;
}

foc_handle_t *Foc_GetStruct(uint8_t motor_num)
{
    return &FOC_Motor[motor_num];
}

#ifdef HFI_ENABLE
foc_state_t Foc_HFI_Enable(uint8_t motor_num)
{
    HFI_Enable(&FOC_Motor[motor_num]);
    return FOC_OK;
}

foc_state_t Foc_HFI_Disable(uint8_t motor_num)
{
    HFI_Disable(&FOC_Motor[motor_num]);
    return FOC_OK;
}

uint8_t Foc_HFI_Is_Valid(uint8_t motor_num)
{
    return FOC_Motor[motor_num].hfi_valid;
}

float Foc_HFI_Get_Angle(uint8_t motor_num)
{
    return FOC_Motor[motor_num].theta_hfi;
}
#endif




