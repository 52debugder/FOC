/**
 * @file foc.c
 * @author MING
 * @brief foc调用库
 * @version 0.7
 * @date 2026-06-01
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "foc.h"
//1111111111111111111
foc_handle_t FOC_Motor[MAX_MOTOR_NUM + 1] = {0};
uint32_t vofa_cnt = 0;

/**
 * @brief FOC位置环状态更新
 * 
 * @param motor 
 */
static void Foc_Update_Position(foc_handle_t *motor)
{
    // 角度误差
    float delta_e = motor->theta - motor->theta_obs_prev;
    if (delta_e > PI)
        delta_e -= _2_PI;
    else if (delta_e < -PI)
        delta_e += _2_PI;

    motor->position_raw += delta_e / POLE_PAIRS;
    motor->position = motor->position_raw - motor->position_offset;
    motor->theta_obs_prev = motor->theta;
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
    motor->hal.pwm_enable(motor_num);            // PWM使能
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

    motor->pi_pll.integral = 0.0f;
    motor->speed_observer = 0.0f;
    motor->theta_Observer = 0.0f;
    motor->theta_obs_prev = 0.0f;

    motor->theta = 0.0f; 

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

    motor->hal.adc_get_value(motor_num, &motor->i_adc_u, &motor->i_adc_v, &motor->i_adc_w);

    switch (motor->mode)
    {
    case MOTOR_STATE_IDLE: // 空状态
        // 每个控制周期都刷新为零电压，防止PWM寄存器残留
        motor->hal.pwm_set_duty(motor->num,
            PWM_ARR / 2, PWM_ARR / 2, PWM_ARR / 2);

        if (fabsf(motor->target_speed) > SPEED_START_THRESHOLD)
        {
            motor->theta             = 0.0f;
            motor->pi_pll.integral   = 0.0f;
            motor->pi_d.integral        = 0.0f;
            motor->pi_q.integral        = 0.0f;
            motor->pi_speed.integral    = 0.0f;
            motor->pi_position.integral = 0.0f;
            motor->theta_obs_prev       = 0.0f;
            motor->id_fw                = 0.0f;

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
        // 1. 定位阶段：给 D 轴施加固定电压，Q 轴为 0，强制转子对齐到 0 度
        motor->u_dq.d = 2.0f; // 2V 左右，根据电机阻抗微调
        motor->u_dq.q = 0.0f;
        motor->theta = 0.0f;
        motor->pi_pll.integral = 0;
        motor->theta_Observer = 0.0f;
        motor->theta_obs_prev = 0.0f;
        FOC_InvPark_Transform(motor);
        FOC_SVPWM_Generate(motor);

        // ALIGN 期间若遥控器归零 → 回 IDLE
        if (fabsf(motor->target_speed) < SPEED_START_THRESHOLD)
        {
            motor->state_timer = 0;
            motor->mode = MOTOR_STATE_IDLE;
            break;
        }

        motor->state_timer++;

        if (motor->state_timer > 4000)
        { // 持续约 100ms (假设频率 18kHz)
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
                motor->mode = MOTOR_STATE_CLOSE;
                break;
            }
#endif
            motor->pi_pll.integral = motor->target_speed > 0 ? OPEN_ELEC_SPEED : -OPEN_ELEC_SPEED;
            motor->mode = MOTOR_STATE_OPEN;
        }
        break;

    case MOTOR_STATE_OPEN:
        // 2. 开环启动：按设定转速匀速旋转
        Foc_Open_Loop(motor, TS);

        // ALIGN 期间若遥控器归零 → 回 IDLE
        if (fabsf(motor->target_speed) < SPEED_START_THRESHOLD)
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
            float speed_rpm = motor->speed_observer * 60.0f / _2_PI_POLE_PAIRS; // 把电角速度转换为圈每秒
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
                    motor->speed_ramp_target = fabsf(motor->speed_observer) * 60.0f / (_2_PI * POLE_PAIRS) + 50.0f;
                else
                    motor->speed_ramp_target = -fabsf(motor->speed_observer) * 60.0f / (_2_PI * POLE_PAIRS) + 50.0f;

                motor->theta_Observer = motor->theta;
                motor->theta_obs_prev = motor->theta;
                motor->pi_position.integral = 0.0f;
#ifdef HFI_ENABLE
                if (motor->hfi_enable)
                    HFI_Reset_Angle(motor, motor->theta);
#endif
                motor->PI_Speed_cnt = 0;
                motor->close_cnt = 0;

                motor->mode = MOTOR_STATE_CLOSE;
            }
        }
#endif

        break;

    case MOTOR_STATE_CLOSE:
        // 3. 闭环运行
        // 闭环期间若遥控器归零 → 回 IDLE
        if (motor->control_mode == FOC_CONTROL_SPEED && fabsf(motor->target_speed) < SPEED_START_THRESHOLD)
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

/**
 * @brief 开环控制
 * 
 * @param motor 
 * @param dt 
 * @return foc_state_t 
 */
foc_state_t Foc_Open_Loop(foc_handle_t *motor, float dt)
{
    foc_state_t foc_state = FOC_OK;
    // 1. 电流校准（减去零点偏移）
    motor->i_uvw.v = -(float)((int32_t)motor->i_adc_u - motor->i_cali_uvw.u) * CURRENT_SCALE;
    motor->i_uvw.u = (float)((int32_t)motor->i_adc_w - motor->i_cali_uvw.w) * CURRENT_SCALE;
    // 2. 电流限幅（保护电机）

    if(fabs(motor->i_uvw.v) >= CURRENT_LIMIT)
    {
        motor->i_uvw.v = motor->i_uvw.v > 0 ? CURRENT_LIMIT : -CURRENT_LIMIT;
        foc_state = FOC_ERR_OVERCURRENT;
    }

    if(fabs(motor->i_uvw.u) >= CURRENT_LIMIT)
    {
        motor->i_uvw.u = motor->i_uvw.u > 0 ? CURRENT_LIMIT : -CURRENT_LIMIT;
        foc_state = FOC_ERR_OVERCURRENT;
    }

    // 3. Clark变换，
    FOC_Clark_Transform(motor);
    // 4. smo观测器推算转子位置，得到电角度和转速
    SMO_Observer(motor, dt, MOTOR_STATE_OPEN);

#ifdef FOC_OPEN_I_DEBUG_EN // 开环电流环调试代码使能
    // 固定角度为0，不依赖观测器
    float control_theta = 0.0f;
    float cos_th = cosf(control_theta);
    float sin_th = sinf(control_theta);

    // Park变换
    motor->i_dq.d = motor->i_ab.alpha;
    motor->i_dq.q = motor->i_ab.beta;

    // d轴电流阶跃信号
    vofa_cnt++;
    if (vofa_cnt >= 34000)
        vofa_cnt = 0;

    if (vofa_cnt >= 17000)
        motor->pi_q.target = CURRENT_LOOP_STEP_HIGH_A;
    else
        motor->pi_q.target = CURRENT_LOOP_STEP_LOW_A;
    motor->pi_d.target = 0.0f;

    float id_target = fmaxf(-CURRENT_TARGET_LIMIT, fminf(CURRENT_TARGET_LIMIT, motor->pi_d.target));
    float iq_target_limit = sqrtf(fmaxf(0.0f, CURRENT_TARGET_LIMIT * CURRENT_TARGET_LIMIT - id_target * id_target));
    motor->pi_d.target = id_target;
    motor->pi_q.target = fmaxf(-iq_target_limit, fminf(iq_target_limit, motor->pi_q.target));

    motor->pi_d.feedback = motor->i_dq.d;
    motor->pi_q.feedback = motor->i_dq.q;
    motor->pi_d.limit = PI_LIMIT;
    motor->pi_q.limit = PI_LIMIT;

    FOC_PI_Regulator(&motor->pi_d, dt); // pid计算
    FOC_PI_Regulator(&motor->pi_q, dt); // pid计算
    motor->u_dq.d = motor->pi_d.output;
    motor->u_dq.q = motor->pi_q.output;

    motor->theta = 0;
#else
    FOC_Park_Transform(motor);

    motor->u_dq.d = 0.0f;             // 通常d轴电流设为0以获得最大转矩效率
    motor->u_dq.q = PWM_VBUS * 0.65f; // q轴电压与期望转矩相关, 电压范围为母线电压的30%~50%

    if(motor->target_speed > 0)
        motor->theta += OPEN_ELEC_SPEED * dt;     // 电角度递增
    else if (motor->target_speed < 0)
        motor->theta -= OPEN_ELEC_SPEED * dt;

    motor->theta = fmod(motor->theta, _2_PI); // 限制在0~2π范围内，保持归一化
    if (motor->theta < 0)
        motor->theta += _2_PI; // 处理负数情况
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
foc_state_t Foc_Close_Loop(foc_handle_t *motor, float dt)
{
    foc_state_t foc_state = FOC_OK;
    // pi输出限幅缓启动
    float pi_limit;
    float voltage_mag;
    float voltage_scale;
    if(motor->close_cnt < 500)           // 缩短到500拍（0.04秒）
    {
        motor->close_cnt++;
        pi_limit = 3.0f + motor->close_cnt * 0.007f;  // 3V→6.5V，0.04秒到位
    }
    else
        pi_limit = PI_LIMIT;

    // 1. 电流校准（减去零点偏移）
    motor->i_uvw.v = -(float)((int32_t)motor->i_adc_u - motor->i_cali_uvw.u) * CURRENT_SCALE;
    motor->i_uvw.u = (float)((int32_t)motor->i_adc_w - motor->i_cali_uvw.w) * CURRENT_SCALE;
    // 2. 电流限幅（保护电机）
    if(fabs(motor->i_uvw.v) >= CURRENT_LIMIT)
    {
        motor->i_uvw.v = motor->i_uvw.v > 0 ? CURRENT_LIMIT : -CURRENT_LIMIT;
        foc_state = FOC_ERR_OVERCURRENT;
    }

    if(fabs(motor->i_uvw.u) >= CURRENT_LIMIT)
    {
        motor->i_uvw.u = motor->i_uvw.u > 0 ? CURRENT_LIMIT : -CURRENT_LIMIT;
        foc_state = FOC_ERR_OVERCURRENT;
    }

    // 3. Clark变换，
    FOC_Clark_Transform(motor);
    // 4. MRAS观测器推算转子位置，得到电角度和转速
    SMO_Observer(motor, dt, MOTOR_STATE_CLOSE);
#ifdef HFI_ENABLE // HFI使能
    HFI_Select_Angle(motor, dt);
#endif
    Foc_Update_Position(motor);

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

    // 6. 速度环PI调节
    #ifdef FOC_SPEED_PI_EN
    
    motor->PI_Speed_cnt++;
    if (motor->PI_Speed_cnt >= 10)
    {
        motor->PI_Speed_cnt = 0;

        float speed_loop_target = motor->speed_ramp_target;

        if (motor->control_mode == FOC_CONTROL_POSITION) // 位置环控制
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
                FOC_PI_Regulator(&motor->pi_position, dt * 10.0f);
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
        else // 速度环控制
        {
            if(motor->target_speed > motor->speed_ramp_target)
            {
                motor->speed_ramp_target += SPEED_RAMP_RATE * dt * 10.0f;
                if (motor->speed_ramp_target > motor->pi_speed.target)
                    motor->speed_ramp_target = motor->pi_speed.target;
            }
            else if(motor->target_speed < motor->speed_ramp_target)
            {
                motor->speed_ramp_target -= SPEED_RAMP_RATE * dt * 10.0f;
                if (motor->speed_ramp_target < motor->pi_speed.target)
                    motor->speed_ramp_target = motor->pi_speed.target;
            }
            speed_loop_target = motor->speed_ramp_target;
        }

        float spd_err = speed_loop_target - motor->speed;

        motor->pi_speed.integral += PI_KI_SPEED * spd_err * (dt * 10.0f); // 速度积分

        motor->pi_speed.integral = fmaxf(-PI_LIMIT_SPEED, fminf(PI_LIMIT_SPEED, motor->pi_speed.integral)); // 速度积分限幅

        float iq_p = PI_KP_SPEED * spd_err; // 速度环比例
        float iq_cmd = iq_p + motor->pi_speed.integral;

        iq_cmd = fmaxf(-PI_LIMIT_SPEED, fminf(PI_LIMIT_SPEED, iq_cmd));

        motor->pi_speed.output = iq_cmd;
        motor->pi_q.target = iq_cmd;
    }
    motor->pi_d.target = 0.0f; // id始终为0

    #ifdef FW_ENABLE
    FOC_FieldWeakening(motor, TS);
    #endif

    #endif

    #ifdef FOC_CLOSE_I_DEBUG_EN
    vofa_cnt++;
    if(vofa_cnt >= 17000 && vofa_cnt < 34000)
    {
        motor->pi_q.target = CURRENT_LOOP_STEP_HIGH_A; // 速度环输出→iq目标
    }
    else if(vofa_cnt > 0 && vofa_cnt < 17000)
        motor->pi_q.target = CURRENT_LOOP_STEP_LOW_A; // 速度环输出→iq目标
    else
        vofa_cnt = 0;
    motor->pi_d.target = 0.0f;                   // id始终为0
    #endif

    float id_target = fmaxf(-CURRENT_TARGET_LIMIT, fminf(CURRENT_TARGET_LIMIT, motor->pi_d.target));
    float iq_target_limit = sqrtf(fmaxf(0.0f, CURRENT_TARGET_LIMIT * CURRENT_TARGET_LIMIT - id_target * id_target));
    motor->pi_d.target = id_target;
    motor->pi_q.target = fmaxf(-iq_target_limit, fminf(iq_target_limit, motor->pi_q.target));

    // 6. 电流环PI调节
    motor->pi_d.feedback = motor->i_dq.d;
    motor->pi_q.feedback = motor->i_dq.q;
    motor->pi_d.limit = pi_limit;
    motor->pi_q.limit = pi_limit;

    FOC_PI_Regulator(&motor->pi_d, dt); // pid计算
    FOC_PI_Regulator(&motor->pi_q, dt); // pid计算
    motor->u_dq.d = motor->pi_d.output;
    motor->u_dq.q = motor->pi_q.output;

    voltage_mag = sqrtf(motor->u_dq.d * motor->u_dq.d + motor->u_dq.q * motor->u_dq.q);
    if (voltage_mag > pi_limit)
    {
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




