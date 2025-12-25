/*********************************************************************************************************************
* Motor Driver - Bike Balance System
* 
* 本文件是自平衡自行车系统的电机驱动实现文件
* 
* 功能说明：
* 1. 电机（通过ESC电调）初始化和配置
* 2. 开环控制：前进、后退、停止
* 3. 速度调节功能
* 
********************************************************************************************************************/

#include "driver_motor.h"
#include "zf_driver_pwm.h"

// ========== 静态变量 ==========
static int16 current_speed = MOTOR_SPEED_STOP;    // 当前速度（-100到+100）
static motor_state_enum current_state = MOTOR_STATE_STOP;  // 当前状态

// ========== 内部辅助函数 ==========

/**
 * @brief 速度限幅函数
 * @param speed 输入速度
 * @return 限幅后的速度
 */
static inline int16 motor_constrain_speed(int16 speed)
{
    if (speed < MOTOR_SPEED_MIN) {
        return MOTOR_SPEED_MIN;
    }
    if (speed > MOTOR_SPEED_MAX) {
        return MOTOR_SPEED_MAX;
    }
    return speed;
}

/**
 * @brief 将速度百分比转换为ESC脉宽
 * @param speed 速度百分比（-100到+100）
 * @return ESC脉宽（微秒）
 */
static int32 motor_speed_to_pulse(int16 speed)
{
    // 限制速度范围
    speed = motor_constrain_speed(speed);
    
    // 计算脉宽
    // speed = -100 -> 1000us
    // speed = 0    -> 1500us
    // speed = +100 -> 2000us
    int32 pulse_width = MOTOR_PULSE_NEUTRAL + (speed * 5);  // 每1%速度对应5us变化
    
    return pulse_width;
}

// ========== 外部接口函数 ==========

/**
 * @brief 初始化电机驱动
 */
void motor_init(void)
{
    // 初始化为停止状态
    current_speed = MOTOR_SPEED_STOP;
    current_state = MOTOR_STATE_STOP;
    
    // 初始化PWM（设置为中立位置）
    pwm_init(MOTOR_ESC_PWM_PIN, MOTOR_ESC_PWM_FREQ, 0);
    
    // 设置停止状态
    motor_stop();
    
    printf("Motor initialized (ESC neutral position)\r\n");
}

/**
 * @brief 直接设置ESC脉宽
 */
void motor_set_pulse_width(int32 pulse_width)
{
    // 限制脉宽范围
    if (pulse_width < MOTOR_PULSE_MIN) {
        pulse_width = MOTOR_PULSE_MIN;
    }
    if (pulse_width > MOTOR_PULSE_MAX) {
        pulse_width = MOTOR_PULSE_MAX;
    }
    
    // 将脉宽（微秒）转换为占空比
    // PWM周期 = 1/50Hz = 20ms = 20000us
    float duty_ratio = (float)pulse_width / 20000.0f;
    uint16 duty = (uint16)(duty_ratio * PWM_DUTY_MAX);
    
    pwm_set_duty(MOTOR_ESC_PWM_PIN, duty);
}

/**
 * @brief 设置电机速度
 */
void motor_set_speed(int16 speed)
{
    // 限制速度范围
    speed = motor_constrain_speed(speed);
    current_speed = speed;
    
    // 更新状态
    if (speed > 0) {
        current_state = MOTOR_STATE_FORWARD;
    } else if (speed < 0) {
        current_state = MOTOR_STATE_BACKWARD;
    } else {
        current_state = MOTOR_STATE_STOP;
    }
    
    // 转换为脉宽并设置
    int32 pulse_width = motor_speed_to_pulse(speed);
    motor_set_pulse_width(pulse_width);
}

/**
 * @brief 停止电机
 */
void motor_stop(void)
{
    motor_set_speed(MOTOR_SPEED_STOP);
}

/**
 * @brief 设置电机前进
 */
void motor_forward(uint8 speed)
{
    // 限制速度范围0-100
    if (speed > 100) {
        speed = 100;
    }
    
    // 转换为有符号速度并设置
    motor_set_speed((int16)speed);
}

/**
 * @brief 设置电机后退
 */
void motor_backward(uint8 speed)
{
    // 限制速度范围0-100
    if (speed > 100) {
        speed = 100;
    }
    
    // 转换为负速度并设置
    motor_set_speed(-(int16)speed);
}

/**
 * @brief 获取当前电机速度
 */
int16 motor_get_speed(void)
{
    return current_speed;
}

/**
 * @brief 获取当前电机状态
 */
motor_state_enum motor_get_state(void)
{
    return current_state;
}
