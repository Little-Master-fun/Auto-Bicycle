/*********************************************************************************************************************
* Servo Motor Driver - Bike Balance System
* 
* 本文件是自平衡自行车系统的舵机驱动实现文件
* 
* 功能说明：
* 1. 舵机PWM初始化和配置
* 2. 舵机角度控制（0-180度）
* 3. 角度限幅保护
* 
********************************************************************************************************************/

#include "driver_servo.h"
#include "zf_driver_pwm.h"

// ========== 静态变量 ==========
static float current_angle = SERVO_ANGLE_CENTER;  // 当前舵机角度

// ========== 内部辅助函数 ==========

/**
 * @brief 角度限幅函数
 * @param angle 输入角度
 * @return 限幅后的角度
 */
static inline float servo_constrain_angle(float angle)
{
    if (angle < SERVO_ANGLE_MIN) {
        return SERVO_ANGLE_MIN;
    }
    if (angle > SERVO_ANGLE_MAX) {
        return SERVO_ANGLE_MAX;
    }
    return angle;
}

/**
 * @brief 将角度转换为PWM占空比
 * @param angle 目标角度（0-180度）
 * @return PWM占空比值
 */
static uint32 servo_angle_to_duty(float angle)
{
    // 确保角度在有效范围内
    angle = servo_constrain_angle(angle);
    
    // 计算脉宽（ms）
    // 0度 -> 0.5ms, 180度 -> 2.5ms
    float pulse_width_ms = SERVO_PULSE_MIN_MS + 
                          (angle / 180.0f) * (SERVO_PULSE_MAX_MS - SERVO_PULSE_MIN_MS);
    
    // 计算PWM周期（ms）
    float period_ms = 1000.0f / SERVO_PWM_FREQ;
    
    // 计算占空比
    float duty_ratio = pulse_width_ms / period_ms;
    
    // 转换为PWM_DUTY_MAX比例的整数值
    return (uint32)(duty_ratio * PWM_DUTY_MAX);
}

// ========== 外部接口函数 ==========

/**
 * @brief 初始化舵机
 */
void servo_init(float init_angle)
{
    // 限制初始角度范围
    init_angle = servo_constrain_angle(init_angle);
    current_angle = init_angle;
    
    // 计算初始占空比
    uint32 init_duty = servo_angle_to_duty(init_angle);
    
    // 初始化PWM
    pwm_init(SERVO_PWM_PIN, SERVO_PWM_FREQ, init_duty);
    
    printf("Servo initialized at angle: %.2f degrees\r\n", current_angle);
}

/**
 * @brief 设置舵机角度
 */
void servo_set_angle(float angle)
{
    // 限制角度范围
    angle = servo_constrain_angle(angle);
    current_angle = angle;
    
    // 转换为占空比并设置
    uint32 duty = servo_angle_to_duty(angle);
    pwm_set_duty(SERVO_PWM_PIN, duty);
}

/**
 * @brief 获取当前舵机角度
 */
float servo_get_angle(void)
{
    return current_angle;
}

/**
 * @brief 设置舵机到中心位置
 */
void servo_set_center(void)
{
    servo_set_angle(SERVO_ANGLE_CENTER);
}
