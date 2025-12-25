/*********************************************************************************************************************
* Servo Motor Driver - Bike Balance System
* 
* 本文件是自平衡自行车系统的舵机驱动头文件
* 
* 功能说明：
* 1. 舵机PWM初始化和配置
* 2. 舵机角度控制（0-180度）
* 3. 角度限幅保护
* 
********************************************************************************************************************/

#ifndef DRIVER_SERVO_H
#define DRIVER_SERVO_H

#include "zf_common_headfile.h"

// ========== 舵机硬件配置 ==========
#define SERVO_PWM_PIN           (ATOM1_CH7_P02_7)    // 舵机PWM引脚
#define SERVO_PWM_FREQ          (50)                 // 舵机PWM频率：50Hz
#define SERVO_PULSE_MIN_MS      (0.5f)               // 最小脉宽：0.5ms (对应0度)
#define SERVO_PULSE_MAX_MS      (2.5f)               // 最大脉宽：2.5ms (对应180度)

// ========== 舵机角度限制 ==========
#define SERVO_ANGLE_MIN         (0.0f)               // 最小角度：0度
#define SERVO_ANGLE_MAX         (180.0f)             // 最大角度：180度
#define SERVO_ANGLE_CENTER      (90.0f)              // 中心角度：90度

// ========== 函数声明 ==========

/**
 * @brief 初始化舵机
 * @param init_angle 初始角度（度），范围0-180
 * @note 会自动配置PWM引脚和频率
 */
void servo_init(float init_angle);

/**
 * @brief 设置舵机角度
 * @param angle 目标角度（度），范围0-180
 * @note 自动进行角度限幅，确保在有效范围内
 */
void servo_set_angle(float angle);

/**
 * @brief 获取当前舵机角度
 * @return 当前角度（度）
 */
float servo_get_angle(void);

/**
 * @brief 设置舵机到中心位置
 */
void servo_set_center(void);

#endif // DRIVER_SERVO_H
