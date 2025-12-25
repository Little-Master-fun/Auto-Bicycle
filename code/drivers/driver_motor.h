/*********************************************************************************************************************
* Motor Driver - Bike Balance System
* 
* 本文件是自平衡自行车系统的电机驱动头文件
* 
* 功能说明：
* 1. 电机（通过ESC电调）初始化和配置
* 2. 开环控制：前进、后退、停止
* 3. 速度调节功能
* 
* 注意：本驱动使用ESC（电子调速器）控制无刷电机
*      ESC接收标准PWM信号（50Hz，脉宽1000-2000us）
* 
********************************************************************************************************************/

#ifndef DRIVER_MOTOR_H
#define DRIVER_MOTOR_H

#include "zf_common_headfile.h"

// ========== 电机硬件配置 ==========
#define MOTOR_ESC_PWM_PIN       (ATOM0_CH6_P02_6)    // ESC电调PWM引脚
#define MOTOR_ESC_PWM_FREQ      (50)                 // ESC PWM频率：50Hz

// ========== ESC脉宽定义（微秒） ==========
#define MOTOR_PULSE_NEUTRAL     (1500)               // 中立位置：1500us（停止）
#define MOTOR_PULSE_MIN         (1000)               // 最小脉宽：1000us（最大反转）
#define MOTOR_PULSE_MAX         (2000)               // 最大脉宽：2000us（最大正转）

// ========== 速度范围定义 ==========
#define MOTOR_SPEED_MIN         (-100)               // 最小速度：-100%（全速后退）
#define MOTOR_SPEED_MAX         (100)                // 最大速度：+100%（全速前进）
#define MOTOR_SPEED_STOP        (0)                  // 停止速度：0%

// ========== 电机状态枚举 ==========
typedef enum {
    MOTOR_STATE_STOP = 0,       // 停止
    MOTOR_STATE_FORWARD,        // 前进
    MOTOR_STATE_BACKWARD        // 后退
} motor_state_enum;

// ========== 函数声明 ==========

/**
 * @brief 初始化电机驱动
 * @note 会自动配置ESC的PWM引脚，并设置为停止状态
 */
void motor_init(void);

/**
 * @brief 设置电机速度（开环控制）
 * @param speed 速度百分比，范围-100到+100
 *              正数：前进，负数：后退，0：停止
 * @note 自动进行速度限幅，确保在有效范围内
 */
void motor_set_speed(int16 speed);

/**
 * @brief 停止电机
 */
void motor_stop(void);

/**
 * @brief 设置电机前进
 * @param speed 前进速度百分比，范围0-100
 */
void motor_forward(uint8 speed);

/**
 * @brief 设置电机后退
 * @param speed 后退速度百分比，范围0-100
 */
void motor_backward(uint8 speed);

/**
 * @brief 获取当前电机速度
 * @return 当前速度百分比（-100到+100）
 */
int16 motor_get_speed(void);

/**
 * @brief 获取当前电机状态
 * @return 电机状态枚举值
 */
motor_state_enum motor_get_state(void);

/**
 * @brief 直接设置ESC脉宽（高级功能）
 * @param pulse_width 脉宽值（微秒），范围1000-2000
 * @note 一般不建议直接使用，建议使用motor_set_speed()
 */
void motor_set_pulse_width(int32 pulse_width);

#endif // DRIVER_MOTOR_H
