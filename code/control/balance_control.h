/* balance_control.h */
#ifndef BALANCE_CONTROL_H
#define BALANCE_CONTROL_H

#include "zf_common_headfile.h"

typedef struct
{
    float roll_deg;
    float roll_filtered_deg;
    float roll_rate_deg_s;
    float target_angle;
    float target_rate;
    float control_output;
} balance_control_state_t;

void balance_control_init(void);
void balance_control_update_5ms_isr(void);

void balance_control_set_target_angle(float angle_deg);
float balance_control_get_target_angle(void);

void balance_control_set_enable(uint8 enable);  // 新增：设置使能/禁止
uint8 balance_control_get_enable(void);         // 新增：获取使能状态

// PID参数调节接口
void balance_control_adjust_angle_kp(float delta);      // 调节角度环Kp
void balance_control_adjust_angle_ki(float delta);      // 调节角度环Ki
void balance_control_adjust_angle_kd(float delta);      // 调节角度环Kd
void balance_control_adjust_velocity_kp(float delta);   // 调节速度环Kp
void balance_control_adjust_velocity_ki(float delta);   // 调节速度环Ki
void balance_control_adjust_velocity_kd(float delta);   // 调节速度环Kd

// 获取PID参数（简化版：仅角度Kp、速度Kp、速度Ki）
void balance_control_get_pid_params(float *angle_kp, float *vel_kp, float *vel_ki);

// 获取完整PID参数（所有6个参数）
void balance_control_get_pid_params_full(float *angle_kp, float *angle_ki, float *angle_kd,
                                          float *vel_kp, float *vel_ki, float *vel_kd);

void balance_control_get_state(balance_control_state_t *out_state);
float balance_control_get_output(void);

#endif
