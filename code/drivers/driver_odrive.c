/*********************************************************************************************************************
* ODrive Driver - Bike Balance System
* 
* 本文件是自平衡自行车系统的ODrive驱动实现文件
* 
* 功能说明：
* 1. ODrive电机控制器初始化和配置
* 2. 力矩控制模式
* 3. 速度读取功能
* 
********************************************************************************************************************/

#include "driver_odrive.h"
#include "zf_driver_uart.h"
#include <stdio.h>

// ========== 静态变量 ==========
static float current_torque = 0.0f;        // 当前设置的力矩
static float wheel_speed_rps = 0.0f;       // 当前轮速（转/秒）

// ========== 内部辅助函数 ==========

/**
 * @brief 力矩限幅函数
 * @param torque 输入力矩
 * @return 限幅后的力矩
 */
static inline float odrive_constrain_torque(float torque)
{
    if (torque > ODRIVE_TORQUE_MAX) {
        return ODRIVE_TORQUE_MAX;
    }
    if (torque < ODRIVE_TORQUE_MIN) {
        return ODRIVE_TORQUE_MIN;
    }
    return torque;
}

// ========== 外部接口函数 ==========

/**
 * @brief 初始化ODrive驱动
 */
void odrive_init(void)
{
    // 初始化UART通信
    uart_init(ODRIVE_UART_INDEX, ODRIVE_BAUDRATE, ODRIVE_TX_PIN, ODRIVE_RX_PIN);
    
    // 初始化状态
    current_torque = 0.0f;
    wheel_speed_rps = 0.0f;
    
    // 等待ODrive启动
    system_delay_ms(100);
    
    // 设置初始力矩为0
    odrive_stop();
    
    printf("ODrive initialized on UART6 (115200 baud)\r\n");
    printf("ODrive torque mode ready.\r\n");
}

/**
 * @brief 设置ODrive输出力矩
 */
void odrive_set_torque(float torque)
{
    char cmd[32];
    
    // 限制力矩范围
    torque = odrive_constrain_torque(torque);
    
    // 保存当前力矩
    current_torque = torque;
    
    // 发送力矩命令: c 0 <torque>
    // ODrive力矩控制命令格式：c <axis> <torque>
    sprintf(cmd, "c 0 %.6f", torque);  // 保留6位小数确保精度
    uart_write_string(ODRIVE_UART_INDEX, cmd);
    uart_write_byte(ODRIVE_UART_INDEX, '\r');  // ODrive命令以回车结束
}

/**
 * @brief 请求读取ODrive速度
 */
void odrive_request_speed(void)
{
    char cmd[64];
    
    // 发送速度查询命令
    sprintf(cmd, "r axis0.encoder.vel_estimate\r\n");
    uart_write_string(ODRIVE_UART_INDEX, cmd);
}

/**
 * @brief 获取当前轮速
 */
float odrive_get_speed(void)
{
    return wheel_speed_rps;
}

/**
 * @brief 停止ODrive电机
 */
void odrive_stop(void)
{
    odrive_set_torque(0.0f);
}
