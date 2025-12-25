/*********************************************************************************************************************
* ODrive Driver - Bike Balance System
* 
* 本文件是自平衡自行车系统的ODrive驱动实现文件
* 
* 功能说明：
* 1. ODrive电机控制器初始化和配置
* 2. 力矩控制模式
* 3. 速度读取功能（非阻塞轮询）
* 
********************************************************************************************************************/

#include "driver_odrive.h"
#include "zf_driver_uart.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// ========== 静态变量 ==========
static float current_torque = 0.0f;              // 当前设置的力矩
static float wheel_speed_rps = 0.0f;             // 当前轮速（转/秒）
static volatile uint8 wheel_speed_valid = 0;    // 轮速数据有效标志

// 行缓冲解析
static char line_buf[128];                       // 行缓冲区
static uint8 line_len = 0;                       // 当前行长度

// 防止重复发读命令导致回包冲突
static uint8 waiting_speed_resp = 0;

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
    wheel_speed_valid = 0;
    line_len = 0;
    waiting_speed_resp = 0;
    
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
 * @brief 请求读取ODrive速度（非阻塞）
 * @note 只负责发送命令，不等待响应
 * @note 需要配合 odrive_poll() 在主循环中高频调用
 */
void odrive_request_speed(void)
{
    if (waiting_speed_resp) return;  // 上次还没收完，先别发新的
    waiting_speed_resp = 1;

    // 读属性：r [property]，回包是一行文本数字
    uart_write_string(ODRIVE_UART_INDEX, "r axis0.encoder.vel_estimate\n");
    // 注意：ODrive 识别 \n/\r 都行，但回包固定 \r\n
}

/**
 * @brief 串口收包轮询：把 UART 收到的字节拼成一行，遇到 '\n' 就解析
 * @note 需要在主循环/任务里频繁调用（1-2ms或每个循环），不要放在ISR里阻塞
 */
void odrive_poll(void)
{
    uint8 ch;

    // 读出当前所有可用字节
    while (uart_query_byte(ODRIVE_UART_INDEX, &ch))
    {
        if (ch == '\r')
        {
            continue; // 忽略 CR
        }

        if (ch == '\n')
        {
            // 一行结束，解析
            line_buf[line_len] = '\0';

            // 期望是一个 float 文本，比如 "12.345"
            char *endp = NULL;
            float v = strtof(line_buf, &endp);
            if (endp != line_buf)
            {
                wheel_speed_rps = v;      // turns/s
                wheel_speed_valid = 1;
            }

            line_len = 0;
            waiting_speed_resp = 0; // 收到回包了，可以发下一次请求
            continue;
        }

        // 普通字符入行缓存
        if (line_len < sizeof(line_buf) - 1)
        {
            line_buf[line_len++] = (char)ch;
        }
        else
        {
            // 行太长：丢弃这一行
            line_len = 0;
            waiting_speed_resp = 0;
        }
    }
}

/**
 * @brief 获取当前轮速
 * @param out_rps 输出参数，存储轮速值（转/秒）
 * @return 1=有效数据，0=无效/未收到数据
 */
uint8 odrive_get_speed(float *out_rps)
{
    if (!out_rps) return 0;
    if (!wheel_speed_valid) return 0;
    *out_rps = wheel_speed_rps;
    return 1;
}

/**
 * @brief 停止ODrive电机
 */
void odrive_stop(void)
{
    odrive_set_torque(0.0f);
}
