/*********************************************************************************************************************
* ODrive Driver - Bike Balance System
* 
* 本文件是自平衡自行车系统的ODrive驱动头文件
* 
* 功能说明：
* 1. ODrive电机控制器初始化和配置
* 2. 力矩控制模式
* 3. 速度读取功能
* 
* 注意：ODrive通过UART接口使用ASCII协议通信
*      命令格式：c 0 <torque> 设置力矩
*      命令格式：r axis0.encoder.vel_estimate 读取速度
* 
********************************************************************************************************************/

#ifndef DRIVER_ODRIVE_H
#define DRIVER_ODRIVE_H

#include "zf_common_headfile.h"

// ========== ODrive硬件配置 ==========
#define ODRIVE_UART_INDEX       (UART_6)              // 使用UART6连接ODrive
#define ODRIVE_BAUDRATE         (115200)              // ODrive默认波特率
#define ODRIVE_TX_PIN           (UART6_TX_P22_0)      // ODrive TX引脚
#define ODRIVE_RX_PIN           (UART6_RX_P23_1)      // ODrive RX引脚

// ========== ODrive参数配置 ==========
#define ODRIVE_TORQUE_MAX       (18.0f)               // 最大力矩限制（Nm）
#define ODRIVE_TORQUE_MIN       (-18.0f)              // 最小力矩限制（Nm）

// ========== 函数声明 ==========

/**
 * @brief 初始化ODrive驱动
 * @note 初始化UART通信并设置为力矩模式
 */
void odrive_init(void);

/**
 * @brief 设置ODrive输出力矩
 * @param torque 目标力矩（Nm），自动限幅在±18Nm范围内
 * @note 发送命令格式: "c 0 <torque>\r"
 */
void odrive_set_torque(float torque);

/**
 * @brief 请求读取ODrive速度（非阻塞）
 * @note 只负责发送命令，不等待响应
 * @note 需要配合 odrive_poll() 在主循环中高频调用
 * @note 建议频率：20-50Hz（每20-50ms调用一次）
 */
void odrive_request_speed(void);

/**
 * @brief 串口收包轮询：非阻塞读取并解析UART数据
 * @note 必须在主循环中高频调用（建议每1-2ms或每个循环都调用）
 * @note 不要放在ISR中调用，会阻塞中断
 */
void odrive_poll(void);

/**
 * @brief 获取当前轮速（转/秒）
 * @param out_rps 输出参数，存储轮速值（turns/s）
 * @return 1=有效数据，0=无效/未收到数据
 * @note 使用示例：
 *       float speed;
 *       if (odrive_get_speed(&speed)) {
 *           printf("Speed: %.2f r/s\n", speed);
 *       }
 */
uint8 odrive_get_speed(float *out_rps);

/**
 * @brief 停止ODrive电机（设置力矩为0）
 */
void odrive_stop(void);

#endif /* DRIVER_ODRIVE_H */
