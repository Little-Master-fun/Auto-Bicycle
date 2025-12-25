/* yis_imu.h */
#ifndef _YIS_IMU_H_
#define _YIS_IMU_H_

#include "zf_common_headfile.h"

/* ================= IMU 数据结构 ================= */
typedef struct
{
    float pitch;   // 俯仰角 (deg)
    float roll;    // 横滚角 (deg)
    float yaw;     // 航向角 (deg)

    float wx;      // X轴角速度 (deg/s)
    float wy;      // Y轴角速度 (deg/s)
    float wz;      // Z轴角速度 (deg/s)

} yis_imu_t;

/* ================= 全局 IMU 数据 ================= */
extern yis_imu_t yis_imu;

/* ================= 接口函数 ================= */
void yis_init(void);          // 初始化 IMU（UART + 中断）
void yis_uart_rx_handler(void); // UART 接收中断回调

#endif
