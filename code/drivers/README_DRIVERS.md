# 舵机和电机驱动使用说明

## 概述

已成功将舵机和电机驱动从 test 项目迁移到 Bicycle 项目，并放置在 `Bicycle\code\drivers` 目录下。

## 文件结构

```
Bicycle/code/drivers/
├── driver_servo.h      - 舵机驱动头文件
├── driver_servo.c      - 舵机驱动实现
├── driver_motor.h      - 电机驱动头文件
└── driver_motor.c      - 电机驱动实现
```

## 舵机驱动 (driver_servo)

### 硬件配置

- **PWM 引脚**: P02_7 (ATOM1_CH7)
- **PWM 频率**: 50Hz
- **脉宽范围**: 0.5ms - 2.5ms
- **角度范围**: 0° - 180°

### 主要函数

```c
// 初始化舵机（在cpu0_main中已自动调用）
void servo_init(float init_angle);

// 设置舵机角度（0-180度）
void servo_set_angle(float angle);

// 设置舵机到中心位置（90度）
void servo_set_center(void);

// 获取当前舵机角度
float servo_get_angle(void);
```

### 使用示例

```c
// 设置舵机到不同角度
servo_set_angle(45.0f);    // 左转
servo_set_center();        // 回中
servo_set_angle(135.0f);   // 右转

// 获取当前角度
float current = servo_get_angle();
printf("Current angle: %.2f\r\n", current);
```

## 电机驱动 (driver_motor)

### 硬件配置

- **ESC PWM 引脚**: P02_6 (ATOM0_CH6)
- **PWM 频率**: 50Hz
- **脉宽范围**: 1000us - 2000us (中立点 1500us)
- **速度范围**: -100% 到 +100%

### 控制模式

电机通过 ESC（电子调速器）进行开环控制，支持：

- **前进**: 正速度值 (0-100%)
- **后退**: 负速度值 (-100-0%)
- **停止**: 速度为 0

### 主要函数

```c
// 初始化电机（在cpu0_main中已自动调用）
void motor_init(void);

// 设置电机速度（-100到+100）
void motor_set_speed(int16 speed);

// 停止电机
void motor_stop(void);

// 前进（速度0-100）
void motor_forward(uint8 speed);

// 后退（速度0-100）
void motor_backward(uint8 speed);

// 获取当前速度和状态
int16 motor_get_speed(void);
motor_state_enum motor_get_state(void);
```

### 使用示例

```c
// 方式1：使用统一的速度设置函数
motor_set_speed(50);      // 50%速度前进
motor_set_speed(-30);     // 30%速度后退
motor_set_speed(0);       // 停止

// 方式2：使用专用的前进/后退函数
motor_forward(80);        // 80%速度前进
motor_backward(60);       // 60%速度后退
motor_stop();             // 停止

// 获取状态
int16 speed = motor_get_speed();
motor_state_enum state = motor_get_state();

switch(state) {
    case MOTOR_STATE_STOP:
        printf("Motor stopped\r\n");
        break;
    case MOTOR_STATE_FORWARD:
        printf("Motor forward at %d%%\r\n", speed);
        break;
    case MOTOR_STATE_BACKWARD:
        printf("Motor backward at %d%%\r\n", -speed);
        break;
}
```

## 集成说明

### 初始化顺序

在 `cpu0_main.c` 中的初始化顺序：

1. 系统时钟和调试串口
2. **硬件驱动**（舵机、电机）
3. 传感器和控制（IMU、平衡控制）
4. 人机交互（屏幕显示）
5. 定时器中断

### 头文件引用

```c
#include "driver_servo.h"
#include "driver_motor.h"
```

## 注意事项

### 舵机

- 角度自动限制在 0-180 度范围内
- 建议使用 90 度作为中心位置
- PWM 频率固定为 50Hz（标准舵机频率）

### 电机

- 使用 ESC 控制，确保 ESC 已正确校准
- 速度自动限制在-100%到+100%范围内
- 中立点为 1500us（停止状态）
- 建议在使用前先测试电机方向是否正确

## 完整示例代码

```c
// 在主循环中使用舵机和电机
while (TRUE)
{
    // 示例1：固定舵机角度，电机前进
    servo_set_angle(90.0f);
    motor_forward(50);
    system_delay_ms(2000);

    // 示例2：舵机转向，电机减速
    servo_set_angle(60.0f);
    motor_forward(30);
    system_delay_ms(2000);

    // 示例3：停止电机，舵机回中
    motor_stop();
    servo_set_center();
    system_delay_ms(1000);

    // 示例4：后退
    motor_backward(40);
    system_delay_ms(1000);
    motor_stop();

    system_delay_ms(10);
}
```

## 进阶使用

### 结合平衡控制

```c
// 根据平衡控制输出调整电机速度
float control_output = balance_control_get_output();
int16 motor_speed = (int16)(control_output * 10);  // 缩放到合适范围
motor_set_speed(motor_speed);
```

### 结合屏幕显示

电机和舵机状态可以通过 `ui_control` 模块显示在屏幕上。

## 故障排查

1. **舵机不动作**

   - 检查 PWM 引脚连接是否正确（P02_7）
   - 检查舵机电源供电
   - 使用示波器检查 PWM 波形

2. **电机不转**

   - 检查 ESC 连接是否正确（P02_6）
   - 确认 ESC 已上电并完成初始化
   - 检查电机电源供电
   - 验证 ESC 是否已校准

3. **电机方向反了**
   - 在代码中对速度值取反
   - 或者交换电机的任意两根相线

## 技术支持

如有问题，请检查：

- 硬件连接
- PWM 信号波形
- 电源供电情况
