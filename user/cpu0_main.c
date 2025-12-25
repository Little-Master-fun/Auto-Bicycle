/*********************************************************************************************************************
* TC387 Opensourec Library ����TC387 ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ��� TC387 ��Դ���һ����
*
* TC387 ��Դ�� ���������
* �����Ը���������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù�������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
*
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
*
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
*
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ����֤Э�� ������������Ϊ���İ汾
* ��������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ����֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
*
* �ļ�����          cpu0_main
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          ADS v1.10.2
* ����ƽ̨          TC387QP
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2022-11-04       pudding            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "driver_imu.h"
#include "driver_servo.h"
#include "driver_motor.h"
#include "driver_odrive.h"
#include "zf_device_key.h"        // 使用库的按键驱动
#include "balance_control.h"
#include "ui_control.h"

// ========== 全局变量 ==========
uint8 system_enable = 0;     // 系统使能标志（0=停止，1=运行）
uint8 param_index = 0;       // 当前调节的参数索引（0=角度Kp, 1=速度Kp, 2=速度Ki）
static const char* param_names[] = {"Angle Kp", "Velocity Kp", "Velocity Ki"};


#pragma section all "cpu0_dsram"
// ���������#pragma section all restore���֮���ȫ�ֱ���������CPU0��RAM��

// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������


// **************************** �������� ****************************
int core0_main(void)
{
    clock_init();                   // ��ȡʱ��Ƶ��<��ر���>
    debug_init();                   // ��ʼ��Ĭ�ϵ��Դ���
    
    // 硬件驱动初始化
    gpio_init(P20_9, GPO, GPIO_LOW, GPO_PUSH_PULL);  // LED指示灯初始化
    servo_init(90.0f);              // 初始化舵机（中心位置）
    motor_init();                   // 初始化电机（停止状态）
    odrive_init();                  // 初始化ODrive动量轮
    key_init(10);                   // 初始化按键（10ms扫描周期）
    
    // 传感器和控制初始化
    yis_init();                     // 初始化IMU
    balance_control_init();         // 初始化平衡控制
    
    // 人机交互初始化
    ui_control_init();              // 初始化屏幕显示
    ui_control_show_splash();       // 显示启动画面
    
    // 定时器初始化
    pit_ms_init(CCU60_CH0, 5);      // 5ms - IMU + balance control update
    // �˴���д�û����� ���������ʼ�������



    // �˴���д�û����� ���������ʼ�������
    cpu_wait_event_ready();         // �ȴ����к��ĳ�ʼ�����
    
    printf("System initialized successfully!\r\n");
    printf("Servo and Motor drivers ready.\r\n");
    
    while (TRUE)
    {
        // 按键扫描（库会自动根据period处理时序）
        key_scanner();
        
        // 按键事件处理
        if (key_get_state(KEY_1) == KEY_SHORT_PRESS) {
            // K1: 启动/停止系统
            key_clear_state(KEY_1);  // 清除按键状态
            system_enable = !system_enable;
            balance_control_set_enable(system_enable);  // 设置控制使能
            if (system_enable) {
                printf("System ENABLED\r\n");
                gpio_high(P20_9);  // LED点亮表示运行
            } else {
                printf("System STOPPED\r\n");
                gpio_low(P20_9);   // LED熄灭表示停止
            }
        }
        
        if (key_get_state(KEY_2) == KEY_SHORT_PRESS) {
            // K2: 切换调节参数
            key_clear_state(KEY_2);
            param_index = (param_index + 1) % 3;
            float angle_kp, vel_kp, vel_ki;
            balance_control_get_pid_params(&angle_kp, &vel_kp, &vel_ki);
            printf("\r\n=== Select: %s ===\r\n", param_names[param_index]);
            printf("Angle Kp=%.3f, Vel Kp=%.3f, Vel Ki=%.3f\r\n", angle_kp, vel_kp, vel_ki);
        }
        
        if (key_get_state(KEY_3) == KEY_SHORT_PRESS) {
            // K3: 增大当前参数
            key_clear_state(KEY_3);
            float angle_kp, vel_kp, vel_ki;
            if (param_index == 0) {
                balance_control_adjust_angle_kp(0.1f);   // 角度Kp步长0.1
            } else if (param_index == 1) {
                balance_control_adjust_velocity_kp(-0.1f);  // 速度Kp步长0.5（负数，所以减去负值=增大绝对值）
            } else {
                balance_control_adjust_velocity_ki(0.001f);  // 速度Ki步长0.01
            }
            balance_control_get_pid_params(&angle_kp, &vel_kp, &vel_ki);
            printf("+ %s: Kp=%.3f, Vel Kp=%.3f, Ki=%.3f\r\n", param_names[param_index], angle_kp, vel_kp, vel_ki);
        }
        
        if (key_get_state(KEY_4) == KEY_SHORT_PRESS) {
            // K4: 减小当前参数
            key_clear_state(KEY_4);
            float angle_kp, vel_kp, vel_ki;
            if (param_index == 0) {
                balance_control_adjust_angle_kp(-0.1f);  // 角度Kp步长0.1
            } else if (param_index == 1) {
                balance_control_adjust_velocity_kp(0.1f);   // 速度Kp步长0.5（负数，所以加正值=减小绝对值）
            } else {
                balance_control_adjust_velocity_ki(-0.001f); // 速度Ki步长0.01
            }
            balance_control_get_pid_params(&angle_kp, &vel_kp, &vel_ki);
            printf("- %s: Kp=%.3f, Vel Kp=%.3f, Ki=%.3f\r\n", param_names[param_index], angle_kp, vel_kp, vel_ki);
        }
        
        system_delay_ms(10);  // 10ms扫描周期
    }
}
#pragma section all restore
// **************************** �������� ****************************
