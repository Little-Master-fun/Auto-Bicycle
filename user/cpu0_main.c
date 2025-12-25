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
uint8 param_index = 0;       // 当前调节的参数索引（0~5）
static const char* param_names[] = {
    "Angle Kp",      // 0: 角度环比例
    "Angle Ki",      // 1: 角度环积分
    "Angle Kd",      // 2: 角度环微分
    "Velocity Kp",   // 3: 速度环比例
    "Velocity Ki",   // 4: 速度环积分
    "Velocity Kd"    // 5: 速度环微分
};


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
    
    uint32 last_speed_request = 0;  // 上次轮速请求时间
    
    while (TRUE)
    {
        // 高频轮询ODrive串口数据（每个循环都调用，非阻塞）
        odrive_poll();
        
        // 低频发送轮速查询请求（每50ms发一次，20Hz）
        uint32 current_time = system_getval_ms();
        if (current_time - last_speed_request >= 50) {
            last_speed_request = current_time;
            odrive_request_speed();  // 仅发送命令，不阻塞
        }
        
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
            param_index = (param_index + 1) % 6;  // 现在有6个参数
            float angle_kp, angle_ki, angle_kd, vel_kp, vel_ki, vel_kd;
            balance_control_get_pid_params_full(&angle_kp, &angle_ki, &angle_kd, &vel_kp, &vel_ki, &vel_kd);
            printf("\r\n=== Select: %s ===\r\n", param_names[param_index]);
            printf("Angle: Kp=%.3f Ki=%.3f Kd=%.3f\r\n", angle_kp, angle_ki, angle_kd);
            printf("Vel:   Kp=%.3f Ki=%.4f Kd=%.3f\r\n", vel_kp, vel_ki, vel_kd);
        }
        
        if (key_get_state(KEY_3) == KEY_SHORT_PRESS) {
            // K3: 增大当前参数
            key_clear_state(KEY_3);
            
            switch (param_index) {
                case 0: balance_control_adjust_angle_kp(0.1f); break;      // 角度Kp 步长0.1
                case 1: balance_control_adjust_angle_ki(0.05f); break;     // 角度Ki 步长0.05
                case 2: balance_control_adjust_angle_kd(0.01f); break;     // 角度Kd 步长0.01
                case 3: balance_control_adjust_velocity_kp(-0.1f); break;  // 速度Kp 步长0.1（负数，减负=增大绝对值）
                case 4: balance_control_adjust_velocity_ki(0.001f); break; // 速度Ki 步长0.001
                case 5: balance_control_adjust_velocity_kd(0.01f); break;  // 速度Kd 步长0.01
            }
            
            float angle_kp, angle_ki, angle_kd, vel_kp, vel_ki, vel_kd;
            balance_control_get_pid_params_full(&angle_kp, &angle_ki, &angle_kd, &vel_kp, &vel_ki, &vel_kd);
            printf("+ %s -> A[%.2f,%.2f,%.2f] V[%.2f,%.3f,%.2f]\r\n", 
                   param_names[param_index], angle_kp, angle_ki, angle_kd, vel_kp, vel_ki, vel_kd);
        }
        
        if (key_get_state(KEY_4) == KEY_SHORT_PRESS) {
            // K4: 减小当前参数
            key_clear_state(KEY_4);
            
            switch (param_index) {
                case 0: balance_control_adjust_angle_kp(-0.1f); break;     // 角度Kp 步长0.1
                case 1: balance_control_adjust_angle_ki(-0.05f); break;    // 角度Ki 步长0.05
                case 2: balance_control_adjust_angle_kd(-0.01f); break;    // 角度Kd 步长0.01
                case 3: balance_control_adjust_velocity_kp(0.1f); break;   // 速度Kp 步长0.1（负数，加正=减小绝对值）
                case 4: balance_control_adjust_velocity_ki(-0.001f); break;// 速度Ki 步长0.001
                case 5: balance_control_adjust_velocity_kd(-0.01f); break; // 速度Kd 步长0.01
            }
            
            float angle_kp, angle_ki, angle_kd, vel_kp, vel_ki, vel_kd;
            balance_control_get_pid_params_full(&angle_kp, &angle_ki, &angle_kd, &vel_kp, &vel_ki, &vel_kd);
            printf("- %s -> A[%.2f,%.2f,%.2f] V[%.2f,%.3f,%.2f]\r\n", 
                   param_names[param_index], angle_kp, angle_ki, angle_kd, vel_kp, vel_ki, vel_kd);
        }
        
        system_delay_ms(10);  // 10ms扫描周期
    }
}
#pragma section all restore
// **************************** �������� ****************************
