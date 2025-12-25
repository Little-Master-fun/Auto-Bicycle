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
#include "balance_control.h"
#include "ui_control.h"


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
    servo_init(90.0f);              // 初始化舵机（中心位置）
    motor_init();                   // 初始化电机（停止状态）
    
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
        // 示例：可以在这里添加舵机和电机控制代码
        // 例如：
        // servo_set_angle(90.0f);      // 设置舵机角度到90度
        // motor_forward(50);           // 电机以50%速度前进
        // motor_stop();                // 停止电机
        
        system_delay_ms(10);
    }
}
#pragma section all restore
// **************************** �������� ****************************
