/*********************************************************************************************************************
* UI Control Module - Bike Balance System Screen Display
* 
* 本文件是自平衡自行车系统的屏幕显示控制模块实现文件
* 
* 功能说明：
* 1. IPS200屏幕初始化和配置
* 2. 实时显示系统状态（角度、角速度、控制输出等）
* 3. 提供安全的屏幕显示函数，避免越界
* 
********************************************************************************************************************/

#include "ui_control.h"
#include "zf_device_ips200.h"
#include <string.h>

// ========== 静态变量 ==========
static uint32 last_display_update = 0;  // 上次屏幕更新时间

// ========== 初始化UI控制模块 ==========
void ui_control_init(void)
{
    printf("Initializing IPS200 Screen...\r\n");

    // 初始化IPS200屏幕
    ips200_init(IPS200_TYPE);

    // 设置显示方向为竖屏
    ips200_set_dir(IPS200_PORTAIT);

    // 设置默认颜色：白色前景，黑色背景
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);

    // 设置字体：8x16字体
    ips200_set_font(IPS200_8X16_FONT);

    // 清空屏幕
    ips200_clear();

    printf("IPS200 Screen initialized successfully.\r\n");
}

// ========== 显示启动画面 ==========
void ui_control_show_splash(void)
{
    ips200_clear();
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(40, 100, "Bike Balance System");
    ips200_set_color(RGB565_GREEN, RGB565_BLACK);
    ips200_show_string(60, 120, "Initializing...");
}

// ========== 安全显示字符串 ==========
void ui_safe_show_string(uint16 x, uint16 y, const char *str)
{
    // 检查坐标是否在有效范围内
    if (x >= ips200_width_max || y >= ips200_height_max) {
        return;
    }

    // 计算字符串显示后的x边界（8x16字体，每个字符宽8像素）
    uint16 str_width = strlen(str) * 8;
    if (x + str_width >= ips200_width_max) {
        // 如果超出右边界，调整x坐标
        x = ips200_width_max - str_width - 1;
    }

    ips200_show_string(x, y, str);
}

// ========== 安全显示浮点数 ==========
void ui_safe_show_float(uint16 x, uint16 y, float value, uint8 int_digits, uint8 dec_digits)
{
    // 检查坐标有效性
    if (x >= ips200_width_max || y >= ips200_height_max) {
        return;
    }

    // 计算浮点数显示的大致宽度（+2 for decimal point and sign）
    uint16 max_width = (int_digits + dec_digits + 2) * 8;
    if (x + max_width >= ips200_width_max) {
        x = ips200_width_max - max_width - 1;
    }

    ips200_show_float(x, y, value, int_digits, dec_digits);
}

// ========== 清空屏幕 ==========
void ui_clear_screen(void)
{
    ips200_clear();
}

// ========== 设置屏幕颜色 ==========
void ui_set_color(uint16 pen_color, uint16 bg_color)
{
    ips200_set_color(pen_color, bg_color);
}

// ========== 更新屏幕显示 ==========
void ui_control_update(const ui_display_data_t *data)
{
    uint32 current_time = system_getval_ms();

    // 检查是否到达更新间隔（第一次调用时强制更新）
    if (last_display_update != 0 && (current_time - last_display_update) < UI_UPDATE_INTERVAL_MS) {
        return;
    }

    last_display_update = current_time;

    // 清空屏幕
    ips200_clear();

    // ========== 显示标题 ==========
    ips200_set_color(RGB565_CYAN, RGB565_BLACK);
    ips200_show_string(35, 2, "Balance Ctrl");
    
    uint16 y = 20;

    // ========== 显示系统状态 ==========
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(5, y, "Status:");
    if (data->system_status == 0) {
        ips200_set_color(RGB565_GREEN, RGB565_BLACK);
        ips200_show_string(60, y, "RUN ");
    } else {
        ips200_set_color(RGB565_RED, RGB565_BLACK);
        ips200_show_string(60, y, "STOP");
    }
    y += 18;

    // ========== 显示实时数据 ==========
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(5, y, "Ang:");
    ips200_show_float(45, y, data->angle, 2, 2);
    ips200_show_string(100, y, "deg");
    y += 16;

    ips200_show_string(5, y, "Rate:");
    ips200_show_float(45, y, data->angular_velocity, 3, 1);
    y += 16;

    ips200_show_string(5, y, "Out:");
    ips200_show_float(45, y, data->control_output, 1, 3);
    ips200_show_string(100, y, "Nm");
    y += 18;

    // ========== 显示PID参数 ==========
    ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
    ips200_show_string(5, y, "-- PID Params --");
    y += 16;

    // 角度Kp
    if (data->selected_param == 0) {
        ips200_set_color(RGB565_GREEN, RGB565_BLACK);  // 选中显示绿色
        ips200_show_string(5, y, ">");
    } else {
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        ips200_show_string(5, y, " ");
    }
    ips200_show_string(15, y, "AKp:");
    ips200_show_float(50, y, data->angle_kp, 1, 2);
    y += 16;

    // 速度Kp
    if (data->selected_param == 1) {
        ips200_set_color(RGB565_GREEN, RGB565_BLACK);
        ips200_show_string(5, y, ">");
    } else {
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        ips200_show_string(5, y, " ");
    }
    ips200_show_string(15, y, "VKp:");
    ips200_show_float(50, y, data->velocity_kp, 2, 2);
    y += 16;

    // 速度Ki
    if (data->selected_param == 2) {
        ips200_set_color(RGB565_GREEN, RGB565_BLACK);
        ips200_show_string(5, y, ">");
    } else {
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        ips200_show_string(5, y, " ");
    }
    ips200_show_string(15, y, "VKi:");
    ips200_show_float(50, y, data->velocity_ki, 1, 3);
    y += 18;

    // ========== 显示操作提示 ==========
    ips200_set_color(RGB565_GRAY, RGB565_BLACK);
    ips200_show_string(5, y, "K1:ON/OFF");
    y += 14;
    ips200_show_string(5, y, "K2:Select K3:+ K4:-");
}
