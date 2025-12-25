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

    // 检查是否到达更新间隔
    if ((current_time - last_display_update) < UI_UPDATE_INTERVAL_MS) {
        return;
    }

    last_display_update = current_time;

    // 清空屏幕
    ips200_clear();

    // 显示标题
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ui_safe_show_string(40, UI_TITLE_Y, "Bike Balance");

    // 初始化显示位置
    uint16 current_y = UI_DATA_START_Y;

    // ========== 显示目标角度 ==========
    if (current_y < ips200_height_max) {
        ui_safe_show_string(UI_COL1_X, current_y, "Target:");
        ui_safe_show_float(UI_COL2_X, current_y, data->target_angle, 2, 2);
        ui_safe_show_string(UI_COL3_X, current_y, "deg");
        current_y += UI_LINE_HEIGHT;
    }

    // ========== 显示当前角度 ==========
    if (current_y < ips200_height_max) {
        ui_safe_show_string(UI_COL1_X, current_y, "Angle:");
        ui_safe_show_float(UI_COL2_X, current_y, data->angle, 3, 2);
        ui_safe_show_string(UI_COL3_X, current_y, "deg");
        current_y += UI_LINE_HEIGHT;
    }

    // ========== 显示角速度 ==========
    if (current_y < ips200_height_max) {
        ui_safe_show_string(UI_COL1_X, current_y, "Rate:");
        ui_safe_show_float(UI_COL2_X, current_y, data->angular_velocity, 3, 2);
        ui_safe_show_string(UI_COL3_X, current_y, "deg/s");
        current_y += UI_LINE_HEIGHT;
    }

    // ========== 显示控制输出 ==========
    if (current_y < ips200_height_max) {
        ui_safe_show_string(UI_COL1_X, current_y, "Output:");
        ui_safe_show_float(UI_COL2_X, current_y, data->control_output, 2, 4);
        ui_safe_show_string(UI_COL3_X, current_y, "Nm");
        current_y += UI_LINE_HEIGHT;
    }

    // ========== 显示系统状态 ==========
    if (current_y < ips200_height_max) {
        ui_safe_show_string(UI_COL1_X, current_y, "Status:");
        
        if (data->system_status == 0) {
            // 运行状态 - 绿色显示
            ips200_set_color(RGB565_GREEN, RGB565_BLACK);
            ui_safe_show_string(UI_COL2_X, current_y, "RUN ");
        } else {
            // 停止状态 - 红色显示
            ips200_set_color(RGB565_RED, RGB565_BLACK);
            ui_safe_show_string(UI_COL2_X, current_y, "STOP");
        }
        
        // 恢复默认颜色
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        current_y += UI_LINE_HEIGHT;
    }
}
