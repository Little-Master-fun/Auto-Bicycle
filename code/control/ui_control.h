/*********************************************************************************************************************
* UI Control Module - Bike Balance System Screen Display
* 
* 本文件是自平衡自行车系统的屏幕显示控制模块头文件
* 
* 功能说明：
* 1. IPS200屏幕初始化和配置
* 2. 实时显示系统状态（角度、角速度、控制输出等）
* 3. 提供安全的屏幕显示函数，避免越界
* 
********************************************************************************************************************/

#ifndef UI_CONTROL_H
#define UI_CONTROL_H

#include "zf_common_headfile.h"

// ========== 屏幕配置 ==========
#define IPS200_TYPE             (IPS200_TYPE_SPI)    // 根据实际屏幕选择：IPS200_TYPE_SPI 或 IPS200_TYPE_PARALLEL8

// ========== 显示布局配置 ==========
#define UI_UPDATE_INTERVAL_MS   (500)                // 屏幕更新间隔（毫秒）
#define UI_TITLE_Y              (5)                  // 标题Y坐标
#define UI_DATA_START_Y         (30)                 // 数据显示起始Y坐标
#define UI_LINE_HEIGHT          (16)                 // 行高
#define UI_COL1_X               (5)                  // 第一列：标签
#define UI_COL2_X               (80)                 // 第二列：数值
#define UI_COL3_X               (140)                // 第三列：单位

// ========== 显示数据结构 ==========
typedef struct {
    float angle;                // 当前角度（度）
    float angular_velocity;     // 当前角速度（度/秒）
    float control_output;       // 控制输出（力矩）
    uint8 system_status;        // 系统状态：0=运行，1=停止
    float target_angle;         // 目标角度（度）
    // PID参数
    float angle_kp;             // 角度环Kp
    float velocity_kp;          // 速度环Kp
    float velocity_ki;          // 速度环Ki
    uint8 selected_param;       // 当前选中的参数（0=角度Kp, 1=速度Kp, 2=速度Ki）
} ui_display_data_t;

// ========== 外部变量声明 ==========
extern uint16 ips200_width_max;     // 屏幕宽度
extern uint16 ips200_height_max;    // 屏幕高度

// ========== 函数声明 ==========

/**
 * @brief 初始化UI控制模块和IPS200屏幕
 * @note 需要在系统初始化时调用一次
 */
void ui_control_init(void);

/**
 * @brief 更新屏幕显示内容（建议在定时中断或主循环中调用）
 * @param data 要显示的数据结构指针
 * @note 内部会自动控制刷新频率，避免频繁更新
 */
void ui_control_update(const ui_display_data_t *data);

/**
 * @brief 显示启动画面
 * @note 通常在初始化完成后调用
 */
void ui_control_show_splash(void);

/**
 * @brief 安全显示字符串（带边界检查）
 * @param x X坐标
 * @param y Y坐标
 * @param str 要显示的字符串
 */
void ui_safe_show_string(uint16 x, uint16 y, const char *str);

/**
 * @brief 安全显示浮点数（带边界检查）
 * @param x X坐标
 * @param y Y坐标
 * @param value 要显示的浮点数值
 * @param int_digits 整数位数
 * @param dec_digits 小数位数
 */
void ui_safe_show_float(uint16 x, uint16 y, float value, uint8 int_digits, uint8 dec_digits);

/**
 * @brief 清空屏幕
 */
void ui_clear_screen(void);

/**
 * @brief 设置屏幕颜色
 * @param pen_color 前景色
 * @param bg_color 背景色
 */
void ui_set_color(uint16 pen_color, uint16 bg_color);

#endif // UI_CONTROL_H
