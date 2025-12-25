/* balance_control.c */
#include "balance_control.h"
#include "driver_imu.h"
#include "driver_odrive.h"
#include <string.h>
#include <math.h>

/* =========================
 * Timing
 * ========================= */
#define BALANCE_CTRL_DT_S              (0.005f)  /* 5ms ISR */
#define BALANCE_ANGLE_TICK_DIV         (3u)      /* run angle loop every 3*5ms=15ms */
#define BALANCE_ANGLE_DT_S             (BALANCE_CTRL_DT_S * (float)BALANCE_ANGLE_TICK_DIV)

/* =========================
 * Units / scaling
 * =========================
 * IMPORTANT:
 *  - If yis_imu.roll is in deg and yis_imu.wx is in deg/s -> set to 1.0f
 *  - If they are in rad / rad/s -> set to 1.0f too, but your tuning + safety thresholds must match rad.
 */
#define BALANCE_IMU_SCALE              (1.0f)

/* Optional safety: stop if tilt too large (in SAME UNIT as scaled roll) */
#define BALANCE_FALL_ANGLE_LIMIT       (35.0f)   /* e.g. 35deg if using deg */

/* =========================
 * Output limits
 * ========================= */
#define BALANCE_TORQUE_LIMIT           (3.0f)    /* final torque command limit to ODrive */

/* =========================
 * Data structures
 * ========================= */
typedef struct
{
    float acc[3];
    float gyr[3];
    float gyr_filtered[3];
    float eul[3];
    float roll_rate;
    float roll_filtered;
} AttitudeData_t;

typedef struct
{
    float last_value;
    float alpha;
} LowPassFilter_t;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float last_error;
    float max_integral;
    float output_limit;
} AnglePID_t;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float last_error;
    float max_integral;
    float output_limit;
} AngularVelocityPID_t;

/* =========================
 * Static variables
 * ========================= */
static AttitudeData_t attitude_data;

/* Gyro LPF: alpha越大越“快”，滞后越小，但噪声更大。建议 0.15~0.35 起步 */
static LowPassFilter_t gyr_lpf = { 0.0f, 0.20f };

/* 外环：角度 -> 目标角速度（单位随你的IMU） */
static AnglePID_t angle_pid =
{
    /* kp   ki   kd */
    1.0f, 0.0f, 0.0f,

    /* integral, last_error */
    0.0f, 0.0f,

    /* max_integral, output_limit */
    50.0f, 80.0f   /* output_limit 是“目标角速度”的最大值 */
};

/* 内环：角速度 -> 力矩命令（最终会被限制到 ±BALANCE_TORQUE_LIMIT）
 * 注意 kp 符号必须和你的坐标/电机方向一致：方向错了怎么调都不稳。
 */
static AngularVelocityPID_t velocity_pid =
{
    /* kp     ki    kd */
    -1.0f, 0.00f, 0.0f,

    /* integral, last_error */
    0.0f, 0.0f,

    /* max_integral, output_limit */
    10.0f, BALANCE_TORQUE_LIMIT
};

static float target_angle = 0.4f;                 /* 目标角度偏置：0.4度 */
static float target_angular_velocity = 0.0f;      /* 外环输出 */
static float torque_cmd = 0.0f;                   /* 最终输出给 ODrive 的扭矩命令 */
static uint8 control_enable = 0;                  /* 控制使能标志 */

/* =========================
 * Utilities
 * ========================= */
static inline float constrain_float(float value, float min_val, float max_val)
{
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

/* PID with simple conditional integration anti-windup */
static float pid_update(float error, float *integral, float *last_error,
                        float kp, float ki, float kd, float dt,
                        float max_integral, float output_limit)
{
    if (dt <= 0.0f) dt = 1e-6f;

    float p_term = kp * error;

    float derivative = (error - *last_error) / dt;
    *last_error = error;
    float d_term = kd * derivative;

    /* Predict output without adding more integral */
    float i_term = ki * (*integral);
    float out_pre = p_term + i_term + d_term;

    /* Determine saturation */
    uint8 saturated_hi = (out_pre > output_limit);
    uint8 saturated_lo = (out_pre < -output_limit);

    /* Only integrate if:
       - not saturated, OR
       - integral will help pull output back (error tends to reduce saturation)
    */
    uint8 allow_integrate = 0;
    if (!saturated_hi && !saturated_lo)
    {
        allow_integrate = 1;
    }
    else
    {
        /* If output is too high, we want error < 0 to pull it down
           If output is too low, we want error > 0 to pull it up
        */
        if (saturated_hi && (error < 0.0f)) allow_integrate = 1;
        if (saturated_lo && (error > 0.0f)) allow_integrate = 1;
    }

    if (allow_integrate)
    {
        *integral += error * dt;
        *integral = constrain_float(*integral, -max_integral, max_integral);
    }

    i_term = ki * (*integral);
    float output = p_term + i_term + d_term;
    return constrain_float(output, -output_limit, output_limit);
}

static float low_pass_filter(LowPassFilter_t *lpf, float input)
{
    lpf->last_value = lpf->last_value * (1.0f - lpf->alpha) + input * lpf->alpha;
    return lpf->last_value;
}

/* =========================
 * IMU read
 * ========================= */
static void read_imu_data(void)
{
    attitude_data.gyr[0] = yis_imu.wx;
    attitude_data.gyr[1] = yis_imu.wy;
    attitude_data.gyr[2] = yis_imu.wz;

    /* 这里只过滤 roll_rate 用到的 x 轴，其他轴你暂时没用到 */
    attitude_data.gyr_filtered[0] = low_pass_filter(&gyr_lpf, attitude_data.gyr[0]);
    attitude_data.gyr_filtered[1] = attitude_data.gyr[1];
    attitude_data.gyr_filtered[2] = attitude_data.gyr[2];

    attitude_data.eul[0] = yis_imu.roll;
    attitude_data.eul[1] = yis_imu.pitch;
    attitude_data.eul[2] = yis_imu.yaw;
}

/* =========================
 * Control loops
 * ========================= */
static void angle_loop_control(void)
{
    const float dt = BALANCE_ANGLE_DT_S;

    /* current_angle in your chosen unit (deg or rad) */
    float current_angle = attitude_data.roll_filtered * BALANCE_IMU_SCALE;
    float angle_error = target_angle - current_angle;

    /* Output: target angular velocity */
    target_angular_velocity = pid_update(angle_error,
                                         &angle_pid.integral,
                                         &angle_pid.last_error,
                                         angle_pid.kp,
                                         angle_pid.ki,
                                         angle_pid.kd,
                                         dt,
                                         angle_pid.max_integral,
                                         angle_pid.output_limit);
}

static void velocity_loop_control(void)
{
    const float dt = BALANCE_CTRL_DT_S;

    float current_rate = attitude_data.roll_rate * BALANCE_IMU_SCALE;
    float rate_error = target_angular_velocity - current_rate;

    /* IMPORTANT:
       velocity_pid.output_limit MUST match actuator limit (torque limit),
       otherwise windup still happens.
    */
    float torque = pid_update(rate_error,
                              &velocity_pid.integral,
                              &velocity_pid.last_error,
                              velocity_pid.kp,
                              velocity_pid.ki,
                              velocity_pid.kd,
                              dt,
                              velocity_pid.max_integral,
                              velocity_pid.output_limit);

    torque_cmd = constrain_float(torque, -BALANCE_TORQUE_LIMIT, BALANCE_TORQUE_LIMIT);

    /* Safety: if fallen over, stop */
    {
        float roll_now = attitude_data.roll_filtered * BALANCE_IMU_SCALE;
        if (fabsf(roll_now) > BALANCE_FALL_ANGLE_LIMIT)
        {
            /* auto disable */
            control_enable = 0;
        }
    }

    /* Send to ODrive */
    if (control_enable)
    {
        odrive_set_torque(torque_cmd);
    }
    else
    {
        odrive_stop();
        torque_cmd = 0.0f;
    }

    /* Extra: if saturated too long and not correcting, decay integral a bit */
    static uint8 saturation_counter = 0;
    if ((fabsf(torque_cmd) > (BALANCE_TORQUE_LIMIT * 0.97f)) &&
        (rate_error * torque_cmd) > 0.0f) /* error and output same sign -> not correcting */
    {
        saturation_counter++;
        if (saturation_counter > 100) /* ~500ms */
        {
            velocity_pid.integral *= 0.5f;
            saturation_counter = 50;
        }
    }
    else
    {
        saturation_counter = 0;
    }
}

/* =========================
 * Public APIs
 * ========================= */
void balance_control_init(void)
{
    memset(&attitude_data, 0, sizeof(attitude_data));

    gyr_lpf.last_value = 0.0f;

    angle_pid.integral = 0.0f;
    angle_pid.last_error = 0.0f;

    velocity_pid.integral = 0.0f;
    velocity_pid.last_error = 0.0f;

    target_angular_velocity = 0.0f;
    torque_cmd = 0.0f;

    control_enable = 0;

    odrive_stop();
}

void balance_control_update_5ms_isr(void)
{
    static uint8 tick = 0;

    read_imu_data();

    /* Here we assume yis_imu.roll is already an estimated roll angle.
       If it is raw/unfiltered, you should run proper attitude estimation. */
    attitude_data.roll_filtered = attitude_data.eul[0];

    /* roll rate from gyro (filtered) */
    attitude_data.roll_rate = attitude_data.gyr_filtered[0];

    if (tick == 0u)
    {
        angle_loop_control();
    }

    velocity_loop_control();

    tick++;
    if (tick >= BALANCE_ANGLE_TICK_DIV)
    {
        tick = 0u;
    }
}

void balance_control_set_target_angle(float angle_deg_or_rad)
{
    target_angle = angle_deg_or_rad;
}

float balance_control_get_target_angle(void)
{
    return target_angle;
}

void balance_control_get_state(balance_control_state_t *out_state)
{
    if (out_state == NULL)
    {
        return;
    }

    out_state->roll_deg = attitude_data.eul[0] * BALANCE_IMU_SCALE;
    out_state->roll_filtered_deg = attitude_data.roll_filtered * BALANCE_IMU_SCALE;
    out_state->roll_rate_deg_s = attitude_data.roll_rate * BALANCE_IMU_SCALE;

    out_state->target_angle = target_angle;
    out_state->target_rate = target_angular_velocity;
    out_state->control_output = torque_cmd;
}

float balance_control_get_output(void)
{
    return torque_cmd;
}

void balance_control_set_enable(uint8 enable)
{
    control_enable = enable;

    if (!enable)
    {
        /* reset integrators when disabling */
        velocity_pid.integral = 0.0f;
        velocity_pid.last_error = 0.0f;

        angle_pid.integral = 0.0f;
        angle_pid.last_error = 0.0f;

        target_angular_velocity = 0.0f;
        torque_cmd = 0.0f;

        odrive_stop();
    }
}

uint8 balance_control_get_enable(void)
{
    return control_enable;
}

void balance_control_adjust_angle_kp(float delta)
{
    angle_pid.kp += delta;
    if (angle_pid.kp < -10.0f) angle_pid.kp = -10.0f;
    if (angle_pid.kp > 10.0f) angle_pid.kp = 10.0f;  // 限制最大值
}

void balance_control_adjust_angle_ki(float delta)
{
    angle_pid.ki += delta;
    if (angle_pid.ki < -10.0f) angle_pid.ki = -10.0f;
    if (angle_pid.ki > 5.0f) angle_pid.ki = 5.0f;  // 限制最大值
}

void balance_control_adjust_angle_kd(float delta)
{
    angle_pid.kd += delta;
    if (angle_pid.kd < -10.0f) angle_pid.kd = -10.0f;
    if (angle_pid.kd > 2.0f) angle_pid.kd = 2.0f;  // 限制最大值
}

void balance_control_adjust_velocity_kp(float delta)
{
    velocity_pid.kp += delta;
    if (velocity_pid.kp > 0.0f) velocity_pid.kp = 0.0f;     // Kp是负数
    if (velocity_pid.kp < -20.0f) velocity_pid.kp = -20.0f; // 限制最小值
}

void balance_control_adjust_velocity_ki(float delta)
{
    velocity_pid.ki += delta;
    if (velocity_pid.ki < -10.0f) velocity_pid.ki = -10.0f;
    if (velocity_pid.ki > 1.0f) velocity_pid.ki = 1.0f;  // 限制最大值
}

void balance_control_adjust_velocity_kd(float delta)
{
    velocity_pid.kd += delta;
    if (velocity_pid.kd < -10.0f) velocity_pid.kd = -10.0f;
    if (velocity_pid.kd > 1.0f) velocity_pid.kd = 1.0f;  // 限制最大值
}

void balance_control_get_pid_params(float *angle_kp, float *vel_kp, float *vel_ki)
{
    if (angle_kp) *angle_kp = angle_pid.kp;
    if (vel_kp) *vel_kp = velocity_pid.kp;
    if (vel_ki) *vel_ki = velocity_pid.ki;
}

void balance_control_get_pid_params_full(float *angle_kp, float *angle_ki, float *angle_kd,
                                          float *vel_kp, float *vel_ki, float *vel_kd)
{
    if (angle_kp) *angle_kp = angle_pid.kp;
    if (angle_ki) *angle_ki = angle_pid.ki;
    if (angle_kd) *angle_kd = angle_pid.kd;
    if (vel_kp) *vel_kp = velocity_pid.kp;
    if (vel_ki) *vel_ki = velocity_pid.ki;
    if (vel_kd) *vel_kd = velocity_pid.kd;
}
