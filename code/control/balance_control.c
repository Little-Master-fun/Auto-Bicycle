/* balance_control.c */
#include "balance_control.h"
#include "driver_imu.h"
#include <string.h>

#define BALANCE_IMU_SCALE              (0.35f)
#define BALANCE_ANGLE_TICK_DIV         (3u)   /* run angle loop every 3 * 5ms = 15ms */

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

static AttitudeData_t attitude_data;
static LowPassFilter_t gyr_lpf = { 0.0f, 0.06f };

static AnglePID_t angle_pid =
{
    1.0f, 0.0f, 0.0f,
    0.0f, 0.0f,
    10000.0f, 100.0f
};

static AngularVelocityPID_t velocity_pid =
{
    -9.8f, 0.10f, 0.0f,
    0.0f, 0.0f,
    10000.0f, 100.0f
};

static float target_angle = 0.35f;
static float target_angular_velocity = 0.0f;
static float target_angular_acceleration = 0.0f;
static float control_output = 0.0f;

static inline float constrain_float(float value, float min_val, float max_val)
{
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

static float pid_update(float error, float *integral, float *last_error,
                        float kp, float ki, float kd, float dt,
                        float max_integral, float output_limit)
{
    float p_term = kp * error;

    *integral += error * dt;
    *integral = constrain_float(*integral, -max_integral, max_integral);
    float i_term = ki * (*integral);

    float derivative = (error - *last_error) / dt;
    *last_error = error;
    float d_term = kd * derivative;

    return constrain_float(p_term + i_term + d_term, -output_limit, output_limit);
}

static float low_pass_filter(LowPassFilter_t *lpf, float input)
{
    lpf->last_value = lpf->last_value * (1.0f - lpf->alpha) + input * lpf->alpha;
    return lpf->last_value;
}

static void read_imu_data(void)
{
    attitude_data.gyr[0] = yis_imu.wx;
    attitude_data.gyr[1] = yis_imu.wy;
    attitude_data.gyr[2] = yis_imu.wz;

    attitude_data.gyr_filtered[0] = low_pass_filter(&gyr_lpf, attitude_data.gyr[0]);

    attitude_data.eul[0] = yis_imu.roll;
    attitude_data.eul[1] = yis_imu.pitch;
    attitude_data.eul[2] = yis_imu.yaw;
}

static void angle_loop_control(void)
{
    float dt = 1.0f;
    float current_angle = attitude_data.roll_filtered * BALANCE_IMU_SCALE;
    float angle_error = target_angle - current_angle;

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
    float dt = 1.0f;
    float current_angular_velocity = attitude_data.roll_rate * BALANCE_IMU_SCALE;
    float velocity_error = target_angular_velocity - current_angular_velocity;

    target_angular_acceleration = pid_update(velocity_error,
                                             &velocity_pid.integral,
                                             &velocity_pid.last_error,
                                             velocity_pid.kp,
                                             velocity_pid.ki,
                                             velocity_pid.kd,
                                             dt,
                                             velocity_pid.max_integral,
                                             velocity_pid.output_limit);

    control_output = -constrain_float(target_angular_acceleration, -3.0f, 3.0f);
}

void balance_control_init(void)
{
    memset(&attitude_data, 0, sizeof(attitude_data));
    gyr_lpf.last_value = 0.0f;

    angle_pid.integral = 0.0f;
    angle_pid.last_error = 0.0f;
    velocity_pid.integral = 0.0f;
    velocity_pid.last_error = 0.0f;

    target_angular_velocity = 0.0f;
    target_angular_acceleration = 0.0f;
    control_output = 0.0f;
}

void balance_control_update_5ms_isr(void)
{
    static uint8 tick = 0;

    read_imu_data();

    attitude_data.roll_filtered = attitude_data.eul[0];
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

void balance_control_set_target_angle(float angle_deg)
{
    target_angle = angle_deg;
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
    out_state->control_output = control_output;
}

float balance_control_get_output(void)
{
    return control_output;
}
