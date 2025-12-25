/* balance_control.h */
#ifndef BALANCE_CONTROL_H
#define BALANCE_CONTROL_H

#include "zf_common_headfile.h"

typedef struct
{
    float roll_deg;
    float roll_filtered_deg;
    float roll_rate_deg_s;
    float target_angle;
    float target_rate;
    float control_output;
} balance_control_state_t;

void balance_control_init(void);
void balance_control_update_5ms_isr(void);

void balance_control_set_target_angle(float angle_deg);
float balance_control_get_target_angle(void);

void balance_control_get_state(balance_control_state_t *out_state);
float balance_control_get_output(void);

#endif
