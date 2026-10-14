#include "pid.h"

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

void pid_init(pid_ctrl_t *pid, float kp, float ki, float kd, float out_limit, float d_alpha)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_limit = out_limit;
    pid->d_alpha = d_alpha;
    pid_reset(pid);
}

void pid_reset(pid_ctrl_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->d_filtered = 0.0f;
    pid->has_prev = 0;
}

float pid_update(pid_ctrl_t *pid, float error, float dt)
{
    if (dt <= 0.0f) {
        return 0.0f;
    }

    if (pid->has_prev) {
        float raw_d = (error - pid->prev_error) / dt;
        pid->d_filtered += pid->d_alpha * (raw_d - pid->d_filtered);
    }
    pid->prev_error = error;
    pid->has_prev = 1;

    float p_term = pid->kp * error;
    float d_term = pid->kd * pid->d_filtered;
    float i_step = pid->ki * error * dt;
    float unsaturated = p_term + pid->integral + i_step + d_term;

    int pushing_high = unsaturated > pid->out_limit && error > 0.0f;
    int pushing_low = unsaturated < -pid->out_limit && error < 0.0f;
    if (!pushing_high && !pushing_low) {
        pid->integral += i_step;
    }

    return clampf(p_term + pid->integral + d_term, -pid->out_limit, pid->out_limit);
}
