#ifndef LF_PID_H
#define LF_PID_H

typedef struct {
    float kp;
    float ki;
    float kd;
    float out_limit;
    float d_alpha;

    float integral;
    float prev_error;
    float d_filtered;
    int has_prev;
} pid_ctrl_t;

void pid_init(pid_ctrl_t *pid, float kp, float ki, float kd, float out_limit, float d_alpha);
void pid_reset(pid_ctrl_t *pid);
float pid_update(pid_ctrl_t *pid, float error, float dt);

#endif
