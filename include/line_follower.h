#ifndef LF_LINE_FOLLOWER_H
#define LF_LINE_FOLLOWER_H

#include "line_sensor.h"
#include "pid.h"

typedef enum {
    LF_FOLLOW,
    LF_SEARCH,
    LF_STOPPED
} lf_state_t;

typedef struct {
    float base_speed;
    float min_speed;
    float max_wheel_speed;
    float slowdown;
    float max_error_m;
    float search_speed;
    float lost_timeout_s;
} lf_params_t;

typedef struct {
    float left;
    float right;
} wheel_cmd_t;

typedef struct {
    lf_params_t prm;
    pid_ctrl_t pid;
    lf_state_t state;
    float last_error;
    float lost_time;
} line_follower_t;

void lf_init(line_follower_t *lf, const lf_params_t *prm, const pid_ctrl_t *tuned_pid);
wheel_cmd_t lf_step(line_follower_t *lf, const line_estimate_t *est, float dt);

#endif
