#ifndef LF_CAR_CONFIG_H
#define LF_CAR_CONFIG_H

#include "line_follower.h"
#include "line_sensor.h"
#include "pid.h"

#define CAR_SENSOR_COUNT 8
#define CAR_SENSOR_PITCH_M 0.012f
#define CAR_CONTROL_HZ 100

static inline void car_default_sensor_cfg(line_sensor_cfg_t *cfg)
{
    cfg->count = CAR_SENSOR_COUNT;
    for (int i = 0; i < CAR_SENSOR_COUNT; ++i) {
        cfg->pos_m[i] = (0.5f * (CAR_SENSOR_COUNT - 1) - (float)i) * CAR_SENSOR_PITCH_M;
        cfg->cal_min[i] = 300;
        cfg->cal_max[i] = 3300;
    }
    cfg->detect_threshold = 0.35f;
}

static inline void car_default_params(lf_params_t *prm, pid_ctrl_t *pid)
{
    prm->base_speed = 0.45f;
    prm->min_speed = 0.15f;
    prm->max_wheel_speed = 0.6f;
    prm->slowdown = 0.6f;
    prm->max_error_m = 0.04f;
    prm->search_speed = 0.15f;
    prm->lost_timeout_s = 1.5f;
    pid_init(pid, 9.0f, 2.0f, 0.35f, 0.5f, 0.3f);
}

#endif
