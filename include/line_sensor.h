#ifndef LF_LINE_SENSOR_H
#define LF_LINE_SENSOR_H

#include <stdint.h>

#define LS_MAX_SENSORS 16

typedef struct {
    int count;
    float pos_m[LS_MAX_SENSORS];
    uint16_t cal_min[LS_MAX_SENSORS];
    uint16_t cal_max[LS_MAX_SENSORS];
    float detect_threshold;
} line_sensor_cfg_t;

typedef struct {
    float position_m;
    float peak;
    int found;
} line_estimate_t;

void line_sensor_normalize(const line_sensor_cfg_t *cfg, const uint16_t *raw, float *out);

line_estimate_t line_sensor_estimate(const line_sensor_cfg_t *cfg, const float *norm);

#endif
