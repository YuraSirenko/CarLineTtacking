#include "line_sensor.h"

void line_sensor_normalize(const line_sensor_cfg_t *cfg, const uint16_t *raw, float *out)
{
    for (int i = 0; i < cfg->count; ++i) {
        float lo = (float)cfg->cal_min[i];
        float hi = (float)cfg->cal_max[i];
        float span = hi > lo ? hi - lo : 1.0f;
        float v = ((float)raw[i] - lo) / span;
        out[i] = v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
    }
}

line_estimate_t line_sensor_estimate(const line_sensor_cfg_t *cfg, const float *norm)
{
    line_estimate_t est = {0.0f, 0.0f, 0};
    if (cfg->count <= 0) {
        return est;
    }

    int peak_i = 0;
    for (int i = 1; i < cfg->count; ++i) {
        if (norm[i] > norm[peak_i]) {
            peak_i = i;
        }
    }
    est.peak = norm[peak_i];
    if (est.peak < cfg->detect_threshold) {
        return est;
    }

    float floor_level = 0.5f * cfg->detect_threshold;
    int lo = peak_i > 0 ? peak_i - 1 : 0;
    int hi = peak_i < cfg->count - 1 ? peak_i + 1 : cfg->count - 1;
    float sum_w = 0.0f;
    float sum_wx = 0.0f;
    for (int i = lo; i <= hi; ++i) {
        float w = norm[i] - floor_level;
        if (w > 0.0f) {
            sum_w += w;
            sum_wx += w * cfg->pos_m[i];
        }
    }

    est.position_m = sum_w > 0.0f ? sum_wx / sum_w : cfg->pos_m[peak_i];
    est.found = 1;
    return est;
}
