#include "car_config.h"
#include "hal.h"
#include "line_follower.h"
#include "line_sensor.h"
#include "pid.h"

#include <stdint.h>
#include <string.h>

int main(void)
{
    hal_init();

    line_sensor_cfg_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    car_default_sensor_cfg(&cfg);

    lf_params_t prm;
    pid_ctrl_t pid;
    car_default_params(&prm, &pid);

    line_follower_t lf;
    lf_init(&lf, &prm, &pid);

    const uint32_t period_us = 1000000u / CAR_CONTROL_HZ;
    uint32_t next = hal_micros();

    for (;;) {
        while ((int32_t)(hal_micros() - next) < 0) {
        }
        next += period_us;

        uint16_t raw[LS_MAX_SENSORS];
        float norm[LS_MAX_SENSORS];
        hal_read_line_sensors(raw, cfg.count);
        line_sensor_normalize(&cfg, raw, norm);
        line_estimate_t est = line_sensor_estimate(&cfg, norm);
        wheel_cmd_t cmd = lf_step(&lf, &est, 1.0f / CAR_CONTROL_HZ);
        hal_set_wheel_speeds(cmd.left, cmd.right);
    }
}
