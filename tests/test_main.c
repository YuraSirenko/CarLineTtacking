#include "car_config.h"
#include "line_follower.h"
#include "line_sensor.h"
#include "pid.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;

#define CHECK(cond)                                                     \
    do {                                                                \
        if (!(cond)) {                                                  \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);      \
            ++failures;                                                 \
        }                                                               \
    } while (0)

static void test_pid_sign_and_limit(void)
{
    pid_ctrl_t pid;
    pid_init(&pid, 2.0f, 0.0f, 0.0f, 1.0f, 1.0f);
    CHECK(fabsf(pid_update(&pid, 0.25f, 0.01f) - 0.5f) < 1e-6f);
    CHECK(pid_update(&pid, 10.0f, 0.01f) == 1.0f);
    CHECK(pid_update(&pid, -10.0f, 0.01f) == -1.0f);
}

static void test_pid_anti_windup(void)
{
    pid_ctrl_t pid;
    pid_init(&pid, 1.0f, 10.0f, 0.0f, 1.0f, 1.0f);
    for (int i = 0; i < 1000; ++i) {
        pid_update(&pid, 5.0f, 0.01f);
    }
    CHECK(pid.integral < 1.0f);
    CHECK(pid_update(&pid, -2.0f, 0.01f) < 0.0f);
}

static void test_sensor_estimate(void)
{
    line_sensor_cfg_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    car_default_sensor_cfg(&cfg);

    float centred[8] = {0, 0, 0, 0.9f, 0.9f, 0, 0, 0};
    line_estimate_t e = line_sensor_estimate(&cfg, centred);
    CHECK(e.found);
    CHECK(fabsf(e.position_m) < 1e-4f);

    float left[8] = {0.2f, 1.0f, 0.4f, 0, 0, 0, 0, 0};
    e = line_sensor_estimate(&cfg, left);
    CHECK(e.found && e.position_m > 0.02f);

    float none[8] = {0.1f, 0.05f, 0.1f, 0.2f, 0.1f, 0, 0.1f, 0};
    e = line_sensor_estimate(&cfg, none);
    CHECK(!e.found);

    float stray[8] = {0, 0, 0, 0.9f, 0.9f, 0, 0, 0.6f};
    e = line_sensor_estimate(&cfg, stray);
    CHECK(fabsf(e.position_m) < 1e-4f);
}

static void test_normalize_clamps(void)
{
    line_sensor_cfg_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    car_default_sensor_cfg(&cfg);
    uint16_t raw[8] = {0, 300, 1800, 3300, 4095, 300, 300, 300};
    float out[8];
    line_sensor_normalize(&cfg, raw, out);
    CHECK(out[0] == 0.0f);
    CHECK(out[1] == 0.0f);
    CHECK(fabsf(out[2] - 0.5f) < 1e-6f);
    CHECK(out[3] == 1.0f);
    CHECK(out[4] == 1.0f);
}

static void test_follower_behaviour(void)
{
    lf_params_t prm;
    pid_ctrl_t pid;
    car_default_params(&prm, &pid);
    line_follower_t lf;
    lf_init(&lf, &prm, &pid);

    line_estimate_t line_left = {0.02f, 1.0f, 1};
    wheel_cmd_t cmd = lf_step(&lf, &line_left, 0.01f);
    CHECK(cmd.right > cmd.left);
    CHECK(fmaxf(fabsf(cmd.left), fabsf(cmd.right)) <= prm.max_wheel_speed + 1e-6f);

    line_estimate_t lost = {0.0f, 0.0f, 0};
    cmd = lf_step(&lf, &lost, 0.01f);
    CHECK(lf.state == LF_SEARCH);
    CHECK(cmd.right > 0.0f && cmd.left < 0.0f);

    for (int i = 0; i < 200; ++i) {
        cmd = lf_step(&lf, &lost, 0.01f);
    }
    CHECK(lf.state == LF_STOPPED);
    CHECK(cmd.left == 0.0f && cmd.right == 0.0f);

    cmd = lf_step(&lf, &line_left, 0.01f);
    CHECK(lf.state == LF_FOLLOW);
}

int main(void)
{
    test_pid_sign_and_limit();
    test_pid_anti_windup();
    test_sensor_estimate();
    test_normalize_clamps();
    test_follower_behaviour();
    if (failures == 0) {
        printf("all unit tests passed\n");
    }
    return failures == 0 ? 0 : 1;
}
