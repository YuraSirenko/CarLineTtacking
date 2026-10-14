#include "car_config.h"
#include "line_follower.h"
#include "line_sensor.h"
#include "pid.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define TRACK_POINTS 1000
#define TWO_PI 6.28318530718f

#define WHEEL_BASE_M 0.15f
#define BAR_AHEAD_M 0.08f
#define MOTOR_TAU_S 0.05f
#define LINE_HALF_WIDTH_M 0.0095f
#define SIM_TIME_S 30.0f
#define SETTLE_TIME_S 2.0f

#define MAX_ALLOWED_ERROR_M 0.03f
#define MIN_LAPS 1.0f

typedef struct {
    float x;
    float y;
} vec2_t;

typedef struct {
    float x;
    float y;
    float heading;
    float v_left;
    float v_right;
} car_t;

static vec2_t track[TRACK_POINTS];
static uint32_t rng_state = 12345u;

static float noise(float amplitude)
{
    rng_state = rng_state * 1664525u + 1013904223u;
    float unit = (float)(rng_state >> 8) / (float)(1u << 24);
    return amplitude * (2.0f * unit - 1.0f);
}

static void build_track(void)
{
    for (int i = 0; i < TRACK_POINTS; ++i) {
        float t = TWO_PI * (float)i / TRACK_POINTS;
        track[i].x = 1.2f * cosf(t);
        track[i].y = 0.7f * sinf(t) + 0.25f * sinf(2.0f * t);
    }
}

static float segment_distance(vec2_t p, vec2_t a, vec2_t b)
{
    float abx = b.x - a.x;
    float aby = b.y - a.y;
    float len2 = abx * abx + aby * aby;
    float t = len2 > 0.0f ? ((p.x - a.x) * abx + (p.y - a.y) * aby) / len2 : 0.0f;
    t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
    float dx = p.x - (a.x + t * abx);
    float dy = p.y - (a.y + t * aby);
    return sqrtf(dx * dx + dy * dy);
}

static float distance_to_track(vec2_t p, int *nearest)
{
    float best = 1e9f;
    for (int i = 0; i < TRACK_POINTS; ++i) {
        float d = segment_distance(p, track[i], track[(i + 1) % TRACK_POINTS]);
        if (d < best) {
            best = d;
            *nearest = i;
        }
    }
    return best;
}

static uint16_t simulate_sensor(float dist_to_line)
{
    float r = 1.0f;
    if (dist_to_line > LINE_HALF_WIDTH_M) {
        float e = (dist_to_line - LINE_HALF_WIDTH_M) / 0.004f;
        r = expf(-e * e);
    }
    float raw = 300.0f + 3000.0f * r + noise(60.0f);
    raw = raw < 0.0f ? 0.0f : (raw > 4095.0f ? 4095.0f : raw);
    return (uint16_t)raw;
}

static void read_sensor_bar(const car_t *car, const line_sensor_cfg_t *cfg, uint16_t *raw)
{
    float c = cosf(car->heading);
    float s = sinf(car->heading);
    int unused;
    for (int i = 0; i < cfg->count; ++i) {
        float lat = cfg->pos_m[i];
        vec2_t p = {car->x + BAR_AHEAD_M * c - lat * s, car->y + BAR_AHEAD_M * s + lat * c};
        raw[i] = simulate_sensor(distance_to_track(p, &unused));
    }
}

static void step_car(car_t *car, wheel_cmd_t cmd, float dt)
{
    car->v_left += (cmd.left - car->v_left) * dt / MOTOR_TAU_S;
    car->v_right += (cmd.right - car->v_right) * dt / MOTOR_TAU_S;

    float v = 0.5f * (car->v_left + car->v_right);
    float w = (car->v_right - car->v_left) / WHEEL_BASE_M;
    car->x += v * cosf(car->heading) * dt;
    car->y += v * sinf(car->heading) * dt;
    car->heading += w * dt;
}

int main(int argc, char **argv)
{
    int csv = argc > 1 && strcmp(argv[1], "--csv") == 0;
    const float dt = 1.0f / CAR_CONTROL_HZ;

    build_track();

    line_sensor_cfg_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    car_default_sensor_cfg(&cfg);

    lf_params_t prm;
    pid_ctrl_t pid;
    car_default_params(&prm, &pid);

    line_follower_t lf;
    lf_init(&lf, &prm, &pid);

    car_t car = {1.2f + 0.02f, 0.0f, 0.5f * 3.14159265f + 0.17f, 0.0f, 0.0f};

    int prev_idx;
    distance_to_track((vec2_t){car.x, car.y}, &prev_idx);
    long progress = 0;
    float sum_err = 0.0f;
    float max_err = 0.0f;
    long samples = 0;

    if (csv) {
        printf("t,x,y,heading,cross_track_m,line_pos_m,state,cmd_left,cmd_right\n");
    }

    int steps = (int)(SIM_TIME_S / dt);
    for (int k = 0; k < steps; ++k) {
        float t = k * dt;

        uint16_t raw[LS_MAX_SENSORS];
        float norm[LS_MAX_SENSORS];
        read_sensor_bar(&car, &cfg, raw);
        line_sensor_normalize(&cfg, raw, norm);
        line_estimate_t est = line_sensor_estimate(&cfg, norm);
        wheel_cmd_t cmd = lf_step(&lf, &est, dt);
        step_car(&car, cmd, dt);

        int idx;
        float err = distance_to_track((vec2_t){car.x, car.y}, &idx);
        int delta = idx - prev_idx;
        if (delta > TRACK_POINTS / 2) {
            delta -= TRACK_POINTS;
        } else if (delta < -TRACK_POINTS / 2) {
            delta += TRACK_POINTS;
        }
        progress += delta;
        prev_idx = idx;

        if (t >= SETTLE_TIME_S) {
            sum_err += err;
            max_err = fmaxf(max_err, err);
            ++samples;
        }

        if (csv) {
            printf("%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%.3f,%.3f\n", t, car.x, car.y, car.heading,
                   err, est.position_m, (int)lf.state, cmd.left, cmd.right);
        }
    }

    float laps = (float)progress / TRACK_POINTS;
    float mean_err = samples > 0 ? sum_err / samples : 0.0f;
    int pass = lf.state != LF_STOPPED && max_err <= MAX_ALLOWED_ERROR_M && laps >= MIN_LAPS;

    FILE *out = csv ? stderr : stdout;
    fprintf(out, "laps: %.2f | mean cross-track: %.1f mm | max cross-track: %.1f mm | %s\n", laps,
            mean_err * 1000.0f, max_err * 1000.0f, pass ? "PASS" : "FAIL");
    return pass ? 0 : 1;
}
