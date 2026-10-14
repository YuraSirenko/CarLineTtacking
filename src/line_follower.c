#include "line_follower.h"

#include <math.h>

void lf_init(line_follower_t *lf, const lf_params_t *prm, const pid_ctrl_t *tuned_pid)
{
    lf->prm = *prm;
    lf->pid = *tuned_pid;
    pid_reset(&lf->pid);
    lf->state = LF_FOLLOW;
    lf->last_error = 0.0f;
    lf->lost_time = 0.0f;
}

static wheel_cmd_t limit_wheels(wheel_cmd_t cmd, float max_speed)
{
    float biggest = fmaxf(fabsf(cmd.left), fabsf(cmd.right));
    if (biggest > max_speed) {
        float k = max_speed / biggest;
        cmd.left *= k;
        cmd.right *= k;
    }
    return cmd;
}

wheel_cmd_t lf_step(line_follower_t *lf, const line_estimate_t *est, float dt)
{
    const lf_params_t *p = &lf->prm;
    wheel_cmd_t cmd = {0.0f, 0.0f};

    if (est->found) {
        if (lf->state != LF_FOLLOW) {
            pid_reset(&lf->pid);
        }
        lf->state = LF_FOLLOW;
        lf->lost_time = 0.0f;
        lf->last_error = est->position_m;

        float turn = pid_update(&lf->pid, est->position_m, dt);

        float ratio = fminf(fabsf(est->position_m) / p->max_error_m, 1.0f);
        float v = fmaxf(p->base_speed * (1.0f - p->slowdown * ratio), p->min_speed);

        cmd.left = v - turn;
        cmd.right = v + turn;
        return limit_wheels(cmd, p->max_wheel_speed);
    }

    if (lf->state == LF_STOPPED) {
        return cmd;
    }

    lf->lost_time += dt;
    if (lf->lost_time > p->lost_timeout_s) {
        lf->state = LF_STOPPED;
        return cmd;
    }

    lf->state = LF_SEARCH;
    float dir = lf->last_error >= 0.0f ? 1.0f : -1.0f;
    cmd.left = -dir * p->search_speed;
    cmd.right = dir * p->search_speed;
    return cmd;
}
