#include "yoomi/motion.h"

#include <string.h>

static int32_t iabs32(int32_t v)
{
    return v < 0 ? -v : v;
}

static void apply_speed(yoomi_motion_t *m)
{
    int pct = m->pct;
    if (pct < 1) {
        pct = 1;
    }
    if (pct > 100) {
        pct = 100;
    }
    m->pct = pct;
    for (int i = 0; i < YOOMI_AXIS_COUNT; i++) {
        yoomi_axis_t *a = &m->ax[i];
        a->vmax = (int32_t)((int64_t)a->base_vmax * pct / 100);
        a->home_vmax = (int32_t)((int64_t)a->base_home_vmax * pct / 100);
        if (a->vmax < 1000) {
            a->vmax = 1000;
        }
        if (a->home_vmax < 1000) {
            a->home_vmax = 1000;
        }
    }
}

static void axis_setup(yoomi_axis_t *a, int32_t vmax, int32_t home_vmax, int32_t amax, int32_t max_um)
{
    memset(a, 0, sizeof(*a));
    a->base_vmax = vmax;
    a->base_home_vmax = home_vmax;
    a->amax = amax;
    a->min_um = 0;
    a->max_um = max_um;
}

void motion_init(yoomi_motion_t *m)
{
    memset(m, 0, sizeof(*m));
    axis_setup(&m->ax[YOOMI_AXIS_X], 250000, 100000, 1200000, 500000);
    axis_setup(&m->ax[YOOMI_AXIS_Y], 250000, 100000, 1200000, 600000);
    axis_setup(&m->ax[YOOMI_AXIS_Z], 150000, 80000, 800000, 180000);
    m->pct = 100;
    m->halt_scale = 1;
    apply_speed(m);
}

void motion_set_speed_pct(yoomi_motion_t *m, int pct)
{
    m->pct = pct;
    apply_speed(m);
}

void motion_set_pos_raw(yoomi_motion_t *m, int axis, int32_t pos_um)
{
    yoomi_axis_t *a = &m->ax[axis];
    a->pos_um = pos_um;
    a->target_um = pos_um;
    a->vel_um_s = 0;
    a->rem = 0;
    a->moved_um = 0;
    a->busy = false;
    a->homing = false;
    a->homed = false;
    a->limit_fault = false;
}

int motion_move_axis(yoomi_motion_t *m, int axis, int32_t target_um)
{
    if (axis < 0 || axis >= YOOMI_AXIS_COUNT) {
        return -1;
    }
    if (m->halt) {
        return -1;
    }
    yoomi_axis_t *a = &m->ax[axis];
    if (target_um < a->min_um || target_um > a->max_um) {
        return -1;
    }
    a->target_um = target_um;
    a->homing = false;
    a->busy = true;
    a->limit_fault = false;
    return 0;
}

int motion_move_xy(yoomi_motion_t *m, int32_t x_um, int32_t y_um)
{
    if (x_um < m->ax[YOOMI_AXIS_X].min_um || x_um > m->ax[YOOMI_AXIS_X].max_um) {
        return -1;
    }
    if (y_um < m->ax[YOOMI_AXIS_Y].min_um || y_um > m->ax[YOOMI_AXIS_Y].max_um) {
        return -1;
    }
    if (motion_move_axis(m, YOOMI_AXIS_X, x_um) != 0) {
        return -1;
    }
    if (motion_move_axis(m, YOOMI_AXIS_Y, y_um) != 0) {
        return -1;
    }
    return 0;
}

int motion_home(yoomi_motion_t *m, int axis)
{
    if (axis < 0 || axis >= YOOMI_AXIS_COUNT || m->halt) {
        return -1;
    }
    yoomi_axis_t *a = &m->ax[axis];
    a->homing = true;
    a->homed = false;
    a->busy = true;
    a->limit_fault = false;
    a->target_um = -5000;
    return 0;
}

void motion_pause(yoomi_motion_t *m, bool paused)
{
    m->pause = paused;
}

void motion_halt(yoomi_motion_t *m, bool emergency)
{
    m->halt = true;
    m->halt_scale = emergency ? 5 : 1;
    m->pause = false;
}

void motion_clear_halt(yoomi_motion_t *m)
{
    m->halt = false;
    m->halt_scale = 1;
    for (int i = 0; i < YOOMI_AXIS_COUNT; i++) {
        m->ax[i].limit_fault = false;
    }
}

static void axis_tick(yoomi_axis_t *a, uint32_t dt_ms, bool pause, bool halt, int halt_scale)
{
    int32_t accel = a->amax;
    if (halt && halt_scale > 1) {
        accel = a->amax * halt_scale;
    }
    int32_t dv = (int32_t)(((int64_t)accel * (int64_t)dt_ms) / 1000);
    if (dv < 1) {
        dv = 1;
    }

    if (!a->homing && !pause && !halt && iabs32(a->target_um - a->pos_um) <= 30 &&
        iabs32(a->vel_um_s) <= dv) {
        a->moved_um += a->target_um - a->pos_um;
        a->pos_um = a->target_um;
        a->vel_um_s = 0;
        a->rem = 0;
        a->busy = false;
        return;
    }

    int32_t dest = a->target_um;
    int32_t vlim = a->homing ? a->home_vmax : a->vmax;
    if (halt || pause) {
        dest = a->pos_um;
    }

    int32_t err = dest - a->pos_um;
    int dir = (err > 0) - (err < 0);
    int32_t v = a->vel_um_s;
    int32_t av = iabs32(v);
    int64_t stop = 0;
    if (accel > 0) {
        stop = ((int64_t)av * (int64_t)av) / (2 * (int64_t)accel);
    }
    bool wrong = (v > 0 && dir < 0) || (v < 0 && dir > 0);
    bool brake = dir == 0 || wrong || (stop >= (int64_t)iabs32(err));

    if (brake) {
        if (av <= dv) {
            v = 0;
        } else {
            v -= (v > 0) ? dv : -dv;
        }
    } else if (av < vlim) {
        v += dir * dv;
        if (v > vlim) {
            v = vlim;
        }
        if (v < -vlim) {
            v = -vlim;
        }
    } else if (av > vlim) {
        int32_t capped = dir * vlim;
        if (iabs32(v - capped) <= dv) {
            v = capped;
        } else {
            v -= (v > 0) ? dv : -dv;
        }
    }

    a->vel_um_s = v;
    int64_t acc = (int64_t)a->rem + (int64_t)a->vel_um_s * (int64_t)dt_ms;
    int32_t delta = (int32_t)(acc / 1000);
    int32_t next = a->pos_um + delta;
    /* Land on the target instead of stepping through it. A 1 ms tick at
     * cruise speed is longer than the last few micrometres, and stepping
     * through Z = 0 would trip the soft limit. */
    if (!a->homing && !pause && !halt) {
        bool reached = (a->target_um >= a->pos_um && next >= a->target_um) ||
                       (a->target_um <= a->pos_um && next <= a->target_um);
        if (reached) {
            a->moved_um += a->target_um - a->pos_um;
            a->pos_um = a->target_um;
            a->vel_um_s = 0;
            a->rem = 0;
            a->busy = false;
            return;
        }
    }
    a->rem = (int32_t)(acc % 1000);
    a->pos_um = next;
    a->moved_um += delta;

    if (a->homing && !pause && !halt && a->pos_um <= 0) {
        a->moved_um -= a->pos_um;
        a->pos_um = 0;
        a->target_um = 0;
        a->vel_um_s = 0;
        a->rem = 0;
        a->busy = false;
        a->homing = false;
        a->homed = true;
        return;
    }

    if (!a->homing && (a->pos_um < a->min_um || a->pos_um > a->max_um)) {
        if (a->pos_um < a->min_um) {
            a->pos_um = a->min_um;
        }
        if (a->pos_um > a->max_um) {
            a->pos_um = a->max_um;
        }
        a->vel_um_s = 0;
        a->rem = 0;
        a->busy = false;
        a->limit_fault = true;
        a->target_um = a->pos_um;
        return;
    }

    if (halt && a->vel_um_s == 0) {
        a->target_um = a->pos_um;
        a->rem = 0;
        a->busy = false;
        a->homing = false;
        return;
    }

    if (!pause && !halt && a->pos_um == a->target_um && a->vel_um_s == 0) {
        a->busy = false;
        a->rem = 0;
    }
}

void motion_tick(yoomi_motion_t *m, uint32_t dt_ms)
{
    if (dt_ms == 0) {
        return;
    }
    for (int i = 0; i < YOOMI_AXIS_COUNT; i++) {
        axis_tick(&m->ax[i], dt_ms, m->pause, m->halt, m->halt_scale);
    }
}

bool motion_settled(const yoomi_motion_t *m)
{
    if (m->pause || m->halt) {
        return false;
    }
    for (int i = 0; i < YOOMI_AXIS_COUNT; i++) {
        const yoomi_axis_t *a = &m->ax[i];
        if (a->busy || a->homing || a->vel_um_s != 0 || a->pos_um != a->target_um) {
            return false;
        }
    }
    return true;
}

bool motion_stopped(const yoomi_motion_t *m)
{
    for (int i = 0; i < YOOMI_AXIS_COUNT; i++) {
        if (m->ax[i].vel_um_s != 0) {
            return false;
        }
    }
    return true;
}

bool motion_homed_all(const yoomi_motion_t *m)
{
    return m->ax[0].homed && m->ax[1].homed && m->ax[2].homed;
}

int32_t motion_pos(const yoomi_motion_t *m, int axis)
{
    return m->ax[axis].pos_um;
}

int32_t motion_vel(const yoomi_motion_t *m, int axis)
{
    return m->ax[axis].vel_um_s;
}

bool motion_limit_fault(const yoomi_motion_t *m)
{
    for (int i = 0; i < YOOMI_AXIS_COUNT; i++) {
        if (m->ax[i].limit_fault) {
            return true;
        }
    }
    return false;
}
