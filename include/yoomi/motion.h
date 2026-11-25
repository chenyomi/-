#ifndef YOOMI_MOTION_H
#define YOOMI_MOTION_H

#include <stdbool.h>
#include <stdint.h>

enum {
    YOOMI_AXIS_X = 0,
    YOOMI_AXIS_Y = 1,
    YOOMI_AXIS_Z = 2,
    YOOMI_AXIS_COUNT = 3
};

typedef struct {
    int32_t pos_um;
    int32_t vel_um_s;
    int32_t target_um;
    int32_t base_vmax;
    int32_t base_home_vmax;
    int32_t vmax;
    int32_t home_vmax;
    int32_t amax;
    int32_t min_um;
    int32_t max_um;
    int32_t rem;
    int32_t moved_um;
    bool busy;
    bool homing;
    bool homed;
    bool limit_fault;
} yoomi_axis_t;

typedef struct {
    yoomi_axis_t ax[YOOMI_AXIS_COUNT];
    bool pause;
    bool halt;
    int halt_scale;
    int pct;
} yoomi_motion_t;

void motion_init(yoomi_motion_t *m);
void motion_set_speed_pct(yoomi_motion_t *m, int pct);
void motion_set_pos_raw(yoomi_motion_t *m, int axis, int32_t pos_um);
int motion_move_axis(yoomi_motion_t *m, int axis, int32_t target_um);
int motion_move_xy(yoomi_motion_t *m, int32_t x_um, int32_t y_um);
int motion_home(yoomi_motion_t *m, int axis);
void motion_pause(yoomi_motion_t *m, bool paused);
void motion_halt(yoomi_motion_t *m, bool emergency);
void motion_clear_halt(yoomi_motion_t *m);
void motion_tick(yoomi_motion_t *m, uint32_t dt_ms);

bool motion_settled(const yoomi_motion_t *m);
bool motion_stopped(const yoomi_motion_t *m);
bool motion_homed_all(const yoomi_motion_t *m);
int32_t motion_pos(const yoomi_motion_t *m, int axis);
int32_t motion_vel(const yoomi_motion_t *m, int axis);
bool motion_limit_fault(const yoomi_motion_t *m);

#endif
