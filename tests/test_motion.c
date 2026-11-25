#include "test.h"

#include "yoomi/motion.h"

#include <stdio.h>

static int run_settled(yoomi_motion_t *m, uint32_t max_ms, int32_t *peak_x)
{
    for (uint32_t t = 0; t < max_ms; t++) {
        motion_tick(m, 1);
        if (peak_x != 0 && motion_pos(m, YOOMI_AXIS_X) > *peak_x) {
            *peak_x = motion_pos(m, YOOMI_AXIS_X);
        }
        if (motion_settled(m)) {
            return 0;
        }
    }
    fprintf(stderr, "motion timeout pos=%ld vel=%ld target=%ld\n",
            (long)motion_pos(m, YOOMI_AXIS_X), (long)motion_vel(m, YOOMI_AXIS_X),
            (long)m->ax[YOOMI_AXIS_X].target_um);
    return -1;
}

void test_motion(void)
{
    yoomi_motion_t m;
    motion_init(&m);
    EXPECT(motion_move_axis(&m, YOOMI_AXIS_X, 900000) != 0);
    EXPECT(motion_pos(&m, YOOMI_AXIS_X) == 0);

    int32_t peak = 0;
    EXPECT(motion_move_axis(&m, YOOMI_AXIS_X, 100000) == 0);
    EXPECT(run_settled(&m, 5000, &peak) == 0);
    EXPECT(motion_pos(&m, YOOMI_AXIS_X) == 100000);
    EXPECT(peak <= 100000 + 500);
    EXPECT(m.ax[YOOMI_AXIS_X].moved_um == 100000);

    motion_init(&m);
    motion_set_pos_raw(&m, YOOMI_AXIS_X, 80000);
    EXPECT(motion_home(&m, YOOMI_AXIS_X) == 0);
    EXPECT(run_settled(&m, 5000, 0) == 0);
    EXPECT(motion_pos(&m, YOOMI_AXIS_X) == 0);
    EXPECT(m.ax[YOOMI_AXIS_X].homed);

    motion_init(&m);
    EXPECT(motion_move_axis(&m, YOOMI_AXIS_X, 300000) == 0);
    for (int i = 0; i < 400; i++) {
        motion_tick(&m, 1);
    }
    EXPECT(!motion_settled(&m));
    int32_t mid = motion_pos(&m, YOOMI_AXIS_X);
    EXPECT(mid > 1000);
    motion_pause(&m, true);
    for (int i = 0; i < 300; i++) {
        motion_tick(&m, 1);
    }
    EXPECT(motion_stopped(&m));
    int32_t held = motion_pos(&m, YOOMI_AXIS_X);
    for (int i = 0; i < 200; i++) {
        motion_tick(&m, 1);
    }
    EXPECT(motion_pos(&m, YOOMI_AXIS_X) == held);
    motion_pause(&m, false);
    EXPECT(run_settled(&m, 5000, 0) == 0);
    EXPECT(motion_pos(&m, YOOMI_AXIS_X) == 300000);

    motion_init(&m);
    EXPECT(motion_move_xy(&m, 120000, 200000) == 0);
    EXPECT(motion_move_axis(&m, YOOMI_AXIS_Z, 50000) == 0);
    EXPECT(run_settled(&m, 5000, 0) == 0);
    EXPECT(motion_pos(&m, YOOMI_AXIS_X) == 120000);
    EXPECT(motion_pos(&m, YOOMI_AXIS_Y) == 200000);
    EXPECT(motion_pos(&m, YOOMI_AXIS_Z) == 50000);

    motion_init(&m);
    motion_set_speed_pct(&m, 50);
    EXPECT(m.ax[YOOMI_AXIS_X].vmax == 125000);
}
