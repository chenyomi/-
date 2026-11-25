#include "yoomi/gripper.h"

#include <string.h>

void gripper_init(yoomi_gripper_t *g)
{
    memset(g, 0, sizeof(*g));
    g->st = GRIP_OPEN;
    g->travel_ms = 200;
}

int gripper_close(yoomi_gripper_t *g)
{
    if (g->st == GRIP_FAULT) {
        return -1;
    }
    if (g->st == GRIP_CLOSED || g->st == GRIP_CLOSING) {
        return 0;
    }
    g->st = GRIP_CLOSING;
    g->elapsed_ms = 0;
    return 0;
}

int gripper_open(yoomi_gripper_t *g)
{
    if (g->st == GRIP_FAULT) {
        return -1;
    }
    if (g->st == GRIP_OPEN || g->st == GRIP_OPENING) {
        return 0;
    }
    g->st = GRIP_OPENING;
    g->elapsed_ms = 0;
    return 0;
}

void gripper_tick(yoomi_gripper_t *g, uint32_t dt_ms, bool part_present)
{
    if (g->st != GRIP_CLOSING && g->st != GRIP_OPENING) {
        return;
    }
    g->elapsed_ms += dt_ms;
    if (g->elapsed_ms < g->travel_ms) {
        return;
    }
    if (g->st == GRIP_CLOSING) {
        g->st = part_present ? GRIP_CLOSED : GRIP_FAULT;
    } else {
        g->st = GRIP_OPEN;
    }
}

void gripper_reset(yoomi_gripper_t *g)
{
    uint32_t travel = g->travel_ms == 0 ? 200 : g->travel_ms;
    memset(g, 0, sizeof(*g));
    g->travel_ms = travel;
    g->st = GRIP_OPEN;
}

bool gripper_moving(const yoomi_gripper_t *g)
{
    return g->st == GRIP_CLOSING || g->st == GRIP_OPENING;
}

grip_state_t gripper_state(const yoomi_gripper_t *g)
{
    return g->st;
}
