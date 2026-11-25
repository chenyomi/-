#ifndef YOOMI_GRIPPER_H
#define YOOMI_GRIPPER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    GRIP_OPEN = 0,
    GRIP_CLOSING,
    GRIP_CLOSED,
    GRIP_OPENING,
    GRIP_FAULT
} grip_state_t;

typedef struct {
    grip_state_t st;
    uint32_t elapsed_ms;
    uint32_t travel_ms;
} yoomi_gripper_t;

void gripper_init(yoomi_gripper_t *g);
int gripper_close(yoomi_gripper_t *g);
int gripper_open(yoomi_gripper_t *g);
void gripper_tick(yoomi_gripper_t *g, uint32_t dt_ms, bool part_present);
void gripper_reset(yoomi_gripper_t *g);
bool gripper_moving(const yoomi_gripper_t *g);
grip_state_t gripper_state(const yoomi_gripper_t *g);

#endif
