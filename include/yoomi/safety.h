#ifndef YOOMI_SAFETY_H
#define YOOMI_SAFETY_H

#include <stdbool.h>
#include <stdint.h>

enum {
    FAULT_NONE = 0,
    FAULT_ESTOP = 1,
    FAULT_DOOR = 2,
    FAULT_SOFT_LIMIT = 3,
    FAULT_GRIP = 4,
    FAULT_WATCHDOG = 5,
    FAULT_SLOT = 6,
    FAULT_DROPPED = 7
};

typedef struct {
    bool estop_in;
    bool door_closed;
    bool tripped;
    int fault;
    uint32_t since_ms;
    uint32_t timeout_ms;
} yoomi_safety_t;

void safety_init(yoomi_safety_t *s);
void safety_set_inputs(yoomi_safety_t *s, bool estop_asserted, bool door_closed);
void safety_kick(yoomi_safety_t *s);
void safety_tick(yoomi_safety_t *s, uint32_t dt_ms);
int safety_reset(yoomi_safety_t *s);
bool safety_ok(const yoomi_safety_t *s);

#endif
