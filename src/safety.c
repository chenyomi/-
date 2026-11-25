#include "yoomi/safety.h"

#include <string.h>

void safety_init(yoomi_safety_t *s)
{
    memset(s, 0, sizeof(*s));
    s->door_closed = true;
    s->timeout_ms = 500;
}

void safety_set_inputs(yoomi_safety_t *s, bool estop_asserted, bool door_closed)
{
    s->estop_in = estop_asserted;
    s->door_closed = door_closed;
}

void safety_kick(yoomi_safety_t *s)
{
    s->since_ms = 0;
}

void safety_tick(yoomi_safety_t *s, uint32_t dt_ms)
{
    s->since_ms += dt_ms;
    if (s->tripped) {
        return;
    }
    if (s->estop_in) {
        s->tripped = true;
        s->fault = FAULT_ESTOP;
        return;
    }
    if (!s->door_closed) {
        s->tripped = true;
        s->fault = FAULT_DOOR;
        return;
    }
    if (s->since_ms >= s->timeout_ms) {
        s->tripped = true;
        s->fault = FAULT_WATCHDOG;
    }
}

int safety_reset(yoomi_safety_t *s)
{
    if (s->estop_in || !s->door_closed) {
        return -1;
    }
    s->tripped = false;
    s->fault = FAULT_NONE;
    s->since_ms = 0;
    return 0;
}

bool safety_ok(const yoomi_safety_t *s)
{
    return !s->tripped && !s->estop_in && s->door_closed;
}
