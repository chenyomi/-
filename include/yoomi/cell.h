#ifndef YOOMI_CELL_H
#define YOOMI_CELL_H

#include "yoomi/gripper.h"
#include "yoomi/json_msg.h"
#include "yoomi/motion.h"
#include "yoomi/safety.h"

#include <stddef.h>
#include <stdint.h>

#define YOOMI_VERSION "1.0.0"
#define YOOMI_COLS 4
#define YOOMI_ROWS 6
#define YOOMI_SLOTS (YOOMI_COLS * YOOMI_ROWS)
#define YOOMI_MODBUS_ADDR 0x01
#define YOOMI_REG_COUNT 16

enum {
    YOOMI_OK = 0,
    YOOMI_ERR_ARG = -1,
    YOOMI_ERR_BUSY = -2,
    YOOMI_ERR_STATE = -3
};

enum {
    YOOMI_JOB_NONE = 0,
    YOOMI_JOB_HOME = 1,
    YOOMI_JOB_OUT = 2,
    YOOMI_JOB_IN = 3,
    YOOMI_JOB_STOP = 4
};

enum {
    REG_STATUS = 0,
    REG_FAULT = 1,
    REG_JOB = 2,
    REG_SPEED = 3,
    REG_X_HI = 4,
    REG_X_LO = 5,
    REG_Y_HI = 6,
    REG_Y_LO = 7,
    REG_Z_HI = 8,
    REG_Z_LO = 9,
    REG_CMD = 10,
    REG_SLOT = 11,
    REG_SPEED_ARG = 12,
    REG_STEP = 13,
    REG_HOLDING = 14,
    REG_HEARTBEAT = 15
};

enum {
    ST_HOMED = 1 << 0,
    ST_BUSY = 1 << 1,
    ST_FAULT = 1 << 2,
    ST_PAUSED = 1 << 3,
    ST_GRIP_CLOSED = 1 << 4,
    ST_HOLDING = 1 << 5,
    ST_DOOR = 1 << 6,
    ST_ESTOP = 1 << 7
};

enum {
    MB_CMD_HOME = 1,
    MB_CMD_OUT = 2,
    MB_CMD_IN = 3,
    MB_CMD_PAUSE = 4,
    MB_CMD_RESUME = 5,
    MB_CMD_STOP = 6,
    MB_CMD_RESET = 7,
    MB_CMD_SPEED = 8
};

typedef void (*yoomi_pub_fn)(const char *topic, const char *json, void *user);

typedef struct {
    yoomi_motion_t motion;
    yoomi_gripper_t grip;
    yoomi_safety_t safety;
    uint16_t regs[YOOMI_REG_COUNT];
    bool slot_has[YOOMI_SLOTS];
    bool port_has;
    bool holding;
    bool sensor_override;
    bool sensor_override_val;
    bool estop_in;
    bool door_closed;
    bool paused;
    bool job_active;
    bool step_issued;
    int job;
    int slot_index;
    int app_fault;
    int task_number;
    int step_i;
    int nsteps;
    uint8_t steps[16];
    int speed_pct;
    uint16_t arg_slot;
    uint16_t arg_speed;
    uint32_t heartbeat;
    uint32_t status_acc_ms;
    uint32_t mb_seq;
    char group_id[YOOMI_ID_MAX];
    char pool[96];
    char task_json[256];
    yoomi_pub_fn pub;
    void *pub_user;
} yoomi_cell_t;

void cell_init(yoomi_cell_t *c);
void cell_tick(yoomi_cell_t *c, uint32_t dt_ms);
void cell_kick(yoomi_cell_t *c);
void cell_set_publisher(yoomi_cell_t *c, yoomi_pub_fn fn, void *user);

void cell_set_estop(yoomi_cell_t *c, bool asserted);
void cell_set_door_closed(yoomi_cell_t *c, bool closed);

int cell_cmd(yoomi_cell_t *c, const yoomi_cmd_t *cmd);
int cell_cmd_json(yoomi_cell_t *c, const char *json);
int cell_modbus(yoomi_cell_t *c, const uint8_t *req, size_t len, uint8_t *rsp, size_t cap);

int cell_fault(const yoomi_cell_t *c);
const char *cell_fault_name(int fault);
bool cell_job_active(const yoomi_cell_t *c);
bool cell_homed(const yoomi_cell_t *c);
int32_t cell_pos_um(const yoomi_cell_t *c, int axis);
int32_t cell_vel_um_s(const yoomi_cell_t *c, int axis);
const char *cell_task_json(const yoomi_cell_t *c);

void cell_sim_set_slot(yoomi_cell_t *c, int slot, bool occupied);
void cell_sim_set_port(yoomi_cell_t *c, bool occupied);
void cell_sim_force_sensor(yoomi_cell_t *c, bool enable, bool present);
bool cell_sim_slot(const yoomi_cell_t *c, int slot);
bool cell_sim_port(const yoomi_cell_t *c);
bool cell_sim_holding(const yoomi_cell_t *c);

int yoomi_slot_index(const char *name);
void yoomi_slot_name(int index, char out[3]);
int32_t yoomi_slot_x_um(int index);
int32_t yoomi_slot_y_um(int index);

#define YOOMI_Z_SAFE_UM 0
#define YOOMI_Z_PICK_UM 150000
#define YOOMI_Z_PORT_UM 140000
#define YOOMI_PORT_X_UM 20000
#define YOOMI_PORT_Y_UM 520000

#endif
