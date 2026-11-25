#include "yoomi/cell.h"

#include "yoomi/modbus.h"

#include <stdio.h>
#include <string.h>

enum {
    S_HOME_Z = 1,
    S_HOME_X,
    S_HOME_Y,
    S_Z_SAFE,
    S_XY_SLOT,
    S_Z_SLOT,
    S_GRIP_CLOSE,
    S_XY_PORT,
    S_Z_PORT,
    S_GRIP_OPEN,
    S_DONE,
    S_STOPPED
};

int yoomi_slot_index(const char *name)
{
    if (name == 0 || name[0] == 0 || name[1] == 0 || name[2] != 0) {
        return -1;
    }
    char c = name[0];
    if (c >= 'a' && c <= 'd') {
        c = (char)(c - 'a' + 'A');
    }
    if (c < 'A' || c > 'D' || name[1] < '1' || name[1] > '6') {
        return -1;
    }
    return (name[1] - '1') * YOOMI_COLS + (c - 'A');
}

void yoomi_slot_name(int index, char out[3])
{
    out[0] = (char)('A' + (index % YOOMI_COLS));
    out[1] = (char)('1' + (index / YOOMI_COLS));
    out[2] = 0;
}

int32_t yoomi_slot_x_um(int index)
{
    return (40 + (index % YOOMI_COLS) * 120) * 1000;
}

int32_t yoomi_slot_y_um(int index)
{
    return (30 + (index / YOOMI_COLS) * 80) * 1000;
}

const char *cell_fault_name(int fault)
{
    switch (fault) {
    case FAULT_NONE:
        return "none";
    case FAULT_ESTOP:
        return "estop";
    case FAULT_DOOR:
        return "door";
    case FAULT_SOFT_LIMIT:
        return "soft_limit";
    case FAULT_GRIP:
        return "grip";
    case FAULT_WATCHDOG:
        return "watchdog";
    case FAULT_SLOT:
        return "slot";
    case FAULT_DROPPED:
        return "dropped";
    default:
        return "unknown";
    }
}

static void copy_text(char *dst, size_t cap, const char *src)
{
    size_t i = 0;
    if (cap == 0) {
        return;
    }
    if (src != 0) {
        for (; src[i] != 0 && i + 1 < cap; i++) {
            dst[i] = src[i];
        }
    }
    dst[i] = 0;
}

static void publish_task(yoomi_cell_t *c, const char *state)
{
    c->task_number += 1;
    if (c->group_id[0] == 0) {
        copy_text(c->group_id, sizeof c->group_id, "LOCAL");
    }
    if (sss_task_json(c->task_json, sizeof c->task_json, c->group_id, c->task_number, state,
                      c->pool) != 0) {
        c->task_json[0] = 0;
    }
    if (c->pub != 0) {
        c->pub("sss/task", c->task_json, c->pub_user);
    }
}

static void set_pool(yoomi_cell_t *c, const char *text)
{
    copy_text(c->pool, sizeof c->pool, text);
}

static const char *fault_text(int fault)
{
    switch (fault) {
    case FAULT_ESTOP:
        return "急停";
    case FAULT_DOOR:
        return "安全门打开";
    case FAULT_SOFT_LIMIT:
        return "软限位";
    case FAULT_GRIP:
        return "夹爪空抓";
    case FAULT_WATCHDOG:
        return "看门狗超时";
    case FAULT_SLOT:
        return "库位状态不对";
    case FAULT_DROPPED:
        return "物料脱落";
    default:
        return "故障";
    }
}

static void fail_job(yoomi_cell_t *c, int fault)
{
    if (c->app_fault == FAULT_NONE) {
        c->app_fault = fault;
    }
    motion_halt(&c->motion, fault == FAULT_ESTOP);
    c->job_active = false;
    c->paused = false;
    set_pool(c, fault_text(cell_fault(c)));
    publish_task(c, "PAUSE");
}

static void put_i32(yoomi_cell_t *c, int reg, int32_t value)
{
    uint32_t u = (uint32_t)value;
    c->regs[reg] = (uint16_t)(u >> 16);
    c->regs[reg + 1] = (uint16_t)(u & 0xFFFF);
}

static void sync_regs(yoomi_cell_t *c)
{
    uint16_t status = 0;
    if (motion_homed_all(&c->motion)) {
        status |= ST_HOMED;
    }
    if (c->job_active) {
        status |= ST_BUSY;
    }
    if (cell_fault(c) != FAULT_NONE) {
        status |= ST_FAULT;
    }
    if (c->paused) {
        status |= ST_PAUSED;
    }
    if (gripper_state(&c->grip) == GRIP_CLOSED) {
        status |= ST_GRIP_CLOSED;
    }
    if (c->holding) {
        status |= ST_HOLDING;
    }
    if (c->door_closed) {
        status |= ST_DOOR;
    }
    if (c->safety.tripped && c->safety.fault == FAULT_ESTOP) {
        status |= ST_ESTOP;
    }
    c->regs[REG_STATUS] = status;
    c->regs[REG_FAULT] = (uint16_t)cell_fault(c);
    c->regs[REG_JOB] = (uint16_t)(c->job_active ? c->job : YOOMI_JOB_NONE);
    c->regs[REG_SPEED] = (uint16_t)c->speed_pct;
    put_i32(c, REG_X_HI, motion_pos(&c->motion, YOOMI_AXIS_X));
    put_i32(c, REG_Y_HI, motion_pos(&c->motion, YOOMI_AXIS_Y));
    put_i32(c, REG_Z_HI, motion_pos(&c->motion, YOOMI_AXIS_Z));
    c->regs[REG_CMD] = 0;
    c->regs[REG_SLOT] = c->arg_slot;
    c->regs[REG_SPEED_ARG] = c->arg_speed;
    c->regs[REG_STEP] = (uint16_t)c->step_i;
    c->regs[REG_HOLDING] = c->holding ? 1 : 0;
    c->regs[REG_HEARTBEAT] = (uint16_t)c->heartbeat;
}

static void assign_group(yoomi_cell_t *c, const char *group)
{
    if (group != 0 && group[0] != 0) {
        copy_text(c->group_id, sizeof c->group_id, group);
        return;
    }
    c->mb_seq += 1;
    (void)snprintf(c->group_id, sizeof c->group_id, "MB%u", c->mb_seq);
}

static int inventory_ok(const yoomi_cell_t *c, int job, int slot)
{
    if (job == YOOMI_JOB_OUT && (!c->slot_has[slot] || c->port_has)) {
        return YOOMI_ERR_STATE;
    }
    if (job == YOOMI_JOB_IN && (c->slot_has[slot] || !c->port_has)) {
        return YOOMI_ERR_STATE;
    }
    return YOOMI_OK;
}

static int start_job(yoomi_cell_t *c, int job, int slot, const char *group)
{
    if (!safety_ok(&c->safety) || c->app_fault != FAULT_NONE || c->motion.halt) {
        return YOOMI_ERR_STATE;
    }
    if (c->job_active) {
        return YOOMI_ERR_BUSY;
    }
    if (job == YOOMI_JOB_OUT || job == YOOMI_JOB_IN) {
        if (slot < 0 || slot >= YOOMI_SLOTS) {
            return YOOMI_ERR_ARG;
        }
        int inv = inventory_ok(c, job, slot);
        if (inv != YOOMI_OK) {
            return inv;
        }
        c->slot_index = slot;
    }
    assign_group(c, group);
    int n = 0;
    if (job == YOOMI_JOB_HOME) {
        c->steps[n++] = S_HOME_Z;
        c->steps[n++] = S_HOME_X;
        c->steps[n++] = S_HOME_Y;
        c->steps[n++] = S_DONE;
    } else {
        if (!motion_homed_all(&c->motion)) {
            c->steps[n++] = S_HOME_Z;
            c->steps[n++] = S_HOME_X;
            c->steps[n++] = S_HOME_Y;
        }
        if (job == YOOMI_JOB_OUT) {
            c->steps[n++] = S_Z_SAFE;
            c->steps[n++] = S_XY_SLOT;
            c->steps[n++] = S_Z_SLOT;
            c->steps[n++] = S_GRIP_CLOSE;
            c->steps[n++] = S_Z_SAFE;
            c->steps[n++] = S_XY_PORT;
            c->steps[n++] = S_Z_PORT;
            c->steps[n++] = S_GRIP_OPEN;
            c->steps[n++] = S_Z_SAFE;
            c->steps[n++] = S_DONE;
        } else if (job == YOOMI_JOB_IN) {
            c->steps[n++] = S_Z_SAFE;
            c->steps[n++] = S_XY_PORT;
            c->steps[n++] = S_Z_PORT;
            c->steps[n++] = S_GRIP_CLOSE;
            c->steps[n++] = S_Z_SAFE;
            c->steps[n++] = S_XY_SLOT;
            c->steps[n++] = S_Z_SLOT;
            c->steps[n++] = S_GRIP_OPEN;
            c->steps[n++] = S_Z_SAFE;
            c->steps[n++] = S_DONE;
        } else {
            return YOOMI_ERR_ARG;
        }
    }
    c->nsteps = n;
    c->step_i = 0;
    c->step_issued = false;
    c->job = job;
    c->job_active = true;
    c->paused = false;
    c->task_number = 0;
    motion_pause(&c->motion, false);
    return YOOMI_OK;
}

static int apply_speed(yoomi_cell_t *c, int pct)
{
    if (pct < 1 || pct > 100) {
        return YOOMI_ERR_ARG;
    }
    motion_set_speed_pct(&c->motion, pct);
    c->speed_pct = c->motion.pct;
    c->arg_speed = (uint16_t)c->speed_pct;
    return YOOMI_OK;
}

static int do_pause(yoomi_cell_t *c)
{
    if (!c->job_active) {
        return YOOMI_OK;
    }
    c->paused = true;
    motion_pause(&c->motion, true);
    set_pool(c, "已暂停");
    publish_task(c, "PAUSE");
    return YOOMI_OK;
}

static int do_resume(yoomi_cell_t *c)
{
    if (cell_fault(c) != FAULT_NONE) {
        return YOOMI_ERR_STATE;
    }
    c->paused = false;
    if (!c->motion.halt) {
        motion_pause(&c->motion, false);
    }
    if (c->job_active) {
        set_pool(c, "继续");
        publish_task(c, "RUN");
    }
    return YOOMI_OK;
}

static int do_stop(yoomi_cell_t *c)
{
    if (cell_fault(c) != FAULT_NONE || c->motion.halt) {
        c->job_active = false;
        return YOOMI_ERR_STATE;
    }
    c->paused = false;
    motion_pause(&c->motion, false);
    (void)motion_move_axis(&c->motion, YOOMI_AXIS_X, motion_pos(&c->motion, YOOMI_AXIS_X));
    (void)motion_move_axis(&c->motion, YOOMI_AXIS_Y, motion_pos(&c->motion, YOOMI_AXIS_Y));
    c->steps[0] = S_Z_SAFE;
    c->steps[1] = S_STOPPED;
    c->nsteps = 2;
    c->step_i = 0;
    c->step_issued = false;
    c->job = YOOMI_JOB_STOP;
    c->job_active = true;
    return YOOMI_OK;
}

static int do_reset(yoomi_cell_t *c)
{
    if (safety_reset(&c->safety) != 0) {
        return YOOMI_ERR_STATE;
    }
    c->app_fault = FAULT_NONE;
    gripper_reset(&c->grip);
    if (c->holding) {
        c->grip.st = GRIP_CLOSED;
    }
    motion_clear_halt(&c->motion);
    c->job_active = false;
    c->paused = false;
    c->step_issued = false;
    c->job = YOOMI_JOB_NONE;
    set_pool(c, "已复位");
    publish_task(c, "PAUSE");
    return YOOMI_OK;
}

int cell_cmd(yoomi_cell_t *c, const yoomi_cmd_t *cmd)
{
    if (c == 0 || cmd == 0 || cmd->cmd[0] == 0) {
        return YOOMI_ERR_ARG;
    }
    if (cmd->has_speed && strcmp(cmd->cmd, "speed") != 0) {
        int spd = apply_speed(c, cmd->speed);
        if (spd != YOOMI_OK) {
            return spd;
        }
    }
    const char *group = cmd->has_group ? cmd->group_id : 0;
    if (strcmp(cmd->cmd, "speed") == 0) {
        if (!cmd->has_speed) {
            return YOOMI_ERR_ARG;
        }
        return apply_speed(c, cmd->speed);
    }
    if (strcmp(cmd->cmd, "pause") == 0) {
        return do_pause(c);
    }
    if (strcmp(cmd->cmd, "resume") == 0) {
        return do_resume(c);
    }
    if (strcmp(cmd->cmd, "stop") == 0) {
        return do_stop(c);
    }
    if (strcmp(cmd->cmd, "reset") == 0) {
        return do_reset(c);
    }
    if (strcmp(cmd->cmd, "home") == 0) {
        return start_job(c, YOOMI_JOB_HOME, -1, group);
    }
    if (strcmp(cmd->cmd, "storage_out") == 0 || strcmp(cmd->cmd, "storage_in") == 0) {
        if (!cmd->has_slot) {
            return YOOMI_ERR_ARG;
        }
        int slot = yoomi_slot_index(cmd->slot);
        int job = strcmp(cmd->cmd, "storage_out") == 0 ? YOOMI_JOB_OUT : YOOMI_JOB_IN;
        return start_job(c, job, slot, group);
    }
    return YOOMI_ERR_ARG;
}

int cell_cmd_json(yoomi_cell_t *c, const char *json)
{
    yoomi_cmd_t cmd;
    if (yoomi_cmd_parse(json, &cmd) != 0) {
        return YOOMI_ERR_ARG;
    }
    return cell_cmd(c, &cmd);
}

static int dispatch_mb(yoomi_cell_t *c, uint16_t code)
{
    yoomi_cmd_t cmd;
    memset(&cmd, 0, sizeof cmd);
    switch (code) {
    case MB_CMD_HOME:
        return start_job(c, YOOMI_JOB_HOME, -1, 0);
    case MB_CMD_OUT:
        return start_job(c, YOOMI_JOB_OUT, (int)c->arg_slot, 0);
    case MB_CMD_IN:
        return start_job(c, YOOMI_JOB_IN, (int)c->arg_slot, 0);
    case MB_CMD_PAUSE:
        copy_text(cmd.cmd, sizeof cmd.cmd, "pause");
        return cell_cmd(c, &cmd);
    case MB_CMD_RESUME:
        copy_text(cmd.cmd, sizeof cmd.cmd, "resume");
        return cell_cmd(c, &cmd);
    case MB_CMD_STOP:
        copy_text(cmd.cmd, sizeof cmd.cmd, "stop");
        return cell_cmd(c, &cmd);
    case MB_CMD_RESET:
        copy_text(cmd.cmd, sizeof cmd.cmd, "reset");
        return cell_cmd(c, &cmd);
    case MB_CMD_SPEED:
        return apply_speed(c, (int)c->arg_speed);
    default:
        return YOOMI_ERR_ARG;
    }
}

static int read_reg(void *user, uint16_t reg, uint16_t *val)
{
    yoomi_cell_t *c = user;
    if (reg >= YOOMI_REG_COUNT) {
        return -1;
    }
    *val = c->regs[reg];
    return 0;
}

static int write_reg(void *user, uint16_t reg, uint16_t val)
{
    yoomi_cell_t *c = user;
    if (reg == REG_SLOT) {
        if (val >= YOOMI_SLOTS) {
            return -2;
        }
        c->arg_slot = val;
        return 0;
    }
    if (reg == REG_SPEED_ARG) {
        if (val < 1 || val > 100) {
            return -2;
        }
        c->arg_speed = val;
        return 0;
    }
    if (reg == REG_CMD) {
        return dispatch_mb(c, val) == YOOMI_OK ? 0 : -2;
    }
    return -1;
}

int cell_modbus(yoomi_cell_t *c, const uint8_t *req, size_t len, uint8_t *rsp, size_t cap)
{
    sync_regs(c);
    modbus_slave_t slave;
    slave.addr = YOOMI_MODBUS_ADDR;
    slave.read_reg = read_reg;
    slave.write_reg = write_reg;
    slave.user = c;
    return modbus_handle(&slave, req, len, rsp, cap);
}

static bool near_i(int32_t a, int32_t b)
{
    int32_t d = a - b;
    if (d < 0) {
        d = -d;
    }
    return d <= 1000;
}

static bool zone_slot(const yoomi_cell_t *c)
{
    if (c->slot_index < 0 || c->slot_index >= YOOMI_SLOTS) {
        return false;
    }
    return near_i(motion_pos(&c->motion, YOOMI_AXIS_X), yoomi_slot_x_um(c->slot_index)) &&
           near_i(motion_pos(&c->motion, YOOMI_AXIS_Y), yoomi_slot_y_um(c->slot_index)) &&
           near_i(motion_pos(&c->motion, YOOMI_AXIS_Z), YOOMI_Z_PICK_UM);
}

static bool zone_port(const yoomi_cell_t *c)
{
    return near_i(motion_pos(&c->motion, YOOMI_AXIS_X), YOOMI_PORT_X_UM) &&
           near_i(motion_pos(&c->motion, YOOMI_AXIS_Y), YOOMI_PORT_Y_UM) &&
           near_i(motion_pos(&c->motion, YOOMI_AXIS_Z), YOOMI_Z_PORT_UM);
}

static bool part_present(const yoomi_cell_t *c)
{
    if (c->sensor_override) {
        return c->sensor_override_val;
    }
    if (c->holding) {
        return true;
    }
    if (zone_slot(c) && c->slot_has[c->slot_index]) {
        return true;
    }
    if (zone_port(c) && c->port_has) {
        return true;
    }
    return false;
}

static int issue(yoomi_cell_t *c, uint8_t step)
{
    char name[3];
    name[0] = 0;
    if (c->slot_index >= 0) {
        yoomi_slot_name(c->slot_index, name);
    }
    switch (step) {
    case S_HOME_Z:
        set_pool(c, "回零 Z");
        return motion_home(&c->motion, YOOMI_AXIS_Z);
    case S_HOME_X:
        set_pool(c, "回零 X");
        return motion_home(&c->motion, YOOMI_AXIS_X);
    case S_HOME_Y:
        set_pool(c, "回零 Y");
        return motion_home(&c->motion, YOOMI_AXIS_Y);
    case S_Z_SAFE:
        set_pool(c, "抬升到安全高度");
        return motion_move_axis(&c->motion, YOOMI_AXIS_Z, YOOMI_Z_SAFE_UM);
    case S_XY_SLOT: {
        char buf[64];
        (void)snprintf(buf, sizeof buf, "前往库位 %s", name);
        set_pool(c, buf);
        return motion_move_xy(&c->motion, yoomi_slot_x_um(c->slot_index), yoomi_slot_y_um(c->slot_index));
    }
    case S_Z_SLOT:
        set_pool(c, c->job == YOOMI_JOB_IN ? "下降放料" : "下降取料");
        return motion_move_axis(&c->motion, YOOMI_AXIS_Z, YOOMI_Z_PICK_UM);
    case S_GRIP_CLOSE:
        set_pool(c, "夹取");
        return gripper_close(&c->grip);
    case S_XY_PORT:
        set_pool(c, "送至出库口");
        return motion_move_xy(&c->motion, YOOMI_PORT_X_UM, YOOMI_PORT_Y_UM);
    case S_Z_PORT:
        set_pool(c, c->job == YOOMI_JOB_OUT ? "下降放料" : "下降取料");
        return motion_move_axis(&c->motion, YOOMI_AXIS_Z, YOOMI_Z_PORT_UM);
    case S_GRIP_OPEN:
        if (c->job == YOOMI_JOB_IN && c->slot_index >= 0 && c->slot_has[c->slot_index]) {
            c->app_fault = FAULT_SLOT;
            return -1;
        }
        if (c->job == YOOMI_JOB_OUT && c->port_has) {
            c->app_fault = FAULT_SLOT;
            return -1;
        }
        set_pool(c, "松开夹爪");
        return gripper_open(&c->grip);
    case S_DONE:
    case S_STOPPED:
        return 0;
    default:
        return -1;
    }
}

static bool step_done(const yoomi_cell_t *c, uint8_t step)
{
    switch (step) {
    case S_HOME_Z:
        return c->motion.ax[YOOMI_AXIS_Z].homed && motion_settled(&c->motion);
    case S_HOME_X:
        return c->motion.ax[YOOMI_AXIS_X].homed && motion_settled(&c->motion);
    case S_HOME_Y:
        return c->motion.ax[YOOMI_AXIS_Y].homed && motion_settled(&c->motion);
    case S_Z_SAFE:
    case S_XY_SLOT:
    case S_Z_SLOT:
    case S_XY_PORT:
    case S_Z_PORT:
        return motion_settled(&c->motion);
    case S_GRIP_CLOSE:
        return !gripper_moving(&c->grip) && gripper_state(&c->grip) == GRIP_CLOSED;
    case S_GRIP_OPEN:
        return !gripper_moving(&c->grip) && gripper_state(&c->grip) == GRIP_OPEN;
    case S_DONE:
    case S_STOPPED:
        return true;
    default:
        return false;
    }
}

static void finish_step_message(yoomi_cell_t *c, uint8_t step)
{
    char name[3];
    name[0] = 0;
    if (c->slot_index >= 0) {
        yoomi_slot_name(c->slot_index, name);
    }
    if (step == S_DONE) {
        char buf[64];
        if (c->job == YOOMI_JOB_OUT) {
            (void)snprintf(buf, sizeof buf, "出库完成 %s", name);
        } else if (c->job == YOOMI_JOB_IN) {
            (void)snprintf(buf, sizeof buf, "入库完成 %s", name);
        } else {
            (void)snprintf(buf, sizeof buf, "回零完成");
        }
        set_pool(c, buf);
        publish_task(c, "FINISH");
        c->job_active = false;
        return;
    }
    if (step == S_STOPPED) {
        set_pool(c, "已停止");
        publish_task(c, "PAUSE");
        c->job_active = false;
    }
}

static void sequencer(yoomi_cell_t *c)
{
    if (c->paused || cell_fault(c) != FAULT_NONE) {
        return;
    }
    for (int guard = 0; guard < 8; guard++) {
        if (!c->job_active || c->step_i >= c->nsteps) {
            c->job_active = false;
            return;
        }
        uint8_t step = c->steps[c->step_i];
        if (!c->step_issued) {
            if (issue(c, step) != 0) {
                if (c->app_fault == FAULT_NONE) {
                    c->app_fault = FAULT_SOFT_LIMIT;
                }
                fail_job(c, c->app_fault);
                return;
            }
            c->step_issued = true;
            if (step == S_DONE || step == S_STOPPED) {
                finish_step_message(c, step);
                c->step_i += 1;
                return;
            }
            publish_task(c, "RUN");
        }
        if (step == S_GRIP_CLOSE && !gripper_moving(&c->grip) &&
            gripper_state(&c->grip) == GRIP_FAULT) {
            fail_job(c, FAULT_GRIP);
            return;
        }
        if (!step_done(c, step)) {
            return;
        }
        c->step_issued = false;
        c->step_i += 1;
    }
}

static void transfer(yoomi_cell_t *c, grip_state_t before, grip_state_t now)
{
    if (before != GRIP_CLOSED && now == GRIP_CLOSED && !c->holding) {
        if (zone_slot(c) && c->slot_has[c->slot_index]) {
            c->slot_has[c->slot_index] = false;
            c->holding = true;
        } else if (zone_port(c) && c->port_has) {
            c->port_has = false;
            c->holding = true;
        }
    }
    if (before != GRIP_OPEN && now == GRIP_OPEN && c->holding) {
        if (zone_port(c)) {
            c->holding = false;
            c->port_has = true;
        } else if (zone_slot(c)) {
            if (c->slot_has[c->slot_index]) {
                fail_job(c, FAULT_SLOT);
                return;
            }
            c->holding = false;
            c->slot_has[c->slot_index] = true;
        } else {
            c->holding = false;
            fail_job(c, FAULT_DROPPED);
        }
    }
}

static void publish_status(yoomi_cell_t *c)
{
    if (c->pub == 0) {
        return;
    }
    char x[16], y[16], z[16];
    int32_t px = motion_pos(&c->motion, YOOMI_AXIS_X);
    int32_t py = motion_pos(&c->motion, YOOMI_AXIS_Y);
    int32_t pz = motion_pos(&c->motion, YOOMI_AXIS_Z);
    (void)snprintf(x, sizeof x, "%ld.%02ld", (long)(px / 1000), (long)((px % 1000) / 10));
    (void)snprintf(y, sizeof y, "%ld.%02ld", (long)(py / 1000), (long)((py % 1000) / 10));
    (void)snprintf(z, sizeof z, "%ld.%02ld", (long)(pz / 1000), (long)((pz % 1000) / 10));
    const char *grip = "open";
    switch (gripper_state(&c->grip)) {
    case GRIP_CLOSING:
        grip = "closing";
        break;
    case GRIP_CLOSED:
        grip = "closed";
        break;
    case GRIP_OPENING:
        grip = "opening";
        break;
    case GRIP_FAULT:
        grip = "fault";
        break;
    case GRIP_OPEN:
    default:
        grip = "open";
        break;
    }
    char buf[192];
    int n = snprintf(buf, sizeof buf,
                     "{\"homed\":%d,\"fault\":%d,\"busy\":%d,\"paused\":%d,\"x\":%s,\"y\":%s,\"z\":%s,"
                     "\"gripper\":\"%s\",\"holding\":%d}",
                     motion_homed_all(&c->motion) ? 1 : 0, cell_fault(c), c->job_active ? 1 : 0,
                     c->paused ? 1 : 0, x, y, z, grip, c->holding ? 1 : 0);
    if (n > 0 && (size_t)n < sizeof buf) {
        c->pub("sss/cell/status", buf, c->pub_user);
    }
}

void cell_init(yoomi_cell_t *c)
{
    memset(c, 0, sizeof(*c));
    motion_init(&c->motion);
    gripper_init(&c->grip);
    safety_init(&c->safety);
    c->door_closed = true;
    c->speed_pct = 100;
    c->arg_speed = 100;
    c->slot_index = -1;
    for (int i = 0; i < YOOMI_SLOTS; i++) {
        c->slot_has[i] = true;
    }
    motion_set_pos_raw(&c->motion, YOOMI_AXIS_X, 80000);
    motion_set_pos_raw(&c->motion, YOOMI_AXIS_Y, 60000);
    motion_set_pos_raw(&c->motion, YOOMI_AXIS_Z, 30000);
    sync_regs(c);
}

void cell_set_publisher(yoomi_cell_t *c, yoomi_pub_fn fn, void *user)
{
    c->pub = fn;
    c->pub_user = user;
}

void cell_kick(yoomi_cell_t *c)
{
    safety_kick(&c->safety);
}

void cell_set_estop(yoomi_cell_t *c, bool asserted)
{
    c->estop_in = asserted;
    safety_set_inputs(&c->safety, c->estop_in, c->door_closed);
}

void cell_set_door_closed(yoomi_cell_t *c, bool closed)
{
    c->door_closed = closed;
    safety_set_inputs(&c->safety, c->estop_in, c->door_closed);
}

void cell_tick(yoomi_cell_t *c, uint32_t dt_ms)
{
    if (dt_ms == 0) {
        return;
    }
    safety_set_inputs(&c->safety, c->estop_in, c->door_closed);
    bool was_tripped = c->safety.tripped;
    safety_tick(&c->safety, dt_ms);
    if (!was_tripped && c->safety.tripped) {
        motion_halt(&c->motion, c->safety.fault == FAULT_ESTOP);
        c->job_active = false;
        c->paused = false;
        set_pool(c, fault_text(c->safety.fault));
        publish_task(c, "PAUSE");
    }
    if (!c->motion.halt) {
        motion_pause(&c->motion, c->paused);
    }
    motion_tick(&c->motion, dt_ms);
    if (motion_limit_fault(&c->motion) && c->app_fault == FAULT_NONE && !c->safety.tripped) {
        fail_job(c, FAULT_SOFT_LIMIT);
    }
    if (cell_fault(c) == FAULT_NONE) {
        grip_state_t before = gripper_state(&c->grip);
        gripper_tick(&c->grip, dt_ms, part_present(c));
        transfer(c, before, gripper_state(&c->grip));
        if (cell_fault(c) == FAULT_NONE) {
            sequencer(c);
        }
    }
    c->status_acc_ms += dt_ms;
    uint32_t beats = c->status_acc_ms / 100U;
    if (beats > 0) {
        c->status_acc_ms %= 100U;
        c->heartbeat += beats;
        publish_status(c);
    }
    sync_regs(c);
}

int cell_fault(const yoomi_cell_t *c)
{
    if (c->safety.tripped) {
        return c->safety.fault;
    }
    return c->app_fault;
}

bool cell_job_active(const yoomi_cell_t *c)
{
    return c->job_active;
}

bool cell_homed(const yoomi_cell_t *c)
{
    return motion_homed_all(&c->motion);
}

int32_t cell_pos_um(const yoomi_cell_t *c, int axis)
{
    return motion_pos(&c->motion, axis);
}

int32_t cell_vel_um_s(const yoomi_cell_t *c, int axis)
{
    return motion_vel(&c->motion, axis);
}

const char *cell_task_json(const yoomi_cell_t *c)
{
    return c->task_json;
}

void cell_sim_set_slot(yoomi_cell_t *c, int slot, bool occupied)
{
    if (slot >= 0 && slot < YOOMI_SLOTS) {
        c->slot_has[slot] = occupied;
    }
}

void cell_sim_set_port(yoomi_cell_t *c, bool occupied)
{
    c->port_has = occupied;
}

void cell_sim_force_sensor(yoomi_cell_t *c, bool enable, bool present)
{
    c->sensor_override = enable;
    c->sensor_override_val = present;
}

bool cell_sim_slot(const yoomi_cell_t *c, int slot)
{
    if (slot < 0 || slot >= YOOMI_SLOTS) {
        return false;
    }
    return c->slot_has[slot];
}

bool cell_sim_port(const yoomi_cell_t *c)
{
    return c->port_has;
}

bool cell_sim_holding(const yoomi_cell_t *c)
{
    return c->holding;
}
