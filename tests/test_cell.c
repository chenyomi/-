#include "test.h"

#include "yoomi/cell.h"
#include "yoomi/modbus.h"

#include <stdio.h>
#include <string.h>

static void kick_tick(yoomi_cell_t *c, uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++) {
        cell_kick(c);
        cell_tick(c, 1);
    }
}

static int run_job(yoomi_cell_t *c, const char *group, uint32_t max_ms)
{
    for (uint32_t t = 0; t < max_ms; t++) {
        cell_kick(c);
        cell_tick(c, 1);
        if (cell_fault(c) != FAULT_NONE) {
            fprintf(stderr, "job fault=%s json=%s x=%ld y=%ld z=%ld\n", cell_fault_name(cell_fault(c)),
                    cell_task_json(c), (long)cell_pos_um(c, YOOMI_AXIS_X), (long)cell_pos_um(c, YOOMI_AXIS_Y),
                    (long)cell_pos_um(c, YOOMI_AXIS_Z));
            return -1;
        }
        const char *js = cell_task_json(c);
        if (!cell_job_active(c) && strstr(js, group) != 0 &&
            strstr(js, "\"groupState\":\"FINISH\"") != 0) {
            return 0;
        }
    }
    fprintf(stderr, "job timeout json=%s x=%ld y=%ld z=%ld\n", cell_task_json(c),
            (long)cell_pos_um(c, YOOMI_AXIS_X), (long)cell_pos_um(c, YOOMI_AXIS_Y),
            (long)cell_pos_um(c, YOOMI_AXIS_Z));
    return -1;
}

static size_t mb_frame(uint8_t *out, const uint8_t *pdu, size_t n)
{
    memcpy(out, pdu, n);
    uint16_t crc = modbus_crc(out, n);
    out[n] = (uint8_t)(crc & 0xFF);
    out[n + 1] = (uint8_t)(crc >> 8);
    return n + 2;
}

void test_cell(void)
{
    yoomi_cell_t cell;
    cell_init(&cell);
    cell_sim_set_slot(&cell, yoomi_slot_index("A1"), false);
    EXPECT(cell_cmd_json(&cell, "{\"cmd\":\"storage_out\",\"groupId\":\"G1\",\"slot\":\"A1\"}") ==
           YOOMI_ERR_STATE);
    EXPECT(!cell_job_active(&cell));
    EXPECT(cell_cmd_json(&cell, "{\"cmd\":\"nope\"}") == YOOMI_ERR_ARG);

    cell_init(&cell);
    EXPECT(cell_cmd_json(&cell, "{\"cmd\":\"storage_out\",\"groupId\":\"G1001\",\"slot\":\"A3\"}") ==
           YOOMI_OK);
    EXPECT(cell_cmd_json(&cell, "{\"cmd\":\"storage_out\",\"groupId\":\"G1009\",\"slot\":\"B1\"}") ==
           YOOMI_ERR_BUSY);
    EXPECT(run_job(&cell, "G1001", 60000) == 0);
    int a3 = yoomi_slot_index("A3");
    EXPECT(!cell_sim_slot(&cell, a3));
    EXPECT(cell_sim_port(&cell));
    EXPECT(!cell_sim_holding(&cell));
    EXPECT(cell_homed(&cell));
    EXPECT(strstr(cell_task_json(&cell), "\"groupId\":\"G1001\"") != 0);
    EXPECT(strstr(cell_task_json(&cell), "\"groupState\":\"FINISH\"") != 0);
    EXPECT(strstr(cell_task_json(&cell), "\"taskNumber\":") != 0);
    EXPECT(strstr(cell_task_json(&cell), "\"poolInfo\":") != 0);
    EXPECT(cell_pos_um(&cell, YOOMI_AXIS_X) == YOOMI_PORT_X_UM);
    EXPECT(cell_pos_um(&cell, YOOMI_AXIS_Y) == YOOMI_PORT_Y_UM);
    EXPECT(cell_pos_um(&cell, YOOMI_AXIS_Z) == YOOMI_Z_SAFE_UM);

    EXPECT(cell_cmd_json(&cell, "{\"cmd\":\"storage_in\",\"groupId\":\"G1002\",\"slot\":\"A3\",\"speed\":80}") ==
           YOOMI_OK);
    EXPECT(run_job(&cell, "G1002", 60000) == 0);
    EXPECT(cell_sim_slot(&cell, a3));
    EXPECT(!cell_sim_port(&cell));
    EXPECT(cell_pos_um(&cell, YOOMI_AXIS_X) == yoomi_slot_x_um(a3));
    EXPECT(cell_pos_um(&cell, YOOMI_AXIS_Y) == yoomi_slot_y_um(a3));
    EXPECT(cell_pos_um(&cell, YOOMI_AXIS_Z) == 0);

    cell_init(&cell);
    EXPECT(cell_cmd_json(&cell, "{\"cmd\":\"storage_out\",\"groupId\":\"P1\",\"slot\":\"B2\"}") == YOOMI_OK);
    kick_tick(&cell, 1500);
    EXPECT(cell_job_active(&cell));
    EXPECT(cell_cmd_json(&cell, "{\"cmd\":\"pause\"}") == YOOMI_OK);
    EXPECT(strstr(cell_task_json(&cell), "\"groupState\":\"PAUSE\"") != 0);
    kick_tick(&cell, 250);
    EXPECT(cell_vel_um_s(&cell, YOOMI_AXIS_X) == 0);
    EXPECT(cell_vel_um_s(&cell, YOOMI_AXIS_Y) == 0);
    EXPECT(cell_vel_um_s(&cell, YOOMI_AXIS_Z) == 0);
    int32_t hx = cell_pos_um(&cell, YOOMI_AXIS_X);
    int32_t hy = cell_pos_um(&cell, YOOMI_AXIS_Y);
    int32_t hz = cell_pos_um(&cell, YOOMI_AXIS_Z);
    kick_tick(&cell, 200);
    EXPECT(cell_pos_um(&cell, YOOMI_AXIS_X) == hx);
    EXPECT(cell_pos_um(&cell, YOOMI_AXIS_Y) == hy);
    EXPECT(cell_pos_um(&cell, YOOMI_AXIS_Z) == hz);
    EXPECT(cell_cmd_json(&cell, "{\"cmd\":\"resume\"}") == YOOMI_OK);
    EXPECT(run_job(&cell, "P1", 60000) == 0);
    EXPECT(!cell_sim_slot(&cell, yoomi_slot_index("B2")));

    cell_init(&cell);
    cell_sim_force_sensor(&cell, true, false);
    EXPECT(cell_cmd_json(&cell, "{\"cmd\":\"storage_out\",\"groupId\":\"E1\",\"slot\":\"C1\"}") == YOOMI_OK);
    int saw_grip = 0;
    for (int i = 0; i < 60000 && cell_fault(&cell) == FAULT_NONE; i++) {
        cell_kick(&cell);
        cell_tick(&cell, 1);
        if (cell_fault(&cell) == FAULT_GRIP) {
            saw_grip = 1;
        }
    }
    EXPECT(saw_grip == 1);
    EXPECT(cell_sim_slot(&cell, yoomi_slot_index("C1")));
    EXPECT(!cell_sim_holding(&cell));
    EXPECT(strstr(cell_task_json(&cell), "夹爪空抓") != 0);

    cell_init(&cell);
    EXPECT(cell_cmd_json(&cell, "{\"cmd\":\"home\",\"groupId\":\"H1\"}") == YOOMI_OK);
    kick_tick(&cell, 400);
    cell_set_estop(&cell, true);
    kick_tick(&cell, 150);
    EXPECT(cell_fault(&cell) == FAULT_ESTOP);
    EXPECT(cell_vel_um_s(&cell, YOOMI_AXIS_X) == 0);
    EXPECT(cell_vel_um_s(&cell, YOOMI_AXIS_Y) == 0);
    EXPECT(cell_vel_um_s(&cell, YOOMI_AXIS_Z) == 0);
    EXPECT(cell_cmd_json(&cell, "{\"cmd\":\"home\"}") == YOOMI_ERR_STATE);
    cell_set_estop(&cell, false);
    EXPECT(cell_cmd_json(&cell, "{\"cmd\":\"reset\"}") == YOOMI_OK);
    EXPECT(cell_fault(&cell) == FAULT_NONE);

    cell_init(&cell);
    cell_set_door_closed(&cell, false);
    cell_kick(&cell);
    cell_tick(&cell, 1);
    EXPECT(cell_fault(&cell) == FAULT_DOOR);
    EXPECT(cell_cmd_json(&cell, "{\"cmd\":\"reset\"}") == YOOMI_ERR_STATE);
    cell_set_door_closed(&cell, true);
    EXPECT(cell_cmd_json(&cell, "{\"cmd\":\"reset\"}") == YOOMI_OK);

    cell_init(&cell);
    for (int i = 0; i < 499; i++) {
        cell_tick(&cell, 1);
    }
    EXPECT(cell_fault(&cell) == FAULT_NONE);
    cell_tick(&cell, 1);
    EXPECT(cell_fault(&cell) == FAULT_WATCHDOG);
    cell_kick(&cell);
    EXPECT(cell_cmd_json(&cell, "{\"cmd\":\"reset\"}") == YOOMI_OK);
    EXPECT(cell_fault(&cell) == FAULT_NONE);

    cell_init(&cell);
    kick_tick(&cell, 250);
    EXPECT(cell.regs[REG_HEARTBEAT] >= 2);
    EXPECT((cell.regs[REG_STATUS] & ST_DOOR) != 0);
    EXPECT((cell.regs[REG_STATUS] & ST_HOMED) == 0);

    uint8_t req[32], rsp[64];
    const uint8_t read4[] = {YOOMI_MODBUS_ADDR, 0x03, 0x00, 0x00, 0x00, 0x04};
    size_t n = mb_frame(req, read4, sizeof read4);
    int rn = cell_modbus(&cell, req, n, rsp, sizeof rsp);
    EXPECT(rn > 5);
    EXPECT(rsp[1] == 0x03 && rsp[2] == 8);
    EXPECT(rsp[3] == 0x00 && rsp[4] == (uint8_t)ST_DOOR);

    const uint8_t slot[] = {YOOMI_MODBUS_ADDR, 0x06, 0x00, 0x0B, 0x00, 0x08};
    n = mb_frame(req, slot, sizeof slot);
    EXPECT(cell_modbus(&cell, req, n, rsp, sizeof rsp) == 8);
    const uint8_t go[] = {YOOMI_MODBUS_ADDR, 0x06, 0x00, 0x0A, 0x00, MB_CMD_OUT};
    n = mb_frame(req, go, sizeof go);
    EXPECT(cell_modbus(&cell, req, n, rsp, sizeof rsp) == 8);
    EXPECT(run_job(&cell, "MB", 60000) == 0);
    EXPECT(!cell_sim_slot(&cell, 8));
    EXPECT(cell_sim_port(&cell));

    const uint8_t bad_slot[] = {YOOMI_MODBUS_ADDR, 0x06, 0x00, 0x0B, 0x00, 0x63};
    n = mb_frame(req, bad_slot, sizeof bad_slot);
    rn = cell_modbus(&cell, req, n, rsp, sizeof rsp);
    EXPECT(rsp[1] == (uint8_t)(0x06 | 0x80));
}
