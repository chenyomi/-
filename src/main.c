#include "yoomi/cell.h"

#include <stdio.h>
#include <string.h>

static void on_pub(const char *topic, const char *json, void *user)
{
    int *tasks = user;
    if (strcmp(topic, "sss/task") == 0) {
        *tasks += 1;
        printf("[sss/task] %s\n", json);
    }
}

static int run_until_done(yoomi_cell_t *cell, const char *group, uint32_t max_ms)
{
    for (uint32_t t = 0; t < max_ms; t++) {
        cell_kick(cell);
        cell_tick(cell, 1);
        if (cell_fault(cell) != FAULT_NONE) {
            return -1;
        }
        const char *js = cell_task_json(cell);
        if (!cell_job_active(cell) && strstr(js, group) != 0 &&
            strstr(js, "\"groupState\":\"FINISH\"") != 0) {
            return 0;
        }
    }
    return -1;
}

static void print_mm(const char *name, int32_t um)
{
    int32_t a = um < 0 ? -um : um;
    printf(" %s=%s%ld.%02ld", name, um < 0 ? "-" : "", (long)(a / 1000), (long)((a % 1000) / 10));
}

int main(void)
{
    yoomi_cell_t cell;
    int task_msgs = 0;
    cell_init(&cell);
    cell_set_publisher(&cell, on_pub, &task_msgs);

    printf("YooMi Cell Controller %s\n", YOOMI_VERSION);
    printf("host SIL  |  4x6 slots  |  MQTT sss/task  |  Modbus RTU slave %u\n",
           (unsigned)YOOMI_MODBUS_ADDR);

    const char *out_cmd =
        "{\"cmd\":\"storage_out\",\"groupId\":\"G1001\",\"slot\":\"A3\",\"speed\":100}";
    if (cell_cmd_json(&cell, out_cmd) != YOOMI_OK || run_until_done(&cell, "G1001", 60000) != 0) {
        printf("RESULT fail stage=out fault=%s json=%s\n", cell_fault_name(cell_fault(&cell)),
               cell_task_json(&cell));
        return 1;
    }
    if (cell_sim_slot(&cell, yoomi_slot_index("A3")) || !cell_sim_port(&cell)) {
        printf("RESULT fail stage=out inventory\n");
        return 1;
    }

    const char *in_cmd =
        "{\"cmd\":\"storage_in\",\"groupId\":\"G1002\",\"slot\":\"A3\",\"speed\":80}";
    if (cell_cmd_json(&cell, in_cmd) != YOOMI_OK || run_until_done(&cell, "G1002", 60000) != 0) {
        printf("RESULT fail stage=in fault=%s json=%s\n", cell_fault_name(cell_fault(&cell)),
               cell_task_json(&cell));
        return 1;
    }

    int a3 = yoomi_slot_index("A3");
    bool slot_ok = cell_sim_slot(&cell, a3) && !cell_sim_port(&cell) && !cell_sim_holding(&cell);
    bool pos_ok = cell_pos_um(&cell, YOOMI_AXIS_X) == yoomi_slot_x_um(a3) &&
                  cell_pos_um(&cell, YOOMI_AXIS_Y) == yoomi_slot_y_um(a3) &&
                  cell_pos_um(&cell, YOOMI_AXIS_Z) == YOOMI_Z_SAFE_UM;
    printf("RESULT %s task_msgs=%d", (slot_ok && pos_ok) ? "ok" : "fail", task_msgs);
    print_mm("x", cell_pos_um(&cell, YOOMI_AXIS_X));
    print_mm("y", cell_pos_um(&cell, YOOMI_AXIS_Y));
    print_mm("z", cell_pos_um(&cell, YOOMI_AXIS_Z));
    printf(" slotA3=%d port=%d\n", cell_sim_slot(&cell, a3) ? 1 : 0, cell_sim_port(&cell) ? 1 : 0);
    return (slot_ok && pos_ok) ? 0 : 1;
}
