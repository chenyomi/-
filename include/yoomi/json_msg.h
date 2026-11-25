#ifndef YOOMI_JSON_MSG_H
#define YOOMI_JSON_MSG_H

#include <stdbool.h>
#include <stddef.h>

#define YOOMI_ID_MAX 24
#define YOOMI_CMD_MAX 24
#define YOOMI_SLOT_MAX 16

typedef struct {
    char cmd[YOOMI_CMD_MAX];
    char group_id[YOOMI_ID_MAX];
    char slot[YOOMI_SLOT_MAX];
    int speed;
    bool has_speed;
    bool has_group;
    bool has_slot;
} yoomi_cmd_t;

int sss_task_json(char *buf, size_t cap, const char *group_id, int task_number,
                  const char *group_state, const char *pool_info);
int yoomi_cmd_parse(const char *json, yoomi_cmd_t *out);

#endif
