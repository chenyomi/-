#include "yoomi/json_msg.h"

#include <stdio.h>
#include <string.h>

static int append_raw(char *dst, size_t cap, size_t *i, const char *s)
{
    for (; *s; s++) {
        if (*i + 1 >= cap) {
            return -1;
        }
        dst[(*i)++] = *s;
    }
    return 0;
}

static int append_escaped(char *dst, size_t cap, size_t *i, const char *s)
{
    for (; *s; s++) {
        unsigned char c = (unsigned char)*s;
        if (c < 0x20) {
            return -1;
        }
        if (c == '"' || c == '\\') {
            if (*i + 2 >= cap) {
                return -1;
            }
            dst[(*i)++] = '\\';
            dst[(*i)++] = (char)c;
        } else {
            if (*i + 1 >= cap) {
                return -1;
            }
            dst[(*i)++] = (char)c;
        }
    }
    return 0;
}

int sss_task_json(char *buf, size_t cap, const char *group_id, int task_number,
                  const char *group_state, const char *pool_info)
{
    if (buf == 0 || cap == 0 || group_id == 0 || group_state == 0 || pool_info == 0) {
        return -1;
    }
    char number[16];
    int nw = snprintf(number, sizeof number, "%d", task_number);
    if (nw <= 0 || (size_t)nw >= sizeof number) {
        return -1;
    }
    size_t i = 0;
    if (append_raw(buf, cap, &i, "{\"groupId\":\"") != 0 ||
        append_escaped(buf, cap, &i, group_id) != 0 ||
        append_raw(buf, cap, &i, "\",\"taskNumber\":") != 0 ||
        append_raw(buf, cap, &i, number) != 0 ||
        append_raw(buf, cap, &i, ",\"groupState\":\"") != 0 ||
        append_escaped(buf, cap, &i, group_state) != 0 ||
        append_raw(buf, cap, &i, "\",\"poolInfo\":\"") != 0 ||
        append_escaped(buf, cap, &i, pool_info) != 0 ||
        append_raw(buf, cap, &i, "\"}") != 0) {
        return -1;
    }
    if (i >= cap) {
        return -1;
    }
    buf[i] = 0;
    return 0;
}

static const char *find_key(const char *json, const char *key)
{
    char pat[40];
    int n = snprintf(pat, sizeof pat, "\"%s\"", key);
    if (n <= 0 || (size_t)n >= sizeof pat) {
        return 0;
    }
    const char *p = strstr(json, pat);
    if (p == 0) {
        return 0;
    }
    p += (size_t)n;
    while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') {
        p++;
    }
    if (*p != ':') {
        return 0;
    }
    p++;
    while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') {
        p++;
    }
    return p;
}

static int take_string(const char *p, char *out, size_t cap)
{
    if (*p != '"') {
        return -1;
    }
    p++;
    size_t i = 0;
    while (*p && *p != '"') {
        if (*p == '\\') {
            return -1;
        }
        if (i + 1 >= cap) {
            return -1;
        }
        out[i++] = *p++;
    }
    if (*p != '"') {
        return -1;
    }
    out[i] = 0;
    return 0;
}

static int ident_ok(const char *s, size_t max_body)
{
    size_t n = strlen(s);
    if (n == 0 || n > max_body) {
        return 0;
    }
    for (size_t i = 0; i < n; i++) {
        char c = s[i];
        int ok = (c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                 c == '_' || c == '-';
        if (!ok) {
            return 0;
        }
    }
    return 1;
}

int yoomi_cmd_parse(const char *json, yoomi_cmd_t *out)
{
    if (json == 0 || out == 0) {
        return -1;
    }
    memset(out, 0, sizeof(*out));
    out->speed = -1;
    const char *cmd = find_key(json, "cmd");
    if (cmd == 0 || take_string(cmd, out->cmd, sizeof out->cmd) != 0 || !ident_ok(out->cmd, 16)) {
        return -1;
    }
    const char *gid = find_key(json, "groupId");
    if (gid != 0) {
        if (take_string(gid, out->group_id, sizeof out->group_id) != 0 ||
            !ident_ok(out->group_id, 16)) {
            return -1;
        }
        out->has_group = true;
    }
    const char *slot = find_key(json, "slot");
    if (slot != 0) {
        if (take_string(slot, out->slot, sizeof out->slot) != 0 || out->slot[0] == 0) {
            return -1;
        }
        out->has_slot = true;
    }
    const char *spd = find_key(json, "speed");
    if (spd != 0) {
        if (*spd == '"') {
            return -1;
        }
        int sign = 1;
        if (*spd == '-') {
            sign = -1;
            spd++;
        }
        if (*spd < '0' || *spd > '9') {
            return -1;
        }
        int value = 0;
        while (*spd >= '0' && *spd <= '9') {
            value = value * 10 + (*spd - '0');
            if (value > 1000) {
                return -1;
            }
            spd++;
        }
        out->speed = sign * value;
        out->has_speed = true;
    }
    return 0;
}
