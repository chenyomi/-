#include "yoomi/mqtt_pkt.h"

#include <string.h>

static int write_varint(uint8_t *p, size_t cap, uint32_t n, size_t *used)
{
    size_t i = 0;
    do {
        if (i >= cap) {
            return -1;
        }
        uint8_t b = (uint8_t)(n % 128U);
        n /= 128U;
        if (n > 0) {
            b |= 0x80;
        }
        p[i++] = b;
    } while (n > 0 && i < 4);
    if (n > 0) {
        return -1;
    }
    *used = i;
    return 0;
}

static int read_varint(const uint8_t *p, size_t len, uint32_t *out, int *used)
{
    uint32_t value = 0;
    uint32_t mul = 1;
    for (int i = 0; i < 4; i++) {
        if ((size_t)i >= len) {
            return -1;
        }
        uint8_t b = p[i];
        value += (uint32_t)(b & 0x7F) * mul;
        if ((b & 0x80) == 0) {
            *out = value;
            *used = i + 1;
            return 0;
        }
        mul *= 128U;
    }
    return -1;
}

static size_t finish(uint8_t *buf, size_t cap, uint8_t header, const uint8_t *body, size_t body_len)
{
    uint8_t var[4];
    size_t var_len = 0;
    if (write_varint(var, sizeof var, (uint32_t)body_len, &var_len) != 0) {
        return 0;
    }
    if (1 + var_len + body_len > cap) {
        return 0;
    }
    buf[0] = header;
    memcpy(buf + 1, var, var_len);
    if (body_len > 0) {
        memcpy(buf + 1 + var_len, body, body_len);
    }
    return 1 + var_len + body_len;
}

static int put_str(uint8_t *p, size_t cap, size_t *i, const char *s)
{
    size_t n = strlen(s);
    if (n > 65535 || *i + 2 + n > cap) {
        return -1;
    }
    p[(*i)++] = (uint8_t)(n >> 8);
    p[(*i)++] = (uint8_t)(n & 0xFF);
    memcpy(p + *i, s, n);
    *i += n;
    return 0;
}

size_t mqtt_enc_connect(uint8_t *buf, size_t cap, const char *client_id, uint16_t keepalive_s)
{
    if (buf == 0 || client_id == 0 || client_id[0] == 0) {
        return 0;
    }
    uint8_t body[160];
    size_t i = 0;
    if (put_str(body, sizeof body, &i, "MQTT") != 0) {
        return 0;
    }
    if (i + 4 > sizeof body) {
        return 0;
    }
    body[i++] = 0x04;
    body[i++] = 0x02;
    body[i++] = (uint8_t)(keepalive_s >> 8);
    body[i++] = (uint8_t)(keepalive_s & 0xFF);
    if (put_str(body, sizeof body, &i, client_id) != 0) {
        return 0;
    }
    return finish(buf, cap, 0x10, body, i);
}

size_t mqtt_enc_subscribe(uint8_t *buf, size_t cap, uint16_t packet_id, const char *topic)
{
    if (buf == 0 || topic == 0 || topic[0] == 0) {
        return 0;
    }
    uint8_t body[160];
    size_t i = 0;
    body[i++] = (uint8_t)(packet_id >> 8);
    body[i++] = (uint8_t)(packet_id & 0xFF);
    if (put_str(body, sizeof body, &i, topic) != 0) {
        return 0;
    }
    if (i >= sizeof body) {
        return 0;
    }
    body[i++] = 0x00;
    return finish(buf, cap, 0x82, body, i);
}

size_t mqtt_enc_publish(uint8_t *buf, size_t cap, const char *topic,
                        const void *payload, size_t payload_len)
{
    if (buf == 0 || topic == 0 || topic[0] == 0) {
        return 0;
    }
    if (payload_len > 0 && payload == 0) {
        return 0;
    }
    if (payload_len > MQTT_PAYLOAD_MAX || strlen(topic) >= MQTT_TOPIC_MAX) {
        return 0;
    }
    uint8_t body[MQTT_PAYLOAD_MAX + MQTT_TOPIC_MAX + 4];
    size_t i = 0;
    if (put_str(body, sizeof body, &i, topic) != 0) {
        return 0;
    }
    if (i + payload_len > sizeof body) {
        return 0;
    }
    if (payload_len > 0) {
        memcpy(body + i, payload, payload_len);
        i += payload_len;
    }
    return finish(buf, cap, 0x30, body, i);
}

size_t mqtt_enc_pingreq(uint8_t *buf, size_t cap)
{
    if (buf == 0 || cap < 2) {
        return 0;
    }
    buf[0] = 0xC0;
    buf[1] = 0x00;
    return 2;
}

int mqtt_parse(const uint8_t *buf, size_t len, mqtt_packet_t *out, size_t *consumed)
{
    if (buf == 0 || out == 0 || consumed == 0 || len < 2) {
        return -1;
    }
    memset(out, 0, sizeof(*out));
    uint32_t rem = 0;
    int used = 0;
    if (read_varint(buf + 1, len - 1, &rem, &used) != 0) {
        return -1;
    }
    size_t total = 1U + (size_t)used + (size_t)rem;
    if (total > len) {
        return -1;
    }
    const uint8_t *p = buf + 1 + used;
    out->type = buf[0] >> 4;
    if (out->type == MQTT_PKT_CONNACK) {
        if (rem < 2) {
            return -1;
        }
        out->return_code = p[1];
    } else if (out->type == MQTT_PKT_PUBLISH) {
        if (rem < 2) {
            return -1;
        }
        uint16_t tlen = (uint16_t)((p[0] << 8) | p[1]);
        if ((size_t)tlen + 2 > rem || tlen == 0 || tlen >= MQTT_TOPIC_MAX) {
            return -1;
        }
        memcpy(out->topic, p + 2, tlen);
        out->topic[tlen] = 0;
        size_t plen = (size_t)rem - 2U - (size_t)tlen;
        if (plen > MQTT_PAYLOAD_MAX) {
            return -1;
        }
        if (plen > 0) {
            memcpy(out->payload, p + 2 + tlen, plen);
        }
        out->payload_len = plen;
    } else if (out->type == MQTT_PKT_SUBACK) {
        if (rem < 3) {
            return -1;
        }
        out->packet_id = (uint16_t)((p[0] << 8) | p[1]);
        out->return_code = p[2];
    } else if (out->type == MQTT_PKT_SUBSCRIBE) {
        if (rem < 2) {
            return -1;
        }
        out->packet_id = (uint16_t)((p[0] << 8) | p[1]);
    }
    *consumed = total;
    return 0;
}
