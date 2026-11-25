#ifndef YOOMI_MQTT_PKT_H
#define YOOMI_MQTT_PKT_H

#include <stddef.h>
#include <stdint.h>

enum {
    MQTT_PKT_CONNECT = 1,
    MQTT_PKT_CONNACK = 2,
    MQTT_PKT_PUBLISH = 3,
    MQTT_PKT_SUBSCRIBE = 8,
    MQTT_PKT_SUBACK = 9,
    MQTT_PKT_PINGREQ = 12,
    MQTT_PKT_PINGRESP = 13
};

#define MQTT_TOPIC_MAX 128
#define MQTT_PAYLOAD_MAX 512

typedef struct {
    int type;
    int return_code;
    uint16_t packet_id;
    char topic[MQTT_TOPIC_MAX];
    uint8_t payload[MQTT_PAYLOAD_MAX];
    size_t payload_len;
} mqtt_packet_t;

size_t mqtt_enc_connect(uint8_t *buf, size_t cap, const char *client_id, uint16_t keepalive_s);
size_t mqtt_enc_subscribe(uint8_t *buf, size_t cap, uint16_t packet_id, const char *topic);
size_t mqtt_enc_publish(uint8_t *buf, size_t cap, const char *topic,
                        const void *payload, size_t payload_len);
size_t mqtt_enc_pingreq(uint8_t *buf, size_t cap);
int mqtt_parse(const uint8_t *buf, size_t len, mqtt_packet_t *out, size_t *consumed);

#endif
