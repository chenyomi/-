#include "test.h"

#include "yoomi/cell.h"
#include "yoomi/modbus.h"
#include "yoomi/mqtt_pkt.h"

#include <string.h>

static int contains(const uint8_t *buf, size_t len, const char *lit)
{
    size_t n = strlen(lit);
    if (n > len) {
        return 0;
    }
    for (size_t i = 0; i + n <= len; i++) {
        if (memcmp(buf + i, lit, n) == 0) {
            return 1;
        }
    }
    return 0;
}

static size_t mb_frame(uint8_t *out, const uint8_t *pdu, size_t n)
{
    memcpy(out, pdu, n);
    uint16_t crc = modbus_crc(out, n);
    out[n] = (uint8_t)(crc & 0xFF);
    out[n + 1] = (uint8_t)(crc >> 8);
    return n + 2;
}

static int read_holding(void *user, uint16_t reg, uint16_t *val)
{
    const uint16_t *regs = user;
    if (reg > 3) {
        return -1;
    }
    *val = regs[reg];
    return 0;
}

static int write_holding(void *user, uint16_t reg, uint16_t val)
{
    uint16_t *regs = user;
    if (reg != 1) {
        return -1;
    }
    if (val > 100) {
        return -2;
    }
    regs[reg] = val;
    return 0;
}

void test_protocol(void)
{
    const uint8_t spec[] = {0x11, 0x03, 0x00, 0x6B, 0x00, 0x03};
    EXPECT(modbus_crc(spec, sizeof spec) == 0x8776);
    const uint8_t qty[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x0A};
    EXPECT(modbus_crc(qty, sizeof qty) == 0xCDC5);

    uint16_t regs[4] = {0x1234, 10, 0, 0};
    modbus_slave_t slave;
    slave.addr = 0x01;
    slave.read_reg = read_holding;
    slave.write_reg = write_holding;
    slave.user = regs;

    uint8_t req[32], rsp[64];
    const uint8_t read_pdu[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02};
    size_t n = mb_frame(req, read_pdu, sizeof read_pdu);
    int rn = modbus_handle(&slave, req, n, rsp, sizeof rsp);
    EXPECT(rn == 9);
    EXPECT(rsp[0] == 0x01 && rsp[1] == 0x03 && rsp[2] == 4);
    EXPECT(rsp[3] == 0x12 && rsp[4] == 0x34);
    EXPECT(rsp[5] == 0x00 && rsp[6] == 0x0A);
    EXPECT(modbus_crc(rsp, (size_t)rn - 2) == (uint16_t)(rsp[rn - 2] | (rsp[rn - 1] << 8)));

    req[n - 1] ^= 0xFF;
    EXPECT(modbus_handle(&slave, req, n, rsp, sizeof rsp) == 0);

    const uint8_t bad_fn[] = {0x01, 0x11, 0x00, 0x00, 0x00, 0x01};
    n = mb_frame(req, bad_fn, sizeof bad_fn);
    rn = modbus_handle(&slave, req, n, rsp, sizeof rsp);
    EXPECT(rn == 5);
    EXPECT(rsp[1] == 0x91 && rsp[2] == 0x01);

    const uint8_t write_pdu[] = {0x01, 0x06, 0x00, 0x01, 0x00, 0x32};
    n = mb_frame(req, write_pdu, sizeof write_pdu);
    rn = modbus_handle(&slave, req, n, rsp, sizeof rsp);
    EXPECT(rn == 8);
    EXPECT(regs[1] == 0x32);
    EXPECT(memcmp(rsp, req, 6) == 0);

    const uint8_t bad_val[] = {0x01, 0x06, 0x00, 0x01, 0x00, 0xC8};
    n = mb_frame(req, bad_val, sizeof bad_val);
    rn = modbus_handle(&slave, req, n, rsp, sizeof rsp);
    EXPECT(rsp[1] == (uint8_t)(0x06 | 0x80) && rsp[2] == 0x03);

    const uint8_t multi[] = {0x01, 0x10, 0x00, 0x01, 0x00, 0x01, 0x02, 0x00, 0x14};
    n = mb_frame(req, multi, sizeof multi);
    rn = modbus_handle(&slave, req, n, rsp, sizeof rsp);
    EXPECT(rn == 8);
    EXPECT(regs[1] == 0x14);
    EXPECT(rsp[1] == 0x10);

    char json[128];
    EXPECT(sss_task_json(json, sizeof json, "G1", 2, "RUN", "取料") == 0);
    EXPECT(strcmp(json, "{\"groupId\":\"G1\",\"taskNumber\":2,\"groupState\":\"RUN\",\"poolInfo\":\"取料\"}") ==
           0);

    yoomi_cmd_t cmd;
    const char *line = "{\"speed\":80,\"cmd\":\"storage_out\",\"slot\":\"B2\",\"groupId\":\"G9\"}";
    EXPECT(yoomi_cmd_parse(line, &cmd) == 0);
    EXPECT(strcmp(cmd.cmd, "storage_out") == 0);
    EXPECT(strcmp(cmd.slot, "B2") == 0);
    EXPECT(strcmp(cmd.group_id, "G9") == 0);
    EXPECT(cmd.has_speed && cmd.speed == 80);
    EXPECT(yoomi_cmd_parse("{", &cmd) != 0);
    EXPECT(yoomi_slot_index("A3") == 8);
    EXPECT(yoomi_slot_index("B1") == 1);
    EXPECT(yoomi_slot_index("D6") == 23);
    EXPECT(yoomi_slot_index("E1") < 0);
    EXPECT(yoomi_slot_x_um(8) == 40000);
    EXPECT(yoomi_slot_y_um(8) == 190000);

    uint8_t pkt[700];
    size_t pn = mqtt_enc_connect(pkt, sizeof pkt, "yoomi-cell-1", 30);
    EXPECT(pn > 10 && pkt[0] == 0x10);
    EXPECT(contains(pkt, pn, "MQTT"));
    mqtt_packet_t msg;
    size_t used = 0;
    EXPECT(mqtt_parse(pkt, pn, &msg, &used) == 0);
    EXPECT(msg.type == MQTT_PKT_CONNECT && used == pn);

    const char *payload = "{\"groupId\":\"G1\",\"taskNumber\":1,\"groupState\":\"RUN\",\"poolInfo\":\"回零 Z\"}";
    pn = mqtt_enc_publish(pkt, sizeof pkt, "sss/task", payload, strlen(payload));
    EXPECT(pkt[0] == 0x30);
    EXPECT(mqtt_parse(pkt, pn, &msg, &used) == 0);
    EXPECT(msg.type == MQTT_PKT_PUBLISH);
    EXPECT(strcmp(msg.topic, "sss/task") == 0);
    EXPECT(msg.payload_len == strlen(payload));
    EXPECT(memcmp(msg.payload, payload, msg.payload_len) == 0);

    uint8_t big[200];
    memset(big, 0x5A, sizeof big);
    pn = mqtt_enc_publish(pkt, sizeof pkt, "sss/cell/status", big, sizeof big);
    EXPECT(mqtt_parse(pkt, pn, &msg, &used) == 0);
    EXPECT(msg.payload_len == sizeof big);
    EXPECT(memcmp(msg.payload, big, sizeof big) == 0);

    pn = mqtt_enc_subscribe(pkt, sizeof pkt, 7, "sss/cell/cmd");
    EXPECT(pkt[0] == 0x82);
    EXPECT(mqtt_parse(pkt, pn, &msg, &used) == 0);
    EXPECT(msg.type == MQTT_PKT_SUBSCRIBE && msg.packet_id == 7);

    const uint8_t suback[] = {0x90, 0x03, 0x00, 0x07, 0x00};
    EXPECT(mqtt_parse(suback, sizeof suback, &msg, &used) == 0);
    EXPECT(msg.type == MQTT_PKT_SUBACK && msg.packet_id == 7 && msg.return_code == 0);

    EXPECT(mqtt_enc_pingreq(pkt, sizeof pkt) == 2);
    EXPECT(pkt[0] == 0xC0 && pkt[1] == 0x00);
}
