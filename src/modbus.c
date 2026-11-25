#include "yoomi/modbus.h"

uint16_t modbus_crc(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 1) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001);
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

static int append_crc(uint8_t *buf, size_t len, size_t cap)
{
    if (len + 2 > cap) {
        return -1;
    }
    uint16_t crc = modbus_crc(buf, len);
    buf[len] = (uint8_t)(crc & 0xFF);
    buf[len + 1] = (uint8_t)(crc >> 8);
    return (int)(len + 2);
}

static int exception_rsp(uint8_t addr, uint8_t func, uint8_t code, uint8_t *rsp, size_t cap)
{
    if (cap < 5) {
        return -1;
    }
    rsp[0] = addr;
    rsp[1] = (uint8_t)(func | 0x80);
    rsp[2] = code;
    return append_crc(rsp, 3, cap);
}

int modbus_handle(const modbus_slave_t *slave, const uint8_t *req, size_t len,
                  uint8_t *rsp, size_t cap)
{
    if (slave == 0 || req == 0 || rsp == 0 || len < 4) {
        return 0;
    }
    uint16_t got = (uint16_t)(req[len - 2] | ((uint16_t)req[len - 1] << 8));
    if (modbus_crc(req, len - 2) != got) {
        return 0;
    }
    if (req[0] != slave->addr) {
        return 0;
    }
    uint8_t func = req[1];
    if (func == 0x03) {
        if (len != 8) {
            return exception_rsp(slave->addr, func, 0x03, rsp, cap);
        }
        uint16_t start = (uint16_t)((req[2] << 8) | req[3]);
        uint16_t qty = (uint16_t)((req[4] << 8) | req[5]);
        if (qty < 1 || qty > 16) {
            return exception_rsp(slave->addr, func, 0x03, rsp, cap);
        }
        if ((uint32_t)start + qty > 16) {
            return exception_rsp(slave->addr, func, 0x02, rsp, cap);
        }
        size_t need = (size_t)(3 + qty * 2 + 2);
        if (need > cap) {
            return -1;
        }
        rsp[0] = slave->addr;
        rsp[1] = 0x03;
        rsp[2] = (uint8_t)(qty * 2);
        for (uint16_t i = 0; i < qty; i++) {
            uint16_t val = 0;
            if (slave->read_reg(slave->user, (uint16_t)(start + i), &val) != 0) {
                return exception_rsp(slave->addr, func, 0x02, rsp, cap);
            }
            rsp[3 + i * 2] = (uint8_t)(val >> 8);
            rsp[4 + i * 2] = (uint8_t)(val & 0xFF);
        }
        return append_crc(rsp, 3 + (size_t)qty * 2, cap);
    }
    if (func == 0x06) {
        if (len != 8) {
            return exception_rsp(slave->addr, func, 0x03, rsp, cap);
        }
        uint16_t reg = (uint16_t)((req[2] << 8) | req[3]);
        uint16_t val = (uint16_t)((req[4] << 8) | req[5]);
        int wr = slave->write_reg(slave->user, reg, val);
        if (wr == -1) {
            return exception_rsp(slave->addr, func, 0x02, rsp, cap);
        }
        if (wr != 0) {
            return exception_rsp(slave->addr, func, 0x03, rsp, cap);
        }
        if (cap < 8) {
            return -1;
        }
        for (size_t i = 0; i < 6; i++) {
            rsp[i] = req[i];
        }
        return append_crc(rsp, 6, cap);
    }
    if (func == 0x10) {
        if (len < 9) {
            return exception_rsp(slave->addr, func, 0x03, rsp, cap);
        }
        uint16_t start = (uint16_t)((req[2] << 8) | req[3]);
        uint16_t qty = (uint16_t)((req[4] << 8) | req[5]);
        uint8_t bytes = req[6];
        if (qty < 1 || qty > 16 || bytes != qty * 2 || len != (size_t)(9 + bytes)) {
            return exception_rsp(slave->addr, func, 0x03, rsp, cap);
        }
        if ((uint32_t)start + qty > 16) {
            return exception_rsp(slave->addr, func, 0x02, rsp, cap);
        }
        uint16_t vals[16];
        for (uint16_t i = 0; i < qty; i++) {
            vals[i] = (uint16_t)((req[7 + i * 2] << 8) | req[8 + i * 2]);
            int probe = slave->write_reg(slave->user, (uint16_t)(start + i), vals[i]);
            if (probe != 0) {
                return exception_rsp(slave->addr, func, probe == -1 ? 0x02 : 0x03, rsp, cap);
            }
        }
        if (cap < 8) {
            return -1;
        }
        rsp[0] = slave->addr;
        rsp[1] = 0x10;
        rsp[2] = req[2];
        rsp[3] = req[3];
        rsp[4] = req[4];
        rsp[5] = req[5];
        return append_crc(rsp, 6, cap);
    }
    return exception_rsp(slave->addr, func, 0x01, rsp, cap);
}
