#ifndef YOOMI_MODBUS_H
#define YOOMI_MODBUS_H

#include <stddef.h>
#include <stdint.h>

uint16_t modbus_crc(const uint8_t *data, size_t len);

typedef struct {
    uint8_t addr;
    int (*read_reg)(void *user, uint16_t reg, uint16_t *val);
    int (*write_reg)(void *user, uint16_t reg, uint16_t val);
    void *user;
} modbus_slave_t;

/* Returns response length, or 0 if the frame is ignored (bad CRC, other addr, too short). */
int modbus_handle(const modbus_slave_t *slave, const uint8_t *req, size_t len,
                  uint8_t *rsp, size_t cap);

#endif
