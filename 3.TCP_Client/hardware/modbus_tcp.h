#ifndef MODBUS_TCP_H
#define MODBUS_TCP_H

#include <stdint.h>

// 读取寄存器值（功能码 0x03）
int modbus_read_register(uint8_t sn, uint8_t *dest_ip, uint16_t dest_port, uint16_t address);

// 写入寄存器值（功能码 0x06）
int modbus_write_register(uint8_t sn, uint8_t *dest_ip, uint16_t dest_port, uint16_t address, uint16_t value);

#endif // MODBUS_TCP_H
