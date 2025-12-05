#include "modbus_tcp.h"
#include "socket.h"
#include <string.h>
#include <stdio.h>

static uint8_t recv_buf[256];
static uint8_t send_buf[256];
static uint16_t trans_id = 1;

/**
 * @brief 读取 Modbus 寄存器（功能码 0x03）
 * @param sn Socket 编号
 * @param dest_ip 目标 IP 地址
 * @param dest_port 目标端口
 * @param address Modbus 地址（例如 40001 对应寄存器 0）
 * @return 读取的寄存器值，失败返回 -1
 */
int modbus_read_register(uint8_t sn, uint8_t *dest_ip, uint16_t dest_port, uint16_t address)
{
    int32_t ret;
    uint16_t reg_addr = address - 40001; // Modbus 地址转换为寄存器地址

    // 检查连接状态
    if (getSn_SR(sn) != SOCK_ESTABLISHED)
    {
        if (getSn_SR(sn) == SOCK_CLOSED)
        {
            socket(sn, Sn_MR_TCP, 0, 0);
        }
        if (getSn_SR(sn) == SOCK_INIT)
        {
            connect(sn, dest_ip, dest_port);
        }
        return -1;
    }

    // 构造 Modbus TCP 读请求帧
    send_buf[0] = (trans_id >> 8) & 0xFF;
    send_buf[1] = trans_id & 0xFF;
    send_buf[2] = 0x00;
    send_buf[3] = 0x00;
    send_buf[4] = 0x00;
    send_buf[5] = 0x06;
    send_buf[6] = 0x01; // 单元 ID
    send_buf[7] = 0x03; // 功能码：读保持寄存器
    send_buf[8] = (reg_addr >> 8) & 0xFF;
    send_buf[9] = reg_addr & 0xFF;
    send_buf[10] = 0x00;
    send_buf[11] = 0x01; // 读取 1 个寄存器
    trans_id++;

    // 发送请求
    ret = send(sn, send_buf, 12);
    if (ret <= 0)
    {
        printf("Send failed\r\n");
        return -1;
    }

    // 等待响应
    delay_ms(100);

    // 接收响应
    ret = getSn_RX_RSR(sn);
    if (ret > 0)
    {
        recv(sn, recv_buf, ret);

        // 检查功能码
        if (recv_buf[7] == 0x03)
        {
            // 解析寄存器值
            uint16_t value = (recv_buf[9] << 8) | recv_buf[10];
            return value;
        }
        else if (recv_buf[7] & 0x80)
        {
            printf("Modbus Exception: 0x%02X\r\n", recv_buf[8]);
            return -1;
        }
    }

    return -1;
}

/**
 * @brief 写入 Modbus 寄存器（功能码 0x06）
 * @param sn Socket 编号
 * @param dest_ip 目标 IP 地址
 * @param dest_port 目标端口
 * @param address Modbus 地址（例如 40002 对应寄存器 1）
 * @param value 写入的值
 * @return 成功返回 0，失败返回 -1
 */
int modbus_write_register(uint8_t sn, uint8_t *dest_ip, uint16_t dest_port, uint16_t address, uint16_t value)
{
    int32_t ret;
    uint16_t reg_addr = address - 40001;

    // 检查连接状态
    if (getSn_SR(sn) != SOCK_ESTABLISHED)
    {
        return -1;
    }

    // 构造 Modbus TCP 写请求帧
    send_buf[0] = (trans_id >> 8) & 0xFF;
    send_buf[1] = trans_id & 0xFF;
    send_buf[2] = 0x00;
    send_buf[3] = 0x00;
    send_buf[4] = 0x00;
    send_buf[5] = 0x06;
    send_buf[6] = 0x01; // 单元 ID
    send_buf[7] = 0x06; // 功能码：写单个寄存器
    send_buf[8] = (reg_addr >> 8) & 0xFF;
    send_buf[9] = reg_addr & 0xFF;
    send_buf[10] = (value >> 8) & 0xFF;
    send_buf[11] = value & 0xFF;
    trans_id++;

    // 发送请求
    ret = send(sn, send_buf, 12);
    if (ret <= 0)
    {
        printf("Send failed\r\n");
        return -1;
    }

    // 等待响应
    delay_ms(100);

    // 接收响应
    ret = getSn_RX_RSR(sn);
    if (ret > 0)
    {
        recv(sn, recv_buf, ret);

        // 检查功能码
        if (recv_buf[7] == 0x06)
        {
            printf("Write Success\r\n");
            return 0;
        }
        else if (recv_buf[7] & 0x80)
        {
            printf("Modbus Exception: 0x%02X\r\n", recv_buf[8]);
            return -1;
        }
    }

    return -1;
		
}
