/*wizchip->STM32 Hardware Pin define*/
//	wizchip_SCS    --->     STM32_GPIOA4
//	wizchip_SCLK	 --->     STM32_GPIOB13
//  wizchip_MISO	 --->     STM32_GPIOB14
//	wizchip_MOSI	 --->     STM32_GPIOB15
//	wizchip_RESET	 --->     STM32_GPIOA3
//	wizchip_INT    --->     STM32_GPIOA2

/* The current routine uses a 12Mhz external crystal. If you use other crystals, you need to modify the system clock.
******************************************************************************
* stm32f10x.h			Modify 118 lines of external crystal frequency
* system_stm32f10x.c	Modify 1055 lines of frequency doubling factor
#endif
******************************************************************************
*/

#include "stm32f10x.h"
#include <stdio.h>
#include "wiz_platform.h"
#include "wizchip_conf.h"
#include "wiz_interface.h"
#include "modbus_tcp.h"
#include "socket.h"

#define SOCKET_ID 0

wiz_NetInfo default_net_info = {
    .mac = {0x00, 0x08, 0xdc, 0x12, 0x22, 0x12},
    .ip = {192, 168, 1, 30},
    .gw = {192, 168, 1, 1},
    .sn = {255, 255, 255, 0},
    .dns = {8, 8, 8, 8},
    .dhcp = NETINFO_STATIC};

uint8_t ethernet_buf[1024 * 2] = {0};
uint8_t dest_ip[4] = {192, 168, 1, 1};
uint16_t dest_port = 502;

int main(void)
{
    int read_value;
    uint16_t write_value = 100;  // 去掉 static
    
    delay_init();
    debug_usart_init();
    wiz_timer_init();
    wiz_spi_init();
    wiz_rst_int_init();
    printf("Modbus TCP Client\r\n");

    wizchip_initialize();
    network_init(ethernet_buf, &default_net_info);
    setSn_KPALVTR(SOCKET_ID, 6);

    while (1)
    {
        uint8_t sock_status = getSn_SR(SOCKET_ID);
        
        // Socket 状态机
        if (sock_status == SOCK_CLOSED)
        {
            printf("Socket Closed, Opening...\r\n");
            socket(SOCKET_ID, Sn_MR_TCP, 0, 0);
        }
        else if (sock_status == SOCK_INIT)
        {
            printf("Connecting to %d.%d.%d.%d:%d\r\n", 
                   dest_ip[0], dest_ip[1], dest_ip[2], dest_ip[3], dest_port);
            connect(SOCKET_ID, dest_ip, dest_port);
        }
        else if (sock_status == SOCK_ESTABLISHED)
        {
            // 检查是否刚连接上
            if (getSn_IR(SOCKET_ID) & Sn_IR_CON)
            {
                setSn_IR(SOCKET_ID, Sn_IR_CON);
                printf("Connected!\r\n");
            }
            
            // 读取寄存器地址 40001 的值
            read_value = modbus_read_register(SOCKET_ID, dest_ip, dest_port, 40001);
            if (read_value >= 0)
            {
                printf("Read 40001: %d\r\n", read_value);
            }

            delay_ms(500);

            // 写入寄存器地址 40002 的值
            modbus_write_register(SOCKET_ID, dest_ip, dest_port, 40002, write_value);
            printf("Write 40002: %d\r\n", write_value);
            write_value++;

            delay_ms(1000);
        }
        else if (sock_status == SOCK_CLOSE_WAIT)
        {
            printf("Close Wait, Disconnecting...\r\n");
            disconnect(SOCKET_ID);
        }
        
        delay_ms(1);  // 避免过快循环
    }
}
