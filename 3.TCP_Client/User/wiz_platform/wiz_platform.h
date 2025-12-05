#ifndef __WIZ_PLATFORM_H__
#define __WIZ_PLATFORM_H__

#include <stdint.h>
#include "stm32f10x.h"

// WIZnet SPI Pins (Modified for STM32F103C8T6)
// SCS (Chip Select) -> PA4
#define WIZ_SCS_PIN GPIO_Pin_4
#define WIZ_SCS_PORT GPIOA

// RST (Reset) -> PA3
#define WIZ_RST_PIN GPIO_Pin_3
#define WIZ_RST_PORT GPIOA

// INT (Interrupt) -> PA2
#define WIZ_INT_PIN GPIO_Pin_2
#define WIZ_INT_PORT GPIOA

// SPI2 Pins (Unchanged)
// SCK -> PB13
#define WIZ_SCK_PIN GPIO_Pin_13
#define WIZ_SCK_PORT GPIOB

// MISO -> PB14
#define WIZ_MISO_PIN GPIO_Pin_14
#define WIZ_MISO_PORT GPIOB

// MOSI -> PB15
#define WIZ_MOSI_PIN GPIO_Pin_15
#define WIZ_MOSI_PORT GPIOB

/**
 * @brief   delay init
 * @param   none
 * @return  none
 */
void delay_init(void);

/**
 * @brief   delay us
 * @param   none
 * @return  none
 */
void delay_us(uint32_t nus);

/**
 * @brief   delay ms
 * @param   none
 * @return  none
 */
void delay_ms(uint32_t nms);

/**
 * @brief   delay s
 * @param   none
 * @return  none
 */
void delay_s(uint32_t ns);

/**
 * @brief   debug usart init
 * @param   none
 * @return  none
 */
void debug_usart_init(void);

/**
 * @brief   wiz timer init
 * @param   none
 * @return  none
 */
void wiz_timer_init(void);

/**
 * @brief   wiz spi init
 * @param   none
 * @return  none
 */
void wiz_spi_init(void);

/**
 * @brief   wiz rst and int pin init
 * @param   none
 * @return  none
 */
void wiz_rst_int_init(void);

/**
 * @brief   hardware reset wizchip
 * @param   none
 * @return  none
 */
void wizchip_reset(void);

/**
 * @brief   SPI select wizchip
 * @param   none
 * @return  none
 */
void wizchip_select(void);

/**
 * @brief   SPI deselect wizchip
 * @param   none
 * @return  none
 */
void wizchip_deselect(void);

/**
 * @brief   SPI write 1 byte to wizchip
 * @param   dat:1 byte data
 * @return  none
 */
void wizchip_write_byte(uint8_t dat);

/**
 * @brief   SPI read 1 byte from wizchip
 * @param   none
 * @return  1 byte data
 */
uint8_t wizchip_read_byte(void);

/**
 * @brief   SPI write buff to wizchip
 * @param   buf:write buff
 * @param   len:write len
 * @return  none
 */
void wizchip_write_buff(uint8_t *buf, uint16_t len);

/**
 * @brief   SPI read buff from wizchip
 * @param   buf:read buff
 * @param   len:read len
 * @return  none
 */
void wizchip_read_buff(uint8_t *buf, uint16_t len);

/**
 * @brief   Register the WIZCHIP SPI callback function
 * @param   none
 * @return  none
 */
void wizchip_spi_cb_reg(void);

#endif
