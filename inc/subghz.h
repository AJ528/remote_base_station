/*
 * subghz.h
 *
 *  Created on: Mar 23, 2024
 *      Author: adevries
 */

#ifndef __SUBGHZ_H
#define __SUBGHZ_H

#include "stm32wlxx_hal_subghz.h"
#include <stdint.h>
#include <stdbool.h>

#define TX_MODE  0
#define RX_MODE  1

void MX_SUBGHZ_Init(void);
HAL_StatusTypeDef subghz_init(SUBGHZ_HandleTypeDef *hsubghz);
void subghz_write_tx_buffer(uint8_t value);
HAL_StatusTypeDef tx_packet(void);
void subghz_read_rx_buffer(void);
HAL_StatusTypeDef continuous_rx(void);
HAL_StatusTypeDef single_rx_blocking(void);

#endif /* __SUBGHZ_H */
