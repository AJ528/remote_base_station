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

typedef enum
{
  RADIO_SWITCH_OFF    = 0,
  RADIO_SWITCH_RX     = 1,
  RADIO_SWITCH_RFO_LP = 2,
  RADIO_SWITCH_RFO_HP = 3,
}BSP_RADIO_Switch_TypeDef;

void MX_SUBGHZ_Init(void);
HAL_StatusTypeDef subghz_init(SUBGHZ_HandleTypeDef *hsubghz);
void subghz_radio_getRxBufferStatus(void);
void subghz_radio_getPacketStatus(void);
void subghz_radio_getstatus(void);
void subghz_read_rx_buffer(void);
HAL_StatusTypeDef single_rx_block(SUBGHZ_HandleTypeDef *hsubghz);
HAL_StatusTypeDef continuous_rx(void);
HAL_StatusTypeDef single_rx_blocking(void);
int32_t ConfigRFSwitch(BSP_RADIO_Switch_TypeDef Config);

#endif /* __SUBGHZ_H */
