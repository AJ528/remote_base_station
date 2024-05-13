/*
 * subghz.h
 *
 *  Created on: Mar 23, 2024
 *      Author: adevries
 */

#ifndef INC_SUBGHZ_H_
#define INC_SUBGHZ_H_

#include "stm32wlxx_hal.h"
#include <stdint.h>

typedef enum
{
  RADIO_SWITCH_OFF    = 0,
  RADIO_SWITCH_RX     = 1,
  RADIO_SWITCH_RFO_LP = 2,
  RADIO_SWITCH_RFO_HP = 3,
}BSP_RADIO_Switch_TypeDef;

HAL_StatusTypeDef subghz_init(SUBGHZ_HandleTypeDef *hsubghz);
HAL_StatusTypeDef single_rx_block(SUBGHZ_HandleTypeDef *hsubghz);
HAL_StatusTypeDef continuous_rx(SUBGHZ_HandleTypeDef *hsubghz);
int32_t ConfigRFSwitch(BSP_RADIO_Switch_TypeDef Config);

#endif /* INC_SUBGHZ_H_ */
