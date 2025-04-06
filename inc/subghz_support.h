#ifndef __SUBGHZ_SUPPORT_H
#define __SUBGHZ_SUPPORT_H

#include "stm32wlxx_ll_gpio.h"
#include "stm32wlxx_hal_subghz.h"

#include <stdint.h>
#include <stdbool.h>

#define OWN_ADDRESS						0x5A

#define RF_FREQ						915000000
#define BIT_RATE					50000
#define FREQ_DEVIATION		25000
#define XTAL_FREQ					32000000

void subghz_init_settings_default(SUBGHZ_HandleTypeDef *hsubghz);
HAL_StatusTypeDef subghz_setPayloadLength(SUBGHZ_HandleTypeDef *hsubghz, uint8_t length);
HAL_StatusTypeDef subghz_setAddress(SUBGHZ_HandleTypeDef *hsubghz, uint8_t address);
uint8_t subghz_radio_getstatus(void);
HAL_StatusTypeDef subghz_setFrequency(SUBGHZ_HandleTypeDef *hsubghz, uint32_t frequency);
HAL_StatusTypeDef subghz_setIRQ(SUBGHZ_HandleTypeDef *hsubghz, uint16_t radio_irq_source);
void subghz_radio_getPacketStatus(uint8_t *buffer, bool print);
void set_RF_switch_RX(void);
void set_RF_switch_TX(void);


#endif /* __SUBGHZ_SUPPORT_H */
