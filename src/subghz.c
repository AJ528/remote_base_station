/*
 * subghz.c
 *
 *  Created on: Mar 23, 2024
 *      Author: adevries
 */

#include "subghz.h"
#include "subghz_support.h"

#include "stm32wlxx_hal_subghz.h"
#include "stm32wlxx_ll_bus.h"
#include "stm32wlxx_ll_rcc.h"

#include "mprintf.h"

#include <stdint.h>


#define RADIO_MODE_STANDBY_RC       0x02
#define RADIO_MODE_STANDBY_HSE32	0x03
#define	RADIO_MODE_FS				0x04
#define	RADIO_MODE_RX				0x05
#define RADIO_MODE_TX               0x06

#define RADIO_MODE_BITFIELD         0x70
#define RADIO_STATUS_BITFIELD       0x0E

SUBGHZ_HandleTypeDef subghz_handle;


static void subghz_init_irq(SUBGHZ_HandleTypeDef *hsubghz);


void subghz_init(void)
{
	// enable clocks
	LL_APB3_GRP1_EnableClock(LL_APB3_GRP1_PERIPH_SUBGHZSPI);
	LL_RCC_HSE_EnableTcxo();
	LL_RCC_HSE_Enable();
	// wait for clock to be ready
	while (LL_RCC_HSE_IsReady() == 0)
	{// do nothing
	}

	// subghz SPI max speed is 16 MHz, but there's no need to go that fast
	subghz_handle.Init.BaudratePrescaler = SUBGHZSPI_BAUDRATEPRESCALER_8;

	// init the subghz HAL module
	if(HAL_SUBGHZ_Init(&subghz_handle) != HAL_OK){
		printf_("error\r\n");
	}
	// now initialize communication parameters (frequency, bandwidth, packet length, etc.)
	if(subghz_init_settings(&subghz_handle) != HAL_OK){
		printf_("error\r\n");
	}

#if (RX_MODE == 1)
	subghz_init_irq(&subghz_handle);
	ConfigRFSwitch(RADIO_SWITCH_RX);
#endif
#if (TX_MODE == 1)
	ConfigRFSwitch(RADIO_SWITCH_RFO_LP);
#endif
}

HAL_StatusTypeDef subghz_init_settings(SUBGHZ_HandleTypeDef *hsubghz)
{
	HAL_StatusTypeDef result;

	// initialize the radio settings to reasonable defaults (I don't expect these settings to change)
	subghz_init_settings_default(hsubghz);

	// set our own address (when receiving) or the address you want to send data to (when transmitting)
	// address comparison/filtering is enabled via the Set_PacketParams() command.
	result = subghz_setAddress(hsubghz, OWN_ADDRESS);
	if(result != HAL_OK){
		return result;
	}

	// get the status of the radio
	uint8_t RadioResult = subghz_radio_getstatus();
	
	//extract the radio mode from the result
	uint8_t RadioMode = ((RadioResult & RADIO_MODE_BITFIELD) >> 4);
	// confirm radio is in standby mode using the internal RC oscillator
	if(RadioMode != RADIO_MODE_STANDBY_RC){
		return HAL_ERROR;
	}
	return HAL_OK;
}

void subghz_read_rx_buffer(void)
{
	uint8_t buf[16];

	HAL_SUBGHZ_ExecGetCmd(&subghz_handle, RADIO_GET_RXBUFFERSTATUS, buf, 4);

	uint32_t payload_len = buf[1] + 1;
  	printf_("Buf Status: %#04x, %#04x, %#04x\r\n", buf[0], buf[1], buf[2]);
	
	// read bytes from rx buffer
	HAL_SUBGHZ_ReadBuffer(&subghz_handle, buf[2], buf, (uint16_t)payload_len);

	uint32_t i;
	printf_("buf = ");
	for(i = 0; i < payload_len; i++){
		printf_("%#04x, ", buf[i]);
	}
	printf_("\r\n");
}

void subghz_write_tx_buffer(uint8_t value)
{
	uint8_t tx_addr;
	uint8_t buf[4];
	uint8_t buf2[3];
	uint32_t i;

	for(i = 0; i < sizeof(buf); i++){
		buf[i] = value++;
	}
	// get the start address of the tx buffer (I think)
	HAL_SUBGHZ_ReadRegister(&subghz_handle, 0x0802, &tx_addr);

	printf_("tx_addr = %#0x\r\n", tx_addr);
	
	// write bytes to the start of the tx buffer
	HAL_SUBGHZ_WriteBuffer(&subghz_handle, 0x80, buf, sizeof(buf));

	printf_("value = %#04x\r\n", value);

	HAL_SUBGHZ_ReadBuffer(&subghz_handle, 0x81, buf2, sizeof(buf2));

	printf_("buf2 = %#04x\r\n", buf2[1]);
}

HAL_StatusTypeDef tx_packet(void)
{
	uint8_t RadioCmd[3] = {0xff, 0xff, 0x00};	// disable timeout
	return(HAL_SUBGHZ_ExecSetCmd(&subghz_handle, RADIO_SET_TX, RadioCmd, 3));
}

HAL_StatusTypeDef continuous_rx(void)
{
	uint8_t RadioCmd[3] = {0xFF, 0xFF, 0xFF};
	return(HAL_SUBGHZ_ExecSetCmd(&subghz_handle, RADIO_SET_RX, RadioCmd, 3));
}

HAL_StatusTypeDef single_rx_blocking(void)
{
	uint8_t RadioCmd[3] = {0};
	return(HAL_SUBGHZ_ExecSetCmd(&subghz_handle, RADIO_SET_RX, RadioCmd, 3));
}

static void subghz_init_irq(SUBGHZ_HandleTypeDef *hsubghz)
{
	subghz_setIRQ(hsubghz, SUBGHZ_IRQ_RXDONE | SUBGHZ_IRQ_ERROR);
  /* SUBGHZ_Radio_IRQn interrupt configuration */
  NVIC_SetPriority(SUBGHZ_Radio_IRQn, 3);
  NVIC_EnableIRQ(SUBGHZ_Radio_IRQn);
}

void SUBGHZ_Radio_IRQHandler(void)
{
	//pass the interrupt to the HAL IRQ handler
  HAL_SUBGHZ_IRQHandler(&subghz_handle);
}