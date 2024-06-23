/*
 * subghz.c
 *
 *  Created on: Mar 23, 2024
 *      Author: adevries
 */

#include "subghz.h"

#include "stm32wlxx_hal_subghz.h"
#include "stm32wlxx_ll_gpio.h"
#include "stm32wlxx_ll_bus.h"
#include "stm32wlxx_ll_rcc.h"
#include "stm32wlxx_ll_utils.h"

#include "mprintf.h"

#include <stdint.h>
#include <stdbool.h>

#define RF_SW_CTRL3_Pin LL_GPIO_PIN_3
#define RF_SW_CTRL3_GPIO_Port GPIOC
#define RF_SW_CTRL2_Pin LL_GPIO_PIN_5
#define RF_SW_CTRL2_GPIO_Port GPIOC
#define RF_SW_CTRL1_Pin LL_GPIO_PIN_4
#define RF_SW_CTRL1_GPIO_Port GPIOC

#define ADDRESS						0x5A

#define RF_FREQ						915000000
#define BIT_RATE					50000
#define FREQ_DEVIATION				25000
#define XTAL_FREQ					32000000


#define SYNCWORD_BASEADDRESS		0x06C0
#define NODE_ADDRESS_REG			0x06CD
#define BROADCAST_ADDRESS_REG		0x06CE		
#define CRC_INIT_MSB_REG			0x06BC
#define CRC_INIT_LSB_REG			0x06BD
#define CRC_POLY_MSB_REG			0x06BE
#define CRC_POLY_LSB_REG			0x06BF



#define RADIO_MODE_STANDBY_RC       0x02
#define RADIO_MODE_STANDBY_HSE32	0x03
#define	RADIO_MODE_FS				0x04
#define	RADIO_MODE_RX				0x05
#define RADIO_MODE_TX               0x06

#define	RADIO_COMMAND_RX_DONE		0x02
#define	RADIO_COMMAND_TIMEOUT		0x03
#define RADIO_COMMAND_TX_DONE       0x06

#define RADIO_MODE_BITFIELD         0x70
#define RADIO_STATUS_BITFIELD       0x0E


#define SX_FREQ_TO_CHANNEL( channel, freq )                                  \
do                                                                           \
{                                                                            \
  channel = (uint32_t) ((((uint64_t) freq)<<25)/(XTAL_FREQ) );               \
}while( 0 )

struct __attribute__((__packed__)) sRadioParams {
	uint16_t PbLength;
	uint8_t PbDetLength;
	uint8_t SyncWordLength;
	uint8_t AddrComp;
	uint8_t PktType;
	uint8_t PayloadLength;
	uint8_t CrcType;
	uint8_t Whitening;
};



static SUBGHZ_HandleTypeDef hsubghz;



static HAL_StatusTypeDef subghz_default_init(SUBGHZ_HandleTypeDef *hsubghz);
HAL_StatusTypeDef SetRfFrequency(SUBGHZ_HandleTypeDef *hsubghz, uint32_t frequency);
HAL_StatusTypeDef SetAddress(SUBGHZ_HandleTypeDef *hsubghz, uint8_t address);
HAL_StatusTypeDef DefaultModulationParams(SUBGHZ_HandleTypeDef *hsubghz);
HAL_StatusTypeDef DefaultCRC(SUBGHZ_HandleTypeDef *hsubghz);
static HAL_StatusTypeDef SetPayloadLength(SUBGHZ_HandleTypeDef *hsubghz, uint8_t length);
static HAL_StatusTypeDef DefaultTxConfig(SUBGHZ_HandleTypeDef *hsubghz);
static HAL_StatusTypeDef SUBGHZ_Radio_Set_IRQ(SUBGHZ_HandleTypeDef *hsubghz, uint16_t radio_irq_source);

static void subghz_irq_init(void);


void MX_SUBGHZ_Init(void)
{
	LL_APB3_GRP1_EnableClock(LL_APB3_GRP1_PERIPH_SUBGHZSPI);
    LL_RCC_HSE_EnableTcxo();
    LL_RCC_HSE_Enable();

    while (LL_RCC_HSE_IsReady() == 0)
    {}

  	hsubghz.Init.BaudratePrescaler = SUBGHZSPI_BAUDRATEPRESCALER_8;

	if (HAL_SUBGHZ_Init(&hsubghz) != HAL_OK)
	{
		printf_("error\r\n");
	}
	if(subghz_init(&hsubghz) != HAL_OK)
	{
		printf_("error\r\n");
	}

#if (RX_MODE == 1)
	subghz_irq_init();
#endif

}


HAL_StatusTypeDef subghz_init(SUBGHZ_HandleTypeDef *hsubghz)
{
	uint8_t RadioResult = 0x00;
	uint8_t RadioMode   = 0x00;

	HAL_StatusTypeDef result;

	subghz_default_init(hsubghz);

	result = SetRfFrequency(hsubghz, RF_FREQ);
	if(result != HAL_OK){
		return result;
	}

#if (RX_MODE == 1)
	result = SUBGHZ_Radio_Set_IRQ(hsubghz, SUBGHZ_IRQ_RXDONE | SUBGHZ_IRQ_ERROR);
	if(result != HAL_OK){
		return result;
	}
#endif
	
	/* Retrieve Status from SUBGHZ Radio */
	result = HAL_SUBGHZ_ExecGetCmd(hsubghz, RADIO_GET_STATUS, &RadioResult, 1);
	if(result != HAL_OK){
		return result;
	}

	/* Format Mode and Status receive from SUBGHZ Radio */
	RadioMode = ((RadioResult & RADIO_MODE_BITFIELD) >> 4);

	/* Check if SUBGHZ Radio is in RADIO_MODE_STANDBY_RC mode */
	if(RadioMode != RADIO_MODE_STANDBY_RC)
	{
		return HAL_ERROR;
	}

	return HAL_OK;
}

static HAL_StatusTypeDef subghz_default_init(SUBGHZ_HandleTypeDef *hsubghz)
{
	HAL_StatusTypeDef result;

	const uint8_t standby_clock = 0x00;		//sets the standby clock to be RC 13 MHz
	result = HAL_SUBGHZ_ExecSetCmd(hsubghz, RADIO_SET_STANDBY, &standby_clock, sizeof(standby_clock));
	if(result != HAL_OK){
		return result;
	}

	const uint8_t buf_base_addr[2] = {0x80, 0x00};
	result = HAL_SUBGHZ_ExecSetCmd(hsubghz, RADIO_SET_BUFFERBASEADDRESS, buf_base_addr, sizeof(buf_base_addr));
	if(result != HAL_OK){
		return result;
	}

	const uint8_t packet_type  = 0x00;
	result = HAL_SUBGHZ_ExecSetCmd(hsubghz, RADIO_SET_PACKETTYPE, &packet_type, sizeof(packet_type));
	if(result != HAL_OK){
		return result;
	}

	const uint8_t syncword[] = {0x48, 0xDF, 0x70, 0x72, 0x00, 0x00, 0x00, 0x00};
	result = HAL_SUBGHZ_WriteRegisters(hsubghz, SYNCWORD_BASEADDRESS, syncword, sizeof(syncword));
	if(result != HAL_OK){
		return result;
	}

	result = SetAddress(hsubghz, ADDRESS);
	if(result != HAL_OK){
		return result;
	}

	result = DefaultCRC(hsubghz);
	if(result != HAL_OK){
		return result;
	}

#if (RX_MODE == 1)
	uint8_t payload_len = 18;
#endif
#if (TX_MODE == 1)
	uint8_t payload_len = 3;
#endif


	// with variable length payloads in RX mode, the length set here is the
	// max payload length accepted before an error is asserted
	// in TX mode, this sets the length of the payload
	result = SetPayloadLength(hsubghz, payload_len);
	if(result != HAL_OK){
		return result;
	}

#if (TX_MODE == 1)
	result = DefaultTxConfig(hsubghz);
	if(result != HAL_OK){
		return result;
	}
#endif

	result = DefaultModulationParams(hsubghz);
	return result;
}

void subghz_read_rx_buffer(void)
{
	uint8_t buf[16];

	HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_RXBUFFERSTATUS, buf, 4);

	uint32_t payload_len = buf[1] + 1;
  	printf_("Buf Status: %#04x, %#04x, %#04x\r\n", buf[0], buf[1], buf[2]);

	if(payload_len != 4){
		printf_("ERROR!\r\n");
	}
	
	// read bytes from rx buffer
	HAL_SUBGHZ_ReadBuffer(&hsubghz, buf[2], buf, (uint16_t)payload_len);

	uint32_t i;
	printf_("buf = ");
	for(i = 0; i < payload_len; i++){
		printf_("%#04x, ", buf[i]);
	}
	printf_("\r\n");
}

void subghz_radio_getRxBufferStatus(void)
{
	uint8_t buf[3] = {0};

	HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_RXBUFFERSTATUS, buf, sizeof(buf));
  	printf_("Buf Status: %#04x, %#04x, %#04x\r\n", buf[0], buf[1], buf[2]);
}

void subghz_radio_getPacketStatus(uint8_t *buffer, bool print)
{
	uint8_t temp_buf[4] = {0};
	uint32_t i;
	HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_PACKETSTATUS, temp_buf, sizeof(temp_buf));

	for(i = 0; i < sizeof(temp_buf); i++){
		buffer[i] = temp_buf[i];
	}
	if(print){
  		printf_("Packet Status: %#04x, %#04x, %#04x, %#04x\r\n", temp_buf[0], temp_buf[1], temp_buf[2], temp_buf[3]);
	}
}

void subghz_radio_getstatus(void)
{
	uint8_t RadioResult = 0x00;
	HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_STATUS, &RadioResult, 1);
  	printf_("RR: 0x%02x\r\n", RadioResult);
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
	HAL_SUBGHZ_ReadRegister(&hsubghz, 0x0802, &tx_addr);

	printf_("tx_addr = %#0x\r\n", tx_addr);
	
	// write bytes to the start of the tx buffer
	HAL_SUBGHZ_WriteBuffer(&hsubghz, 0x80, buf, sizeof(buf));

	printf_("value = %#04x\r\n", value);

	HAL_SUBGHZ_ReadBuffer(&hsubghz, 0x81, buf2, sizeof(buf2));

	printf_("buf2 = %#04x\r\n", buf2[1]);

}

HAL_StatusTypeDef tx_packet(void)
{
	uint8_t RadioCmd[3] = {0xff, 0xff, 0x00};	// disable timeout
	return(HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_TX, RadioCmd, 3));
}

static HAL_StatusTypeDef DefaultTxConfig(SUBGHZ_HandleTypeDef *hsubghz)
{
	HAL_StatusTypeDef result;

	uint8_t buf[4] = {0};

	// configures output power for +10 dBm
	buf[0] = 0x01;	// set PA duty cycle to 1
	buf[1] = 0x00; 	// set HP PA output power to 0
	buf[2] = 0x01;	// select the LP PA
	buf[3] = 0x01;	// must be 0x01

	result = HAL_SUBGHZ_ExecSetCmd(hsubghz, RADIO_SET_PACONFIG, buf, 4);

	// reuse buffer for next command
	buf[0] = 0x0D;	// set output to +13 dBm (somehow turns into +10 dBm when you actually reach the output)
	buf[1] = 0x04;	// set the PA ramp up time to 200 us for no reason other than it's in the middle
	
	result = HAL_SUBGHZ_ExecSetCmd(hsubghz, RADIO_SET_TXPARAMS, buf, 2);

	return result;
}

HAL_StatusTypeDef single_rx_block(SUBGHZ_HandleTypeDef *hsubghz)
{
	uint8_t RadioCmd[3] = {0xFF, 0xFF, 0xFE};
	return(HAL_SUBGHZ_ExecSetCmd(hsubghz, RADIO_SET_RX, RadioCmd, 3));

	uint8_t RadioMode = 0;
	uint8_t RadioResult = 0;

	do
	{
	  /* Retrieve Status from SUBGHZ Radio */
	  uint8_t result = HAL_SUBGHZ_ExecGetCmd(hsubghz, RADIO_GET_STATUS, &RadioResult, 1);
	  if (result != HAL_OK)
	  {
		printf_("error: 0x%02x\r\n", result);
	  }
	  printf_("RR: 0x%02x\r\n", RadioResult);

	  printf_("delay\r\n");
	  LL_mDelay(500);

	  /* Format Mode and Status receive from SUBGHZ Radio */
	  RadioMode = ((RadioResult & RADIO_MODE_BITFIELD) >> 4);
	}
	while (RadioMode != RADIO_MODE_STANDBY_RC);
}

HAL_StatusTypeDef continuous_rx(void)
{
	uint8_t RadioCmd[3] = {0xFF, 0xFF, 0xFF};
	return(HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_RX, RadioCmd, 3));
}

HAL_StatusTypeDef single_rx_blocking(void)
{
	uint8_t RadioCmd[3] = {0};
	return(HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_RX, RadioCmd, 3));
}

/**
  * @brief  Configure Radio Switch.
  * @param  Config: Specifies the Radio RF switch path to be set.
  *         This parameter can be one of following parameters:
  *           @arg RADIO_SWITCH_OFF
  *           @arg RADIO_SWITCH_RX
  *           @arg RADIO_SWITCH_RFO_LP
  *           @arg RADIO_SWITCH_RFO_HP
  * @retval BSP status
  */
int32_t ConfigRFSwitch(BSP_RADIO_Switch_TypeDef Config)
{
  switch (Config)
  {
    case RADIO_SWITCH_OFF:
    {
      /* Turn off switch */
	  LL_GPIO_ResetOutputPin(GPIOC, RF_SW_CTRL1_Pin | RF_SW_CTRL2_Pin | RF_SW_CTRL3_Pin);
      break;
    }
    case RADIO_SWITCH_RX:
    {
      /*Turns On in Rx Mode the RF Switch */
	  LL_GPIO_SetOutputPin(GPIOC, RF_SW_CTRL1_Pin | RF_SW_CTRL3_Pin);
	  LL_GPIO_ResetOutputPin(GPIOC, RF_SW_CTRL2_Pin);
      break;
    }
    case RADIO_SWITCH_RFO_LP:
    {
      /*Turns On in Tx Low Power the RF Switch */
	  LL_GPIO_SetOutputPin(GPIOC, RF_SW_CTRL1_Pin | RF_SW_CTRL2_Pin | RF_SW_CTRL3_Pin);
      break;
    }
    case RADIO_SWITCH_RFO_HP:
    {
      /*Turns On in Tx High Power the RF Switch */
	  LL_GPIO_SetOutputPin(GPIOC, RF_SW_CTRL3_Pin);
	  LL_GPIO_ResetOutputPin(GPIOC, RF_SW_CTRL1_Pin);
	  LL_GPIO_SetOutputPin(GPIOC, RF_SW_CTRL2_Pin);
      break;
    }
    default:
      break;
  }

  return 0;
}


HAL_StatusTypeDef SetRfFrequency(SUBGHZ_HandleTypeDef *hsubghz, uint32_t frequency)
{
    uint8_t buf[4];
    uint32_t chan = 0;
	HAL_StatusTypeDef result;
	const uint32_t freq_lower_limit = 902000000;
	const uint32_t freq_upper_limit = 928000000;

	if(frequency < freq_lower_limit){
		frequency = freq_lower_limit;
	}
	if(frequency > freq_upper_limit){
		frequency = freq_upper_limit;
	}


    SX_FREQ_TO_CHANNEL(chan, frequency);
    buf[0] = ( uint8_t )( ( chan >> 24 ) & 0xFF );
    buf[1] = ( uint8_t )( ( chan >> 16 ) & 0xFF );
    buf[2] = ( uint8_t )( ( chan >> 8 ) & 0xFF );
    buf[3] = ( uint8_t )( chan & 0xFF );

    result = HAL_SUBGHZ_ExecSetCmd(hsubghz, RADIO_SET_RFFREQUENCY, buf, 4);
	if(result != HAL_OK){
		return result;
	}

	// calibrate after setting frequency
	// calibrate for center frequency +/- 4 MHz

	const uint32_t delta_freq = 4000000;

	uint32_t upper_freq = frequency + delta_freq;
	uint32_t lower_freq = frequency - delta_freq;

	uint8_t cal_buf[2];

	cal_buf[0] = (uint8_t)(lower_freq / 4000000);
	cal_buf[1] = (uint8_t)(upper_freq / 4000000);

	result = HAL_SUBGHZ_ExecSetCmd(hsubghz, RADIO_CALIBRATEIMAGE, cal_buf, 2);

	return result;
}

HAL_StatusTypeDef SetAddress(SUBGHZ_HandleTypeDef *hsubghz, uint8_t address)
{
	return HAL_SUBGHZ_WriteRegisters(hsubghz, NODE_ADDRESS_REG, &address, sizeof(address));
}

HAL_StatusTypeDef DefaultCRC(SUBGHZ_HandleTypeDef *hsubghz)
{
	HAL_StatusTypeDef result;
	//implements CRC16-CCITT
	static const uint8_t CRC_init[2] = {0x1D, 0x0F};
	static const uint8_t CRC_poly[2] = {0x10, 0x21};

	result = HAL_SUBGHZ_WriteRegisters(hsubghz, CRC_INIT_MSB_REG, CRC_init, sizeof(CRC_init));
	if(result != HAL_OK){
		return result;
	}
	result = HAL_SUBGHZ_WriteRegisters(hsubghz, CRC_POLY_MSB_REG, CRC_poly, sizeof(CRC_poly));
	return result;


}

HAL_StatusTypeDef DefaultModulationParams(SUBGHZ_HandleTypeDef *hsubghz)
{
    uint8_t buf[8] = {0};

	uint32_t tempVal = ( uint32_t )(( 32 * XTAL_FREQ ) / BIT_RATE );
	buf[0] = ( tempVal >> 16 ) & 0xFF;		// BR
	buf[1] = ( tempVal >> 8 ) & 0xFF;		// BR
	buf[2] = tempVal & 0xFF;				// BR
	buf[3] = 0;					// modulation
	buf[4] = 0x13;				// RX bandwidth
	//calculate Fdev
	SX_FREQ_TO_CHANNEL(tempVal, FREQ_DEVIATION);
	buf[5] = ( tempVal >> 16 ) & 0xFF;		// FDev
	buf[6] = ( tempVal >> 8 ) & 0xFF;		// FDev
	buf[7] = ( tempVal& 0xFF );				// FDev

	return(HAL_SUBGHZ_ExecSetCmd(hsubghz, RADIO_SET_MODULATIONPARAMS, buf, 8));
}

static HAL_StatusTypeDef SetPayloadLength(SUBGHZ_HandleTypeDef *hsubghz, uint8_t length)
{
	HAL_StatusTypeDef result;

	struct sRadioParams params = {
		.PbLength = 32,
		.PbDetLength = 0x07,
		.SyncWordLength = 32,
		.AddrComp = 0x01,		// filter on node address
		.PktType = 1,
		.PayloadLength = length,
		.CrcType = 2,			// 2 byte CRC
		.Whitening = 0
	};

	result = HAL_SUBGHZ_ExecSetCmd(hsubghz, RADIO_SET_PACKETPARAMS, (uint8_t*)&params, 9);
		
	return result;
}

static HAL_StatusTypeDef SUBGHZ_Radio_Set_IRQ(SUBGHZ_HandleTypeDef *hsubghz, uint16_t radio_irq_source)
{
	HAL_StatusTypeDef result;
	uint8_t buf[8] = {0};

	// store the 16-bit values in big-endian format
	buf[0] = buf[2] = (uint8_t)(radio_irq_source >> 8);
	buf[1] = buf[3] = (uint8_t)radio_irq_source;

	// final 4 bytes can remain zero, they don't matter

	result = HAL_SUBGHZ_ExecSetCmd(hsubghz, RADIO_CFG_DIOIRQ, buf, sizeof(buf));
	return result;
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void subghz_irq_init(void)
{
  /* SUBGHZ_Radio_IRQn interrupt configuration */
  NVIC_SetPriority(SUBGHZ_Radio_IRQn, 0);
  NVIC_EnableIRQ(SUBGHZ_Radio_IRQn);
}

void SUBGHZ_Radio_IRQHandler(void)
{
  HAL_SUBGHZ_IRQHandler(&hsubghz);
}