// subghz_support.c -- subghz support functions

#include "subghz_support.h"
#include "subghz.h"

#include "stm32wlxx_hal_subghz.h"
#include "mprintf.h"

#include <stdint.h>
#include <stdbool.h>



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

#define SYNCWORD_BASEADDRESS		0x06C0
#define NODE_ADDRESS_REG			0x06CD
#define BROADCAST_ADDRESS_REG		0x06CE		
#define CRC_INIT_MSB_REG			0x06BC
#define CRC_INIT_LSB_REG			0x06BD
#define CRC_POLY_MSB_REG			0x06BE
#define CRC_POLY_LSB_REG			0x06BF


extern SUBGHZ_HandleTypeDef subghz_handle;

static HAL_StatusTypeDef DefaultTxConfig(SUBGHZ_HandleTypeDef *hsubghz);
static HAL_StatusTypeDef DefaultModulationParams(SUBGHZ_HandleTypeDef *hsubghz);
static HAL_StatusTypeDef DefaultCRC(SUBGHZ_HandleTypeDef *hsubghz);

HAL_StatusTypeDef subghz_default_init(SUBGHZ_HandleTypeDef *hsubghz)
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

HAL_StatusTypeDef SetAddress(SUBGHZ_HandleTypeDef *hsubghz, uint8_t address)
{
	return HAL_SUBGHZ_WriteRegisters(hsubghz, NODE_ADDRESS_REG, &address, sizeof(address));
}

void subghz_radio_getstatus(void)
{
	uint8_t RadioResult = 0x00;
	HAL_SUBGHZ_ExecGetCmd(&subghz_handle, RADIO_GET_STATUS, &RadioResult, 1);
  	printf_("RR: 0x%02x\r\n", RadioResult);
}

void subghz_radio_getRxBufferStatus(void)
{
	uint8_t buf[3] = {0};

	HAL_SUBGHZ_ExecGetCmd(&subghz_handle, RADIO_GET_RXBUFFERSTATUS, buf, sizeof(buf));
  	printf_("Buf Status: %#04x, %#04x, %#04x\r\n", buf[0], buf[1], buf[2]);
}

void subghz_radio_getPacketStatus(uint8_t *buffer, bool print)
{
	uint8_t temp_buf[4] = {0};
	uint32_t i;
	HAL_SUBGHZ_ExecGetCmd(&subghz_handle, RADIO_GET_PACKETSTATUS, temp_buf, sizeof(temp_buf));

	for(i = 0; i < sizeof(temp_buf); i++){
		buffer[i] = temp_buf[i];
	}
	if(print){
  		printf_("Packet Status: %#04x, %#04x, %#04x, %#04x\r\n", temp_buf[0], temp_buf[1], temp_buf[2], temp_buf[3]);
	}
}

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

HAL_StatusTypeDef SetPayloadLength(SUBGHZ_HandleTypeDef *hsubghz, uint8_t length)
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

static HAL_StatusTypeDef DefaultCRC(SUBGHZ_HandleTypeDef *hsubghz)
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

static HAL_StatusTypeDef DefaultModulationParams(SUBGHZ_HandleTypeDef *hsubghz)
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

HAL_StatusTypeDef SUBGHZ_Radio_Set_IRQ(SUBGHZ_HandleTypeDef *hsubghz, uint16_t radio_irq_source)
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