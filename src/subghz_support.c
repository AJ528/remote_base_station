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

HAL_StatusTypeDef subghz_init_settings_default(SUBGHZ_HandleTypeDef *hsubghz)
{
	HAL_StatusTypeDef result;

	// set the standby clock
	// 0x00 = 13 MHz internal RC oscillator
	// 0x01 = High Speed 32 MHz external oscillator (HSE32)
	const uint8_t standby_clock = 0x00;		
	result = HAL_SUBGHZ_ExecSetCmd(hsubghz, RADIO_SET_STANDBY, &standby_clock, 1);
	if(result != HAL_OK){
		return result;
	}

	// set the relative locations of the TX and RX buffers inside the 256-byte data buffer
	// default is TX addr = 0x80 and RX addr = 0x00 ({0x80, 0x00})
	const uint8_t buf_base_addr[2] = {0x80, 0x00};
	result = HAL_SUBGHZ_ExecSetCmd(hsubghz, RADIO_SET_BUFFERBASEADDRESS, buf_base_addr, 2);
	if(result != HAL_OK){
		return result;
	}

	// set the format of the packets that will be transmitted/received
	// 0x00 = FSK packet
	// 0x01 = LoRa packet
	// 0x02 = BPSK packet
	// 0x03 = MSK packet
	const uint8_t packet_type  = 0x00;
	result = HAL_SUBGHZ_ExecSetCmd(hsubghz, RADIO_SET_PACKETTYPE, &packet_type, 1);
	if(result != HAL_OK){
		return result;
	}

	// set the sync word the radio will listen for
	// receiving a preamble followed by this pattern of data tells the radio a transmission has started
	// syncwords can be a max of 64 bits long. The syncword length is set via the Set_PacketParams() command.
	const uint8_t syncword[] = {0x48, 0xDF, 0x70, 0x72, 0x00, 0x00, 0x00, 0x00};
	result = HAL_SUBGHZ_WriteRegisters(hsubghz, SYNCWORD_BASEADDRESS, syncword, sizeof(syncword));
	if(result != HAL_OK){
		return result;
	}

	// set the default CRC values to be used when error-checking the received packets.
	// CRC verification is enabled via the Set_PacketParams() command.
	result = DefaultCRC(hsubghz);
	if(result != HAL_OK){
		return result;
	}

	uint8_t payload_len;
#if (RX_MODE == 1)
	payload_len = 18;
#endif
#if (TX_MODE == 1)
	payload_len = 3;
#endif
	// set the payload length
	// when receiving variable-length packets, the length set here is the max allowed before error
	// variable length packets are enabled via the Set_PacketParams() command (called in SetPayloadLength()).
	result = subghz_setPayloadLength(hsubghz, payload_len);
	if(result != HAL_OK){
		return result;
	}

#if (TX_MODE == 1)
	// configure the output power and power amplifier used when transmitting
	result = DefaultTxConfig(hsubghz);
	if(result != HAL_OK){
		return result;
	}
#endif

	// set the default modulation parameters (bit rate, frequency deviation, rx bandwidth)
	result = DefaultModulationParams(hsubghz);
	if(result != HAL_OK){
		return result;
	}

	// set the frequency of the carrier wave
	result = subghz_setFrequency(hsubghz, RF_FREQ);
	return result;
}

HAL_StatusTypeDef subghz_setAddress(SUBGHZ_HandleTypeDef *hsubghz, uint8_t address)
{
	return HAL_SUBGHZ_WriteRegisters(hsubghz, NODE_ADDRESS_REG, &address, sizeof(address));
}

uint8_t subghz_radio_getstatus(void)
{
	uint8_t RadioStatus;
	HAL_SUBGHZ_ExecGetCmd(&subghz_handle, RADIO_GET_STATUS, &RadioStatus, 1);
	printf_("Radio Status: %#04x\r\n", RadioStatus);
	return RadioStatus;
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

HAL_StatusTypeDef subghz_setPayloadLength(SUBGHZ_HandleTypeDef *hsubghz, uint8_t length)
{
	HAL_StatusTypeDef result;

	struct sRadioParams params = {
		.PbLength = 32,						// send a preamble 32-bits long
		.PbDetLength = 0x07,			// detect a preamble 32-bits long
		.SyncWordLength = 32,			// the sync word length is 32 bits
		.AddrComp = 0x01,					// filter on node address (ignore packets that don't match our address)
		.PktType = 1,							// enable variable-length payloads
		.PayloadLength = length,	// set the max payload length that can be received without error (RX-mode)
															// otherwise, sets the payload length that will be transmitted (TX-mode)
		.CrcType = 2,							// enables CRC verification, and sets the CRC length to 2 bytes
		.Whitening = 0						// disables whitening
	};

	result = HAL_SUBGHZ_ExecSetCmd(hsubghz, RADIO_SET_PACKETPARAMS, (uint8_t*)&params, 9);
		
	return result;
}

// sets the carrier wave frequency. Also limits the frequency to acceptable values.
HAL_StatusTypeDef subghz_setFrequency(SUBGHZ_HandleTypeDef *hsubghz, uint32_t frequency)
{
	HAL_StatusTypeDef result;
	const uint32_t freq_lower_limit = 902000000;
	const uint32_t freq_upper_limit = 928000000;

	if(frequency < freq_lower_limit){
		frequency = freq_lower_limit;
	}
	if(frequency > freq_upper_limit){
		frequency = freq_upper_limit;
	}

	// RF-PLL channel value
	uint32_t channel;
	uint8_t buffer[4];
	// calculate the channel value from the desired frequency
	SX_FREQ_TO_CHANNEL(channel, frequency);
	// store the calculated value in the buffer
	buffer[0] = ( uint8_t )( ( channel >> 24 ) & 0xFF );
	buffer[1] = ( uint8_t )( ( channel >> 16 ) & 0xFF );
	buffer[2] = ( uint8_t )( ( channel >> 8 ) & 0xFF );
	buffer[3] = ( uint8_t )( channel & 0xFF );

	result = HAL_SUBGHZ_ExecSetCmd(hsubghz, RADIO_SET_RFFREQUENCY, buffer, 4);
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

	result = HAL_SUBGHZ_WriteRegisters(hsubghz, CRC_INIT_MSB_REG, CRC_init, 2);
	if(result != HAL_OK){
		return result;
	}
	result = HAL_SUBGHZ_WriteRegisters(hsubghz, CRC_POLY_MSB_REG, CRC_poly, 2);
	return result;


}

static HAL_StatusTypeDef DefaultModulationParams(SUBGHZ_HandleTypeDef *hsubghz)
{
	uint8_t buf[8] = {0};
	// calculate the value of BR from the clock freq and desired bit rate
	uint32_t tempVal = ( uint32_t )(( 32 * XTAL_FREQ ) / BIT_RATE );
	buf[0] = ( tempVal >> 16 ) & 0xFF;	// BR
	buf[1] = ( tempVal >> 8 ) & 0xFF;		// BR
	buf[2] = tempVal & 0xFF;						// BR
	// set the pulse-shape filter
	buf[3] = 0;						// no filter
	// set the RX filter bandwidth
	buf[4] = 0x0B;				// RX bandwidth of 117 kHz
	//calculate Fdev from the desired frequency deviation
	SX_FREQ_TO_CHANNEL(tempVal, FREQ_DEVIATION);
	buf[5] = ( tempVal >> 16 ) & 0xFF;	// FDev
	buf[6] = ( tempVal >> 8 ) & 0xFF;		// FDev
	buf[7] = ( tempVal& 0xFF );					// FDev

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

HAL_StatusTypeDef subghz_setIRQ(SUBGHZ_HandleTypeDef *hsubghz, uint16_t radio_irq_source)
{
	HAL_StatusTypeDef result;
	uint8_t buf[8] = {0};

	// store the 16-bit value twice in big-endian format
	// overall takes up 4 bytes
	buf[0] = buf[2] = (uint8_t)(radio_irq_source >> 8);
	buf[1] = buf[3] = (uint8_t)radio_irq_source;

	// the final 4 bytes can remain zero, they don't matter
	result = HAL_SUBGHZ_ExecSetCmd(hsubghz, RADIO_CFG_DIOIRQ, buf, 8);
	return result;
}