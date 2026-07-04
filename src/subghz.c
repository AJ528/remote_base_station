/*
 * subghz.c
 *
 *  Created on: Mar 23, 2024
 *      Author: adevries
 */

#include "subghz.h"
#include "stm32wlxx.h"
#include "subghz_support.h"
#include "IR_lib.h"
#include "error.h"
#include "mprintf.h"
#include "utils.h"

#include "gpio.h"
#include "sysclk.h"
#include "stm32wlxx_ll_rcc.h"

#include "stm32wlxx_hal_subghz.h"
#include "stm32wlxx_ll_bus.h"

#include <stdint.h>
#include <stdbool.h>


#define SMPS_CTRL0_REG_ADDR         0x0916
#define SUBGHZ_PCR_ADDR             0x091A
#define SUBGHZ_REGDRVCR_ADDR        0x091F
#define SUBGHZ_SMPSC2R_ADDR         0x0923

#define RADIO_MODE_STANDBY_RC       0x02
#define RADIO_MODE_STANDBY_HSE32    0x03
#define	RADIO_MODE_FS               0x04
#define	RADIO_MODE_RX               0x05
#define RADIO_MODE_TX               0x06

#define RADIO_MODE_BITFIELD         0x70
#define RADIO_STATUS_BITFIELD       0x0E

SUBGHZ_HandleTypeDef subghz_handle;

static HAL_StatusTypeDef subghz_configure_settings(SUBGHZ_HandleTypeDef *hsubghz);
static uint32_t str_to_uint32(const char * restrict nptr, const char ** restrict endptr, uint32_t base);
static void subghz_init_irq(SUBGHZ_HandleTypeDef *hsubghz);


void subghz_init(void)
{

  uint8_t regVal = 0;

  // enable clocks
  LL_APB3_GRP1_EnableClock(LL_APB3_GRP1_PERIPH_SUBGHZSPI);

  // subghz SPI max speed is 16 MHz, but there's no need to go that fast
  subghz_handle.Init.BaudratePrescaler = SUBGHZSPI_BAUDRATEPRESCALER_8;

  // init the subghz HAL module
  ERROR_CHECK(HAL_SUBGHZ_Init(&subghz_handle));

  /*  Put the SUBGHZ module into standby mode. Since this is the first command sent 
      after initialization, the module also goes through calibration (takes 1.6ms) */
  const uint8_t standby_clock = 0x00; // sets the standby clock to 13MHz internal RC oscillator
  ERROR_CHECK(HAL_SUBGHZ_ExecSetCmd(&subghz_handle, RADIO_SET_STANDBY, &standby_clock, 1));

  // TODO: verify there are no errors as the subghz module gets initialized
  uint8_t err_buff[4]={0};
  ERROR_CHECK(HAL_SUBGHZ_ExecGetCmd(&subghz_handle, RADIO_GET_ERROR, err_buff, 3));

  /*  Set VDDTCXO to 2.2V. When combined with the series impedance on the CCA, the TCXO will
      create a 32MHz clock signal at a safe voltage level for the MCU */
  // for some reason, the timeout cannot be 0x00 or there are HAL errors
  const uint8_t tcxo_settings[4] = {0x03, 0x00, 0x00, 0x40};		// sets VDDTCXO to output 2.2V and disables the timeout
  ERROR_CHECK(HAL_SUBGHZ_ExecSetCmd(&subghz_handle, RADIO_SET_TCXOMODE, tcxo_settings, 4));

  // TODO: see if you need to enable calibrations before actually calibrating.
  // uint8_t cal_enable = 0x7f;
  // ERROR_CHECK(HAL_SUBGHZ_ExecSetCmd(&subghz_handle, RADIO_CALIBRATE, &cal_enable, 1));

  /*  Enable the clock detection circuitry. This should automatically disable the onboard SMPS if the HSE32 signal ever fails.
      The reference manual says it's not necessary when using TCXO powered from VDDTCXO, but it probably doesn't hurt. */
  const uint8_t clk_detect = 0x40;
  HAL_SUBGHZ_ReadRegister(&subghz_handle, SMPS_CTRL0_REG_ADDR, &regVal);
  regVal = regVal | clk_detect;
  ERROR_CHECK(HAL_SUBGHZ_WriteRegister(&subghz_handle, SMPS_CTRL0_REG_ADDR, regVal));

  /*  Set the SUBGHZ module to use the SMPS when in active modes */
  const uint8_t regulator_mode = 0x01;  // sets active mode power supply to SMPS
  ERROR_CHECK(HAL_SUBGHZ_ExecSetCmd(&subghz_handle, RADIO_SET_REGULATORMODE, &regulator_mode, 1));

  /* You cannot trust the "reset values" listed in the reference manual for the following 3 registers.
     Therefore, do a read -> modify -> write on the registers instead of just writing a value to them. */
  const uint8_t pcr_val = 0x60;
  HAL_SUBGHZ_ReadRegister(&subghz_handle, SUBGHZ_PCR_ADDR, &regVal);
  regVal &= ~(pcr_val);
  regVal |= pcr_val;
  HAL_SUBGHZ_WriteRegister(&subghz_handle, SUBGHZ_PCR_ADDR, regVal);

  const uint8_t reg_drv_val = 0x09;
  HAL_SUBGHZ_ReadRegister(&subghz_handle, SUBGHZ_REGDRVCR_ADDR, &regVal);
  regVal &= ~(reg_drv_val);
  regVal |= reg_drv_val;
  HAL_SUBGHZ_WriteRegister(&subghz_handle, SUBGHZ_REGDRVCR_ADDR, regVal);

  const uint8_t smps_drv_val = 0x06;
  HAL_SUBGHZ_ReadRegister(&subghz_handle, SUBGHZ_SMPSC2R_ADDR, &regVal);
  regVal &= ~(smps_drv_val);
  regVal |= smps_drv_val;
  HAL_SUBGHZ_WriteRegister(&subghz_handle, SUBGHZ_SMPSC2R_ADDR, regVal);
}

void subghz_config(void)
{
  // initialize communication parameters (frequency, bandwidth, packet length, etc.)
  ERROR_CHECK(subghz_configure_settings(&subghz_handle));

#if (RX_MODE == 1)
  subghz_init_irq(&subghz_handle);
  set_RF_switch_RX();
#endif
#if (TX_MODE == 1)
  set_RF_switch_TX();
#endif
}

static HAL_StatusTypeDef subghz_configure_settings(SUBGHZ_HandleTypeDef *hsubghz)
{
  // initialize the radio settings to reasonable defaults (I don't expect these settings to change)
  subghz_init_settings_default(hsubghz);

  // set our own address (when receiving) or the address you want to send data to (when transmitting)
  // address comparison/filtering is enabled via the Set_PacketParams() command.
  ERROR_CHECK(subghz_setAddress(hsubghz, OWN_ADDRESS));

  // get the status of the radio
  uint8_t RadioResult = subghz_radio_getstatus();
  
  // extract the radio mode from the result
  uint8_t RadioMode = ((RadioResult & RADIO_MODE_BITFIELD) >> 4);
  // confirm radio is in standby mode using the internal RC oscillator
  if(RadioMode != RADIO_MODE_STANDBY_RC){
    return HAL_ERROR;
  }
  return HAL_OK;
}

int32_t tx_cmd(uint32_t argc, char* argv[])
{
#if (TX_MODE == 1)
  // radio RAM has a 256-byte data buffer. Limit max to 128 bytes for now.
  uint8_t buf[4] = {0};
  uint16_t buf_index = 0;
  char const * endptr = NULL;
  uint32_t i;

  for(i = 1; i < argc; i++){
    if(buf_index < sizeof(buf)){
      buf[buf_index++] = (uint8_t)str_to_uint32(argv[i], &endptr, 0);
    }
  }

  puts_("tx_buf = ");
  for(i = 0; i < buf_index; i++){
    printf_("%#04x, ", buf[i]);
  }
  print_newline();

  subghz_write_tx_buffer(buf, buf_index);
  // subghz_write_tx_buffer((uint8_t[]){0x02, 0x07, 0x07, 0x07}, 4);
  tx_packet();
  toggle_status_LED();


#endif
#if (RX_MODE == 1)
  printfln_("Firmware in RX mode, function not available.");
#endif

  return 0;
}

static uint32_t str_to_uint32(const char * restrict nptr, const char ** restrict endptr, uint32_t base)
{
  bool is_valid = false;
  bool is_negative = false;
  char const *c_Ptr;
  uint32_t digit;
  uint32_t result = 0;

  //basic error checking
  if((nptr == NULL) || (base == 1) || (base > 36)){
    return UINT32_MAX;
  }

  c_Ptr = nptr;
  // if c_Ptr is pointing at whitespace, go forward until it's not
  while((*c_Ptr == ' ') || (*c_Ptr == '\t')){
    c_Ptr++;
  }

  // check for leading '+' or '-'
  if(*c_Ptr == '-'){
    is_negative = true;
    c_Ptr++;
  }else if(*c_Ptr == '+'){
    c_Ptr++;
  }

  //see if the value starts with "0x" (indicating base-16)
  if(((base == 0) || (base == 16)) && 
  (*c_Ptr == '0') && 
  ((c_Ptr[1] == 'x') || (c_Ptr[1] == 'X'))){
    c_Ptr += 2;   // jump past "0x"
    base = 16;
  } // if not, check for "0b" (indicating base-2)
  else if(((base == 0) || (base == 2)) && 
  (*c_Ptr == '0') && 
  ((c_Ptr[1] == 'b') || (c_Ptr[1] == 'B'))){
    c_Ptr += 2;   // jump past "0b"
    base = 2;
  }

  // if base is 0 and we haven't found any special prefixes, assume base-10
  if(base == 0){
    base = 10;
  }

  // infinite loop (until we "break" out of it)
  while(true){
    // examine current character to see if it's a potential digit
    if((*c_Ptr >= '0') && (*c_Ptr <= '9')){
      digit = *c_Ptr - '0';
      c_Ptr++;
    }else if((*c_Ptr >= 'a') && (*c_Ptr <= 'z')){
      digit = *c_Ptr - 'a' + 10;
      c_Ptr++;
    }else if((*c_Ptr >= 'A') && (*c_Ptr <= 'Z')){
      digit = *c_Ptr - 'A' + 10;
      c_Ptr++;
    }else{
      // if not a potential digit, stop converting
      break;
    }
    // see if the potential digit is allowed in the base being converted
    if(digit >= base){
      // digit is not allowed
      // move pointer back one and exit loop
      c_Ptr--;
      break;
    }
    // if we reach this point the digit is valid, so add it to the result
    // first multiply the working result by the base
    result *= base;
    result += digit;
    
    // if we reach this point at least once the result is now valid
    is_valid = true;
  }

  if(endptr != NULL){
    if(is_valid){ // if the result is valid, endptr is set to first invalid character
      *endptr = c_Ptr;
    }else{ // otherwise endptr = nptr
      *endptr = nptr;
    }
  }

  if(is_negative){
    return (0 - result);
  }else{
    return result;
  }
}

// this function requires dest_buffer to be larger than the data being copied over
int32_t subghz_read_rx_buffer(uint8_t *dest_buffer)
{
  uint8_t buf[16];

  HAL_SUBGHZ_ExecGetCmd(&subghz_handle, RADIO_GET_RXBUFFERSTATUS, buf, 4);

  uint32_t payload_len = buf[1] + 1;
    printfln_("Buf Status: %#04x, %#04x, %#04x", buf[0], buf[1], buf[2]);
  
  // read bytes from rx buffer
  HAL_SUBGHZ_ReadBuffer(&subghz_handle, buf[2], dest_buffer, (uint16_t)payload_len);

  uint32_t i;
  puts_("rx payload = ");
  for(i = 0; i < payload_len; i++){
    printf_("%#04x, ", dest_buffer[i]);
  }
  print_newline();

  return payload_len;
}

void subghz_write_tx_buffer(uint8_t *value, uint16_t val_len)
{
  // uint8_t tx_addr;

  // get the tx buffer current location
  // HAL_SUBGHZ_ReadRegister(&subghz_handle, 0x0802, &tx_addr);
  // printfln_("tx_addr = %#0x", tx_addr);
  
  // write bytes to the start of the tx buffer
  HAL_SUBGHZ_WriteBuffer(&subghz_handle, 0x80, value, val_len);
}

HAL_StatusTypeDef tx_packet(void)
{
  const uint8_t RadioCmd[3] = {0xff, 0xff, 0x00};	// disable timeout
  return(HAL_SUBGHZ_ExecSetCmd(&subghz_handle, RADIO_SET_TX, RadioCmd, 3));
}

HAL_StatusTypeDef continuous_rx_enable(void)
{
  const uint8_t RadioCmd[3] = {0xFF, 0xFF, 0xFF};
  return(HAL_SUBGHZ_ExecSetCmd(&subghz_handle, RADIO_SET_RX, RadioCmd, 3));
}

HAL_StatusTypeDef single_rx_blocking(void)
{
  const uint8_t RadioCmd[3] = {0};
  return(HAL_SUBGHZ_ExecSetCmd(&subghz_handle, RADIO_SET_RX, RadioCmd, 3));
}

static void subghz_init_irq(SUBGHZ_HandleTypeDef *hsubghz)
{
  subghz_setIRQ(hsubghz, SUBGHZ_IRQ_RXDONE | SUBGHZ_IRQ_ERROR);
  /* SUBGHZ_Radio_IRQn interrupt configuration */
  // set interrupt priority to 3. Lower numbers have higher priority
  NVIC_SetPriority(SUBGHZ_Radio_IRQn, 3);
  NVIC_EnableIRQ(SUBGHZ_Radio_IRQn);
}

void SUBGHZ_Radio_IRQHandler(void)
{
  // pass the interrupt to the HAL IRQ handler
  // HAL_SUBGHZ_IRQHandler(&subghz_handle);

  // moving HAL IRQ handler body here for the moment

  uint8_t tmpisr[3U] = {0U};
  uint16_t itsource;
  uint8_t tmp_buf[4] = {0};

  /* Retrieve Interrupts from SUBGHZ Irq Register */
  HAL_SUBGHZ_ExecGetCmd(&subghz_handle, RADIO_GET_IRQSTATUS, tmpisr, 3U);
  itsource = tmpisr[1U];
  itsource = (itsource << 8U) | tmpisr[2U];

  /* Clear SUBGHZ Irq Register */
  HAL_SUBGHZ_ExecSetCmd(&subghz_handle, RADIO_CLR_IRQSTATUS, tmpisr+1, 2U);

  // if there is an error, don't do anything else
  if (SUBGHZ_CHECK_IT_SOURCE(itsource, SUBGHZ_IRQ_ERROR) != RESET)
  {
    // if you need more info about the error source, look at the packet status
    printfln_("SUBGHZ Error!");
    subghz_radio_getPacketStatus(tmp_buf, true);
    return;
  }

  /* Packet transmission completed Interrupt */
  if (SUBGHZ_CHECK_IT_SOURCE(itsource, SUBGHZ_IRQ_TXDONE) != RESET)
  {
    // do something
  }

  /* Packet received Interrupt */
  if (SUBGHZ_CHECK_IT_SOURCE(itsource, SUBGHZ_IRQ_RXDONE) != RESET)
  {
    uint8_t rx_buf[16];
    uint32_t data_len;
    // do something
    toggle_status_LED();
    printfln_("packet received!");
    data_len = subghz_read_rx_buffer(rx_buf);
    subghz_radio_getPacketStatus(tmp_buf, false);
    int32_t rssi_avg = (tmp_buf[3] / 2) * -1;
    printfln_("rssi avg = %d dBm", rssi_avg);
    if(data_len == 5){
      receive_RF_command(&rx_buf[1], 4);
    }
  }

  /* Preamble Detected Interrupt */
  if (SUBGHZ_CHECK_IT_SOURCE(itsource, SUBGHZ_IRQ_PREAMBLE_DETECTED) != RESET)
  {
    // do something
  }

  /*  Valid sync word detected Interrupt */
  if (SUBGHZ_CHECK_IT_SOURCE(itsource, SUBGHZ_IRQ_SYNCWORD_VALID) != RESET)
  {
    // do something
  }

  /* Rx or Tx Timeout Interrupt */
  if (SUBGHZ_CHECK_IT_SOURCE(itsource, SUBGHZ_IRQ_RX_TX_TIMEOUT) != RESET)
  {
    // do something
  }
}