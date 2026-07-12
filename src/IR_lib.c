#include "IR_lib.h"
#include "mprintf.h"
#include "timer.h"
#include "asm_funcs.h"
#include "device_protocol_structs.h"
#include "utils.h"
#include "gpio.h"

#include "stm32wlxx_ll_utils.h"

#include "mprintf.h"
#include "stm32wlxx.h"

#include <stdint.h>


// This is the maximum number of entries the output buffer can hold
// Each entry is 6 bytes
#define   MAX_ENTRIES   32
// size of command ring buffer (in bytes)
#define   RF_CMD_RING_BUF_SIZE   32

// ring buffer to hold received characters until they are processed
static ringBuf RF_cmd_Buffer = {
  .data = (uint8_t[RF_CMD_RING_BUF_SIZE]){},
  .size = RF_CMD_RING_BUF_SIZE,
  .writeIndex = 0,
  .readIndex = 0,
  .overflow = false
};

// Output buffer to hold the timer period, duty cycle, and repetition information.
// Once completed, the output buffer is sent to the timer module to correctly blink the IR LED.
static uint16_t output_buffer[MAX_ENTRIES * 3] = {0};
static const uint32_t output_buffer_size = COUNT_OF(output_buffer);
static uint32_t output_buffer_index = 0;

static const struct protocol *protocol_list[] = {0, &NEC1, &NECx2};

// Private Function Declarations
static int32_t encode_number(const struct protocol *protocol, uint32_t number, uint32_t bitlen);
static uint32_t num_repeating_bits(uint32_t number);
static int32_t convert_time_array(const int16_t *enc_time_arr, uint32_t enc_time_arr_len, 
                                    uint16_t repeat_num);
static int32_t add_extent_delay(uint32_t delay_us);
static int32_t output_buffer_add_entry(uint16_t period_us, uint16_t repeat_num,
                                        uint16_t high_time_us);
static void output_buffer_init(void);

int32_t execute_command(const struct command *cmd, bool is_ditto)
{
  const struct protocol *protocol_used = cmd->device->prot_used;

  // reset the buffer before use
  // the reset function also puts a dummy entry in index 0 so the IRTIM works properly
  output_buffer_init();

  set_IR_frequency(protocol_used->carrier_freq);

  int32_t result = protocol_used->fmt_func(cmd, is_ditto);
  CHECK(result);

  // enable the IR LED before use
  GPIO_IR_Pins_Enable();

  send_pulses(output_buffer, output_buffer_index);

  while(DMA_busy()){
    // enter LPM here?
    // LL_mDelay(1);
  }

  // disable the IR LEDs when not in use
  GPIO_IR_Pins_Disable();

  return (0);
}

int32_t execute_command_RF(uint8_t protocol_id, uint8_t device_id, uint8_t subdevice_id, uint8_t function_code)
{
  const struct protocol *protocol_used = protocol_list[protocol_id];

  // initialize the buffer before use
  // the function resets the index and puts a dummy entry in index 0 so the IRTIM works properly
  output_buffer_init();

  set_IR_frequency(protocol_used->carrier_freq);

  int32_t result = protocol_used->fmt_func_RF(device_id, subdevice_id, function_code, false);
  CHECK(result);

  // enable the IR LED before use
  GPIO_IR_Pins_Enable();

  send_pulses(output_buffer, output_buffer_index);
  while(DMA_busy()){
    // enter LPM here?
    // LL_mDelay(1);
  }

  output_buffer_init();

  // disable the IR LEDs when not in use
  GPIO_IR_Pins_Disable();

  return (0);
}

void receive_RF_command(uint8_t *RF_cmd, uint32_t RF_cmd_len)
{
  uint32_t i;
  int32_t result;
  for(i = 0; i < RF_cmd_len; i++){
    result = bufPush(&RF_cmd_Buffer, RF_cmd[i]);
    if(result < 0){
      RF_cmd_Buffer.overflow = true;
    }
  }
}

void handle_RF_command_buffer(void)
{
  if(!bufIsEmpty(&RF_cmd_Buffer)){
    uint8_t protocol_id = bufPop(&RF_cmd_Buffer);
    uint8_t device_id = bufPop(&RF_cmd_Buffer);
    uint8_t subdevice_id = bufPop(&RF_cmd_Buffer);
    uint8_t function_code = bufPop(&RF_cmd_Buffer);

    printfln_("executing IR command: %#04x, %#04x, %#04x, %#04x", protocol_id,
                                                    device_id, subdevice_id, function_code);
    execute_command_RF(protocol_id, device_id, subdevice_id, function_code);
  }else{
    GPIO_set_status_LED(GREEN);
  }

}

int32_t format_NEC1_command(const struct command *cmd, bool is_ditto)
{
  int32_t result;
  uint32_t time_sum = 0;
  const struct protocol *const cur_protocol = &NEC1;
  struct stream_char *cur_char;
  // because NEC1 has a special way to handle dittos, determine which stream characteristics to use
  if(is_ditto){
    cur_char = &(cur_protocol->ditto_stream);
  }else{
    cur_char = &(cur_protocol->primary_stream); 
  }
  // encode lead-in
  result = convert_time_array(cur_char->lead_in, cur_char->lead_in_len, 0);
  CHECK(result);
  time_sum += result;
  if(is_ditto == false){
    // encode device ID
    result = encode_number(cur_protocol, cmd->device->device_id, cmd->device->device_len);
    CHECK(result);
    time_sum += result;
    // encode subdevice ID
    result = encode_number(cur_protocol, cmd->device->subdevice_id, cmd->device->subdevice_len);
    CHECK(result);
    time_sum += result;
    // encode function
    result = encode_number(cur_protocol, cmd->function, cmd->function_len);
    CHECK(result);
    time_sum += result;
    // encode the inverse function
    result = encode_number(cur_protocol, ~(cmd->function), cmd->function_len);
    CHECK(result);
    time_sum += result;
  }
  // encode lead-out
  result = convert_time_array(cur_char->lead_out, cur_char->lead_out_len, 0);
  CHECK(result);
  time_sum += result;

  if(cur_char->extent_ms != 0){
    int32_t extent_us = (cur_char->extent_ms) * 1000;
    int32_t extent_remainder = extent_us - time_sum;
    if(extent_remainder < 0){
      //something has gone wrong
      return (-1);
    }
    result = add_extent_delay(extent_remainder);
    CHECK(result);
  }
  return (0);
}

int32_t format_NEC1_command_RF(uint8_t device_id, uint8_t subdevice_id, uint8_t function_code, bool is_ditto)
{
  int32_t result;
  uint32_t time_sum = 0;
  const struct protocol *const cur_protocol = &NEC1;
  struct stream_char *cur_char;
  // because NEC1 has a special way to handle dittos, determine which stream characteristics to use
  if(is_ditto){
    cur_char = &(cur_protocol->ditto_stream);
  }else{
    cur_char = &(cur_protocol->primary_stream); 
  }
  // encode lead-in
  result = convert_time_array(cur_char->lead_in, cur_char->lead_in_len, 0);
  CHECK(result);
  time_sum += result;
  if(is_ditto == false){
    // encode device ID
    result = encode_number(cur_protocol, device_id, 8);
    CHECK(result);
    time_sum += result;
    // encode subdevice ID
    result = encode_number(cur_protocol, subdevice_id, 8);
    CHECK(result);
    time_sum += result;
    // encode function
    result = encode_number(cur_protocol, function_code, 8);
    CHECK(result);
    time_sum += result;
    // encode the inverse function
    result = encode_number(cur_protocol, ~(function_code), 8);
    CHECK(result);
    time_sum += result;
  }
  // encode lead-out
  result = convert_time_array(cur_char->lead_out, cur_char->lead_out_len, 0);
  CHECK(result);
  time_sum += result;

  if(cur_char->extent_ms != 0){
    int32_t extent_us = (cur_char->extent_ms) * 1000;
    int32_t extent_remainder = extent_us - time_sum;
    if(extent_remainder < 0){
      //something has gone wrong
      return (-1);
    }
    result = add_extent_delay(extent_remainder);
    CHECK(result);
  }
  return (0);
}

int32_t format_NECx2_command(const struct command *cmd, bool is_ditto)
{
  int32_t result;
  uint32_t time_sum = 0;
  const struct protocol *const cur_protocol = &NECx2;
  struct stream_char *cur_char;
  // NECx2 doesn't have special ditto stream, so we always use primary stream
  cur_char = &(cur_protocol->primary_stream);
  // encode lead-in
  result = convert_time_array(cur_char->lead_in, cur_char->lead_in_len, 0);
  CHECK(result);
  time_sum += result;
  // encode device ID
  result = encode_number(cur_protocol, cmd->device->device_id, cmd->device->device_len);
  CHECK(result);
  time_sum += result;
  // encode subdevice ID
  result = encode_number(cur_protocol, cmd->device->subdevice_id, cmd->device->subdevice_len);
  CHECK(result);
  time_sum += result;
  // encode function
  result = encode_number(cur_protocol, cmd->function, cmd->function_len);
  CHECK(result);
  time_sum += result;
  // encode the inverse function
  result = encode_number(cur_protocol, ~(cmd->function), cmd->function_len);
  CHECK(result);
  time_sum += result;
  // encode lead-out
  result = convert_time_array(cur_char->lead_out, cur_char->lead_out_len, 0);
  CHECK(result);
  time_sum += result;

  if(cur_char->extent_ms != 0){
    int32_t extent_us = (cur_char->extent_ms) * 1000;
    int32_t extent_remainder = extent_us - time_sum;
    if(extent_remainder < 0){
      //something has gone wrong
      return (-1);
    }
    result = add_extent_delay(extent_remainder);
    CHECK(result);
  }
  return (0);
}

int32_t format_NECx2_command_RF(uint8_t device_id, uint8_t subdevice_id, uint8_t function_code, bool is_ditto)
{
  int32_t result;
  uint32_t time_sum = 0;
  const struct protocol *const cur_protocol = &NECx2;
  struct stream_char *cur_char;
  // NECx2 doesn't have special ditto stream, so we always use primary stream
  cur_char = &(cur_protocol->primary_stream);
  // encode lead-in
  result = convert_time_array(cur_char->lead_in, cur_char->lead_in_len, 0);
  CHECK(result);
  time_sum += result;
  // encode device ID
  result = encode_number(cur_protocol, device_id, 8);
  CHECK(result);
  time_sum += result;
  // encode subdevice ID
  result = encode_number(cur_protocol, subdevice_id, 8);
  CHECK(result);
  time_sum += result;
  // encode function
  result = encode_number(cur_protocol, function_code, 8);
  CHECK(result);
  time_sum += result;
  // encode the inverse function
  result = encode_number(cur_protocol, ~(function_code), 8);
  CHECK(result);
  time_sum += result;
  // encode lead-out
  result = convert_time_array(cur_char->lead_out, cur_char->lead_out_len, 0);
  CHECK(result);
  time_sum += result;

  if(cur_char->extent_ms != 0){
    int32_t extent_us = (cur_char->extent_ms) * 1000;
    int32_t extent_remainder = extent_us - time_sum;
    if(extent_remainder < 0){
      //something has gone wrong
      return (-1);
    }
    result = add_extent_delay(extent_remainder);
    CHECK(result);
  }
  return (0);
}

/*
    this function takes a number and encodes it per the protocol provided
    returns negative number on error, otherwise returns the length of time encoded
*/
static int32_t encode_number(const struct protocol *protocol, uint32_t number, uint32_t bitlen)
{
  uint32_t input_num;
  int32_t result;
  int32_t time_sum = 0;
  // if least-significant bit is transmitted first
  if(protocol->LSB == false){
    //reverse the bit order of the number
    input_num = rev_bit(number, bitlen);
  }else{
    input_num = number;
  }

  uint32_t bit_index = 0;
  uint32_t num_repeats = 0;

  while(bit_index != bitlen){
    //check for repeat bits and encode them
    num_repeats = num_repeating_bits(input_num);
    //can't repeat longer than the bit length
    if(num_repeats > (bitlen - bit_index - 1)){
      num_repeats = bitlen - bit_index - 1;
    }

    if(input_num & 0x01){
      result = convert_time_array(protocol->enc_one, 2, num_repeats);
    }else{
      result = convert_time_array(protocol->enc_zero, 2, num_repeats);
    }

    CHECK(result);
    time_sum += (result * (num_repeats + 1));

    input_num = input_num >> (num_repeats + 1);
    bit_index += (num_repeats + 1);
  }
  return (time_sum);
}

/*  
    returns the number of times bit 0 is repeated
    examples:
    0b00001111 - returns 3
    0b00101000 - returns 2
    0b11111110 - returns 0
*/
static uint32_t num_repeating_bits(uint32_t number)
{
  uint32_t num_repeats = 0;
  uint32_t bit0 = number & 0x01;
  uint32_t tmp_num = number >> 1;

  while((tmp_num & 0x01) == bit0){
    num_repeats++;
    tmp_num = tmp_num >> 1;
    if(num_repeats == 31){
      break;
    }
  }
  return num_repeats;
}

/*
    this function converts an encoded time array to 0, 1, or 2 output buffer entries
    the output buffer entries tell us the length, duty cycle, and number of repeats of the timer

    returns negative number on error, otherwise returns the length of time encoded when not repeated
*/
static int32_t convert_time_array(const int16_t *enc_time_arr, uint32_t enc_time_arr_len, 
                            uint16_t repeat_num)
{
  int32_t result;
  int32_t time_sum;
    //encoding array should contain 1 or 2 entries, but 0 isn't wrong...
    switch(enc_time_arr_len){
    case 0:
      return (0);
      break;
    case 1:{
      uint16_t abs_val = (uint16_t)abs_int(enc_time_arr[0]);
      time_sum = abs_val;
      if(enc_time_arr[0] < 0){
        result = output_buffer_add_entry(abs_val, repeat_num, 0);
      }else{
        result = output_buffer_add_entry(abs_val, repeat_num, abs_val);
      }
      break;
    }
    case 2:{
      // if both entries are positive or negative
      if(((enc_time_arr[0] < 0) && (enc_time_arr[1] < 0)) || (!(enc_time_arr[0] < 0) && !(enc_time_arr[1] < 0))){
        // return an error. I don't want to deal with this.
        return (-1);
      }
      // if the first entry is negative, no repeats allowed
      if((enc_time_arr[0] < 0) && (repeat_num > 0)){
        return (-1);
      }
      uint16_t abs_val0 = (uint16_t)abs_int(enc_time_arr[0]);
      uint16_t abs_val1 = (uint16_t)abs_int(enc_time_arr[1]);
      time_sum = abs_val0 + abs_val1;

      if(enc_time_arr[0] < 0){
        result = output_buffer_add_entry(abs_val0, 0, 0);
        result = output_buffer_add_entry(abs_val1, 0, abs_val1);
      }else{
        result = output_buffer_add_entry(abs_val0 + abs_val1, repeat_num, abs_val0);
      }
      break;
    }
    default:
      return (-1);
      break;
    }
    CHECK(result);
    return time_sum;
}

/*
    this function adds entries to the end of the output buffer that are dead time
    this is used when protocols have an extent and are required to take a fixed amount of time to transmit
*/
static int32_t add_extent_delay(uint32_t delay_us)
{
  int32_t result;
  uint32_t upper_bytes = delay_us >> 16;

  // if the delay is larger than 0x00FF_FFFF, it can't be contained in only 2 buffer entries
  // also, that means the delay is longer than 16 seconds. Throw an error
  if((upper_bytes & 0xFF00) != 0){
    return(-1);
  }

  // largest output buffer entry can only be 16 bits, but delay time can be larger than that

  // if the delay time is longer than 16 bits
  if(upper_bytes != 0){
    result = output_buffer_add_entry(0xFFFF, upper_bytes, 0);
    CHECK(result);
  }
  uint16_t lower_bytes = delay_us & 0x0000FFFF;
  result = output_buffer_add_entry(lower_bytes, 0, 0);
  return result;
}

/*
    This function adds an entry to output buffer if there is space.
    returns 0 if successful, -1 if the buffer is full
*/
static int32_t output_buffer_add_entry(uint16_t period_us, uint16_t repeat_num, uint16_t high_time_us)
{
  if(period_us == 0){
    return (0);
  }

  // this assumes (output_buffer_size) mod (3) always equals 0
  // and output_buffer_index always stays aligned
  if(output_buffer_index < output_buffer_size){
    output_buffer[output_buffer_index++] = period_us;
    output_buffer[output_buffer_index++] = repeat_num;
    output_buffer[output_buffer_index++] = high_time_us;
    return (0);
  }else{
    return (-1);
  }
}

static void output_buffer_init(void)
{
  output_buffer_index = 0;
  /*
  There is a bug where IRTIM will output high until it sees
  the first edge from TIM16 OC1. The workaround is to always
  have the first buffer entry be a dummy one that cycles TIM16.
  */
  output_buffer_add_entry(1, 0, 0);
}