#include "IR_lib.h"
#include "timer.h"

#include "stm32wlxx_ll_utils.h"

#include <stdint.h>


#define PULSE_PERIOD    0x0800

uint16_t dma_data_arr[] = {
  PULSE_PERIOD, 0, PULSE_PERIOD/2,
  PULSE_PERIOD, 0, PULSE_PERIOD/2,
  PULSE_PERIOD, 0, PULSE_PERIOD/4,
  PULSE_PERIOD, 0, PULSE_PERIOD/32
};
uint32_t arr_size = sizeof(dma_data_arr)/sizeof(dma_data_arr[0]);

uint16_t dma_data_arr2[] = {
  PULSE_PERIOD, 0, PULSE_PERIOD/2,
  PULSE_PERIOD, 0, PULSE_PERIOD/2,
  PULSE_PERIOD, 0, PULSE_PERIOD/2,
  PULSE_PERIOD, 0, PULSE_PERIOD/2
};
uint32_t arr_size2 = sizeof(dma_data_arr2)/sizeof(dma_data_arr2[0]);

void send_command(void)
{
  send_pulses(dma_data_arr, arr_size);
  while(DMA_busy()){
    LL_mDelay(50);
  }
  send_pulses(dma_data_arr2, arr_size2);
}

int16_t execute_command(const struct command *cmd, bool is_ditto)
{
    const struct protocol *protocol_used = cmd->device->prot_used;
    const struct stream_char *cur_char;   //current characteristics
    const uint32_t SMCLK_freq = CS_getSMCLK();
    uint16_t extent;

    if(is_ditto){
        if(protocol_used->has_ditto == true){
            cur_char = &(protocol_used->ditto_stream);
            extent = cmd->device->prot_used->ditto_stream.extent_ms;
        }else{
            cur_char = &(protocol_used->primary_stream);
            extent = cmd->device->prot_used->primary_stream.extent_ms;
        }
    }else{
        cur_char = &(protocol_used->primary_stream);
        extent = cmd->device->prot_used->primary_stream.extent_ms;
    }
/*
    //set up carrier freq and SPI timing
    stop_carrier_wave();
    enable_carrier_wave(SMCLK_freq, protocol_used->carrier_freq);

    disable_SPI();
    set_unit_freq(SMCLK_freq, protocol_used->unit_freq);


    int16_t result = protocol_used->fmt_func(output_buf, OUTPUT_BUF_SIZE, cur_char, cmd, is_ditto);

    if(result < 0)
        return result;

    if(extent != 0)
        start_extent_timer(extent);
    else
        set_extent_passed();

    TXData_size = result;
    TXData_index = 0;
    enable_SPI();
    enable_SPI_int();

    //enter LPM0 and wait for command to execute
    __bis_SR_register(LPM0_bits);

    clear_buffer(output_buf, TXData_size);

    if(extent_passed() == false){
        //enter LPM3 and wait for the extent time period to pass
        __bis_SR_register(LPM3_bits);
    }

    reset_extent_passed();
*/

    return (0);
}