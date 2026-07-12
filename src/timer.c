#include "timer.h"
#include "asm_funcs.h"

#include "mprintf.h"

#include "stm32wlxx_ll_tim.h"
#include "stm32wlxx_ll_bus.h"
#include "stm32wlxx_ll_dma.h"
#include "stm32wlxx_ll_rcc.h"

#include <stdint.h>
#include <stdbool.h>

// #define TIM16_PERIOD    4000
// #define TIM17_PERIOD    820

// private functions
static uint16_t find_best_count_value(uint32_t src_clk, uint16_t target_clk);

static bool busy_sending_pulses = false;

void timer_init(void)
{
  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM16);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM17);

  // set TIM17 to 39,024 Hz
  LL_TIM_StructInit(&TIM_InitStruct);
  TIM_InitStruct.Prescaler = 0;
  // autoreload value isn't known at init time. Will be defined later
  TIM_InitStruct.Autoreload = 0xffff;
  LL_TIM_Init(TIM17, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM17);

  LL_TIM_OC_StructInit(&TIM_OC_InitStruct);
  // Output mode is forced inactive currently. Will enable immediately before use
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_INACTIVE;
  // TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_ENABLE;
  // comparevalue isn't known at this time. Will be defined later so duty cycle is 50%
  TIM_OC_InitStruct.CompareValue = 0xffff;
  LL_TIM_OC_Init(TIM17, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_EnablePreload(TIM17, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_DisableFast(TIM17, LL_TIM_CHANNEL_CH1);
  
  LL_TIM_StructInit(&TIM_InitStruct);
  // Effective clock = input clock / (prescaler + 1)
  // Therefore, TIM16 prescaler is set to 31 so TIM16 takes precisely 1us to count each number
  TIM_InitStruct.Prescaler = 31;   
  // autoreload value isn't known at init time. Will be defined later
  TIM_InitStruct.Autoreload = 0xffff;
  LL_TIM_Init(TIM16, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM16);

  LL_TIM_OC_StructInit(&TIM_OC_InitStruct);
  // Output mode is forced inactive currently. Will enable immediately before use
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_INACTIVE;
  // TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_ENABLE;
  // comparevalue value isn't known at init time. Will be defined later
  TIM_OC_InitStruct.CompareValue = 0xffff;
  LL_TIM_OC_Init(TIM16, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_EnablePreload(TIM16, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_DisableFast(TIM16, LL_TIM_CHANNEL_CH1);  

  // TIM16 will send a DMA request on an update event (instead of when a capture/compare event happens)
  LL_TIM_CC_SetDMAReqTrigger(TIM16, LL_TIM_CCDMAREQUEST_UPDATE);
  // configure the DMA to set 3 sequential registers, starting with TIM16 auto-reload register, each time it is called
  LL_TIM_ConfigDMABurst(TIM16, LL_TIM_DMABURST_BASEADDR_ARR, LL_TIM_DMABURST_LENGTH_3TRANSFERS);
  // enable DMA requests when an update event occurs on TIM16
  LL_TIM_EnableDMAReq_UPDATE(TIM16);
  // set interrupt priority to 3. Lower numbers have higher priority
  NVIC_SetPriority(TIM16_IRQn, 3);
  // note TIM16 IRQ is not enabled at this point

    /*
    There is a bug where if TIM17.MOE is set before TIM16.MOE,
    IRTIM will output high until TIM17 counts to TIM17.CCR1 
  */
  LL_TIM_EnableAllOutputs(TIM16);
  LL_TIM_EnableAllOutputs(TIM17);
}

void dma_init(void)
{
  // turn on the DMA peripheral by enabling clocks to the DMA and DMA request MUX module
  LL_DMA_InitTypeDef DMA_InitStruct = {0};
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  // initialize DMA Channel 1 with the following settings:
  // peripheral address is TIM16 DMA register (dedicated register for handling DMA)
  DMA_InitStruct.PeriphOrM2MSrcAddress = (uint32_t)&(TIM16->DMAR);
  // memory address is unknown now, will be set later
  DMA_InitStruct.MemoryOrM2MDstAddress = 0;
  // DMA direction is going to move data from memory to the peripheral register
  DMA_InitStruct.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
  // DMA is going to be in normal (not circular) mode
  DMA_InitStruct.Mode = LL_DMA_MODE_NORMAL;
  // do not increment the peripheral address each time data is transferred to it
  DMA_InitStruct.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  // do increment the memory address each time data is read
  DMA_InitStruct.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  // write a halfword (16 bits) each time to the destination address
  DMA_InitStruct.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_HALFWORD;
  // read a halfword (16 bits) each time from the source address
  DMA_InitStruct.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_HALFWORD;
  // number of data transfers to perform is unknown currently. Will set later
  DMA_InitStruct.NbData = 0;
  // this DMA is triggered by an updated event in TIM16
  DMA_InitStruct.PeriphRequest = LL_DMAMUX_REQ_TIM16_UP;
  // this DMA transfer has medium priority (compared to other potential DMA transfers)
  DMA_InitStruct.Priority = LL_DMA_PRIORITY_MEDIUM;

  LL_DMA_Init(DMA1, LL_DMA_CHANNEL_1, &DMA_InitStruct);
  // enable DMA interrupt when transfer is complete
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
  // set interrupt priority to 2. Lower numbers have higher priority
  NVIC_SetPriority(DMA1_Channel1_IRQn, 2);
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

void send_pulses(uint16_t *pulse_array, uint32_t array_size)
{
  // point DMA channel 1 to the location of the pulse array data
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)pulse_array);
  // tell DMA channel 1 how long the data is
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, array_size);

  // update interrupt flag may be set; clear it 
  LL_TIM_ClearFlag_UPDATE(TIM16);

  // clear any pending DMA requests before enabling DMA channel
  LL_TIM_DisableDMAReq_UPDATE(TIM16);
  while(LL_TIM_IsEnabledDMAReq_UPDATE(TIM16));
  LL_TIM_EnableDMAReq_UPDATE(TIM16);

  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);


  // generate an update event. The DMA moves data to the preload registers
  LL_TIM_GenerateEvent_UPDATE(TIM16);

  // don't read TIM_EGR_UG here to determine when to trigger the next event
  // that does not indicate when you can trigger another event
  while(!(LL_TIM_IsActiveFlag_UPDATE(TIM16)));
  // generate a second update event. First batch of data moves into active registers
  LL_TIM_GenerateEvent_UPDATE(TIM16);

  // set the outputs active
  LL_TIM_OC_SetMode(TIM16, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
  LL_TIM_OC_SetMode(TIM17, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);

  // enable TIM16 and TIM17 so they start counting
  LL_TIM_EnableCounter(TIM16);
  LL_TIM_EnableCounter(TIM17);

  busy_sending_pulses = true;
}

void set_IR_frequency(uint16_t target_freq)
{
  // a target freq of 0 Hz is garbage, don't do anything
  if(target_freq == 0){
    return;
  }
  // get speed of clock going to TIM17
  LL_RCC_ClocksTypeDef clk_struct;
  LL_RCC_GetSystemClocksFreq(&clk_struct);

  uint32_t TM17_src_clk = clk_struct.PCLK2_Frequency;
  // find the count value that gets you closest to the target frequency
  uint16_t TIM17_period = find_best_count_value(TM17_src_clk, target_freq);
  // printfln_("Target Frequency is %d Hz, TIM17 will count to %d", target_freq, TIM17_period);
  // update TIM17 period and keep the duty cycle at 50%
  LL_TIM_SetAutoReload(TIM17, TIM17_period);
  LL_TIM_OC_SetCompareCH1(TIM17, TIM17_period/2);

  // generate an update event to move the new period and duty cycle from preload to shadow registers
  LL_TIM_GenerateEvent_UPDATE(TIM17);
}

// find the best count value for a timer to achieve an arbitrary target_clk
static uint16_t find_best_count_value(uint32_t src_clk, uint16_t target_clk)
{
  // if src_clk or target_clk is 0, that's a bad input so just return 0
  // same if target_clk is more than src_clk
  if((src_clk == 0) || (target_clk == 0) || (target_clk > src_clk)){
    return 0;
  }
  uint16_t count1 = src_clk / target_clk;

  uint32_t diff1 = (src_clk / count1) - target_clk;


  if(diff1 == 0){
      return count1;
  }else{
      uint16_t count2 = count1 + 1;
      if(count2 == 0){
        return count1;
      }
      uint32_t diff2 = abs_int((src_clk / count2) - target_clk);
      if(diff1 > diff2)
          return count2;
      else
          return count1;
  }
}

bool DMA_busy(void)
{
  return busy_sending_pulses;
}

void TIM16_IRQHandler(void)
{
  LL_TIM_ClearFlag_UPDATE(TIM16);

  // if the timer is enabled, that means we just finished sending the second-to-last pulse.
  // enable one pulse mode so the timer stops after the next pulse.
  if(LL_TIM_IsEnabledCounter(TIM16)){
    LL_TIM_SetOnePulseMode(TIM16, LL_TIM_ONEPULSEMODE_SINGLE);
  }else{  //if the timer is no longer enabled, that means the final pulse just finished.
    // disable one pulse mode
    LL_TIM_SetOnePulseMode(TIM16, LL_TIM_ONEPULSEMODE_REPETITIVE);
    // disable TIM16 interrupt
    NVIC_DisableIRQ(TIM16_IRQn);
    // disable TIM16 update interrupt flag setting
    LL_TIM_DisableIT_UPDATE(TIM16);
    // clear any update interrupt flags that may be set
    LL_TIM_ClearFlag_UPDATE(TIM16);
    // stop TIM17 from counting (TIM16 should already be stopped due to OPM)
    LL_TIM_DisableCounter(TIM17);
    // force the outputs low
    LL_TIM_OC_SetMode(TIM16, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_INACTIVE);
    LL_TIM_OC_SetMode(TIM17, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_INACTIVE);
    // disable TIM16 and TIM17 outputs
    // LL_TIM_DisableAllOutputs(TIM17);
    // LL_TIM_DisableAllOutputs(TIM16);
    busy_sending_pulses = false;
  }
}

void DMA1_Channel1_IRQHandler(void)
{
  // if this IRQ is called, the DMA transfer is completed
  // clear the DMA1 channel 1 global flag
  LL_DMA_ClearFlag_GI1(DMA1);
  // disable DMA1 channel 1
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
  // clear the update interrupt flag (if it's asserted)
  LL_TIM_ClearFlag_UPDATE(TIM16);
  // enable the update interrupt to flag to trigger TIM16 IRQ
  LL_TIM_EnableIT_UPDATE(TIM16);
  // enable TIM16 IRQ so there's an interrupt with the last pulse is sent
  NVIC_EnableIRQ(TIM16_IRQn);
}