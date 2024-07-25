#include "timer.h"

// #include "pin_defs.h"

#include "stm32wlxx_ll_tim.h"

#include "stm32wlxx_ll_bus.h"
#include "stm32wlxx_ll_dma.h"
#include <stdint.h>


#define TIM16_PERIOD    0x2000
#define TIM17_PERIOD    0x0800


void timer_init(void)
{
  /*initial attempt just to verify I can get the timers configured correctly*/

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM16);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM17);

  // uint32_t TIM17_period = 0x0800;
  // uint32_t TIM17_period = 820;
  //set TIM17 to 39,024 Hz
  //TODO: verify the frequency is not 38,976 Hz
  TIM_InitStruct.Prescaler = 0x4000;                                     
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;                           
  TIM_InitStruct.Autoreload = TIM17_PERIOD;                                       
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1; 
  LL_TIM_Init(TIM17, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM17);

  LL_TIM_OC_StructInit(&TIM_OC_InitStruct);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_ENABLE;
  TIM_OC_InitStruct.CompareValue = 4 * TIM17_PERIOD / 2;
  LL_TIM_OC_Init(TIM17, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisablePreload(TIM17, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_DisableFast(TIM17, LL_TIM_CHANNEL_CH1);

  NVIC_SetPriority(TIM17_IRQn, 2);
  NVIC_EnableIRQ(TIM17_IRQn);

  LL_TIM_EnableIT_UPDATE(TIM17);

  LL_TIM_EnableCounter(TIM17);
  LL_TIM_EnableAllOutputs(TIM17);

  // uint32_t TIM16_period = 0x2000;
  TIM_InitStruct.Prescaler = 0x2000;                                     
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;                           
  TIM_InitStruct.Autoreload = TIM16_PERIOD;                                       
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1; 
  LL_TIM_Init(TIM16, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM16);

  LL_TIM_OC_StructInit(&TIM_OC_InitStruct);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_ENABLE;
  TIM_OC_InitStruct.CompareValue = TIM16_PERIOD / 2;
  LL_TIM_OC_Init(TIM16, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_EnablePreload(TIM16, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_DisableFast(TIM16, LL_TIM_CHANNEL_CH1);  

  LL_TIM_EnableCounter(TIM16);
  LL_TIM_EnableAllOutputs(TIM16);
}

void dma_init(void)
{
  LL_DMA_InitTypeDef DMA_InitStruct = {0};
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  // DMA_InitStruct.PeriphOrM2MSrcAddress = TIM16_BASE;
  DMA_InitStruct.PeriphOrM2MSrcAddress = (uint32_t)&(TIM16->DMAR);
  // DMA_InitStruct.MemoryOrM2MDstAddress = TBD;
  DMA_InitStruct.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
  DMA_InitStruct.Mode = LL_DMA_MODE_NORMAL;
  DMA_InitStruct.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  DMA_InitStruct.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  DMA_InitStruct.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_HALFWORD;
  DMA_InitStruct.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_HALFWORD;
  // DMA_InitStruct.NbData = TBD;
  DMA_InitStruct.PeriphRequest = LL_DMAMUX_REQ_TIM16_UP;
  DMA_InitStruct.Priority = LL_DMA_PRIORITY_MEDIUM;

}

uint16_t duty_cycle_arr[] = {TIM17_PERIOD/2, TIM17_PERIOD/4, TIM17_PERIOD/8, TIM17_PERIOD/16, TIM17_PERIOD/32};
uint32_t arr_size = sizeof(duty_cycle_arr)/sizeof(duty_cycle_arr[0]);

void TIM17_IRQHandler(void)
{
  LL_TIM_ClearFlag_UPDATE(TIM17);

  static uint32_t i = 0;

  LL_TIM_OC_SetCompareCH1(TIM17, duty_cycle_arr[i]);

  if((i+1) != arr_size){
    i++;
  }else{
    LL_TIM_SetOnePulseMode(TIM17, LL_TIM_ONEPULSEMODE_SINGLE);
  }

}