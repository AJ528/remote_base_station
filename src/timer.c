#include "timer.h"
#include "pin_defs.h"
#include "mprintf.h"

#include "stm32wlxx.h"
#include "stm32wlxx_ll_tim.h"

#include "stm32wlxx_ll_bus.h"
#include "stm32wlxx_ll_dma.h"
#include "stm32wlxx_ll_gpio.h"

#include <stdint.h>
#include <stdbool.h>


#define TIM16_PERIOD    0x2000
#define TIM17_PERIOD    0x0800

static bool busy_sending_pulses = false;

// uint16_t dma_data_arr[] = {
//   TIM17_PERIOD, 0, TIM17_PERIOD/2,
//   TIM17_PERIOD, 0, TIM17_PERIOD/2,
//   TIM17_PERIOD, 0, TIM17_PERIOD/4,
//   TIM17_PERIOD, 0, TIM17_PERIOD/32
// };
// uint32_t arr_size = sizeof(dma_data_arr)/sizeof(dma_data_arr[0]);



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
  LL_TIM_EnableARRPreload(TIM17);

  LL_TIM_OC_StructInit(&TIM_OC_InitStruct);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_ENABLE;
  TIM_OC_InitStruct.CompareValue = TIM17_PERIOD / 2;
  LL_TIM_OC_Init(TIM17, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_EnablePreload(TIM17, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_DisableFast(TIM17, LL_TIM_CHANNEL_CH1);

  LL_TIM_CC_SetDMAReqTrigger(TIM17, LL_TIM_CCDMAREQUEST_UPDATE);
  LL_TIM_ConfigDMABurst(TIM17, LL_TIM_DMABURST_BASEADDR_ARR, LL_TIM_DMABURST_LENGTH_3TRANSFERS);

  LL_TIM_EnableDMAReq_UPDATE(TIM17);
  NVIC_SetPriority(TIM17_IRQn, 3);

  LL_TIM_EnableAllOutputs(TIM17);
  
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
  DMA_InitStruct.PeriphOrM2MSrcAddress = (uint32_t)&(TIM17->DMAR);
  DMA_InitStruct.MemoryOrM2MDstAddress = 0;   //unknown now, will set later
  DMA_InitStruct.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
  DMA_InitStruct.Mode = LL_DMA_MODE_NORMAL;
  DMA_InitStruct.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  DMA_InitStruct.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  DMA_InitStruct.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_HALFWORD;
  DMA_InitStruct.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_HALFWORD;
  DMA_InitStruct.NbData = 0;    //unknown now, will set later
  DMA_InitStruct.PeriphRequest = LL_DMAMUX_REQ_TIM17_UP;
  DMA_InitStruct.Priority = LL_DMA_PRIORITY_MEDIUM;

  LL_DMA_Init(DMA1, LL_DMA_CHANNEL_1, &DMA_InitStruct);
  
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
  NVIC_SetPriority(DMA1_Channel1_IRQn, 2);
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

void send_pulses(uint16_t *pulse_array, uint32_t array_size)
{
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)pulse_array);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, array_size);

  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

  LL_TIM_GenerateEvent_UPDATE(TIM17);
  while(READ_BIT(TIM17->EGR, TIM_EGR_UG));
  LL_TIM_GenerateEvent_UPDATE(TIM17);

  LL_TIM_EnableCounter(TIM17);

  busy_sending_pulses = true;
}

bool DMA_busy(void)
{
  return busy_sending_pulses;
}

void TIM17_IRQHandler(void)
{
  LL_TIM_ClearFlag_UPDATE(TIM17);

  // if the timer is enabled, that means we just finished sending the second-to-last pulse.
  // enable one pulse mode so the timer stops after the next pulse.
  if(LL_TIM_IsEnabledCounter(TIM17)){
    LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);
    LL_TIM_SetOnePulseMode(TIM17, LL_TIM_ONEPULSEMODE_SINGLE);
  }else{  //if the timer is no longer enabled, that means the final pulse just finished.
    LL_TIM_SetOnePulseMode(TIM17, LL_TIM_ONEPULSEMODE_REPETITIVE);
    NVIC_DisableIRQ(TIM17_IRQn);
    LL_TIM_DisableIT_UPDATE(TIM17);
    LL_TIM_ClearFlag_UPDATE(TIM17);

    busy_sending_pulses = false;
    
    // NVIC_ClearPendingIRQ(TIM17_IRQn);
    
  }


}

void DMA1_Channel1_IRQHandler(void)
{
  LL_DMA_ClearFlag_GI1(DMA1);
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);

  LL_GPIO_SetOutputPin(LED1_GPIO_Port, LED1_Pin);

  LL_TIM_ClearFlag_UPDATE(TIM17);
  LL_TIM_EnableIT_UPDATE(TIM17);
  // NVIC_ClearPendingIRQ(TIM17_IRQn);
  NVIC_EnableIRQ(TIM17_IRQn);

}