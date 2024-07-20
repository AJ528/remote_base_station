#include "timer.h"

// #include "pin_defs.h"

#include "stm32wlxx_ll_tim.h"


#include "stm32wlxx_ll_exti.h"
#include "stm32wlxx_ll_bus.h"
#include "stm32wlxx_ll_system.h"


void timer_init(void)
{
  /*initial attempt just to verify I can get the timers configured correctly*/

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM16);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM17);

  uint32_t TIM17_period = 0x1000;
  // uint32_t TIM17_period = 820;
  //set TIM17 to 39,024 Hz
  //TODO: verify the frequency is not 38,976 Hz
  TIM_InitStruct.Prescaler = 0x1000;                                     
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;                           
  TIM_InitStruct.Autoreload = TIM17_period;                                       
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1; 
  LL_TIM_Init(TIM17, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM17);

  LL_TIM_OC_StructInit(&TIM_OC_InitStruct);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_ENABLE;
  TIM_OC_InitStruct.CompareValue = TIM17_period / 3;
  LL_TIM_OC_Init(TIM17, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_EnablePreload(TIM17, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_DisableFast(TIM17, LL_TIM_CHANNEL_CH1);

  LL_TIM_CC_EnableChannel(TIM17, LL_TIM_CHANNEL_CH1);
  LL_TIM_EnableCounter(TIM17);
  LL_TIM_GenerateEvent_UPDATE(TIM17);
}