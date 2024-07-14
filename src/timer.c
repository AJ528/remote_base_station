#include "timer.h"

#include "pin_defs.h"

#include "stm32wlxx_ll_tim.h"


#include "stm32wlxx_ll_exti.h"
#include "stm32wlxx_ll_bus.h"
#include "stm32wlxx_ll_system.h"


void timer_init(void)
{
  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM16);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM17);

  // TIM_InitStruct.Prescaler = ;                                     
  // TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;                           
  // TIM_InitStruct.Autoreload = ;                                       
  // TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1; 

}