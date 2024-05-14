

#include "stm32wlxx_ll_system.h"
#include "stm32wlxx_ll_pwr.h"
#include "stm32wlxx_ll_rcc.h"
#include "stm32wlxx_ll_utils.h"


void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
  {
  }

  /** Configure the main internal regulator output voltage
  */
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  while(LL_PWR_IsActiveFlag_VOS() == 1); // delay until VOS flag is 0

  LL_RCC_HSE_EnableTcxo();
  LL_RCC_HSE_Enable();

  // delay until HSE is ready
  while (LL_RCC_HSE_IsReady() == 0U)
  {
  }

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSE);

  // delay until HSE is system clock
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSE)
  {
  }

  LL_RCC_ClocksTypeDef clk_struct;

  LL_RCC_GetSystemClocksFreq(&clk_struct);
  LL_Init1msTick(clk_struct.HCLK1_Frequency);
}