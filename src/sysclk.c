

#include "stm32wlxx_ll_system.h"
#include "stm32wlxx_ll_pwr.h"
#include "stm32wlxx_ll_rcc.h"
#include "stm32wlxx_ll_utils.h"
#include "stm32wlxx_ll_cortex.h"

#include <stdint.h>

static uint32_t tick_count = 0;

void sysclk_init(void)
{
  //set up to run off the 32MHz high speed external clock
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
  {
  }

  // Configure the main internal regulator output voltage
  // set voltage scale to range 1, the high performance mode
  // this sets the internal main regulator to 1.2V and SYSCLK can be up to 64MHz
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  while(LL_PWR_IsActiveFlag_VOS() == 1); // delay until VOS flag is 0

  LL_RCC_HSE_EnableTcxo();
  LL_RCC_HSE_Enable();
  LL_RCC_HSE_EnableCSS();     // enable clock security to detect HSE failure

  // delay until HSE is ready
  while (LL_RCC_HSE_IsReady() == 0U)
  {
  }

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSE);

  // delay until HSE is system clock
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSE)
  {
  }

  // update the global variable SystemCoreClock to reflect the new core clock
  SystemCoreClockUpdate();

  // now that we are running off HSE32, enable the SMPS to improve power efficiency
  LL_PWR_SMPS_Enable();

  LL_RCC_ClocksTypeDef clk_struct;

  LL_RCC_GetSystemClocksFreq(&clk_struct);
  LL_Init1msTick(clk_struct.HCLK1_Frequency);

  LL_SYSTICK_EnableIT();
}

uint32_t get_tick(void)
{
  return tick_count;
}

void SysTick_Handler(void)
{
  tick_count++;
}