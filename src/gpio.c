
#include "gpio.h"

#include "pin_defs.h"

#include "stm32wlxx_ll_gpio.h"
#include "stm32wlxx_ll_exti.h"
#include "stm32wlxx_ll_bus.h"
#include "stm32wlxx_ll_system.h"


void GPIO_init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);

  /*Configure GPIO pin Output Level */
  LL_GPIO_ResetOutputPin(GPIOB, LED1_Pin|LED2_Pin|LED3_Pin);

  /*Configure GPIO pin Output Level */
  LL_GPIO_ResetOutputPin(GPIOC, RF_SW_CTRL3_Pin|RF_SW_CTRL2_Pin|RF_SW_CTRL1_Pin);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RF_SW_CTRL3_Pin RF_SW_CTRL2_Pin RF_SW_CTRL1_Pin */
  GPIO_InitStruct.Pin = RF_SW_CTRL3_Pin|RF_SW_CTRL2_Pin|RF_SW_CTRL1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_SW1_Pin BUTTON_SW2_Pin */
  GPIO_InitStruct.Pin = BUTTON_SW1_Pin|BUTTON_SW2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;

  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);
  LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_0);
  LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_0);

  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE1);
  LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_1);
  LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_1);

  LL_GPIO_Init(BUTTON_SW1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_SW3_Pin */
  GPIO_InitStruct.Pin = BUTTON_SW3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;

  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE6);
  LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_6);
  LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_6);

  LL_GPIO_Init(BUTTON_SW3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : T_VCP_RX_Pin T_VCP_TX_Pin */
  GPIO_InitStruct.Pin = T_VCP_RX_Pin|T_VCP_TX_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*
  While the LED2 pin is configured above, it gets reconfigured here to be the TIM17 output.
  This will let me verify I have everything configured correctly.
  */

  /*Reconfigure GPIO pins : LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
  // GPIO_InitStruct.Alternate = LL_GPIO_AF_14;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}