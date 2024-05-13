#include "stm32wlxx_hal.h"

// #include "core_cm4.h"
#include "subghz.h"
#include "mprintf.h"

#include "stm32wlxx_ll_system.h"
#include "stm32wlxx_ll_pwr.h"
#include "stm32wlxx_ll_rcc.h"
#include "stm32wlxx_ll_bus.h"


#define LED1_Pin GPIO_PIN_15
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_9
#define LED2_GPIO_Port GPIOB
#define RF_SW_CTRL3_Pin GPIO_PIN_3
#define RF_SW_CTRL3_GPIO_Port GPIOC
#define BUTTON_SW1_Pin GPIO_PIN_0
#define BUTTON_SW1_GPIO_Port GPIOA
#define RF_SW_CTRL2_Pin GPIO_PIN_5
#define RF_SW_CTRL2_GPIO_Port GPIOC
#define RF_SW_CTRL1_Pin GPIO_PIN_4
#define RF_SW_CTRL1_GPIO_Port GPIOC
#define BUTTON_SW3_Pin GPIO_PIN_6
#define BUTTON_SW3_GPIO_Port GPIOC
#define BUTTON_SW2_Pin GPIO_PIN_1
#define BUTTON_SW2_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_11
#define LED3_GPIO_Port GPIOB
#define T_VCP_RX_Pin GPIO_PIN_3
#define T_VCP_RX_GPIO_Port GPIOA
#define T_VCP_RXA2_Pin GPIO_PIN_2
#define T_VCP_RXA2_GPIO_Port GPIOA



SUBGHZ_HandleTypeDef hsubghz;


uint8_t RadioResult = 0x00;

void Error_Handler(void);
void MX_SUBGHZ_Init(void);

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_NVIC_Init(void);

void HAL_SUBGHZ_MspInit(SUBGHZ_HandleTypeDef* hsubghz);
void HAL_SUBGHZ_MspDeInit(SUBGHZ_HandleTypeDef* hsubghz);



int main(void)
{

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SUBGHZ_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();

  ConfigRFSwitch(RADIO_SWITCH_RX);

  RadioResult = 0x00;
  HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_STATUS, &RadioResult, 1);
  printf_("RR: 0x%02x\n", RadioResult);

  continuous_rx(&hsubghz);

  while (1)
  {
	RadioResult = 0x00;
	HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_STATUS, &RadioResult, 1);
  	printf_("RR: 0x%02x\n", RadioResult);
  	HAL_Delay(1000);

  }
}

void SystemClock_Config(void)
{
  // RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  // RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  // SET_BIT(PWR->CR1, PWR_CR1_DBP);
  // LL_RCC_LSE_SetDriveCapability(RCC_LSEDRIVE_LOW);

    LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
  {
  }

  /** Configure the main internal regulator output voltage
  */
  // __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
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


  // /** Initializes the CPU, AHB and APB buses clocks
  // */
  // RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  // RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  // RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  // RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  // RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  // RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  // if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  // {
  //   Error_Handler();
  // }

  // /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  // */
  // RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
  //                             |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
  //                             |RCC_CLOCKTYPE_PCLK2;
  // RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  // RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  // RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  // RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  // RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  // if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  // {
  //   Error_Handler();
  // }

}

void HAL_SUBGHZ_MspInit(SUBGHZ_HandleTypeDef* hsubghz)
{
    // __HAL_RCC_SUBGHZSPI_CLK_ENABLE();
    LL_APB3_GRP1_EnableClock(LL_APB3_GRP1_PERIPH_SUBGHZSPI);
    LL_RCC_HSE_EnableTcxo();
    LL_RCC_HSE_Enable();

    while (LL_RCC_HSE_IsReady() == 0)
    {}
}

void HAL_SUBGHZ_MspDeInit(SUBGHZ_HandleTypeDef* hsubghz)
{
    // __HAL_RCC_SUBGHZSPI_CLK_DISABLE();
    LL_APB3_GRP1_DisableClock(LL_APB3_GRP1_PERIPH_SUBGHZSPI);
    NVIC_DisableIRQ(SUBGHZ_Radio_IRQn);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* SUBGHZ_Radio_IRQn interrupt configuration */
  NVIC_SetPriority(SUBGHZ_Radio_IRQn, 0);

  NVIC_EnableIRQ(SUBGHZ_Radio_IRQn);
}

/**
  * @brief SUBGHZ Initialization Function
  * @param None
  * @retval None
  */
void MX_SUBGHZ_Init(void)
{
  hsubghz.Init.BaudratePrescaler = SUBGHZSPI_BAUDRATEPRESCALER_8;
  if (HAL_SUBGHZ_Init(&hsubghz) != HAL_OK)
  {
    Error_Handler();
  }
  if(subghz_init(&hsubghz) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RF_SW_CTRL3_Pin|RF_SW_CTRL2_Pin|RF_SW_CTRL1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RF_SW_CTRL3_Pin RF_SW_CTRL2_Pin RF_SW_CTRL1_Pin */
  GPIO_InitStruct.Pin = RF_SW_CTRL3_Pin|RF_SW_CTRL2_Pin|RF_SW_CTRL1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_SW1_Pin BUTTON_SW2_Pin */
  GPIO_InitStruct.Pin = BUTTON_SW1_Pin|BUTTON_SW2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_SW3_Pin */
  GPIO_InitStruct.Pin = BUTTON_SW3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_SW3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : T_VCP_RX_Pin T_VCP_RXA2_Pin */
  GPIO_InitStruct.Pin = T_VCP_RX_Pin|T_VCP_RXA2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

int32_t putchar_(char c)
{
  ITM_SendChar((uint32_t)c);

  return (uint32_t)c;
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */
