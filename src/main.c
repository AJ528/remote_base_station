
#include "sysclk.h"
#include "gpio.h"
#include "subghz.h"

#include "stm32wlxx_ll_utils.h"

void Error_Handler(void);



int main(void)
{
  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  GPIO_init();
  MX_SUBGHZ_Init();

  ConfigRFSwitch(RADIO_SWITCH_RX);

  continuous_rx();

  while (1)
  {
    subghz_radio_getstatus();
  	LL_mDelay(1000);

  }
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
