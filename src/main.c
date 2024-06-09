
#include "sysclk.h"
#include "gpio.h"
#include "subghz.h"
#include "uart.h"

#include "stm32wlxx_ll_utils.h"
#include "stm32wlxx_ll_lpuart.h"

void Error_Handler(void);



int main(void)
{
  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  GPIO_init();
  UART_init();
  MX_SUBGHZ_Init();

  ConfigRFSwitch(RADIO_SWITCH_RX);

  // continuous_rx();

  while (1)
  {
    // subghz_radio_getstatus();
    single_rx_blocking();
  	LL_mDelay(500);
  }
}

int32_t putchar_(char c)
{
  // loop while the LPUART_TDR register is full
  while(LL_LPUART_IsActiveFlag_TXE_TXFNF(LPUART1) != 1);
  // once the LPUART_TDR register is empty, fill it with char c
  LL_LPUART_TransmitData8(LPUART1, (uint8_t)c);
  return (c);
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
