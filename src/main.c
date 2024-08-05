
#include "sysclk.h"
#include "gpio.h"
#include "subghz.h"
#include "uart.h"
#include "timer.h"
#include "IR_lib.h"

#include "pin_defs.h"
#include "stm32wlxx_ll_gpio.h"

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
  dma_init();
  timer_init();
  GPIO_IR_OUT_init();

  send_command();

#if (RX_MODE == 1)
  // continuous_rx();

  while (1)
  {
    // subghz_radio_getstatus();
    single_rx_blocking();
  	LL_mDelay(500);

  }
#endif

#if (TX_MODE == 1)
  uint8_t i = 0;

  while (1)
  {
    subghz_write_tx_buffer(i++);
    tx_packet();
    LL_mDelay(100);
    subghz_radio_getstatus();
    LL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
  	LL_mDelay(1000);
  }

#endif
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
