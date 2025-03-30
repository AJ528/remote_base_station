
#include "sysclk.h"
#include "gpio.h"
#include "subghz.h"
#include "uart.h"
#include "timer.h"
#include "IR_lib.h"
#include "cmd_assoc_structs.h"

#include "mprintf.h"

#include "pin_defs.h"
#include "stm32wlxx_ll_gpio.h"

#include "stm32wlxx_ll_utils.h"
#include "stm32wlxx_ll_lpuart.h"

#include <stdbool.h>

void Error_Handler(void);


int main(void)
{
  /* initialize the subghz module so the voltage of VDD_TCXO can be adjusted */
  subghz_init();

  /* Configure the system clock to run off HSE32 */
  sysclk_init();

  /* Initialize all configured peripherals */
  GPIO_init();
  UART_init();
  dma_init();
  timer_init();

  /* Configure the SUBGHZ module to listen for commands */
  // subghz_config();

  printf_("about to execute loop!\n");
  execute_command(&SB_PWR_TOG, false);


#if (RX_MODE == 1)
  // continuous_rx();

  while (1)
  {
    // subghz_radio_getstatus();
    // single_rx_blocking();
    LL_GPIO_TogglePin(STATUS_LED_PORT, STATUS_LED_PIN);
  	LL_mDelay(1000);

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
