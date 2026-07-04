
#include "sysclk.h"
#include "gpio.h"
#include "subghz.h"
#include "uart.h"
#include "mcli.h"
#include "timer.h"
#include "IR_lib.h"
#include "cmd_assoc_structs.h"

// #include "mprintf.h"

#include "stm32wlxx_ll_utils.h"
#include "stm32wlxx_ll_lpuart.h"

#include <stdint.h>
#include <stdbool.h>

static inline void enable_cycle_count(void);

extern uint32_t _vector_table_offset;

int main(void)
{
  SCB->VTOR = (uint32_t)(&_vector_table_offset);  // set the vector table offset
  enable_cycle_count();
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
  subghz_config();


#if (RX_MODE == 1)
  execute_command(&LHTV_VLUP, false);
  // RX mode one-time set up
  continuous_rx_enable();
#endif

#if (TX_MODE == 1)
  // TX mode one-time set up
  // uint8_t i = 0;
  // uint32_t ref_time = get_tick();
  // const uint32_t delay_ms_time = 1000;

  subghz_write_tx_buffer((uint8_t[]){0x02, 0x07, 0x07, 0x07}, 4);
  tx_packet();
  toggle_status_LED();
#endif

// infinite loop
  while(1){

    // if UART data is present, receive it
    // TODO: trigger this off an interrupt?
    if(LL_LPUART_IsActiveFlag_RXNE_RXFNE(LPUART1)){
      char c = (char)LL_LPUART_ReceiveData8(LPUART1);
      cli_input(c);
    }
    cli_process();

#if (RX_MODE == 1)
  // continuous RX commands
    handle_RF_command_buffer();
#endif

#if (TX_MODE == 1)
  // continuous TX commands
    // uint32_t current_time = get_tick();

    // if(current_time - ref_time >= delay_ms_time){
    //   ref_time = current_time;
    //   subghz_write_tx_buffer(&i, 1);
    //   i++;
    //   tx_packet();
    //   toggle_status_LED();
    // }
#endif

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

static inline void enable_cycle_count(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->LSUCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk | DWT_CTRL_LSUEVTENA_Msk;
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
