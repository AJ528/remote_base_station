#include "uart.h"

#include "stm32wlxx_ll_bus.h"
#include "stm32wlxx_ll_rcc.h"
#include "stm32wlxx_ll_lpuart.h"

void UART_init(void)
{
    // set the LPUART clock source to the peripheral clock
    LL_RCC_SetLPUARTClockSource(LL_RCC_LPUART1_CLKSOURCE_PCLK1);

    // enable clocks for LPUART
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_LPUART1);

    LL_LPUART_InitTypeDef LPUART_InitStruct = {
        .PrescalerValue = LL_LPUART_PRESCALER_DIV1,
        .BaudRate = 115200,
        .DataWidth = LL_LPUART_DATAWIDTH_8B,
        .StopBits = LL_LPUART_STOPBITS_1,
        .Parity = LL_LPUART_PARITY_NONE,
        .TransferDirection = LL_LPUART_DIRECTION_TX_RX,
        .HardwareFlowControl = LL_LPUART_HWCONTROL_NONE
    };
    LL_LPUART_Init(LPUART1, &LPUART_InitStruct);
    LL_LPUART_Enable(LPUART1);

    // wait for the LPUART module to send an idle frame and finish initialization
    while(!(LL_LPUART_IsActiveFlag_TEACK(LPUART1)) || !(LL_LPUART_IsActiveFlag_REACK(LPUART1)));
}