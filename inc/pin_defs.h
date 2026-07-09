#ifndef __PIN_DEFS_H
#define __PIN_DEFS_H

#define STATUS_LED_PIN                  LL_GPIO_PIN_1
#define STATUS_LED_PORT                 GPIOA
#define STATUS_LED_CTRL0_PIN            LL_GPIO_PIN_8
#define STATUS_LED_CTRL1_PIN            LL_GPIO_PIN_7
#define STATUS_LED_CTRLN_PORT           GPIOB
// [CTRL1, CTRL0] | Color
//          [0,0] | Nothing
//          [0,1] | Green
//          [1,0] | Yellow
//          [1,1] | Red

#define IR_MOD_PIN                      LL_GPIO_PIN_6
#define IR_CARRIER_PIN                  LL_GPIO_PIN_7
#define IR_SIGNALS_PORT                 GPIOA

#define UART_TX_PIN                     LL_GPIO_PIN_2
#define UART_RX_PIN                     LL_GPIO_PIN_3
#define UART_PORT                       GPIOA

#define RF_SWITCH                       LL_GPIO_PIN_9
#define RF_SWITCH_PORT                  GPIOA

#endif /* __PIN_DEFS_H */
