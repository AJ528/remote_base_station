#ifndef __PIN_DEFS_H
#define __PIN_DEFS_H

#define STATUS_LED_PIN                  LL_GPIO_PIN_1
#define STATUS_LED_PORT                 GPIOA
#define STATUS_LED_CTRL0_PIN            LL_GPIO_PIN_8
#define STATUS_LED_CTRL1_PIN            LL_GPIO_PIN_7
#define STATUS_LED_CTRLN_PORT           GPIOB

#define IR_MOD_PIN                      LL_GPIO_PIN_6
#define IR_CARRIER_PIN                  LL_GPIO_PIN_7
#define IR_SIGNALS_PORT                 GPIOA

#define UART_TX_PIN                     LL_GPIO_PIN_2
#define UART_RX_PIN                     LL_GPIO_PIN_3
#define UART_PORT                       GPIOA


#define LED1_Pin                  LL_GPIO_PIN_15
#define LED1_GPIO_Port            GPIOB
#define LED2_Pin                  LL_GPIO_PIN_9
#define LED2_GPIO_Port            GPIOB
#define RF_SW_CTRL3_Pin           LL_GPIO_PIN_3
#define RF_SW_CTRL3_GPIO_Port     GPIOC
#define BUTTON_SW1_Pin            LL_GPIO_PIN_0
#define BUTTON_SW1_GPIO_Port      GPIOA
#define RF_SW_CTRL2_Pin LL_GPIO_PIN_5
#define RF_SW_CTRL2_GPIO_Port GPIOC
#define RF_SW_CTRL1_Pin LL_GPIO_PIN_4
#define RF_SW_CTRL1_GPIO_Port GPIOC
#define BUTTON_SW3_Pin LL_GPIO_PIN_6
#define BUTTON_SW3_GPIO_Port GPIOC
#define BUTTON_SW2_Pin LL_GPIO_PIN_1
#define BUTTON_SW2_GPIO_Port GPIOA
#define LED3_Pin LL_GPIO_PIN_11
#define LED3_GPIO_Port GPIOB
#define T_VCP_RX_Pin LL_GPIO_PIN_3
#define T_VCP_RX_GPIO_Port GPIOA
#define T_VCP_TX_Pin LL_GPIO_PIN_2
#define T_VCP_TX_GPIO_Port GPIOA


#endif /* __PIN_DEFS_H */
