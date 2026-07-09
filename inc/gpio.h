

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_H
#define __GPIO_H

typedef enum {
  GREEN,
  YELLOW,
  RED
} statusLEDColor;

// typedef enum {
//   OFF,
//   BLINK,
//   ON
// } statusLEDPattern;

void GPIO_init(void);
void GPIO_IR_Pins_Enable(void);
void GPIO_IR_Pins_Disable(void);

void toggle_status_LED(void);
void GPIO_set_status_LED(statusLEDColor color);
// void GPIO_set_status_LED(statusLEDColor color, statusLEDPattern pattern);
// void GPIO_handle_status_LED(void);

#endif /* __GPIO_H */