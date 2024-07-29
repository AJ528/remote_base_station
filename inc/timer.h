#ifndef __TIMER_H
#define __TIMER_H

#include <stdint.h>
#include <stdbool.h>

void timer_init(void);
void dma_init(void);
void send_pulses(uint16_t *pulse_array, uint32_t array_size);
bool DMA_busy(void);

#endif /* __TIMER_H */
