#ifndef __SYSCLK_H
#define __SYSCLK_H

#include <stdint.h>

void sysclk_init(void);
uint32_t get_tick(void);

#endif /* __SYSCLK_H */
