
#include "stm32wlxx.h"

#include <stdint.h>

// TODO: make this better

struct boot_vectable_ {
    uint32_t Initial_SP;
    void (*Reset_Handler)(void);
};

#define BOOTVTAB	((struct boot_vectable_ *)0x1fff0000)

int32_t update_cmd(uint32_t argc, char* argv[])
{
  (void)argc;
  (void)argv;

  // call the system bootloader function (does not return)
  __set_MSP(BOOTVTAB->Initial_SP);
  BOOTVTAB->Reset_Handler();
}