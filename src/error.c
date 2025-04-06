
#include "error.h"
#include "mprintf.h"

#include "stm32wlxx.h"


void handle_error(int32_t err_num)
{
  __disable_irq();
  println_("ERROR!");

  while (1)
  {
  }

}