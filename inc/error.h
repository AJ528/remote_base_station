#ifndef __ERROR_H
#define __ERROR_H

#include <stdint.h>

#define ERROR_CHECK(x) do { \
  int32_t v = (x); \
  if(v != 0){ \
    handle_error(v); \
  } \
} while(0)


void handle_error(int32_t err_num);



#endif /* __ERROR_H */
