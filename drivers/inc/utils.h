#ifndef __UTILS_H
#define __UTILS_H

#include <stdint.h>

#define NULL    ((void *) 0)

#define CHECK(x) do { \
  int32_t v = (x); \
  if(v < 0){ \
    return(v); \
  } \
} while(0)

#define COUNT_OF(arr) (sizeof(arr)/sizeof(arr[0]))

// Unlike the other string.h functions, memset cannot end in an underscore.
// This is so the gnu linker will find and call the function when needed.
void* memset(void *ptr, uint8_t value, uint32_t num);

int32_t strcmp_(const char *str1, const char *str2);
void* memmove_(void *destination, const void *source, uint32_t num);
void* memcpy_(void *destination, const void *source, uint32_t num);

#endif /* __UTILS_H */
