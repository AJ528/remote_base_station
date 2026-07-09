#ifndef __UTILS_H
#define __UTILS_H

#include <stdint.h>
#include <stdbool.h>

#define NULL    ((void *) 0)

#define CHECK(x) do { \
  int32_t v = (x); \
  if(v < 0){ \
    return(v); \
  } \
} while(0)

#define COUNT_OF(arr) (sizeof(arr)/sizeof(arr[0]))

typedef struct {
  // data points to the block of memory where data is stored
  uint8_t *data;
  // size is the maximum size the buffer can hold
  uint32_t size;
  // writeIndex is the location where the next data is written
  uint32_t writeIndex;
  // readIndex is the location where data should be read from
  uint32_t readIndex;
  // overflow is true when you try to push too much data into the buffer
  bool overflow;
} ringBuf;

// Unlike the other string.h functions, memset cannot end in an underscore.
// This is so the gnu linker will find and call the function when needed.
void* memset(void *ptr, uint8_t value, uint32_t num);

int32_t strcmp_(const char *str1, const char *str2);
void* memmove_(void *destination, const void *source, uint32_t num);
void* memcpy_(void *destination, const void *source, uint32_t num);


uint8_t bufPop(ringBuf *buf);
int32_t bufPush(ringBuf *buf, uint8_t value);
bool bufIsEmpty(ringBuf *buf);

#endif /* __UTILS_H */
