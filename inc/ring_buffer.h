#ifndef __RING_BUFFER_H
#define __RING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>

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

uint8_t bufPop(ringBuf *buf);
int32_t bufPush(ringBuf *buf, uint8_t value);
bool bufIsEmpty(ringBuf *buf);

#endif /* __RING_BUFFER_H */
