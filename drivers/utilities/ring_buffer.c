#include "utils.h"

#include <stdint.h>




// this function pushes a byte of data into a ring buffer
// it returns 0 if successful, otherwise -1
int32_t bufPush(ringBuf *buf, uint8_t value)
{
  // assuming the buffer size is a power of 2, 
  // modulus isn't needed to wrap the index
  uint32_t nextWI = (buf->writeIndex + 1) & (buf->size - 1);
  // as long as the next write index isn't the read index
  if(nextWI != buf->readIndex){
    // write the value and increment the write index
    buf->data[buf->writeIndex] = value;
    buf->writeIndex = nextWI;
    return (0);
  }else{  // if the next write index is the read index, buffer is full
    return(-1);
  }
}

// this function pops a byte of data out of the ring buffer
// if the buffer has data, it returns the value at the read index
// if the buffer is empty, it returns 0
uint8_t bufPop(ringBuf *buf)
{
  // initialize the return value with a default value of 0
  uint8_t retval = 0;
  // if the read and write index don't match, the buffer contains data
  if(buf->readIndex != buf->writeIndex){
    // retrieve the data from the read index
    retval = buf->data[buf->readIndex];
    // increment the read index
    // assuming the buffer size is a power of 2, 
    // modulus isn't needed to wrap the index
    buf->readIndex = (buf->readIndex + 1) & (buf->size - 1);
  }
  // regardless of if the buffer is empty, return retval
  return(retval);
}

// this function returns true if the buffer is empty and false if not
bool bufIsEmpty(ringBuf *buf)
{
  // buffer is empty if the read index matches the write index
  return((buf->readIndex) == (buf->writeIndex));
}
