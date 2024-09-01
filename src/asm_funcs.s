
.syntax unified
.thumb

  .section .text.abs_int
  .type abs_int, STT_FUNC
  // uint32_t abs_int(int32_t num)
  // returns the absolute values of num
abs_int:
  movs  r1, r0        // stores input argument (r0) into r1 and updates flags N and Z
  it    mi            // if the result is negative (mi = minus)
  rsbmi r0, r1, #0    // then subtract the value in r1 from 0 and store result in r0
  bx    lr            // return from subroutine
