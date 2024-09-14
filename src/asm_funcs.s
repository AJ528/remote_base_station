
.syntax unified
.thumb

  .section .text.abs_int
  .global abs_int
  .type abs_int, STT_FUNC
  // uint32_t abs_int(int32_t num)
  // returns the absolute values of num
abs_int:
  movs  r1, r0        // stores input argument (r0) into r1 and updates flags N and Z
  it    mi            // if the result is negative (mi = minus)
  rsbmi r0, r1, #0    // then subtract the value in r1 from 0 and store result in r0
  bx    lr            // return from subroutine


  .section .text.rev_bit
  .global rev_bit
  .type rev_bit, STT_FUNC
  // uint32_t rev_bit(uint32_t num, uint32_t bitlen)
  // reverses the bits in a 32-bit number, then right-shifts the number until the
  // new LSB is in bit position 0.
rev_bit:
  rbit  r2, r0        // reverses the bits of the number in r0 and stores the result in r2
  rsb   r1, #32       // subtract the bitlen from 32 to get the number of LSRs to do
  lsr   r0, r2, r1    // LSR the number in r2 by r1 and store the result in r0
  bx    lr            // return from subroutine

  