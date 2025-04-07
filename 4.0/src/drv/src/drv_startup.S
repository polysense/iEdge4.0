  .syntax unified
  .cpu cortex-m0plus
  .fpu softvfp
  .thumb

.global g_hdrVectors

/* start address for the initialization values of the .data section.
defined in linker script */
.word _sidata
/* start address for the .data section. defined in linker script */
.word _sdata
/* end address for the .data section. defined in linker script */
.word _edata
/* start address for the .bss section. defined in linker script */
.word _sbss
/* end address for the .bss section. defined in linker script */
.word _ebss

  .section .text.main_start
  .weak main_start
  .type main_start, %function
main_start:
  bx lr
.size main_start, .-main_start

   .section .hdr_vector,"a",%progbits
  .type g_hdrVectors, %object
  .size g_hdrVectors, .-g_hdrVectors

g_hdrVectors:
  .word 0x8274ef37 /* magic word */
  .word main_start
  .word g_export
  .word 0

  .word _estack
  .word _sidata
  .word _sdata
  .word _edata

  .word _sbss
  .word _ebss
  .word 0
  .word 0

