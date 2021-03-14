
/* Linker script for STM32F722RE Device with 512KByte FLASH, 256KByte RAM */

/* Specify the memory areas */
MEMORY
{
  RAM_DATA (xrw)  : ORIGIN = 0x20000000, LENGTH = 128K
  RAM_CODE (rx)   : ORIGIN = 0x20000000 + LENGTH(RAM_DATA), LENGTH = 128K
  FLASH (rx)      : ORIGIN = 0x8000000, LENGTH = 512K
}

INCLUDE "variants/ldscript_gen.ld"
