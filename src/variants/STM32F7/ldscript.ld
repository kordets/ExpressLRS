
/* Linker script for STM32F722RE Device with 512KByte FLASH, 256KByte RAM */

/* Specify the memory areas */
MEMORY
{
  RAM_CODE (rwx)  : ORIGIN = 0x20000000, LENGTH = 128K
  RAM_DATA (rwx)  : ORIGIN = 0x20000000 + LENGTH(RAM_CODE), LENGTH = 256K - LENGTH(RAM_CODE)
  FLASH (rx)      : ORIGIN = 0x8000000, LENGTH = 512K
}

INCLUDE "variants/ldscript_gen.ld"
