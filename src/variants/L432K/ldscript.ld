
/* Linker script for STM32L432K Device with 256KByte FLASH, 64KByte RAM */

/* Specify the memory areas */
MEMORY
{
  RAM_DATA (xrw)  : ORIGIN = 0x20000000, LENGTH = 32K
  RAM_CODE (rx)   : ORIGIN = 0x20000000 + LENGTH(RAM_DATA), LENGTH = 64K - LENGTH(RAM_DATA)
  FLASH (rx)      : ORIGIN = 0x8000000 + FLASH_APP_OFFSET, LENGTH = FLASH_SIZE - FLASH_APP_OFFSET
}

INCLUDE "variants/ldscript_gen.ld"
