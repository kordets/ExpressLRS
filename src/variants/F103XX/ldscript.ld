

/* Linker script for STM32F103C(8-B)Tx Device with 32/64/128KByte FLASH, 10/20KByte RAM */

/* Specify the memory areas */
MEMORY
{
  RAM_CODE (rx)   : ORIGIN = 0x20000000, LENGTH = DEFINED(RAM_CODE_IGNORE) ? 0K : 0x2A00 /* 10.5K */
  RAM_DATA (xrw)  : ORIGIN = 0x20000000 + LENGTH(RAM_CODE), LENGTH = RAM_SIZE - LENGTH(RAM_CODE)
  FLASH (rx)      : ORIGIN = 0x8000000 + FLASH_APP_OFFSET, LENGTH = FLASH_SIZE - FLASH_APP_OFFSET
}

INCLUDE "variants/ldscript_gen.ld"
