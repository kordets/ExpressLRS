
/* Linker script for STM32L432K Device with 256KByte FLASH, 48KByte RAM1 + 16KByte RAM2 */

/* Specify the memory areas */
MEMORY
{
  /* SRAM1 */
  RAM_CODE (rx)   : ORIGIN = 0x20000000, LENGTH = 24K
  RAM_DATA (rwx)  : ORIGIN = 0x20000000 + LENGTH(RAM_CODE), LENGTH = 48K - LENGTH(RAM_CODE)
  /* SRAM2 with hardware parity check */
  SRAM2 (rwx)     : ORIGIN = 0x2000C000, LENGTH = 16K
  FLASH (rx)      : ORIGIN = 0x8000000 + FLASH_APP_OFFSET, LENGTH = FLASH_SIZE - FLASH_APP_OFFSET
}

REGION_ALIAS("DMA_DATA", RAM_DATA)

INCLUDE "variants/ldscript_gen.ld"
