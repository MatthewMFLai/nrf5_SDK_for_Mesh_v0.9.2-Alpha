/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
FLASH (rx) : ORIGIN = 0x0001B000, LENGTH = 0x0001F000
RAM (rwx)  : ORIGIN = 0x200022D8, LENGTH = 0x00005A28
}

INCLUDE "nrf5x_common.ld"
