/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
FLASH (rx) : ORIGIN = 0x0001F000, LENGTH = 0x00061000
RAM (rwx)  : ORIGIN = 0x200031B0, LENGTH = 0x0000CE50
}

INCLUDE "nrf5x_common.ld"
