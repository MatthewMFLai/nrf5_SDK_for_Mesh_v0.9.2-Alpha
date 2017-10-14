/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
FLASH (rx) : ORIGIN = 0x00024000, LENGTH = 0x000DC000 /* Reserve 0x8000 for the bootloader */
RAM (rwx)  : ORIGIN = 0x200031B8, LENGTH = 0x0003AE4A /* Reserve 0x1000 for the bootloader */
}

INCLUDE "nrf5x_common.ld"
