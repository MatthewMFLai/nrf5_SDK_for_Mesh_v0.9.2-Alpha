/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
FLASH (rx) : ORIGIN = 0x00023000, LENGTH = 0x00055000 /* Reserve 0x8000 for the bootloader */
RAM (rwx)  : ORIGIN = 0x200031B0, LENGTH = 0x0000BE50 /* Reserve 0x1000 for the bootloader */
}

INCLUDE "nrf5x_common.ld"
