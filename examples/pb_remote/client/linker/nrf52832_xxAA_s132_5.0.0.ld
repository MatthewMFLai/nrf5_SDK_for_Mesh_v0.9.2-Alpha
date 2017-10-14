/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
FLASH (rx) : ORIGIN = 0x00023000, LENGTH = 0x0005D000
RAM (rwx)  : ORIGIN = 0x200031B0, LENGTH = 0x0000CE50
}

INCLUDE "nrf5x_common.ld"
