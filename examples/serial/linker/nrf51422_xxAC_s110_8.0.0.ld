/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
FLASH (rx)      : ORIGIN = 0x00018000, LENGTH = 0x00022000
RAM (rwx)       : ORIGIN = 0x20002000, LENGTH = 0x00005D00
}

INCLUDE "nrf5x_common.ld"
