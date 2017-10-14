/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
FLASH (rx) : ORIGIN = 0x0001F000, LENGTH = 0x00059000 /* Reserve 0x8000 for the bootloader */
RAM (rwx)  : ORIGIN = 0x200019C0, LENGTH = 0x0000D640 /* Reserve 0x1000 for the bootloader */
}

INCLUDE "nrf5x_common.ld"
