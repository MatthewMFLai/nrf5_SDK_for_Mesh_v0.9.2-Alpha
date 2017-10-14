/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
    FLASH (rx) : ORIGIN = 0x0001B000, LENGTH = 0x0001F000 /* Reserve 0x6000 for the bootloader */
    RAM (rwx)  : ORIGIN = 0x200022D8, LENGTH = 0x00005A28 /* Reserve 0x300 for the bootloader */
}

INCLUDE "nrf5x_common.ld"
