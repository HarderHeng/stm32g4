MEMORY
{
    /* APP: 106KB starting at 0x08005800 (after bootloader 20KB + boot flag 2KB) */
    FLASH : ORIGIN = 0x08005800, LENGTH = 106K
    RAM   : ORIGIN = 0x20000000, LENGTH = 32K
}