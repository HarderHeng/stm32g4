MEMORY
{
    /* APP: 110KB starting at 0x08008800 (after bootloader 16KB + boot flag 2KB) */
    FLASH : ORIGIN = 0x08008800, LENGTH = 110K
    RAM   : ORIGIN = 0x20000000, LENGTH = 32K
}