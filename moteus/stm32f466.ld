/* The only special thing we want to do is keep the ISR vector in the
first flash sector, and shove everything else to the back to make room
for PersistentConfig's region. */

MEMORY
{
  FLASH_ISR (rx) : ORIGIN = 0x8000000, LENGTH = 0x004000
}

SECTIONS
{
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } >FLASH_ISR
}

/* And now delegate everything else to the regular mbed linker script */
INCLUDE external/com_github_ARMmbed_mbed-os/linker_script.ld
