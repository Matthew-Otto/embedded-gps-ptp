/* STM32H563ZIT6U Linker Script */

ENTRY(reset_handler)

/* Memories definition */
MEMORY
{
    FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 2M
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 640K
}

/* Define stack top (end of SRAM) */
_estack = ORIGIN(SRAM) + LENGTH(SRAM);

SECTIONS
{
    /* Vector table */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } > FLASH

    .text : {
        . = ALIGN(4);
        *(.text*)
        *(.rodata*)
        . = ALIGN(4);
    } > FLASH

    /* Used by the startup to initialize data */
    _sidata = LOADADDR(.data);

    .data : {
        _sdata = .;
        *(.data*)
        _edata = .;
    } > SRAM AT > FLASH

    .bss : {
        _sbss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
    } > SRAM

    .heap (NOLOAD) : ALIGN(4) {
        KEEP(*(.heap))
    } > SRAM

    .eth_rx_buffer (NOLOAD) : ALIGN(4) {
        KEEP(*(.eth_rx_buffer))
    } > SRAM
    
    .eth_tx_buffer (NOLOAD) : ALIGN(4) {
        KEEP(*(.eth_rx_buffer))
    } > SRAM


    .stack (COPY) : {
        __stack_top = ORIGIN(SRAM) + LENGTH(SRAM) - 4;
    }
}
