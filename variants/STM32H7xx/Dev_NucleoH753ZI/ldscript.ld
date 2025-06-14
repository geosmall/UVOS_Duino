/*
******************************************************************************
**
**  File        : LinkerScript.ld
**
**  Author      : STM32CubeIDE
**
**  Abstract    : Linker script for STM32H7x3 series
**                2048Kbytes FLASH and 1056Kbytes RAM
**
**                Set heap size, stack size and stack location according
**                to application requirements.
**
**                Set memory bank area and size if external memory is used.
**
**  Target      : STMicroelectronics STM32
**
**  Distribution: The file is distributed as is, without any warranty
**                of any kind.
**
*****************************************************************************
** @attention
**
** Copyright (c) 2019 STMicroelectronics.
** All rights reserved.
**
** This software component is licensed by ST under BSD 3-Clause license,
** the "License"; You may not use this file except in compliance with the
** License. You may obtain a copy of the License at:
**                        opensource.org/licenses/BSD-3-Clause
**
****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = ORIGIN(RAM_D1) + LENGTH(RAM_D1);    /* end of RAM_D1 */
/* Generate a link error if heap and stack don't fit into RAM_D1 */
_Min_Heap_Size = 0x2000 ;      /* required amount of heap  */
_Min_Stack_Size = 0x2000 ; /* required amount of stack */

/* Specify the memory areas */
MEMORY
{
  FLASH (rx)       : ORIGIN = 0x08000000, LENGTH = 1920K
  CONFIG (rx)      : ORIGIN = 0x081E0000, LENGTH = 128K
  DTCMRAM (xrw)    : ORIGIN = 0x20000000, LENGTH = 128K  /* DTCM */
  RAM_D1 (xrw)     : ORIGIN = 0x24000000, LENGTH = 512K  /* AXI SRAM */
  RAM_D2 (xrw)     : ORIGIN = 0x30000000, LENGTH = 288K  /* SRAM1, SRAM2, SRAM3 */
  RAM_D3 (xrw)     : ORIGIN = 0x38000000, LENGTH = 64K   /* SRAM4 */
  BACKUP_RAM (xrw) : ORIGIN = 0x38800000, LENGTH = 4K    /* Backup SRAM */
  ITCMRAM (xrw)    : ORIGIN = 0x00000000, LENGTH = 64K   /* ITCM */
}

/* Define output sections */
SECTIONS
{
  /* The startup code goes first into FLASH */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(4);
  } >FLASH

  /* The program code and other data goes into FLASH */
  .text :
  {
    . = ALIGN(4);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;        /* define a global symbols at end of code */
  } >FLASH

  /* Constant data goes into FLASH */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    . = ALIGN(4);
  } >FLASH

  .ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM : {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array     :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH
  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH
  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH

  /* used by the startup to initialize data */
  _sidata = LOADADDR(.data);

  /* Initialized data sections goes into RAM_D1, load LMA copy after code */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */
    *(.RamFunc)        /* .RamFunc sections */
    *(.RamFunc*)       /* .RamFunc* sections */

    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */
  } >RAM_D1 AT> FLASH

  /* Uninitialized data section */
  . = ALIGN(4);
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss section */
    _sbss = .;         /* define a global symbol at bss start */
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
  } >RAM_D1

  /* User_heap_stack section, used to check that there is enough RAM_D1 left */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM_D1

  .sram1_bss (NOLOAD) :
  {
    . = ALIGN(4);
    _ssram1_bss = .;

    PROVIDE(__sram1_bss_start__ = _sram1_bss);
    *(.sram1_bss)
    *(.sram1_bss*)
    . = ALIGN(4);
    _esram1_bss = .;

    PROVIDE(__sram1_bss_end__ = _esram1_bss);
  } > RAM_D2

  .dtcmram_bss (NOLOAD) :
  {
    . = ALIGN(4);
    _sdtcmram_bss = .;

    PROVIDE(__dtcmram_bss_start__ = _sdtcmram_bss);
    *(.dtcmram_bss)
    *(.dtcmram_bss*)
    . = ALIGN(4);
    _edtcmram_bss = .;

    PROVIDE(__dtcmram_bss_end__ = _edtcmram_bss);
  } >DTCMRAM

  .backup_ram (NOLOAD) :
  {
    . = ALIGN(4);
    _sbackup_ram = .;

    PROVIDE(__backup_ram_start = _sbackup_ram);
    *(.backup_ram)
    *(.backup_ram*)
    . = ALIGN(4);
    _ebackup_ram = .;

    PROVIDE(__backup_ram_end = _ebackup_ram);
  } >BACKUP_RAM

  /* Remove information from the standard libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}


