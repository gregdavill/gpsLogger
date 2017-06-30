/* ============================================================================ */
/* Copyright (c) 2015, Texas Instruments Incorporated                           */
/*  All rights reserved.                                                        */
/*                                                                              */
/*  Redistribution and use in source and binary forms, with or without          */
/*  modification, are permitted provided that the following conditions          */
/*  are met:                                                                    */
/*                                                                              */
/*  *  Redistributions of source code must retain the above copyright           */
/*     notice, this list of conditions and the following disclaimer.            */
/*                                                                              */
/*  *  Redistributions in binary form must reproduce the above copyright        */
/*     notice, this list of conditions and the following disclaimer in the      */
/*     documentation and/or other materials provided with the distribution.     */
/*                                                                              */
/*  *  Neither the name of Texas Instruments Incorporated nor the names of      */
/*     its contributors may be used to endorse or promote products derived      */
/*     from this software without specific prior written permission.            */
/*                                                                              */
/*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" */
/*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,       */
/*  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR      */
/*  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR            */
/*  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,       */
/*  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,         */
/*  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; */
/*  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,    */
/*  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR     */
/*  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,              */
/*  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                          */
/* ============================================================================ */

/* This file supports MSP430F5514 devices. */
/* Version: 1.159 */
/* Default linker script, for normal executables */

OUTPUT_ARCH(msp430)
ENTRY(_start)

MEMORY {
  SFR              : ORIGIN = 0x0000, LENGTH = 0x0010 /* END=0x0010, size 16 */
  PERIPHERAL_8BIT  : ORIGIN = 0x0010, LENGTH = 0x00F0 /* END=0x0100, size 240 */
  PERIPHERAL_16BIT : ORIGIN = 0x0100, LENGTH = 0x0100 /* END=0x0200, size 256 */
  RAM              : ORIGIN = 0x2400, LENGTH = 0x0F80 /* END=0x337F, size 4096 */
  INFOMEM          : ORIGIN = 0x1800, LENGTH = 0x0200 /* END=0x19FF, size 512 as 4 128-byte segments */
  INFOA            : ORIGIN = 0x1980, LENGTH = 0x0080 /* END=0x19FF, size 128 */
  INFOB            : ORIGIN = 0x1900, LENGTH = 0x0080 /* END=0x197F, size 128 */
  INFOC            : ORIGIN = 0x1880, LENGTH = 0x0080 /* END=0x18FF, size 128 */
  INFOD            : ORIGIN = 0x1800, LENGTH = 0x0080 /* END=0x187F, size 128 */
  ROM (rx)         : ORIGIN = 0x4400, LENGTH = 0x9B80 /* END=0xDF7F, size 39806 */
  VECT42           : ORIGIN = 0xDFD2, LENGTH = 0x0002
  VECT43           : ORIGIN = 0xDFD2, LENGTH = 0x0002
  VECT44           : ORIGIN = 0xDFD4, LENGTH = 0x0002
  VECT45           : ORIGIN = 0xDFD6, LENGTH = 0x0002
  VECT46           : ORIGIN = 0xDFD8, LENGTH = 0x0002
  VECT47           : ORIGIN = 0xDFDA, LENGTH = 0x0002
  VECT48           : ORIGIN = 0xDFDC, LENGTH = 0x0002
  VECT49           : ORIGIN = 0xDFDE, LENGTH = 0x0002
  VECT50           : ORIGIN = 0xDFE0, LENGTH = 0x0002
  VECT51           : ORIGIN = 0xDFE2, LENGTH = 0x0002
  VECT52           : ORIGIN = 0xDFE4, LENGTH = 0x0002
  VECT53           : ORIGIN = 0xDFE6, LENGTH = 0x0002
  VECT54           : ORIGIN = 0xDFE8, LENGTH = 0x0002
  VECT55           : ORIGIN = 0xDFEA, LENGTH = 0x0002
  VECT56           : ORIGIN = 0xDFEC, LENGTH = 0x0002
  VECT57           : ORIGIN = 0xDFEE, LENGTH = 0x0002
  VECT58           : ORIGIN = 0xDFF0, LENGTH = 0x0002
  VECT59           : ORIGIN = 0xDFF2, LENGTH = 0x0002
  VECT60           : ORIGIN = 0xDFF4, LENGTH = 0x0002
  VECT61           : ORIGIN = 0xDFF6, LENGTH = 0x0002
  VECT62           : ORIGIN = 0xDFF8, LENGTH = 0x0002
  VECT63           : ORIGIN = 0xDFFA, LENGTH = 0x0002
  RESETVEC         : ORIGIN = 0xDFFC, LENGTH = 0x0002
  CRCVAL           : ORIGIN = 0xDFFE, LENGTH = 0x0002
  BSL              : ORIGIN = 0x1000, LENGTH = 0x0800
  USBRAM           : ORIGIN = 0x1C00, LENGTH = 0x0800
  HIROM (rx)       : ORIGIN = 0x00010000, LENGTH = 0x000043FF
}

SECTIONS
{
  __interrupt_vector_42  : { KEEP (*(__interrupt_vector_42)) KEEP (*(__interrupt_vector_rtc)) } > VECT42
  __interrupt_vector_43  : { KEEP (*(__interrupt_vector_43)) KEEP (*(__interrupt_vector_port2)) } > VECT43
  __interrupt_vector_44  : { KEEP (*(__interrupt_vector_44)) KEEP (*(__interrupt_vector_timer2_a1)) } > VECT44
  __interrupt_vector_45  : { KEEP (*(__interrupt_vector_45)) KEEP (*(__interrupt_vector_timer2_a0)) } > VECT45
  __interrupt_vector_46  : { KEEP (*(__interrupt_vector_46)) KEEP (*(__interrupt_vector_usci_b1)) } > VECT46
  __interrupt_vector_47  : { KEEP (*(__interrupt_vector_47)) KEEP (*(__interrupt_vector_usci_a1)) } > VECT47
  __interrupt_vector_48  : { KEEP (*(__interrupt_vector_48)) KEEP (*(__interrupt_vector_port1)) } > VECT48
  __interrupt_vector_49  : { KEEP (*(__interrupt_vector_49)) KEEP (*(__interrupt_vector_timer1_a1)) } > VECT49
  __interrupt_vector_50  : { KEEP (*(__interrupt_vector_50)) KEEP (*(__interrupt_vector_timer1_a0)) } > VECT50
  __interrupt_vector_51  : { KEEP (*(__interrupt_vector_51)) KEEP (*(__interrupt_vector_dma)) } > VECT51
  __interrupt_vector_52  : { KEEP (*(__interrupt_vector_52)) KEEP (*(__interrupt_vector_usb_ubm)) } > VECT52
  __interrupt_vector_53  : { KEEP (*(__interrupt_vector_53)) KEEP (*(__interrupt_vector_timer0_a1)) } > VECT53
  __interrupt_vector_54  : { KEEP (*(__interrupt_vector_54)) KEEP (*(__interrupt_vector_timer0_a0)) } > VECT54
  __interrupt_vector_55  : { KEEP (*(__interrupt_vector_55)) } > VECT55
  __interrupt_vector_56  : { KEEP (*(__interrupt_vector_56)) KEEP (*(__interrupt_vector_usci_b0)) } > VECT56
  __interrupt_vector_57  : { KEEP (*(__interrupt_vector_57)) KEEP (*(__interrupt_vector_usci_a0)) } > VECT57
  __interrupt_vector_58  : { KEEP (*(__interrupt_vector_58)) KEEP (*(__interrupt_vector_wdt)) } > VECT58
  __interrupt_vector_59  : { KEEP (*(__interrupt_vector_59)) KEEP (*(__interrupt_vector_timer0_b1)) } > VECT59
  __interrupt_vector_60  : { KEEP (*(__interrupt_vector_60)) KEEP (*(__interrupt_vector_timer0_b0)) } > VECT60
  __interrupt_vector_61  : { KEEP (*(__interrupt_vector_61)) KEEP (*(__interrupt_vector_comp_b)) } > VECT61
  __interrupt_vector_62  : { KEEP (*(__interrupt_vector_62)) KEEP (*(__interrupt_vector_unmi)) } > VECT62
  __interrupt_vector_63  : { KEEP (*(__interrupt_vector_63)) KEEP (*(__interrupt_vector_sysnmi)) } > VECT63
  __reset_vector : 
  { 
    KEEP (*(__interrupt_vector_64)) 
    KEEP (*(__interrupt_vector_reset))
    KEEP (*(.resetvec))
  } > RESETVEC

  .crc_val : {KEEP (*(.crc_val)) } > CRCVAL

  .rodata :
  {
    . = ALIGN(2);
   /* KEEP (*(.crc_val)) */
    *(.plt)
    . = ALIGN(2);
    *(.lower.rodata.* .lower.rodata)
    . = ALIGN(2);
    *(.rodata .rodata.* .gnu.linkonce.r.* .const .const:*)
    *(.rodata1)

    *(.eh_frame_hdr)
    KEEP (*(.eh_frame))
    KEEP (*(.gcc_except_table)) *(.gcc_except_table.*)
    PROVIDE (__preinit_array_start = .);
    KEEP (*(.preinit_array))
    PROVIDE (__preinit_array_end = .);
    PROVIDE (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array))
    PROVIDE (__init_array_end = .);
    PROVIDE (__fini_array_start = .);
    KEEP (*(.fini_array))
    KEEP (*(SORT(.fini_array.*)))
    PROVIDE (__fini_array_end = .);
    LONG(0); /* Sentinel.  */

    /* gcc uses crtbegin.o to find the start of the constructors, so
       we make sure it is first.  Because this is a wildcard, it
       doesn't matter if the user does not actually link against
       crtbegin.o; the linker won't look for a file to match a
       wildcard.  The wildcard also means that it doesn't matter which
       directory crtbegin.o is in.  */
    KEEP (*crtbegin*.o(.ctors))

    /* We don't want to include the .ctor section from from the
       crtend.o file until after the sorted ctors.  The .ctor section
       from the crtend file contains the end of ctors marker and it
       must be last */
    KEEP (*(EXCLUDE_FILE (*crtend*.o ) .ctors))
    KEEP (*(SORT(.ctors.*)))
    KEEP (*(.ctors))

    KEEP (*crtbegin*.o(.dtors))
    KEEP (*(EXCLUDE_FILE (*crtend*.o ) .dtors))
    KEEP (*(SORT(.dtors.*)))
    KEEP (*(.dtors))
  } > ROM

  .upper.rodata :
  {
    *(.upper.rodata.* .upper.rodata)
  } > HIROM

  .data :
  {
    . = ALIGN(2);
    PROVIDE (__datastart = .);
    *(.lower.data.* .lower.data)

    . = ALIGN(2);
    *(.either.data.* .either.data)

    . = ALIGN(2);
    KEEP (*(.jcr))
    *(.data.rel.ro.local) *(.data.rel.ro*)
    *(.dynamic)

    . = ALIGN(2);
    *(.data .data.* .gnu.linkonce.d.*)
    KEEP (*(.gnu.linkonce.d.*personality*))
    SORT(CONSTRUCTORS)
    *(.data1)
    *(.got.plt) *(.got)

    /* We want the small data sections together, so single-instruction offsets
       can access them all, and initialized data all before uninitialized, so
       we can shorten the on-disk segment size.  */
    . = ALIGN(2);
    *(.sdata .sdata.* .gnu.linkonce.s.* D_2 D_1)

    . = ALIGN(2);
    _edata = .;
    PROVIDE (edata = .);
    PROVIDE (__dataend = .);
  } > RAM AT> ROM

  /* Note that crt0 assumes this is a multiple of two; all the
     start/stop symbols are also assumed word-aligned.  */
  PROVIDE(__romdatastart = LOADADDR(.data));
  PROVIDE (__romdatacopysize = SIZEOF(.data));

  .bss :
  {
    . = ALIGN(2);
    PROVIDE (__bssstart = .);
    *(.lower.bss.* .lower.bss)
    . = ALIGN(2);
    *(.either.bss.* .either.bss)
    *(.dynbss)
    *(.sbss .sbss.*)
    *(.bss .bss.* .gnu.linkonce.b.*)
    . = ALIGN(2);
    *(COMMON)
    PROVIDE (__bssend = .);
  } > RAM
  PROVIDE (__bsssize = SIZEOF(.bss));

  /* This section contains data that is not initialised at startup.  */
  .noinit :
  {
    . = ALIGN(2);
    PROVIDE (__noinit_start = .);
    *(.noinit)
    . = ALIGN(2);
    PROVIDE (__noinit_end = .);
  } > RAM

  _end = .;
  PROVIDE (end = .);

  .stack (ORIGIN (RAM) + LENGTH(RAM)) :
  {
    PROVIDE (__stack = .);
    *(.stack)
  }

  /* This is just for crt0.S and interrupt handlers.  */
  .lowtext           :
  {
    PROVIDE (_start = .);
    . = ALIGN(2);
    KEEP (*(SORT(.crt_*)))
    KEEP (*(.lowtext))
  } > ROM

  .text :
  {
    . = ALIGN(2);
    *(.lower.text.* .lower.text)

    . = ALIGN(2);
    *(.text .stub .text.* .gnu.linkonce.t.* .text:*)

    KEEP (*(.text.*personality*))
    /* .gnu.warning sections are handled specially by elf32.em.  */
    *(.gnu.warning)
    *(.interp .hash .dynsym .dynstr .gnu.version*)
    PROVIDE (__etext = .);
    PROVIDE (_etext = .);
    PROVIDE (etext = .);
    . = ALIGN(2);
    KEEP (*(.init))
    KEEP (*(.fini))
    KEEP (*(.tm_clone_table))
  } > ROM

  .upper.text :
  {
    . = ALIGN(2);
    *(.upper.text.* .upper.text)
  } > HIROM

  .infoA     : {} > INFOA              /* MSP430 INFO FLASH MEMORY SEGMENTS */
  .infoB     : {} > INFOB
  .infoC     : {} > INFOC
  .infoD     : {} > INFOD

  /* Make sure that upper data sections are not used.  */
  .upper :
  {
    *(.upper.bss.* .upper.bss)
    *(.upper.data.* .upper.data)
    ASSERT (SIZEOF(.upper) == 0, "This MCU does not support placing read/write data into high memory");
  }

  /* The rest are all not normally part of the runtime image.  */

  .MP430.attributes 0 :
  {
    KEEP (*(.MSP430.attributes))
    KEEP (*(.gnu.attributes))
    KEEP (*(__TI_build_attributes))
  }

  /* Stabs debugging sections.  */
  .stab          0 : { *(.stab) }
  .stabstr       0 : { *(.stabstr) }
  .stab.excl     0 : { *(.stab.excl) }
  .stab.exclstr  0 : { *(.stab.exclstr) }
  .stab.index    0 : { *(.stab.index) }
  .stab.indexstr 0 : { *(.stab.indexstr) }
  .comment       0 : { *(.comment) }
  /* DWARF debug sections.
     Symbols in the DWARF debugging sections are relative to the beginning
     of the section so we begin them at 0.  */
  /* DWARF 1.  */
  .debug          0 : { *(.debug) }
  .line           0 : { *(.line) }
  /* GNU DWARF 1 extensions.  */
  .debug_srcinfo  0 : { *(.debug_srcinfo) }
  .debug_sfnames  0 : { *(.debug_sfnames) }
  /* DWARF 1.1 and DWARF 2.  */
  .debug_aranges  0 : { *(.debug_aranges) }
  .debug_pubnames 0 : { *(.debug_pubnames) }
  /* DWARF 2.  */
  .debug_info     0 : { *(.debug_info .gnu.linkonce.wi.*) }
  .debug_abbrev   0 : { *(.debug_abbrev) }
  .debug_line     0 : { *(.debug_line .debug_line.* .debug_line_end ) }
  .debug_frame    0 : { *(.debug_frame) }
  .debug_str      0 : { *(.debug_str) }
  .debug_loc      0 : { *(.debug_loc) }
  .debug_macinfo  0 : { *(.debug_macinfo) }
  /* SGI/MIPS DWARF 2 extensions.  */
  .debug_weaknames 0 : { *(.debug_weaknames) }
  .debug_funcnames 0 : { *(.debug_funcnames) }
  .debug_typenames 0 : { *(.debug_typenames) }
  .debug_varnames  0 : { *(.debug_varnames) }
  /* DWARF 3 */
  .debug_pubtypes 0 : { *(.debug_pubtypes) }
  .debug_ranges   0 : { *(.debug_ranges) }
  /* DWARF Extension.  */
  .debug_macro    0 : { *(.debug_macro) }

  /DISCARD/ : { *(.note.GNU-stack) }
}

