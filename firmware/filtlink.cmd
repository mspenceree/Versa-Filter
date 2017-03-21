/* This is the linker command file that defines the memory map for
   the SPS Filter Module based on the TMS320C203.                    */


/*c:\work\sps\filter\filtsoft\filt.obj

/* c:\c2xxti\RTS2XX.LIB     /* Put input filenames here if desired */

-stack  400                 /* alocate stack space (mostly for "float coefs[128]" in compute_fir) */

MEMORY
{  /* Program Memory */
    PAGE 0: VECS:   origin = 0000h, length = 0040h  /* External RAM */
            CODE:   origin = 0040h, length = 5000h  /* External RAM (max. code size: 5f7eh-40h=5f3eh) */
            /* Important: Set boot2 function in asm. to the same values of origin and length as above */
            /* Don't change origin: SN resides at 40h */
            /* Maximum programm size: origin+length = 5040h words */

   /* Data Memory    */
    PAGE 1: RAMB2:  origin = 0060h, length = 0020h  /* Internal RAM bank B2 (Dual access) */
            RAMB0:  origin = 0200h, length = 0100h  /* Internal RAM bank B0 (Dual access) */
            RAMB1:  origin = 0300h, length = 0100h  /* Internal RAM bank B1 (Dual access) */
            RAMEX:  origin = 5040h, length = 2fc0h  /* External RAM */
}

SECTIONS
{
    vectors: > VECS  PAGE = 0   /* interrupt vector locations... */
                                    /* use .sect "vectors" directive */
    bank0:   > RAMB0 PAGE = 1   /* internal RAM section */
    bank1:   > RAMB1 PAGE = 1   /* internal RAM section */
    bank2:   > RAMB2 PAGE = 1   /* internal RAM section */
    .text:   > CODE  PAGE = 0 /* executable code and floating-point const. */
    .cinit:  > CODE  PAGE = 0 /* tables for explicitly initialized global and static vars.*/
    .const:  > CODE  PAGE = 0 /* string literals, initialized global and static const. vars. */
    .switch: > CODE  PAGE = 0 /* switch statement tables */

/*  .const:  > RAMEX PAGE = 1 /* string literals, initialized global and static const. vars. */

    .data:   > RAMEX PAGE = 1 /* initialized data section (not used by C, may be used in assembly) */
    .bss:    > RAMEX PAGE = 1 /* uninitialized global and static variables */
    .stack:  > RAMEX PAGE = 1 /* software stack: used to pass args. to functions and to allocate space for local vars. */
                              /* the stack size is set above with linker switch */
    .sysmem: > RAMEX PAGE = 1 /* uninitialized section for dynamic memory from malloc functions */

}
