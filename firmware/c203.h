/*  
 *  ======== c203.h ========
 *
 *  This header file contains defines useful in programming
 *  TMS320C203 devices.
 *
 */
 
/* DEFINES for DATA space memory mapped registers */

#define IMR     0x0004      /* Interrupt mask register           */
#define GREG    0x0005      /* Global memory allocation register */
#define IFR     0x0006      /* Interrupt flag register           */

/* DEFINES for IO space memory mapped registers */

#define CLK     0xffe8      /* CLK register                               */
#define IC      0xffec      /* Interrupt control register                 */
#define SDTR    0xfff0      /* Synchronous Serial Port Xmit/Rcv register  */
#define SSPCR   0xfff1      /* Synchronous Serial Port control register   */
#define ADTR    0xfff4      /* Asynchronous Serial Port xmit/rcv register */
#define ASPCR   0xfff5      /* Asynchronous Serial Port control register  */
#define IOSR    0xfff6      /* Input/Output status register               */
#define BRD     0xfff7      /* Baud rate division register                */
#define TCR     0xfff8      /* Timer control register                     */
#define PRD     0xfff9      /* Timer period register                      */
#define TIM     0xfffa      /* Timer counter register                     */
#define WSGR    0xfffc      /* Wait-state generator control register      */

/* DEFINES for enabling and disabling interrupts */

/* Constants for the IMR register */

#define EN_INT1     0x0001  /* Enable external user interrupt 1                   */
#define EN_INT23    0x0002  /* Enable external user interrupts 2 & 3              */
#define EN_TINT     0x0004  /* Enable timer interrupt                             */
#define EN_RINT     0x0008  /* Enable synchronous serial port receive interrupt   */
#define EN_XINT     0x0010  /* Enable synchronous serial port transmit interrupt  */
#define EN_TXRXINT  0x0020  /* Enable asynchronous serial port xmit/rcv interrupt */

/* Constants for the IFR register */

#define CLR_INT1    0x0001  /* Disable external user interrupt 1                   */
#define CLR_INT23   0x0002  /* Disable external user interrupts 2 & 3              */
#define CLR_TINT    0x0004  /* Disable timer interrupt                             */
#define CLR_RINT    0x0008  /* Disable synchronous serial port receive interrupt   */
#define CLR_XINT    0x0010  /* Disable synchronous serial port transmit interrupt  */
#define CLR_TXRXINT 0x0020  /* Disable asynchronous serial port xmit/rcv interrupt */
#define CLR_ALL     0xffff  /* Disable all interrupts                              */

/* Timer control register bit masks */

#define TCR_TDDR        0x000f  /* Timer divide down bits */
#define TCR_TSS         0x0010  /* Timer stop status bit  */
#define TCR_TRB         0x0020  /* Timer reload bit       */
#define TCR_PSC         0x03c0  /* Timer prescale counter */
#define TCR_TSOFT       0x0400  /* Timer SOFT bit         */
#define TCR_TFREE       0x0800  /* Timer FREE bit         */

/* Wait state generator register bit masks */

#define WSGR_PSLWS      0x0007  /* Program space lower wait state bits */
#define WSGR_PSUWS      0x0038  /* Program space upper wait state bits */
#define WSGR_DSWS       0x01c0  /* Data space wait state bits          */
#define WSGR_ISWS       0x0e00  /* IO space wait state bits            */

/* Synchronous serial port control register bit masks                  */
                                                                      
#define SSPCR_DLB       0x0001  /* Digital loopback mode bit          */
#define SSPCR_FSM       0x0002  /* Frame sync mode bit                */
#define SSPCR_MCM       0x0004  /* Clock mode bit                     */
#define SSPCR_TXM       0x0008  /* Transmit mode bit                  */
#define SSPCR_RRST      0x0010  /* Receive reset bit                  */
#define SSPCR_XRST      0x0020  /* Transmit reset bit                 */
#define SSPCR_IN0       0x0040  /* Input bit 0                        */
#define SSPCR_IN1       0x0080  /* Input bit 1                        */
#define SSPCR_FR0       0x0100  /* FIFO receive bit 0                 */
#define SSPCR_FR1       0x0200  /* FIFO receive bit 1                 */
#define SSPCR_FT0       0x0400  /* FIFO transmit bit 0                */
#define SSPCR_FT1       0x0800  /* FIFO transmit bit 1                */
#define SSPCR_RFNE      0x1000  /* Receive FIFO buffer not empty bit  */
#define SSPCR_TCOMP     0x2000  /* Transmit complete bit              */
#define SSPCR_SOFT      0x4000  /* SOFT bit                           */
#define SSPCR_FREE      0x8000  /* FREE bit                           */

/* Asynchronous serial port control register bit masks                */

#define ASPCR_CIO0      0x0001  /* CIO0 configuration bit             */
#define ASPCR_CIO1      0x0002  /* CIO1 configuration bit             */
#define ASPCR_CIO2      0x0004  /* CIO2 configuration bit             */
#define ASPCR_CIO3      0x0008  /* CIO3 configuration bit             */
#define ASPCR_SETBRK    0x0010  /* Set break bit                      */
#define ASPCR_CAD       0x0020  /* Calibrate A detect bit             */
#define ASPCR_STB       0x0040  /* Stop bit                           */
#define ASPCR_RIM       0x0080  /* Receive interrupt mask bit         */
#define ASPCR_TIM       0x0100  /* Transmit interrupt mask bit        */
#define ASPCR_DIM       0x0200  /* Delta interrupt mask bit           */
#define ASPCR_URST      0x2000  /* Reset asynchronous serial port bit */
#define ASPCR_SOFT      0x4000  /* SOFT bit                           */
#define ASPCR_FREE      0x8000  /* FREE bit                           */

/* Baud rate settings for the BRD register.  Assumes CLKOUT1 = 40MHz  */

#define CLKOUT1_F       40e6    /* CLKOUT1 frequency */
#define BRD_50          CLKOUT1_F/(16*50);  /* 0xc350   /* 50     bits/sec */
#define BRD_75          0x8235  /* 75     bits/sec */
#define BRD_300         0x208d  /* 300    bits/sec */
#define BRD_1200        0x0823  /* 1200   bits/sec */
#define BRD_2400        0x0411  /* 2400   bits/sec */
#define BRD_4800        0x0208  /* 4800   bits/sec */
#define BRD_9600        0x0104  /* 9600   bits/sec */
#define BRD_57600       0x002b  /* 57600  bits/sec */
#define BRD_115200      0x0015  /* 115200 bits/sec */

/* IOSR register bit masks */

#define IOSR_IO0        0x0001  /* IO0 status bit                      */
#define IOSR_IO1        0x0002  /* IO1 status bit                      */
#define IOSR_IO2        0x0004  /* IO2 status bit                      */
#define IOSR_IO3        0x0008  /* IO3 status bit                      */
#define IOSR_DIO0       0x0010  /* IO0 change detect bit               */
#define IOSR_DIO1       0x0020  /* IO1 change detect bit               */
#define IOSR_DIO2       0x0040  /* IO2 change detect bit               */
#define IOSR_DIO3       0x0080  /* IO3 change detect bit               */
#define IOSR_DR         0x0100  /* Data ready indicator bit            */
#define IOSR_OE         0x0200  /* Receive register overrun error      */
#define IOSR_FE         0x0400  /* Framing error indicator             */
#define IOSR_THRE       0x0800  /* Transmit data has been sent to AXSR */
#define IOSR_TEMT       0x1000  /* Transmit empty indicator            */
#define IOSR_BI         0x2000  /* Break interrupt                     */
#define IOSR_ADC        0x4000  /* 'A' detect complete                 */

/* IO Port access macros */

unsigned int ioData;

/* 
 *  ======== IO read macros ========
 *
 *  These macros will read from the specified port and store the results
 *  in the data memory address supplied in the argument list.
 *
 *  Example:    unsigned int dataVal;
 *              read_WSGR(dataVal);
 *
 *  will store the current wait state register contents in the variable
 *  dataVal.        
 *  (M.S. Don't use read and write! Does not work in subroutines,
 *   data pointer problem.)
 */

/* #define  read_CLK(a)     asm("   in  _ioData, 0ffe8h ");     a = ioData;
#define read_IC(a)      asm("   in  _ioData, 0ffech ");     a = ioData;
#define read_SDTR(a)    asm("   in  _ioData, 0fff0h ");     a = ioData;
#define read_SSPCR(a)   asm("   in  _ioData, 0fff1h ");     a = ioData;
#define read_ADTR(a)    asm("   in  _ioData, 0fff4h ");     a = ioData;
#define read_ASPCR(a)   asm("   in  _ioData, 0fff5h ");     a = ioData;
#define read_IOSR(a)    asm("   in  _ioData, 0fff6h ");     a = ioData;
#define read_BRD(a)     asm("   in  _ioData, 0fff7h ");     a = ioData;
#define read_TCR(a)     asm("   in  _ioData, 0fff8h ");     a = ioData;
#define read_PRD(a)     asm("   in  _ioData, 0fff9h ");     a = ioData;
#define read_TIM(a)     asm("   in  _ioData, 0fffah ");     a = ioData;
#define read_WSGR(a)    asm("   in  _ioData, 0fffch ");     a = ioData;  */
                                                 
/* 
 *  ======== IO write macros ========
 *
 *  These macros will write the contents of the data memory address
 *  supplied in the argument list to the IO port called.
 *
 *  Example:    unsigned int dataVal = 0;
 *              write_WSGR(dataVal);
 *
 *  will store the value 0 to the wait state register thus setting
 *  all memory accesses to zero wait states.
 *
 */

/* #define  write_CLK(a)    ioData = a;     asm("   out _ioData, 0ffe8h ");
#define write_IC(a)     ioData = a;     asm("   out _ioData, 0ffech ");
#define write_SDTR(a)   ioData = a;     asm("   out _ioData, 0fff0h ");
#define write_SSPCR(a)  ioData = a;     asm("   out _ioData, 0fff1h ");
#define write_ADTR(a)   ioData = a;     asm("   out _ioData, 0fff4h ");
#define write_ASPCR(a)  ioData = a;     asm("   out _ioData, 0fff5h ");
#define write_IOSR(a)   ioData = a;     asm("   out _ioData, 0fff6h ");
#define write_BRD(a)    ioData = a;     asm("   out _ioData, 0fff7h ");
#define write_TCR(a)    ioData = a;     asm("   out _ioData, 0fff8h ");
#define write_PRD(a)    ioData = a;     asm("   out _ioData, 0fff9h ");
#define write_TIM(a)    ioData = a;     asm("   out _ioData, 0fffah ");
#define write_WSGR(a)   ioData = a;     asm("   out _ioData, 0fffch "); */

/* Misc defines */

#define FALSE   0
#define TRUE    1
                                                            