;
;
;   ======== c203.inc ========
;
;   This assembly language header file contains defines useful in programming
;   TMS320C203 devices.
;

;   DEFINES for DATA space memory mapped registers

IMR             .set    0004h       ; Interrupt mask register
GREG            .set    0005h       ; Global memory allocation register
IFR             .set    0006h       ; Interrupt flag register

;   DEFINES for IO space memory mapped registers

CLK             .set    0ffe8h      ; CLK register 
IC              .set    0ffech      ; Interrupt control register                 
SDTR            .set    0fff0h      ; Synchronous Serial Port Xmit/Rcv register  
SSPCR           .set    0fff1h      ; Synchronous Serial Port control register   
ADTR            .set    0fff4h      ; Asynchronous Serial Port xmit/rcv register 
ASPCR           .set    0fff5h      ; Asynchronous Serial Port control register  
IOSR            .set    0fff6h      ; Input/Output status register               
BRD             .set    0fff7h      ; Baud rate division register                
TCR             .set    0fff8h      ; Timer control register                     
PRD             .set    0fff9h      ; Timer period register                      
TIM             .set    0fffah      ; Timer counter register                     
WSGR            .set    0fffch      ; Wait-state generator control register      

;   DEFINES for enabling and disabling interrupts 

;   These use the IMR register 

EN_INT1         .set    0001h       ; Enable external user interrupt 1                    
EN_INT23        .set    0002h       ; Enable external user interrupts 2 & 3              
EN_TINT         .set    0004h       ; Enable timer interrupt                             
EN_RINT         .set    0008h       ; Enable synchronous serial port receive interrupt   
EN_XINT         .set    0010h       ; Enable synchronous serial port transmit interrupt  
EN_TXRXINT      .set    0020h       ; Enable asynchronous serial port xmit/rcv interrupt 

;   These use the IFR register 
                        
CLR_INT1        .set    0001h       ; Clear the external user interrupt 1                  
CLR_INT23       .set    0002h       ; Clear the external user interrupts 2 & 3              
CLR_TINT        .set    0004h       ; Clear the timer interrupt                             
CLR_RINT        .set    0008h       ; Clear the synchronous serial port receive interrupt   
CLR_XINT        .set    0010h       ; Clear the synchronous serial port transmit interrupt  
CLR_TXRXINT     .set    0020h       ; Clear the asynchronous serial port xmit/rcv interrupt 
CLR_ALL         .set    0ffffh      ; Clear all pending interrupts
                                  
;   Timer control register bit masks 

TDDR            .set    000fh       ; Timer divide down bits 
TSS             .set    0010h       ; Timer stop status bit  
TRB             .set    0020h       ; Timer reload bit       
PSC             .set    03c0h       ; Timer prescale counter 
TSOFT           .set    0400h       ; Timer SOFT bit         
TFREE           .set    0800h       ; Timer FREE bit         
                                  
;   Wait state generator register bit masks 

PSLWS           .set    0007h       ; Program space lower wait state bits 
PSUWS           .set    0038h       ; Program space upper wait state bits 
DSWS            .set    01c0h       ; Data space wait state bits          
ISWS            .set    0e00h       ; IO space wait state bits            

;   Synchronous serial port control register bit masks 

DLB             .set    0001h       ; Digital loopback mode bit         
FSM             .set    0002h       ; Frame sync mode bit               
MCM             .set    0004h       ; Clock mode bit                    
TXM             .set    0008h       ; Transmit mode bit                 
RRST            .set    0010h       ; Receive reset bit                 
XRST            .set    0020h       ; Transmit reset bit                
IN0             .set    0040h       ; Input bit 0                       
IN1             .set    0080h       ; Input bit 1                       
FR0             .set    0100h       ; FIFO receive bit 0                
FR1             .set    0200h       ; FIFO receive bit 1                
FT0             .set    0400h       ; FIFO transmit bit 0               
FT1             .set    0800h       ; FIFO transmit bit 1               
RFNE            .set    1000h       ; Receive FIFO buffer not empty bit 
TCOMP           .set    2000h       ; Transmit complete bit             
SPSOFT          .set    4000h       ; SOFT bit                          
SPFREE          .set    8000h       ; FREE bit                          

;   Asynchronous serial port control register bit masks 

CIO0            .set    0001h       ; CIO0 configuration bit             
CIO1            .set    0002h       ; CIO1 configuration bit             
CIO2            .set    0004h       ; CIO2 configuration bit             
CIO3            .set    0008h       ; CIO3 configuration bit             
SETBRK          .set    0010h       ; Set break bit                      
CAD             .set    0020h       ; Calibrate A detect bit             
STB             .set    0040h       ; Stop bit                           
RIM             .set    0080h       ; Receive interrupt mask bit         
ASPTIM          .set    0100h       ; Transmit interrupt mask bit        
DIM             .set    0200h       ; Delta interrupt mask bit           
URST            .set    2000h       ; Reset asynchronous serial port bit 
ASPSOFT         .set    4000h       ; SOFT bit                           
ASPFREE         .set    8000h       ; FREE bit
                                  