; Filename: filtasm.asm
;
; This C2xx assembly source file contains code for the filter module.
; It performs all the real-time filtering.
; 
; History:
;           10/3/97 Orignial - Michael Spencer

;----- Define any Program Constants ----------------------------------------

; Include useful constants for the TMS320C203 assembly language
        .include    "c203.inc"

;----- Define program variable locations -----------------------------------
; Variable starting with _ and defined .global can be accessed from c-code

temp        .usect  "bank2",1
temp2       .usect  "bank2",1
randnum     .usect  "bank2",1   ; random number register
;_in_new_a  .usect  "bank2",1   ; define CODEC I/O words
_in_a       .usect  "bank2",1
            .global  _in_a
_in_b       .usect  "bank2",1
            .global  _in_b
_in_error   .usect  "bank2",1
            .global  _in_error
_in_error_stick .usect  "bank2",1 ; version of in_error where bits are only set, not cleared
            .global  _in_error_stick
_in_digital .usect  "bank2",1
            .global _in_digital
_out_a      .usect  "bank2",1
            .global _out_a
_out_old_a  .usect  "bank2",1   ; must be after _out_a (because of dmov instruction)
_out_b      .usect  "bank2",1
            .global  _out_b
_out_gain   .usect  "bank2",1
            .global  _out_gain
_out_atten  .usect  "bank2",1
            .global  _out_atten
_in_a_hold  .usect  "bank2",1   ; used to hold the peak values for the VU Meter
            .global _in_a_hold
_in_b_hold  .usect  "bank2",1   ; used to hold the peak values for the VU Meter
            .global _in_b_hold
_out_a_hold .usect  "bank2",1   ; used to hold the peak values for the VU Meter
            .global _out_a_hold
_out_b_hold .usect  "bank2",1   ; used to hold the peak values for the VU Meter
            .global _out_b_hold
_t_reg_scale_a  .usect  "bank2",1   ; used to scale ch a data after filter
            .global  _t_reg_scale_a
_t_reg_scale_b  .usect  "bank2",1   ; used to scale ch b data after filter
            .global  _t_reg_scale_b
_func_addr_a  .usect    "bank2",1   ; used to point to address of current function A
            .global  _func_addr_a
_func_addr_b  .usect    "bank2",1   ; used to point to address of current function B
            .global  _func_addr_b
_assembly_flag  .usect  "bank2",1   ; used to flag the assembly code from C code:
            .global  _assembly_flag ; bit position  function
                                    ; 15 (LSB)      Flags VU Meter peak hold code (also uses bit 14)
                                    ; 13            Flags cascade Ch A and Ch B
                                    ; 12            Flags the white noise generator

_coef_ptr_a  .usect "bank2",1   ; used to point to starting address of filter A coefs (changed in c-code to ping-pong)
            .global  _coef_ptr_a
_coef_ptr_b  .usect "bank2",1   ; used to point to starting address of filter B coefs (changed in c-code to ping-pong)
            .global  _coef_ptr_b
_data_ptr_a .usect  "bank2",1   ; used to point to starting address of ch a filter data
            .global  _data_ptr_a
_data_ptr_b .usect  "bank2",1   ; used to point to starting address of ch b filter data
            .global  _data_ptr_b
_orderm2_a  .usect  "bank2",1   ; used to hold FIR filter A order minus 2
            .global  _orderm2_a
_orderm2_b  .usect  "bank2",1   ; used to hold FIR filter B order minus 2
            .global  _orderm2_b
_iosr_copy  .usect  "bank2",1   ; used to hold a copy of the IO status register (txrxint interrups)
            .global  _iosr_copy

_k7f00h     .usect  "bank2",1   ; define memory for costants (for speed)
            .global  _k7f00h    ; value assigned in c-code
_kf80fh     .usect  "bank2",1   ; define memory for costants (for speed)
            .global  _kf80fh    ; value assigned in c-code
_kfff0h     .usect  "bank2",1   ; define memory for costants (for speed)
            .global  _kfff0h    ; value assigned in c-code


; Reserve FIR filter coeficient storage for two channels in internal DATA memory.
; FIR: Two filters up to 128 taps each are stored or one long filter up to 256 taps
; Memory bank B0 is placed into program space by setting CNF bit.
;   Channel A   -   up to 128 tap weights (or up to 256 tap weights for one filter)
;   Channel B   -   up to 128 tap weights

_fir_coef   .usect  "bank0",256
            .global _fir_coef   ; declare it as external so c-code can write values

; Reserve FIR state data and IIR coeficient and state data for two channels in
; internal DATA memory.
; Ch A ocupies 380-3ff (or 300-3ff for one long filter)
; Ch B ocupies 300-37f
; For FIR: An Nth order filter requires N data locations
; For IIR: Each second order section requires 3 state locations.
;          Each second order section requires 5 coeficients.
_coefdata   .usect  "bank1",256
            .global _coefdata   ; declare as external so c-code can write state


;***** Define Interrupt Vector Table ***************************************
        .sect   "vectors"
;start: b   _c_int0     ; branch to C-Code entry point on reset
start:  b   _boot2      ; on reset: branch to our custom boot routine to load sector 2 of FLSAH
int1:   b   start       ; restart ; install interrupt vector for external int1 pin
int2:   b   start       ; restart
tint:   b   start       ; restart
rint:   b   rint_asm    ; install interrupt vector for serial receive (CODEC)
;       b   _rint_c     ; install interrupt vector for serial receive (CODEC)
;       .global _rint_c
xint:   b   start       ; restart
txrxint: b  txrxint_asm ; install interrupt vector for asynchronous serial port & IO pins
;       .global _txrxint_c
        b   start       ; restart
int8:   b   start       ; restart
int9:   b   start       ; restart
int10:  b   start       ; restart
int11:  b   start       ; restart
int12:  b   start       ; restart
int13:  b   start       ; restart
int14:  b   start       ; restart
int15:  b   start       ; restart
int16:  b   start       ; restart
trap:   b   start       ; restart
nmi:    b   start       ; restart
        b   start       ; restart
int20:  b   start       ; restart
int21:  b   start       ; restart
int22:  b   start       ; restart
int23:  b   start       ; restart
int24:  b   start       ; restart
int25:  b   start       ; restart
int26:  b   start       ; restart
int27:  b   start       ; restart
int28:  b   start       ; restart
int29:  b   start       ; restart
int30:  b   start       ; restart
int31:  b   start       ; restart


        .text           ; put following into begining of code section
serial_no:
        .string "9999997768"    ; The SN is the first 6 digits, followed by
                                ; 4 check sum digits: ######oess
                                ; This SN resides from 40h to 49h
;       .string "Copyrighted 1997-1998, Signal Processing Solutions"
;===========================================================================
;  BEGIN ASSEMBLLY FUNCTIONS AND SUBROUTINE DEFINITIONS.
;  Functions starting with a "_" can be called by C code.
;===========================================================================

        .global _boot2  ; called by c (during testing only)
_boot2:
;**********************************************************************
; This function boots the code stored in sector 2 (or 6 if knob is pressed)
; to code memory locations 3f7e-543f inclusive.
;
;**********************************************************************
        ; Init:
        ldp     #0          ; set data pointer to page 0
        splk    #2e00h,temp ; set ARP=1, OVM=1, INTM=1, DP=0
        lst     #0,temp
        splk    #21fch,temp ; set ARB=1, CNF=0, SXM=0, XF=1, PM=0
        lst     #1,temp
        splk    #80h,GREG   ; GREG <- 80h, Designates locations 8000-ffffh as global

        ; Raise the CLIP pins to map in FLASH sector 2(6):
        splk    #4300h,temp ; put sync serial port into reset
        out     temp, 0fff1h ; write the SSPCR (sync. serial port control reg.)
        splk    #4330h,temp ; take sync serial port out of reset
        out     temp, 0fff1h ; write the SSPCR (sync. serial port control reg.)

        splk    #000ch,temp ; load temp with CODEC code for rasing CLIP pins
        lacc    #1000h,3    ; ACC <- 1000h
w_loop: out temp, 0fff0h    ; write to the SDTR (sync. serial port data reg.)
                            ; This causes the continuous transmision of ones untill
        sub     #1          ; ACC <- ACC - 1
        bcnd    w_loop,GT   ; loop till ACC <= 0

        ; Set pointers and addresses:                                                                   
        ; Important: 40h    <=> code origin in the linker command file: Filtlink.cmd
        ;            5000h  <=> code length in the linker command file: Filtlink.cmd
        lar     ar1,#8000h  ; ar1 <- first source address in FLASH
        splk    #(8192+8064-2),_in_a    ; _in_a <- destination address in code memory
        lar     ar0,#(40h + 5000h - (8192+8064-2) - 1)  ; ar1 <- length of data to be transfered minus 1

        ; Transfer code:
boot_loop:
        out temp, 0fff0h    ; keep CLIP high by writing to the SDTR (sync. serial port data reg.)
        lacc    *+,8        ; ACC <- high byte of code shifted by 8 bits
        sacl    _in_b       ; store high byte
        lacl    *+,AR0      ; ACC <- low byte of code
        and     #0ffh       ; mask off upper 24 bits
        or      _in_b       ; OR ACC with high byte to form 16-bit code word
        sacl    _out_a      ; _out_a <- code word
        lacl    _in_a       ; load destination address
        tblw    _out_a      ; transfer code to destination address
        add     #1          ; inc. destination address
        sacl    _in_a       ; save destination address
        banz    boot_loop,*-,ar1    ; branch if not end of code

        splk    #0,GREG     ; disable global memory
        b       _c_int0     ; branch to c initialization code section
        .global _c_int0


        .global _get_serial ; function called by c
_get_serial:
;**********************************************************************
; This function puts the serial number string (the string at label-serial_no:)
; and a terminating null into the 11 element character array: serial_str[]
; when called by c-code with:
;
;          get_serial(&serial_str);
;
; fix: we can shrink this code by looping over the blpd command.
;**********************************************************************
        popd    *+          ; pop return address, push on c-stack
        sar     ar0,*+      ; push c Frame Pointer
        sar     ar1,*       ; push c Stack Pointer
        lar     ar0,#1h     ; size of Frame
        lar     ar0,*0+,ar2 ; set up FP and SP
        
        ldp     #temp       ; Load data pointer to page 0
        clrc    sxm         ; clear sign extension mode
        
        lar     ar2,#0fffdh
        mar     *0+
        
        lar     ar3,*,ar3
        blpd    #serial_no,temp ; put first word of serial number into temp
        lacc    temp, 8     ; put high byte into ACC high
        sach    *,0,ar2     ; save
        lar     ar3,*,ar3
        adrk    #1h
        lacc    temp, 16    ; put into ACC high
        and     #00ffh,16   ; mask of high byte
        sach    *,0,ar2     ; save

        lar     ar3,*,ar3
        adrk    #2h
        blpd    #(serial_no+1),temp ; put first word of serial number into temp
        lacc    temp, 8     ; put high byte into ACC high
        sach    *,0,ar2     ; save
        lar     ar3,*,ar3
        adrk    #3h
        lacc    temp, 16    ; put into ACC high
        and     #00ffh,16   ; mask of high byte
        sach    *,0,ar2     ; save

        lar     ar3,*,ar3
        adrk    #4h
        blpd    #(serial_no+2),temp ; put first word of serial number into temp
        lacc    temp, 8     ; put high byte into ACC high
        sach    *,0,ar2     ; save
        lar     ar3,*,ar3
        adrk    #5h
        lacc    temp, 16    ; put into ACC high
        and     #00ffh,16   ; mask of high byte
        sach    *,0,ar2     ; save

        lar     ar3,*,ar3
        adrk    #6h
        blpd    #(serial_no+3),temp ; put first word of serial number into temp
        lacc    temp, 8     ; put high byte into ACC high
        sach    *,0,ar2     ; save
        lar     ar3,*,ar3
        adrk    #7h
        lacc    temp, 16    ; put into ACC high
        and     #00ffh,16   ; mask of high byte
        sach    *,0,ar2     ; save

        lar     ar3,*,ar3
        adrk    #8h
        blpd    #(serial_no+4),temp ; put first word of serial number into temp
        lacc    temp, 8     ; put high byte into ACC high
        sach    *,0,ar2     ; save
        lar     ar3,*,ar3
        adrk    #9h
        lacc    temp, 16    ; put into ACC high
        and     #00ffh,16   ; mask of high byte
        sach    *,0,ar2     ; save

        lar     ar3,*,ar3
        adrk    #0ah
        lacl    #0          ; 0 -> ACC, put the null character into ACC
        sach    *,0,ar2     ; save

        larp    ar1
        sbrk    #2h         ; deallocate Frame
        lar     ar0,*-      ; pop Frame Pointer     
        pshd    *           ; push return address on hardware stack
        ret                 ; return


txrxint_asm:
;**********************************************************************
; Interrupt routine is for servicing delta IO and RS-232 async. serial port
; interrupts.
;
; This ISR calls the c-functions: txrxint_c
;
; Registers destroyed: None, this is an ISR!
; 
;**********************************************************************
        .global _txrxint_c

; save all registers on c stack (except the hardware stack):
        MAR     *, AR1  ; CURRENT ARP = C-STACK POINTER
        ADRK    #1h     ; SKIP TOP ELEMENT ON STACK
        SST     #1,*+   ; SAVE STATUS REGISTERS
        SST     #0,*+

        POPD    *+      ; pop the return address, push on c-stack
                        ; This is nessary to avoid hardware stack overflow since this
                        ; ISR can be interuppted by itself.
        POPD    *+
        POPD    *+
        POPD    *+
        POPD    *+
        POPD    *+
        POPD    *+
        POPD    *+
        

    ldp     #temp           ; Load data pointer to page 0

;   splk    #0080h,temp     ; Signal the entry into body of txrxint_asm
;   out     temp, 0         ; by setting module pin28 (this causes a display crash if this ISR is called while display is updateing
;   splk    #8,temp
;   out     temp,IOSR       ; Set IO3 bit to signal the entry into ISR (I think this is trouble too

    in      _iosr_copy,IOSR ; Get copy of IOSR
;   out     _kfff0h,IOSR    ; Reset any bits that could have caused the interrupt
    splk    #0008h, IMR     ; Mask TXRXINT interrupt so this ISR is not re-interrupted
;   splk    #CLR_TXRXINT, IFR   ; Clear TXRXINT flag in IFR to avoid double ints
    clrc    INTM            ; Re-enable interrupts so we don't miss a sample int.
    

        SACH    *+  ; SAVE ACCUMULATOR
        SACL    *+

;       CLRC    OVM     ; TURN OFF OVERFLOW MODE (never modified by c-code, p. 4-10)
;       SPM     0       ; PRODUCT SHIFT COUNT OF 0 (never modified by c-code, p. 4-10)

        SPH      *+     ; SAVE P REGISTER (assmues PM = 0)
        SPL      *+

        MPY     #1h     ; SAVE T REGISTER 
        SPL     *+

        SAR     AR0,*+  ; SAVE AUXILIARY REGISTERS
        SAR     AR2,*+
        SAR     AR3,*+
        SAR     AR4,*+
        SAR     AR5,*+
        SAR     AR6,*+
        SAR     AR7,*+,AR1
        
; body...

        mar     *,AR1       ; Prepare for the C-function call
        call    _txrxint_c  ; Call c-code to get the switch or RS-232 action and put into FIFO

;       ldp     #temp       ; Load data pointer to page 0
;       splk    #0028h, IMR ; Unmask TXRXINT interrupt so that the next switch action is not missed

; restore registers and return
        MAR     *, AR1
        MAR     *-
        LAR     AR7,*-  ; RESTORE AUX REGISTERS
        LAR     AR6,*-
        LAR     AR5,*-
        LAR     AR4,*-
        LAR     AR3,*-
        LAR     AR2,*-
        LAR     AR0,*-

        MAR     *-  ; SKIP T REGISTER (FOR NOW)
        LT      *+              
        MPY     #1h ; RESTORE LOW PRODUCT REGISTER
        
        LT      *-  ; RESTORE T REGISTER
        MAR     *-  ; SKIP LOW PRODUCT REGISTER VALUE
        LPH     *-  ; RESTORE HIGH PRODUCT REGISTER

        LACL    *-  ; RESTORE ACCUMULATOR
        ADD     *-,16

    ldp     #temp           ; Load data pointer to page 0
;   splk    #00f0h,temp
;   out     temp,IOSR       ; Reset delta bits in case of switch bounce
;   out     _kfff0h,IOSR    ; Reset delta bits in case of switch bounce

    out     _kfff0h,IOSR    ; Reset any bits that could have caused the interrupt or since the int.
    splk    #CLR_TXRXINT, IFR   ; Clear TXRXINT flag in IFR to avoid double ints


;   splk    #0000h,temp     ; fix: Signal the exit from body of txrxint_asm
;   out     temp, 0         ; by clearing module pin28
;   splk    #0,temp
;   out     temp,IOSR       ; Clear IO3 bit to signal exit from ISR

    setc    INTM            ; Disable interrupts so we can guarantee a return before another
                            ; txrxint interrupt (avoids a hardware stack overflow).
    splk    #0028h, IMR     ; Unmask TXRXINT interrupt for next time

        PSHD    *-      ; push return address on hardware stack
        PSHD    *-      ; push return address on hardware stack
        PSHD    *-      ; push return address on hardware stack
        PSHD    *-      ; push return address on hardware stack
        PSHD    *-      ; push return address on hardware stack
        PSHD    *-      ; push return address on hardware stack
        PSHD    *-      ; push return address on hardware stack
        PSHD    *-      ; push return address on hardware stack

        LST     #0,*-   ; RESTORE STATUS REGISTERS
        LST     #1,*-   ; OLD ARP IS RESTORED *NOW*

    clrc    INTM    ; Enable interrupts
        ret             ; return from interrupt


;       .global  _rint_asm  ; declare as global for c to call
rint_asm:
;**********************************************************************
; Interrupt service routine for receive interrupt. It services the
; CODEC by reading and writing 4 words in the synchronus serial port
; FIFOs.
;
; Registers destroyed: None, this is an ISR!
; 
;**********************************************************************

; save all modified registers on c stack:
        MAR     *, AR1  ; CURRENT ARP = STACK POINTER
        ADRK    #1h     ; SKIP TOP ELEMENT ON STACK
        SST     #1,*+   ; SAVE STATUS REGISTERS
        SST     #0,*+

    ldp     #temp       ; Load data pointer to page 0
    in      _in_b,SDTR  ; Read in sample from ch b of CODEC     Frame n
    out     _out_gain,SDTR  ; Write gain word out

;   splk    #8,temp
;   out     temp,IOSR   ; Fix: Set IO3 bit to signal the entry into body of rint_asm
                        ; Note this screws up read_flash() and write_flash() in c
    
        SACH    *+  ; SAVE ACCUMULATOR
        SACL    *+

;       clrc    OVM     ; TURN OFF OVERFLOW MODE (never modified by c-code, p. 4-10)
;       SPM     0       ; PRODUCT SHIFT COUNT OF 0 (never modified by c-code, p. 4-10)

        SPH     *+      ; SAVE P REGISTER (assmues PM = 0)
        SPL     *+

        MPY     #1h     ; SAVE T REGISTER 
        SPL     *+

        SAR     AR0,*+ ; SAVE AUXILIARY REGISTERS 0 and 2
        SAR     AR2,*,AR2

        
; body...

; Read and Write CODEC registers:
        in      _in_error,SDTR  ; Read in error word from CODEC
        out     _out_old_a,SDTR ; Write out ch a sample to CODEC        Frame
        dmov    _out_a          ; Delay Ch A output by one sample: _out_a -> _out_old_a
        in      _in_a,SDTR      ; Read in sample from ch b of CODEC     Frame
        out     _out_atten,SDTR ; Write attenuation word out

; Check for shift error in received word:
        lacl    _kf80fh         ; load masking constant from memory (for speed)
        and     _in_error
        sub     #8              ; ACC <- ACC - 8
        bcnd    skip, NEQ       ; skip last I/O if there was a receive error

;       nop     ; need 2-NOPs here to delay for 8KHz ch. B !!!
;       nop

        setc    sxm     ; set sign extension mode, requred for random number gen. and _notch functions
        setc    ovm     ; set overflow mode to hard limit accumulations

        bit     _assembly_flag, 12  ; white noise bit -> TC
        bcnd    rand_skip,NTC       ; skip cascade hold if flag not set
; Random number generator (see Numerical Recipies p. 284 to learn this is not the way to do a rand gen.):
        lt      randnum
        mpy     #53     ; 53
        pac             ; load ACC with PREG
        add     #15473  ; 15473
        sacl    randnum
        lacc    randnum ; 16 bit data -> ACC with sign extention (assumes sxm is set)
        sfr             ; ACC/2 -> ACC (to reduce the amplitude of the noise)
        sacl    _in_a
        sacl    _in_b
rand_skip:        

        in      _in_digital,SDTR ; Read digital input word of CODEC
        out     _out_b,SDTR     ; Write out ch b sample to CODEC        Frame

; Set any bits in in_error_stick so we can turn on the CLIP LEDs in c-code:
        lacl    _in_error_stick ; put sticky error code into ACC
        or      _in_error       ; logical OR with current error code to set any bits
        sacl    _in_error_stick ; save new sticky error code

; Call to _func_addr_a:
        lacl    _func_addr_a    ; get the current A function address ...
        cala                    ; and call it


        bit     _assembly_flag, 15  ; vu_flag -> TC
        bcnd    rint_out_b,NTC  ; skip peak hold if flag not set
;       b   rint_out_b
; Hold peak values for VU Meter functionality in c-code (overflow mode (ovm) must be set):
        lacc    _in_a,16        ; load ACC with data
        abs                     ; |data| -> ACC
        sub     _in_a_hold,16   ; ACC - hold -> ACC
        bcnd    rint_in_a,LEQ   ; branch if this sample is not a maximum
        add     _in_a_hold,16   ; recalulate: |data| -> ACC
        sach    _in_a_hold      ; save new maximum
rint_in_a:

        lacc    _in_b,16        ; load ACC with data
        abs                     ; |data| -> ACC
        sub     _in_b_hold,16   ; ACC - hold -> ACC
        bcnd    rint_in_b,LEQ   ; branch if this sample is not a maximum
        add     _in_b_hold,16   ; recalulate: |data| -> ACC
        sach    _in_b_hold      ; save new maximum
rint_in_b:

        lacc    _out_a,16       ; load ACC with data
        abs                     ; |data| -> ACC
        sub     _out_a_hold,16  ; ACC - hold -> ACC
        bcnd    rint_out_a,LEQ  ; branch if this sample is not a maximum
        add     _out_a_hold,16  ; recalulate: |data| -> ACC
        sach    _out_a_hold     ; save new maximum
rint_out_a:

        lacc    _out_b,16       ; load ACC with data
        abs                     ; |data| -> ACC
        sub     _out_b_hold,16  ; ACC - hold -> ACC
        bcnd    rint_out_b,LEQ  ; branch if this sample is not a maximum
        add     _out_b_hold,16  ; recalulate: |data| -> ACC
        sach    _out_b_hold     ; save new maximum
rint_out_b:


; Scale _out_a, _out_b and hard limit (clip):
; (_t_reg_scale_x/256) * _out_x -> _out_x
        spm     1               ; set product mode (PM) to 1 (if not)

        lt      _t_reg_scale_a  ; _t_reg_scale_a -> T
        mpy     _out_a          ; _out_x x T -> P
        pac                     ; 2 * P -> ACC
; Largest pos# in ACC = 0000 000[0 1111 1111 1111 111]X XXXX XXXX, []=_out_x
; Hard limit ACC if there was a positive or negative overflow:
        add     _k7f00h,16
        sub     _k7f00h,16
        sub     _k7f00h,16
        add     _k7f00h,16
        sach    _out_a, 7       ; int[ACC x 2^(7-16)] -> _out_b

        lt      _t_reg_scale_b  ; _t_reg_scale_b -> T
        mpy     _out_b          ; _out_x x T -> P
        pac                     ; 2 * P -> ACC
; Largest pos# in ACC = 0000 000[0 1111 1111 1111 111]X XXXX XXXX, []=_out_x
; Hard limit ACC if there was a positive or negative overflow:
        add     _k7f00h,16
        sub     _k7f00h,16
        sub     _k7f00h,16
        add     _k7f00h,16
        sach    _out_b, 7       ; int[ACC x 2^(7-16)] -> _out_b
        
;        rpt        #10     ; waste time
;        nop
        
skip:
        splk    #CLR_RINT, IFR  ; Clear receive interrupt flag in IFR to avoid double ints

;       clrc    ovm     ; TURN OFF OVERFLOW MODE (must be 0 for c-code, p. 4-10)
                        ; this is restored with ST0 below !
                        
; restore registers and return
        MAR     *, AR1
        LAR     AR2,*-  ; RESTORE AUX REGISTERS 0 and 2
        LAR     AR0,*-

        MAR     *-  ; SKIP T REGISTER (FOR NOW)
        LT      *+              
        MPY     #1h ; RESTORE LOW PRODUCT REGISTER
        
        LT      *-  ; RESTORE T REGISTER
        MAR     *-  ; SKIP LOW PRODUCT REGISTER VALUE
        LPH     *-  ; RESTORE HIGH PRODUCT REGISTER

        LACL    *-  ; RESTORE ACCUMULATOR
        ADD     *-,16

;   splk    #0,temp
;   out     temp,IOSR   ; fix: Clear IO3 bit to signal exit from body of rint_asm

        LST     #0,*-   ; RESTORE STATUS REGISTERS
        LST     #1,*-   ; OLD ARP IS RESTORED *NOW*
        clrc    INTM ; Enable interrupts
        ret         ; return from interrupt
 

;**********************************************************************
; no functions for A and B:
; 
;**********************************************************************
        .global _no_func_a  ; declare function as global so c-code can find it
_no_func_a: 
        splk    #0h,_out_a  ; output zero value

; End of Ch A
        lacl    _func_addr_b    ; get the current B function address ...
        bacc                    ; and branch to it

;**********************************************************************
        .global _no_func_b  ; declare function as global so c-code can find it
_no_func_b: 
        splk    #0h,_out_b  ; output zero value
        ret

;**********************************************************************
; allpass functions for A and B:
; 
;**********************************************************************
        .global _allpass_func_a ; declare function as global so c-code can find it
_allpass_func_a:
        lacl    _in_a           ; in -> ACC
        sacl    _out_a          ; ACC low -> out
; End of Ch A
        lacl    _func_addr_b    ; get the current B function address ...
        bacc                    ; and branch to it

;**********************************************************************
        .global _allpass_func_b ; declare function as global so c-code can find it
_allpass_func_b:
; Start of Ch B
        lacl    _in_b           ; in -> ACC
        bit     _assembly_flag, 13  ; cascade_flag -> TC
        bcnd    allpass_skip,NTC    ; skip cascade hold if flag not set
        lacl    _out_a          ; Ch A output -> ACC
allpass_skip:
        sacl    _out_b          ; ACC low -> out
        ret


;**********************************************************************
; FIR Filter functions for Ch A and B
; Use different function for different max impulse response value:
;                       fir_15_a    - Channel A, s1=15, s2=0, PM=1
;                       fir_15_b    - Channel B, s1=15, s2=0, PM=1
;
;                       fir_16_a    - Channel A, s1=16, s2=0, PM=0
;                       fir_16_b    - Channel B, s1=16, s2=0, PM=0
;
; C-code sets the following values:
;
;   _func_addr_a    =   address of function A to call
;   _func_addr_b    =   address of function B to call
;   _orderm2_a      =   N - 2, N = 3 to 128 for 2 or to 256 for ch A filter only
;   _orderm2_b      =   N - 2, N = 3 to 128 for both filters
;   _data_ptr_a =   0x03ff-N+1, (points to first address filter data ch a: d0)
;   _data_ptr_b =   0x037f-N+1, (points to first address filter data ch b: d0)
;
;       _data_ptr_b:    d0      - ch b
;                       d1 ...
;       0x037f:         d(N-1)
;
;       _data_ptr_a:    d0      - ch a
;                       d1 ...
;       0x03ff:         d(N-1)
;
;   _fir_coef[] = array of coeficient data:         fir_coef[0] = h(N-1) - ch a
;       Clear CNF bit to write fir_coef[]           fir_coef[1] = h(N-2)
;       Then set CNF bit to map to prog memory.     ...
;       (_fir_coef is in B0: 200-2ff)               fir_coef[N-1] = h(0)
;
;                                                   fir_coef[128] = h(N-1)  - ch b
;                                                   fir_coef[129] = h(N-2)
;                                                   ...
;                                                   fir_coef[128+N-1] = h(0)
;
; Coeficent values are stored = int or round[(2^s1)*true_coef_value]
;
; For optimum scaling use different functions for different maximum
; impulse response values as follows:
; coef scaling shift: s1 = 16 - s2 - PM
; output shift:       s2 = 16 - s1 + PM
; Valid combinations (assuming truncation of scaled ideal coefs):
;           max(h)          use:    s1  s2  PM  function
;       [0.5       to 1)            15  0   1   _fir_15_x
;       [0         to 0.5)          16  0   0   _fir_16_x
;
;       [0.015625  to 0.03125)      _fir_20_x (not used)
;       [0.0078125 to 0.015625)     _fir_21_x (not used)
;       [0         to 0.0078125)    _fir_22_x (not used)
;                       
;**********************************************************************
        .global _fir_15_a   ; declare function as global so c-code can find it
_fir_15_a:  ; Ch A FIR filter, s1 = 15
        spm     1               ; set product mode (PM) to 1
        mar     *,AR2           ; AR2 -> ARP

        lacl    _in_a           ; in -> ACC
        lar     AR2, _data_ptr_a ; point to state data location d0
        sacl    *,AR0           ; ACC -> d0
        lar     AR0, #03ffh     ; point to first state data addr used -> AR0
 
        lacl    #0              ; 0 -> ACC
        mpy     #0              ; 0 -> P
        mac     0ff00h,*-       ; ACC + shifted(P) -> ACC
                                ; d(N-1) -> T
                                ; d(N-1) * coef(N-1) -> P
        rpt     _orderm2_a      ; i = 2 to N (_orderm2 = N - 2)
        macd    0ff01h, *-      ; ACC + shifted(P) -> ACC
                                ; d(N-i) -> T
                                ; d(N-i) * coef(N-i) -> P
                                ; d(N-i) -> d(N-i+1)
        apac                    ; ACC + shifted(P) -> ACC
        sach    _out_a,0        ; shifted(ACC) -> out (shift by s2)

; End of Ch A
        lacl    _func_addr_b    ; get the current B function address ...
        bacc                    ; and branch to it

;**********************************************************************
        .global _fir_15_b   ; declare function as global so c-code can find it
_fir_15_b:  ; Ch B FIR filter, s1 = 15
        spm     1               ; set product mode (PM) to 1
        mar     *,AR2           ; AR2 -> ARP
        
        lacl    _in_b           ; in -> ACC
        bit     _assembly_flag, 13  ; cascade_flag -> TC
        bcnd    fir_15_skip,NTC ; skip cascade hold if flag not set
        lacl    _out_a          ; Ch A output -> ACC
fir_15_skip:

        lar     AR2, _data_ptr_b ; point to state data location d0
        sacl    *,AR0           ; ACC -> d0
        lar     AR0, #037fh     ; point to first state data addr used -> AR0

        lacl    #0              ; 0 -> ACC
        mpy     #0              ; 0 -> P
        mac     0ff80h,*-       ; ACC + shifted(P) -> ACC
                                ; d(N-1) -> T
                                ; d(N-1) * coef(N-1) -> P
        rpt     _orderm2_b      ; i = 2 to N (_orderm2 = N - 2)
        macd    0ff81h, *-      ; ACC + shifted(P) -> ACC
                                ; d(N-i) -> T
                                ; d(N-i) * coef(N-i) -> P
                                ; d(N-i) -> d(N-i+1)
        apac                    ; ACC + shifted(P) -> ACC
        sach    _out_b,0        ; shifted(ACC) -> out (shift by s2)
        ret
        
;**********************************************************************
        .global _fir_16_a   ; declare function as global so c-code can find it
_fir_16_a:  ; Ch A FIR filter, s1 = 15
        spm     0               ; clear product mode (PM) to 0
        mar     *,AR2           ; AR2 -> ARP

        lacl    _in_a           ; in -> ACC
        lar     AR2, _data_ptr_a ; point to state data location d0
        sacl    *,AR0           ; ACC -> d0
        lar     AR0, #03ffh     ; point to first state data addr used -> AR0
 
        lacl    #0              ; 0 -> ACC
        mpy     #0              ; 0 -> P
        mac     0ff00h,*-       ; ACC + shifted(P) -> ACC
                                ; d(N-1) -> T
                                ; d(N-1) * coef(N-1) -> P
        rpt     _orderm2_a      ; i = 2 to N (_orderm2 = N - 2)
        macd    0ff01h, *-      ; ACC + shifted(P) -> ACC
                                ; d(N-i) -> T
                                ; d(N-i) * coef(N-i) -> P
                                ; d(N-i) -> d(N-i+1)
        apac                    ; ACC + shifted(P) -> ACC
;   lacc    _in_a, 16
        sach    _out_a,0        ; shifted(ACC) -> out (shift by s2)

; End of Ch A
        lacl    _func_addr_b    ; get the current B function address ...
        bacc                    ; and branch to it

;**********************************************************************
        .global _fir_16_b   ; declare function as global so c-code can find it
_fir_16_b:  ; Ch B FIR filter, s1 = 15
        spm     0               ; clear product mode (PM) to 0
        mar     *,AR2           ; AR2 -> ARP
        
        lacl    _in_b           ; in -> ACC
        bit     _assembly_flag, 13  ; cascade_flag -> TC
        bcnd    fir_16_skip,NTC ; skip cascade hold if flag not set
        lacl    _out_a          ; Ch A output -> ACC
fir_16_skip:

        lar     AR2, _data_ptr_b ; point to state data location d0
        sacl    *,AR0           ; ACC -> d0
        lar     AR0, #037fh     ; point to first state data addr used -> AR0

        lacl    #0              ; 0 -> ACC
        mpy     #0              ; 0 -> P
        mac     0ff80h,*-       ; ACC + shifted(P) -> ACC
                                ; d(N-1) -> T
                                ; d(N-1) * coef(N-1) -> P
        rpt     _orderm2_b      ; i = 2 to N (_orderm2 = N - 2)
        macd    0ff81h, *-      ; ACC + shifted(P) -> ACC
                                ; d(N-i) -> T
                                ; d(N-i) * coef(N-i) -> P
                                ; d(N-i) -> d(N-i+1)
        apac                    ; ACC + shifted(P) -> ACC
;   lacc    _in_b, 16
        sach    _out_b,0        ; shifted(ACC) -> out (shift by s2)
        ret
        

;**********************************************************************
; Notch, Inverse Notch, and Equalizer Filter functions:
;                       notch_a     - Channel A, 2nd order notch
;                       notch_b     - Channel B, 2nd order notch
;
; The notch filter is realized as the weighted sum of a 2nd order
; lattice allpass filter and the input (see IEEE Procedings, Jan. 1998, p. 29).
;
; C-code sets the following values:
;
;   _func_addr_a    =   address of Ch A function
;   _func_addr_b    =   address of Ch B function
;
;   _coef_ptr_b     =   points to first filter coeficient  Ch B (in B1: 300h):
;   _data_ptr_b     =   points to first used state address Ch B (in B1: 37fh)
;   _coef_ptr_a     =   points to first filter coeficient  Ch A (in B1: 380h):
;   _data_ptr_a     =   points to first used state address Ch A (in B1: 3ffh)
;
; 300h  _coef_ptr_b:    c(2)    - Ch B, section 2
;                       k(2)
;                       d(2)
;                       k(2) (copy)
;                       c(1)    - Ch B, section 1
;                       k(1)
;                       d(1)
;                       k(1) (copy)
;                       g(1)
;                       g(2)
;
;                       ... (coef and data grow towards each other)
;                       state(1)    - Ch B, section 1
;                       state(2)    - Ch B, section 2
; 37fh  _data_ptr_b:    state(3)    - Ch B, memory location for allpass output
;
;
; 380h  _coef_ptr_a:    c(2)    - Ch A, section 2
;                       k(2)
;                       d(2)
;                       k(2) (copy)
;                       c(1)    - Ch A, section 1
;                       k(1)
;                       d(1)
;                       k(1) (copy)
;                       g(1)
;                       g(2)
;
;                       ... (coef and data grow towards each other)
;                       state(1)    - Ch A, section 1
;                       state(2)    - Ch A, section 2
; 3ffh  _data_ptr_a:    state(3)    - Ch A, memory location for allpass output
;
;
; Coeficent values are stored = int[(2^15)*true_coef_value]
;                             = int[32768*true_coef_value]
;
;**********************************************************************
        .global _notch_a    ; declare function as global so c-code can find it
_notch_a:   ; Ch A, 2nd order notch filter:

        spm     1           ; set product mode (PM) to 1
        lar     AR2, _coef_ptr_a ; point to first coef address
        lar     AR0, _data_ptr_a ; point to state data location N+1
        mar     *,AR2       ; AR2 -> ARP (point to coefs)
        
        lacc    _in_a,15    ; _in_a/2 -> temp2(_out_a)
        sach    _out_a

;   clrc    ovm             ; fix: clear overflow mode to see overflows better

    ; Section N=2:
    ; Forward:      
        lacl    #0          ; 0 -> ACC
        lt      _out_a      ; temp2(_out_a) -> T
        mpy     *+,AR0      ; T*c2 -> P
        sbrk    1           ; AR0-1 -> AR0 (point to state2)
        lt      *+,AR2      ; state2 -> T
        mpya    *+          ; ACC+P -> ACC, T*k2 -> P
        spac                ; ACC-P -> ACC
        sach    temp        ; ACC -> temp
    ; Backward:
        lacl    #0          ; 0 -> ACC
        mpy     *+          ; T*d2 -> P
        lt      _out_a      ; temp2(_out_a) -> T
        mpya    *+,AR0      ; ACC+P -> ACC, T*k2 -> P
        apac                ; ACC+P -> ACC
        sach    *-,1,AR2    ; 2*ACC -> state3

    ; Section N-1=1:
    ; Forward:      
        lacl    #0          ; 0 -> ACC
        lt      temp        ; temp -> T
        mpy     *+,AR0      ; T*c1 -> P
        sbrk    1           ; AR0-1 -> AR0 (point to state1)
        lt      *+,AR2      ; state1 -> T
        mpya    *+          ; ACC+P -> ACC, T*k1 -> P
        spac                ; ACC-P -> ACC
        sach    _out_a      ; ACC -> temp2(_out_a)
    ; Backward:
        lacl    #0          ; 0 -> ACC
        mpy     *+          ; T*d1 -> P
        lt      temp        ; temp -> T
        mpya    *+,AR0      ; ACC+P -> ACC, T*k1 -> P
        apac                ; ACC+P -> ACC
        sach    *-          ; ACC -> state2

    ; Feedback:
        lacl    _out_a      ; temp2(_out_a) -> ACC
        sacl    *           ; ACC -> state1

    ; Weighted sum of allpass part and input:
        spm     2           ; set product mode (PM) to 2
        adrk    2           ; AR0+2 -> AR0 (point to state3)

        lacc    _in_a       ; _in_a -> ACC
        add     *,0         ; ACC+state3(allpas output) -> ACC
        sfr                 ; ACC/2 -> ACC
        sacl    temp        ; ACC -> temp

        lacc    _in_a       ; _in_a -> ACC
        sub     *,0,AR2     ; ACC-state3(allpas output) -> ACC
        sfr                 ; ACC/2 -> ACC
        sacl    _out_a      ; ACC -> _out_a
        
        lacl    #0          ; 0 -> ACC
        lt      temp        ; temp -> T
        mpy     *+          ; T*g1 -> P
        lt      _out_a      ; _out_a -> T
        mpya    *+          ; ACC+P -> ACC, T*g2 -> P
        apac                ; ACC+P -> ACC
        sach    _out_a      ; ACC -> out


; End of Ch A
        lacl    _func_addr_b    ; get the current B function address ...
        bacc                    ; and branch to it

;**********************************************************************
        .global _notch_b    ; declare function as global so c-code can find it
_notch_b:   ; Ch B, 2nd order notch filter:

; Start of Ch B
        lacl    _in_b       ; _in_b/2 -> ACC
        bit     _assembly_flag, 13  ; cascade_flag -> TC
        bcnd    notch_skip,NTC  ; skip cascade hold if flag not set
        lacl    _out_a      ; _out_a/2 -> ACC
notch_skip:
        sacl    temp2       ; ACC -> temp2 (input)

        lacc    temp2,15    ; input/2 -> temp2(_out_b)
        sach    _out_b

        spm     1           ; set product mode (PM) to 1
        lar     AR2, _coef_ptr_b ; point to first coef address
        lar     AR0, _data_ptr_b ; point to state data location N+1
        mar     *,AR2       ; AR2 -> ARP (point to coefs)

;   clrc    ovm             ; fix: clear overflow mode to see overflows better
        
    ; Section N=2:
    ; Forward:      
        lacl    #0          ; 0 -> ACC
        lt      _out_b      ; temp3(_out_b) -> T
        mpy     *+,AR0      ; T*c2 -> P
        sbrk    1           ; AR0-1 -> AR0 (point to state2)
        lt      *+,AR2      ; state2 -> T
        mpya    *+          ; ACC+P -> ACC, T*k2 -> P
        spac                ; ACC-P -> ACC
        sach    temp        ; ACC -> temp
    ; Backward:
        lacl    #0          ; 0 -> ACC
        mpy     *+          ; T*d2 -> P
        lt      _out_b      ; temp3(_out_b) -> T
        mpya    *+,AR0      ; ACC+P -> ACC, T*k2 -> P
        apac                ; ACC+P -> ACC
        sach    *-,1,AR2    ; 2*ACC -> state3

    ; Section N-1=1:
    ; Forward:      
        lacl    #0          ; 0 -> ACC
        lt      temp        ; temp -> T
        mpy     *+,AR0      ; T*c1 -> P
        sbrk    1           ; AR0-1 -> AR0 (point to state1)
        lt      *+,AR2      ; state1 -> T
        mpya    *+          ; ACC+P -> ACC, T*k1 -> P
        spac                ; ACC-P -> ACC
        sach    _out_b      ; ACC -> temp2(_out_b)
    ; Backward:
        lacl    #0          ; 0 -> ACC
        mpy     *+          ; T*d1 -> P
        lt      temp        ; temp -> T
        mpya    *+,AR0      ; ACC+P -> ACC, T*k1 -> P
        apac                ; ACC+P -> ACC
        sach    *-          ; ACC -> state2

    ; Feedback:
        lacl    _out_b      ; temp2(_out_b) -> ACC
        sacl    *           ; ACC -> state1

    ; Weighted sum of allpass part and input:
        spm     2           ; set product mode (PM) to 2
        adrk    2           ; AR0+2 -> AR0 (point to state3)
        
        lacc    temp2       ; temp2 -> ACC
        add     *,0         ; ACC+state3(allpas output) -> ACC
        sfr                 ; ACC/2 -> ACC
        sacl    temp        ; ACC -> temp

        lacc    temp2       ; temp2 -> ACC
        sub     *,0,AR2     ; ACC-state3(allpas output) -> ACC
        sfr                 ; ACC/2 -> ACC
        sacl    _out_b      ; ACC -> _out_b
        
        lacl    #0          ; 0 -> ACC
        lt      temp        ; temp -> T
        mpy     *+          ; T*g1 -> P
        lt      _out_b      ; _out_b -> T
        mpya    *+          ; ACC+P -> ACC, T*g2 -> P
        apac                ; ACC+P -> ACC
        sach    _out_b      ; ACC -> out

        ret
