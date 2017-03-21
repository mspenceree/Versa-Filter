/**************************************************************************
 *
 *  filt.c source file
 *
 *  This c source file contains code for the filter module.
 *  It performs initialization, sets up ISRs and enters main loop and
 *  flashes cursor until action is required.
 *
 *  History:
 *  V1.00   10/3/97 Started coding orignial - Michael Spencer
 *  V1.01 to V1.60  Lots of additions and modifications...
 *  V1.61   1/24/99 Commented out the command display test lines
 *  V1.62   1/31/99 Fixed the "valid_serial()" func. to compute correct check sums
 *  V1.63   2/6/99  Added the "display:" option that brings back the normal display
 *  V1.64   2/9/99  Added serial command interface
 *  V1.65   2/28/99 Re-aranged sign-on message so recall 0 is not delayed
 *  V1.66   3/13/99 Added display of UserFIR coefs.
 *  V1.67   3/14/99 Added value checking on RS-232 paramters
 *  V2.00   4/26/00 Added calibration msg. This is the feature complete version.
 *  V2.10   4/26/00 Added support for alternate (direction reversed) rotary encoder.
 *  V2.20   6/8/01  Fixed error in parser that updates min_value and max_value limits (param_ptr_temp).
 *                  Removed "Calibrate". Added "Erase Mem".
 *
 **************************************************************************/

/******* Build Parameters *************************************************/
#define MAIN 1              /* 1 - builds main (normal) version of Firmware */
                            /* 0 - builds recovery version of Firmware with */
                            /*     with only serial port programming support */
#define ENCODER_TYPE    1   /* Rotary encoder type: 0 - panasonic, 1 - Switch Channel */
#define SIGN_ON_FLAG_Versa_Filter   1   /* set to one for standard sign on message */
#define SIGN_ON_FLAG_AccuQuest      0   /* set to one for AccuQuest sign on message */

/******* Program Parameters ***********************************************/
#define VERSION 220             /* Firmware Version # (3 digit#: 123 = V1.23) */
#define CURSOR_PERIOD 50        /* cursor flashing period (in multiples of 10ms) */
/*#define HOLD_TIME 300         /* hold time for push/hold to become active (in multiples of 10ms) */
#define OVERFLOW_STICK 20       /* overload LED stick time (on after overload) (in multiples of 5ms) */
#define VU_DECAY 2              /* decay time between 6dB decrements of VU Meter (in multiples of 5ms) */
#define SENDSN_WAIT 22          /* (~0.75sec.) max wait time for sendsn command (in multiples of 32767us) */
#define SW_DEBOUNCE  500            /* switch/encoder debounce interval in us (set to ~1000) */
#define SERIAL_BUF_LEN 128          /* (128) length of serial input command buffer (MUST BE POWER OF 2) */
/* #define ORDER_MIN 2              /* minimum FIR filter order */
/* #define ORDER_MAX 127            /* maximum FIR filter order */
#define FCUT_MIN    200.0/48000.0   /* 600 minimum cutoff freq. for LP and HP (fraction of sampling rate) */
#define FCUT_MAX    20000.0/48000.0 /* 19200 maximum cutoff freq. for LP and HP (fraction of sampling rate) */
#define F1_MIN      200.0/48000.0   /* 600 minimum f1 freq. for LP and HP (fraction of sampling rate) */
#define F1_MAX      19600.0/48000.0 /* 18200 maximum f1 freq. for LP and HP (fraction of sampling rate) */
#define F2_MIN      600.0/48000.0   /* 1600 minimum f2 freq. for LP and HP (fraction of sampling rate) */
#define F2_MAX      20000.0/48000.0 /* 19200 maximum f2 freq. for LP and HP (fraction of sampling rate) */
#define FWIDTH_MIN  400.0/48000.0   /* 1000 minimum width of BP filter (fraction of sampling rate) */
#define FNFNOTCH_MIN  200.0/48000.0     /* 300 minimum freq of notch filter (fraction of sampling rate) */
#define FNFNOTCH_MAX  20000.0/48000.0   /* 19200 maximum freq of notch filter (fraction of sampling rate) */
#define FNFWIDTH_MIN  10.0/48000.0      /* 10 minimum width of notch filter (fraction of sampling rate) */
#define FNFWIDTH_MAX  10000.0/48000.0   /* 10000 maximum width of notch filter (fraction of sampling rate) */
#define GAIN_MAX    10000       /* maximum gain: 100 */
#define LAST_MEM_LOC 4          /* last memory loction for store and recall functions
                                   (9 max because of retrieved_flag[] ) */

/*#define FIR_LENGTH 256    /* temp */

#define SERIAL_LOC  0x0040      /* location of serial number in code space */
#define XTAL        12.288e6    /* frequency of DSP crystal */
#define FLASH_WAITS 4           /* Number of FLASH memory wait states (7 max.) */
#define IO_WAITS    1           /* Number of I/O wait states (7 max.) */
/*#define CLKOUT1       2.0*XTAL    /* frequency of master DSP clock = (1/cycle time) */
/*#define PSCPERIOD (16.0*1e6)/(CLKOUT1) /* set the period out of the prescaler (in microseconds) */
/*#define LOG2D4    0.0752574989    /* log10(2)/4 for computing ideal calibration constants */
#define PI  3.14159265359
#define PID2 1.5707963268        /* PI/2 */
#define PIT2 6.28318530717959   /* PI*2

/***** Define complex data type *******************************************/
/* typedef struct FCOMPLEX {float r,i;} fcomplex;   /* define the fcomplex structre for complex arithmetic */

/***** Include files here *************************************************/
#include    "c203.h"    /* Include useful constants and macros for the TMS320C203 */
#include    <stdlib.h>  /* Include standard library header */
#include    <math.h>    /* Include math library header */
#include    <string.h>  /* Include string function header */
#include    <ctype.h>   /* Include character function header */
#include    <limits.h>  /* define LONG_MIN and LONG_MAX */
#include    "filt.h"    /* generated by filt.m */

/***** Globals ************************************************************/

/* Define arrays of string pointers for text menus: */
/* (note: C stores duplicate string constants throughout the program only once!) */
/* Basic function strings: */
char *func_text[]={
    "NoFunc     ",
    "AllPass    ",
    "LowPass    ",
    "HighPass   ",
    "BandPass   ",
    "BandStop   ",
    "Notch      ",
    "InvNotch   ",
    "UserFIR    ",
/*  "Sine       ", */
    ""
    };
/* Parameter label strings: */
char *revertolevels_text[]={
    "N",
    "Y",
    ""
    };
char *samplerate_text[]={
    " 8KHz",
    "48KHz",
    ""
    };
char *inputsrc_text[]={
    "Analog ",
    "WtNoise",
    ""
    };      
char *mode_text[]={
    "A&B Common ",
    "A&BSeparate",
    "Ch A Only  ",
    ""
    };
char *cascade_ch_a_b_text[]={
    "N",
    "Y",
    ""
    };
char *master_mode_text[]={
    "N   ",
    "Y   ",
    ""
    };
char *initialize_text[]={
    "press",
    ""
    };
char *null_text[]={
    ""
    };

/* Define data-structure array that holds all system parameters: param_struct[]
 *
 *  .text   - pointer to parameter-string that is displayed on the LCD.
 *            Embed numbers in text as follows: ### - int, ###._# -float
 *            The "_" is the first curssor position in the number. It controls
 *            the increment/decrement step size.
 *
 *  For "text  text" display:
 *  .flag   - flag bits = 1pss ss-- nnnn nnnn
 *  .label  - unsinged integer that points to the first parameter label string pointer
 *
 *  For "text int" and "text float" displays:
 *  .flag   - flag bits = 0pss ssoo olll lfff
 *  .label  - Not used
 *
 *  where: d =          derived parameter bit, set if param. is derived from other params (not used now)
 *         p =          press flag bit (signals that this is a press/hold param.)
 *         ssss =       (nstart) start display position for right hand label text or number
 *         nnnn nnnn =  number of parameter label text strings for this param
 *         ooo =        (n_offset) first cursor position in number (relative to ssss) (marked by "_", eg. "##_.#")
 *         llll =       (nlength) length of displayed number
 *         fff =        number of fractional digits to display to right of "."
 *                      if fff=0 the number is assumed an interger
 *
 *  note: ssss, nnnn nnnn, llll, and fff are initialized in the initialize() function
 *        based on the text strings entered in the structure definition below.
 *        The p needs to be initialized in structure definition.
 */

struct pstruct {
  char *text;
  unsigned int flag;
  unsigned int label;
  };

struct pstruct param_struct[]={
/*  0 */    {" FUNC:",          0, (unsigned int)func_text},
 
/* Options: */ 
/*  1%*/    {"Levels-In  Out  ",0, (unsigned int)null_text},
/*  2 */    {"RevertToLevels:", 0, (unsigned int)revertolevels_text},
/*  3 */    {"FullScalIn:#_Vpp",0, 0},
/*  4 */    {"SampleRate:",     0, (unsigned int)samplerate_text},
/*  5 */    {"InputSrc:",       0, (unsigned int)inputsrc_text},
/*  6 */    {"Mode:",           0, (unsigned int)mode_text},
/*  7 */    {"Cascade Ch A&B:", 0, (unsigned int)cascade_ch_a_b_text},
/*  8 */    {"Master Mode:",    0, (unsigned int)master_mode_text},
/*  9%      {"Calibrate:",      0x4000, (unsigned int)calibrate_text}, */
/*  9%*/    {"Initialize:",     0x4000,  (unsigned int)initialize_text},
/* 10 */    {"Store:  # press ",0x4000, 0},
/* 11 */    {"Recall: # press ",0x4000, 0},
/* 12%*/    {"Firmware:  V",    0, (unsigned int)null_text},
/* 13%*/    {"Serial No:",      0, (unsigned int)null_text},

/* Function Parameters: */ 
/* 14 */    {" NFgain:###_.##x",0, 0},

/* 15 */    {" APgain:###_.##x",0, 0},

/* 16 */    {" LPfcut: #####Hz",0, 0},
/* 17 */    {" LPorder:    ###",0, 0},
/* 18 */    {" LPgain:###_.##x",0, 0},

/* 19 */    {" HPfcut: #####Hz",0, 0},
/* 20 */    {" HPorder:    ###",0, 0},
/* 21 */    {" HPgain:###_.##x",0, 0},

/* 22 */    {" BPf1:   #####Hz",0, 0},
/* 23 */    {" BPf2:   #####Hz",0, 0},
/* 24 */    {" BPfcntr:#####Hz",0, 0},
/* 25 */    {" BPfwdth:#####Hz",0, 0},
/* 26 */    {" BPorder:    ###",0, 0},
/* 27 */    {" BPgain:###_.##x",0, 0},

/* 28 */    {" BSf1:   #####Hz",0, 0},
/* 29 */    {" BSf2:   #####Hz",0, 0},
/* 30 */    {" BSfcntr:#####Hz",0, 0},
/* 31 */    {" BSfwdth:#####Hz",0, 0},
/* 32 */    {" BSorder:    ###",0, 0},
/* 33 */    {" BSgain:###_.##x",0, 0},

/* 34 */    {" Nfnotch:#####Hz",0, 0},
/* 35 */    {" Nfwidth:#####Hz",0, 0},
/* 36 */    {" Ngain: ###_.##x",0, 0},

/* 37 */    {" INfcntr:#####Hz",0, 0},
/* 38 */    {" INfwdth:#####Hz",0, 0},
/* 39 */    {" INgain:###_.##x",0, 0},

/* 40 */    {" UForder:    ###",0, 0},
/* 41 */    {" UFtap:##_      ",0, 0},  /* fix: 0x4000 */
/* 42 */    {" UFgain:###_.##x",0, 0},

/* 43 */    {" Sfreq:#####.#Hz",0, 0},
/* 44 */    {" Sphase:###.#deg",0, 0},
/* 45 */    {" Samp:   ###_.##",0, 0},

            };
/* % - memory not used */

/* Setup pointers to parameter boundaries: */
int param_ptr_start[]={14, 15, 16, 19, 22, 28, 34, 37, 40, 43};
int param_ptr_end[]=  {14, 15, 18, 21, 27, 33, 36, 39, 42, 45}; /* this should always be the gain parameter */

#define OPTIONS_START   1
#define OPTIONS_END     13

#define NPARAMSTRUCT    (sizeof param_struct)/(sizeof param_struct[0])
#define NPARAMS         NPARAMSTRUCT + 128      /* reserve additional space for user filter coefs. */
#define NPARAMSM6       NPARAMS + NPARAMS + NPARAMS + NPARAMS + NPARAMS + NPARAMS

/* define the array that holds all parameter values and user filter coefs. */
long params[NPARAMS][3];


/* Access the parameters depending on the display type:
 *
 * For "text text" display, access as follows:
 *    pointer to parameter text= param_struct[param_ptr].text
 *    flag value =                  param_struct[param_ptr].flag
 *    pointer to label text = (char*)(*(char*)(param_struct[param_ptr].label + (parameter value)))
 *
 * For "text int" and "text float" displays, access as follows:
 *    pointer to parameter text=    param_struct[param_ptr].text
 *    flag value =                  param_struct[param_ptr].flag
 *
 * For params[][]:
 *    parameter value for A =       params[param_ptr][0]
 *    parameter value for B =       params[param_ptr][1]
 *    parameter value for Common =  params[param_ptr][2]
 *
 * Filter coeficients are stored from params[NPARAMSTRUCT][] to params[NPARAMSTRUCT + 128][]
 * and packed into params[][] 2 coefs per location: odd (MS 16 bits), even coef (LS 16 bits).
 *
 */


/********* Globals Variables (values set in initialize()) ********************/
unsigned int port0_copy;
unsigned flash_data[0x2000];    /* (8192 bytes) used to hold data to program the FLASH */
int sw_down, sw_down_old, sw_pressed, sw_released; /* used by txrxint_c */
int gray_code, gray_code_old, cw, ccw, testcount;
int flash_locked;   /* lock programming of FLASH memory when set */
int cursor_pos, cursor_pos_vu, cursor_flag, flash_cursor_flag, down_turn_flag, press_flag;
int param_ptr, write_ptr, read_ptr, p_state, flag_options;
int index_ab;           /* flag to specify current menu state:
                        0 - function A
                        1 - function B
                        2 - function A&B (Common)
                        0 - Options section (and flag_options==1)                   */
int params_changed;     /* flag to indicate how params chaged:
                        0 - no change
                        1 - param_ptr changed (update only min_value and max_value)
                        2 - parameter has changed
                        3 - button press action has been confirmed                  */
int params_changed_copy;
int led_counter, vu_counter;
int max_in_level;   /* used to hold calibration constant */
long min_value, max_value;  /* used to limit min and max values of current parameter */
long p_long, p_sum, serial_number;  /* used in the parse function: */
unsigned p_uint1, p_uint2;
int sn_ok_flag, quietsn_flag;
int serial_error_flag;
unsigned data_count;
int sign_mult, index_ab_p;
long check_sum;
unsigned func_addr_temp_a, func_addr_temp_b;
float fsample;  /* current sampling rate in Hz*/
char serial_str[11];    /* serial number string of module (including check sums and terminating null) */
char serial_in_buf[SERIAL_BUF_LEN]; /* RS-232 serial input buffer */
char parameter_str[17], value_str[17];  /* used by cammand parser to hold incomming command strings */
char vu_chars[]={' ', 8, 9, 10, 11, 12, 13, 14, 15, 'X', 'X', 'X', 'X'};    /* used by vu_update() */
unsigned gopt_a, gopt_b, aopt_a, aopt_b;    /* optimal CODEC gain and attenuation settings */
unsigned in_a_vu_level, in_b_vu_level, out_a_vu_level, out_b_vu_level;
float scale_k_a, scale_k_b;
int auto_vu_count;
int iorder_old;
float window[128];      /* holds first half of pre-computed modified-Blackman-window */

#define RECORD_LENGTH   6 + NPARAMSM6       /*  Set to (6 + 6*NPARAMS) = (6 + 6*(46+128)) = 1050 */
unsigned record[RECORD_LENGTH];

/* float in_cal_levels_a[16], in_cal_levels_b[16], out_cal_a[32], out_cal_b[32];

    
/********* Globals pointers used for internal data RAM access: *****/
volatile unsigned int *imr_ptr, *greg_ptr, *ifr_ptr;

/***** External Variables to access assembly variables ********************/
extern int k7f00h, kf80fh, kfff0h;
extern int in_a, in_b, out_a, out_b, t_reg_scale_a, t_reg_scale_b, assembly_flag;
extern int in_a_hold, in_b_hold, out_a_hold, out_b_hold;
extern unsigned in_error, in_error_stick, in_digital, out_gain, out_atten, iosr_copy;
extern unsigned func_addr_a, func_addr_b, coef_ptr_a, coef_ptr_b, data_ptr_a, data_ptr_b;
extern unsigned orderm2_a, orderm2_b;
extern int fir_coef[256], coefdata[256];

/********* define I/O port variables **************************************/
ioport  unsigned    port0;      /* I/O port pins IO4-IO11 on module */
ioport  unsigned    portffe8;   /* CLK I/O address */
ioport  unsigned    portffec;   /* ICR I/O address */
ioport  unsigned    portfff0;   /* SDTR I/O address */
ioport  unsigned    portfff1;   /* SSPCR I/O address */
ioport  unsigned    portfff4;   /* ADTR I/O address */
ioport  unsigned    portfff5;   /* ASPCR I/O address */
ioport  unsigned    portfff6;   /* IOSR I/O address */
ioport  unsigned    portfff7;   /* BRD I/O address */
ioport  unsigned    portfff8;   /* TCR I/O address */
ioport  unsigned    portfff9;   /* PRD I/O address */
ioport  unsigned    portfffa;   /* TIM I/O address */
ioport  unsigned    portfffc;   /* WSGR I/O address */

/***** Declare external assembly functions ********************************/
extern void no_func_a(void);
extern void no_func_b(void);
extern void allpass_func_a(void);
extern void allpass_func_b(void);
extern void iir_4_a(void);
extern void iir_4_b(void);
extern void fir_15_a(void);
extern void fir_15_b(void);
extern void fir_16_a(void);
extern void fir_16_b(void);
extern void lattice_2_a(void);
extern void lattice_2_b(void);
extern void lattice_4_a(void);
extern void lattice_4_b(void);
extern void lattice_8_a(void);
extern void lattice_8_b(void);
extern void notch_a(void);
extern void notch_b(void);
extern void get_serial();

/***** Function Prototypes ************************************************/
void txrxint_c(void);
int inc_dec_param_ptr(void);
void inc_dec_mod(int *ptr, int mod);
void add_sub_pow(long *ptr, int pow);
void initialize(void);
void set_fsample(void);
void init_params(void);
void init_freq_params(void);
void reset_port(void);
void sign_on(void);
void reset_lcd(void);
void update_disp_left(void);
void update_disp_right(int pos);
void disp_text(char *text, int position, int cursor_on);
void disp_num(long num, int position, int width, int nfrac);
char* num2string(long num, int nfrac, int *strlength, char *str17);
void write_lcd_inst(int byte, int wait_usec);
void write_lcd_data(int byte, int wait_usec);
void write_lcd_nibble(int nibble, int wait_usec);
int delta_t(unsigned count_start);
void wait(long wait_usec);
void wait_n_samples(int n);
int prog_flash(unsigned start, unsigned length, unsigned *datawords, int erase_flag, char *error_text);
void read_flash(unsigned start, unsigned length, unsigned *datawords);
void compute_fir(float f1, float f2, int order, int index_ab_tmp);
void load_userfir(int iorder, int index_ab_tmp);
void compute_notch(float fn, float fw, int index_ab_tmp);
void xmit(char *text);
void parse_command(void);
void update_dsp(int param_ptr_tmp, int index_ab_tmp);
void set_all_gains(void);
void gain(int index_ab_tmp);
void vu_update(void);
unsigned convert_to_vu(float scale, int signal);
void led_update(void);
void store(void);
void recall(void);
void beep(unsigned duration, unsigned period);
int record_bad(void);
void store_all(void);
/* fcomplex Cadd(fcomplex a, fcomplex b);
fcomplex Csub(fcomplex a, fcomplex b);
fcomplex Cmul(fcomplex a, fcomplex b);
fcomplex Complex(float re, float im);
fcomplex Conjg(fcomplex z);
fcomplex Cdiv(fcomplex a, fcomplex b);
fcomplex Csqrt(fcomplex z);
fcomplex RCmul(float x, fcomplex a); */


/*========================================================================
 =
 =  main() 
 =
 =========================================================================*/
void main(void) 
{
unsigned count_start;
unsigned ptr;
int itemp;
float ftemp;

/* unsigned long idum;
unsigned int utemp; */

/* boot2(); /* Test the boot2 code */

initialize();   /* initialize DSP hardware */

/* temp = memtest(0x1000,0x7fff,0xabcd);    /* start, stop, value, returns: 0 if OK */


#if(MAIN)

#if(1)     /* fix: make 1 */
/* Check validity of FLASH data memory (sector 3), and write defaults if necessary: */
/* Search for end of record data in FLASH */
ptr = 0x6000;   /* point to first record */
read_flash(ptr, 5, record);     /* read part of record header */
itemp = 0;
if(record[0]==0xffff){
  store_all();  /* store current settings to all flash memory locations */
}
while(record[0]!=0xffff){
  /* Check for valid version and SN in each record: */
  if(record_bad()){
    store_all();    /* store current settings to all flash memory locations */
    break;
  }
  ptr = record[0];              /* point to next location */
  read_flash(ptr, 5, record);   /* read part of record header */
}


/* Recall settings from memory location 0: */
params[11][0] = 0;  /* load recall location value */
params_changed_copy = 3;
recall();           /* recall location 0 settings */


#if(0)  /* fix: set to 0 */
  /* multiple store() test: */
  params[10][0] = 0;    /* load store location value */
  params_changed_copy = 3; /* set from above */
  for(itemp=0;itemp<100;itemp++){
    store();
  } 
#endif

#endif  /* #if(1) */

#endif  /* #if(MAIN) */


sign_on();  /* display the sign-on message */


#if(0)
/* random number testers: */
idum = 0L;
idum = 1664525L*idum + 1013904223L;

utemp = 0;
utemp = 19*utemp + 15473;

#endif



#if(0)
/* fix: waste code memory: */
  for(itemp=0;itemp<1;itemp++){
    itemp = itemp;
    ftemp = pow(log10(itemp*3.12345),log(itemp));
  }
  for(itemp=0;itemp<1;itemp++){
    itemp = itemp;
    ftemp = pow(log10(itemp*3.12345),log(itemp));
  }
  for(itemp=0;itemp<1;itemp++){
    itemp = itemp;
    ftemp = pow(log10(itemp*3.12345),log(itemp));
  }
  for(itemp=0;itemp<1;itemp++){
    itemp = itemp;
    ftemp = pow(log10(itemp*3.12345),log(itemp));
  }
  for(itemp=0;itemp<1;itemp++){
    itemp = itemp;
    ftemp = pow(log10(itemp*3.12345),log(itemp));
  }
  for(itemp=0;itemp<1;itemp++){
    itemp = itemp;
    ftemp = pow(log10(itemp*3.12345),log(itemp));
  }

#endif


/**************************************************************************/
/***** Start forground loop: **********************************************/
while(1){   /* Loop here and flash cursor until a parmeter change is detected or */
            /* or untill serial data is received: */
  if(cursor_flag&&(!sw_down)){
/*    portfff5 &= ~0x0200;  /* suspend delta interrupts while updating display */
    disp_text("", cursor_pos, -1);  /* turn cursor off */
/*    portfff5 |= 0x0200;   /* re-enable delta interupts */
    if(auto_vu_count){  
      if((auto_vu_count++)>10){
        cursor_pos_vu = cursor_pos; /* save cursor position when auto VU is turned on */                        
/*        portfff5 &= ~0x0200;  /* suspend delta interrupts while updating display */
        disp_text(param_struct[1].text, 1, 0);  /* write "Levels:In  Out  " to display */
        disp_text("", cursor_pos_vu, 0);    /* put cursor back */
/*        portfff5 |= 0x0200;   /* re-enable delta interupts */
        assembly_flag |= 3;     /* turn on VU display (auto VU meter code 3) */
        auto_vu_count = (int)params[2][0];  /* restart counter; set to 0 or 1 depending on RevertToLevels */
      }
    }
  }
  else if(press_flag==0){   /* keep from turning on cursor when press action is pending */
/*    portfff5 &= ~0x0200;  /* suspend delta interrupts while updating display */
    disp_text("", cursor_pos, flash_cursor_flag);   /* turn cursor on, if flash_cursor_flag==1 */
/*    portfff5 |= 0x0200;   /* re-enable delta interupts */
  }
  
  for(itemp=0;itemp<CURSOR_PERIOD;itemp++){     /* loop for multiple of 5ms */
    count_start = portfffa;     /* grab current timer value (for loop interval timing) */
    while(delta_t(count_start)<5000){   /* loop until 5ms time interval is over */

      if(write_ptr!=read_ptr){  /* something in serial_in_buf[] */
        parse_command();        /* check if command complete, parse and execute */
        count_start = portfffa; /* grab current timer value to avoid delta_t() overflow (resets interval) */
      }

      if(serial_error_flag==1){ /* RS-232 comm. error in txrxint_c() */
        serial_error_flag = 0;  /* reset flag */
/*        disp_text("Reduce Baud Rate", 1, -1); */
/*        disp_text("Baud Rate Error!", 1, -1); */
        disp_text("RS-232 Error!   ", 1, -1);
        disp_text("", 1, 0);    /* send cursor home */
        beep(200, 400);
/*        assembly_flag &= ~3;  /* Turn off the VU Meter */
/*        auto_vu_count = (int)params[2][0];    /* restart counter; set to 0 or 1 depending on RevertToLevels */
        reset_port();
      }


#if(MAIN)
      if(params_changed){
        portfff5 &= ~0x0200;    /* suspend delta interupts so params_changed flag is not modifed here: */
        params_changed_copy = params_changed;   /* make working copy for update_dsp() */
        params_changed = 0;     /* reset change flag */
        portfff5 |= 0x0200;     /* re-enable delta interupts */
        update_dsp(param_ptr, index_ab);    /* update the DSP's function to reflect current parameters */
        count_start = portfffa; /* grab current timer value to avoid delta_t() overflow (resets interval) */
      }
#endif
    
    }   /* end while() */

#if(MAIN)
    led_update();           /* set overload LEDs to reflect the status of the overload bits */

    portfff5 &= ~0x0200;    /* suspend delta interupts while testing VU flag and vu levels to display vu_update() */
    if(assembly_flag&3){
      vu_update();          /* update VU meters (this is usually called every 5ms) */
    }
    portfff5 |= 0x0200;     /* re-enable delta interupts */

    rand();         /* Call the rand number generator so muliple modules will tend to get out of step.
                       Used for the random delay for the "sendsn" command. (usually called every 5ms) */
#endif

  } /* end for() */


  /* asm("  idle    ");         /* hold here and save power (untill interrupt returns) */
  /* asm("  rpt     #45 ");     /* test non-interuptable foreground 42 OK */
  asm(" nop         ");


}  /* END loop forever */

} /* end main */
/*========================================================================*/


/**************************************************************************
 *
 *  initialize(): Initialize DSP hardware
 *
 **************************************************************************/

void initialize(void)
{
int i, j, nstart, n_offset, nlength, nfrac, temp;
char ctemp;
unsigned flag_temp, cursor_temp, utemp;
unsigned* uptr;                                                 


asm("   setc    INTM    ; disable all interrupts    ");
asm("   setc    SXM     ; set sign extension bit (sign-extension on)");
asm("   clrc    CNF     ; map internal memory block B0 into Data space");
asm("   clrc    OVM     ; clear overflow mode bit => normal overflow (assumed by c)");
asm("   spm     0       ; set product shift mode to No Shift (assumed by c)");

/* Initialize global variables: */
sw_down=0, sw_pressed=0, sw_released=0; /* used by txrxint_c */
cw=0, ccw=0, testcount=0;
down_turn_flag=0, press_flag=0;
param_ptr=0, write_ptr=0, read_ptr=0, p_state=0, flag_options=0;
serial_error_flag = 0;  /* no RS-232 error */
led_counter=0, vu_counter=0;
in_a_vu_level=0, in_b_vu_level=0, out_a_vu_level=0, out_b_vu_level=0;
quietsn_flag = 0;
iorder_old=0;
flash_locked=1; /* lock programming of FLASH memory when set */
flash_cursor_flag = 1;  /* start with cursor flashing */
params_changed=2;   /* flag parameter has changed */
max_in_level=20;    /* set calibration constant */
min_value=LONG_MIN, max_value=LONG_MAX; /* limit min and max values of current parameter */

init_params();      /* init. params[][] */
set_fsample();      /* Set the CODEC sampling rate (must call after init_params()) */
init_freq_params();


/* Setup pointers to absolute internal data memory locations: */
imr_ptr = (volatile unsigned int *)4;   /* pointer to IMR (Interrupt Mask Register) */
greg_ptr = (volatile unsigned int *)5;  /* pointer to GREG (Global Memory Allocation Reg) */
ifr_ptr = (volatile unsigned int *)6;   /* pointer to IFR (Interrupt Flag Register) */

*imr_ptr = 0;       /* mask all interrupts */
*ifr_ptr = CLR_ALL; /* clear pending interrupts */
*greg_ptr = 0x0080; /* map flash memory to global data space 0x8000 to 0xffff */ 
portffe8 = 1;       /* CLK: turn off CLKOUT1 DSP pin to reduce noise */
portffec = 0x01c;   /* ICR: (Single-edge MODE, mask INT2) */
/* Set wait states (WSGR): I/O, DATA=0, HiProg=0, LowProg=0 (p. 8-15): */
portfffc = (IO_WAITS*0x0200 + 0*0x0040 + 0*0x0008 + 0);

/* Initialize assembly language constants and variables: */
k7f00h = 0x7f00;
kf80fh = 0xf80f;
kfff0h = 0xfff0;
/* MUTE, gainb, gaina: */
out_gain = 0*0x0400 + 0*0x0010 + 0;
/* attenb, attena: */
out_atten = 0*0x0200 + 0*0x0010;
/* VU meter peak hold variables: */
in_a_hold = in_b_hold = out_a_hold = out_b_hold = 0;
assembly_flag = 0;

/* Start the timer free running for wait() and delta_t() usage: */
portfff9 = 0xffff;      /* (PRD) set reload value */
portfff8 = 0x03fc;      /* (TCR) set prescaler for divide by 13 (CLKOUT1/13 = ~1.8904MHz) and reload timer */
portfff8 = 0x03cc;      /* (TCR) start timer */
                        /* TIM counter (portfffa) now counts down from 0xffff and repeats */

func_addr_a = (unsigned)&no_func_a;  /* set function A to none */
func_addr_b = (unsigned)&no_func_b;  /* set function B to none */

coef_ptr_a = 0x0380;    /* point to begining of Ch A IIR coef space */
coef_ptr_b = 0x0300;    /* point to benining of Ch B IIR coef space */

/* orderm2_a = FIR_LENGTH - 2;  /* FIR filter A order minus 2 */
/* orderm2_b = FIR_LENGTH - 2;  /* FIR filter B order minus 2 */


/* Load the Module's serial number (with check sums and terminating null) from code memory: */
get_serial(&serial_str);
/* carray[0] = 0;   /* put null at begining of temp array */
/* strncat(carray, serial_str, 6);  /* get first 6 characters of serial number string */
/* serial_number = atol(carray);    /* convert to long. Don't use atol, it's a long func. */
serial_number = 0;
for(i=0;i<6;i++){
  serial_number = 10*serial_number + (serial_str[i]&0x0f);
}

/* Initialize Liquid Crystal Display and sign on msg. */
port0_copy = 0x0080;    /* Initialize port0 (IO4-11) */
port0 = port0_copy;
reset_lcd();

/* Enable async. port (ASPCR), enable receive and delta interrupts,
   set auto baud-rate detect bit and make IO3 an output: */
reset_port();

/* Crystal CS4218 CODEC setup section: */
*ifr_ptr = CLR_ALL; /* clear pending interrupts again */
portfff1 = 0x4300;  /* SSPCR: Put port into reset, int. when FIFO has 4 words */
portfff1 = 0x4330;  /* SSPCR: Take out of reset */
*imr_ptr = EN_RINT|EN_TXRXINT;  /* Unmask Receive interrupt, Unmask SSP  int. */

auto_vu_count = (int)params[2][0];  /* restart counter; set to 0 or 1 depending on RevertToLevels */
index_ab = params[6][0] ? 0:2;  /* if Mode:A&B Common (params[6][0]==0) then index common params, else index A params */   


#if(MAIN)
/* Read the Module's parameter structure and write the housekeeping data: */
for(i=0;i<NPARAMSTRUCT;i++){
/* Search for first '#' or end-of-string and set nstart, n_offset, nlength and nfrac: */
  nstart = 15;
  nfrac = 0;
  n_offset = 0;
  for(j=0;j<=16;j++){
    ctemp = *(param_struct[i].text + j);
    if(nstart==15){
      if((ctemp=='#')||(ctemp=='_')){
        nstart = j;
        param_struct[i].flag &= 0x7fff; /* clear the text type bit */
      }
      else if(ctemp=='\0'){     /* '#' or '_' not found => "text text" type */
        nstart = j;
        param_struct[i].flag |= 0x8000; /* set the text type bit */
        break;
      }
    }
    else{   /* nstart!=15 */
      if(ctemp=='_'){
        n_offset = j;
      }  
      if(ctemp=='.'){
        nfrac = j;
      }
      else if(!((ctemp=='#')||(ctemp=='_')||(ctemp=='.'))){
        nlength = j - nstart;
        break;
      }
    }
  }
  if(n_offset){
    n_offset -= nstart;
  }
  if(nfrac){
    nfrac = nstart + nlength - nfrac - 1;   /* adjust nfrac to the number of fraction digits */
  }
  if(0x8000&param_struct[i].flag){  /* "text text" type */
    j = -1;
    while(*(char*)(*(char*)(param_struct[i].label + (++j)))!='\0'){} /* count number entries in label pointer */
    param_struct[i].flag = (0xc000&param_struct[i].flag)|(nstart<<10)|j; /* load .flag with --ss ss-- nnnn nnnn */
  }
  else{                             /* "text int" or "text float" type */
    param_struct[i].flag = (0xc000&param_struct[i].flag)|(nstart<<10)|(n_offset<<7)|(nlength<<3)|nfrac; /* load .flag with --ss ssoo olll lfff */
  }
}   /* end for(i=0;i<NPARAMSTRUCT;i++) */


/* Initialize calibration constants with ideal values. Table used, not pow() */
/*for(i=0;i<16;i++){
  in_cal_levels_a[i] = 0.815*pow(10.0, LOG2D4*i)/8.0;
  in_cal_levels_b[i] = 0.815*pow(10.0, LOG2D4*i)/8.0;
}
for(i=0;i<32;i++){
  out_cal_a[i] = 1.69*7.1*pow(10.0, -LOG2D4*i);
  out_cal_b[i] = 1.69*7.1*pow(10.0, -LOG2D4*i);
}  */

set_all_gains();    /* set optimal CODEC gains and attenuations */
#endif

asm("   clrc    INTM        ; Enable interrupts");
wait_n_samples(10); /* wait for ~10 sampling intervals (after turning on ints.) for in_error to update */
in_error_stick = 0; /* reset the sticky bits */
gray_code = 0x0003&portfff6;    /* put rotary encoder bits into gray_code so first sw action is taken */

srand(in_a + serial_number%32768);  /* seed the random number generator with an input sample value plus
                                       the modules serial # mod 2^15. Srand used for the "sendsn" command */

update_disp_left();     /* update display to reflect parameter settings */
update_disp_right(1);

} /* end initialize() */


/**************************************************************************
 * init_params
 * This function initializes the parameters in params[][].
 * Frequency related parameters are set to zero.
 *
 **************************************************************************/
void init_params(void)
{
int j;
unsigned* uptr;

/* load all of params[][] with zeros: */
uptr = (unsigned*)(&params[0][0]);
for(j=0;j<NPARAMSM6;j++){
  *uptr++ = 0;
}

params[0][0] = params[0][1] = params[0][2] = 1; /* Default FUNC: AllPass */
params[3][0] = 20;  /* FullScalIn:20Vpp */
params[4][0] = 1;   /* SampleRate:48KHz */

params[15][0] = params[18][0] = params[21][0] = params[27][0] = params[33][0] = params[36][0] = params[39][0] = params[42][0] = params[45][0] =
params[15][1] = params[18][1] = params[21][1] = params[27][1] = params[33][1] = params[36][1] = params[39][1] = params[42][1] = params[45][1] =
params[15][2] = params[18][2] = params[21][2] = params[27][2] = params[33][2] = params[36][2] = params[39][2] = params[42][2] = params[45][2] = 100; /* gains */

params[17][0] = params[20][0] = params[26][0] = params[32][0] = params[40][0] = 
params[17][1] = params[20][1] = params[26][1] = params[32][1] = params[40][1] = 
params[17][2] = params[20][2] = params[26][2] = params[32][2] = params[40][2] = 127; /* filtorder: 127 */

params[41][0] = params[41][1] = params[41][2] = 1;  /* first coef. pointer */
}


/**************************************************************************
 * init_freq_params
 * This function initializes the frequency related parameters in params[][].
 *
 **************************************************************************/
void init_freq_params(void)
{

params[16][0] = params[19][0] = params[22][0] = params[25][0] = params[28][0] = params[31][0] =
params[16][1] = params[19][1] = params[22][1] = params[25][1] = params[28][1] = params[31][1] =
params[16][2] = params[19][2] = params[22][2] = params[25][2] = params[28][2] = params[31][2] = 

params[34][0] = params[37][0] =
params[34][1] = params[37][1] =
params[34][2] = params[37][2] = 1000; /* fcut, f1, fwdth, fnotch */

params[23][0] = params[29][0] =
params[23][1] = params[29][1] =
params[23][2] = params[29][2] = 2000;   /* f2 */

params[24][0] = params[30][0] =
params[24][1] = params[30][1] =
params[24][2] = params[30][2] = 1500;   /* fcntr */

params[35][0] = params[38][0] =
params[35][1] = params[38][1] =
params[35][2] = params[38][2] = 1000; /* Notch width and Inv.notch width */

params[43][0] =
params[43][1] =
params[43][2] = 10000;  /* Sine freq (1000Hz) */

}


/**************************************************************************
 * set_fsample
 * This function sets fsample and the state of the XF pin.
 *
 **************************************************************************/
void set_fsample(void)
{
long ltemp;

/* out_gain |= 0x0400;      /* mute the outputs (still pops) */
if(params[4][0]){   /* if (desired sample rate = 48KHz) */
  asm("     clrc    XF      ; Set the CODEC sampling rate to 48 Ksps");
  fsample = 48000.0;        /* set new sampling rate */
}   
else{
  asm("     setc    XF      ; Set the CODEC sampling rate to 8 Ksps");
  fsample = 8000.0;         /* set new sampling rate */
}


/* wait_n_samples(200); /* wait for ~200 sampling intervals for CODEC to recalibrate */
/* out_gain &= ~0x0400; /* un-mute the outputs */
}


/**************************************************************************
 * reset_port
 * This function resets the async. port (ASPCR), enables receive and
 * delta interrupts, sets auto baud-rate detect bit and makes IO3 an output.
 *
 **************************************************************************/
void reset_port(void)
{
portfff5 = 0;       /* make shure port is reset */
portfff6 = 0x66f0;  /* IOSR: reset all bits */
portfff5 = ASPCR_URST;
portfff5 = 0;
#if(MAIN)
portfff5 = (ASPCR_URST|ASPCR_DIM|ASPCR_RIM|ASPCR_CAD|ASPCR_CIO3);
#else
portfff5 = (ASPCR_URST|          ASPCR_RIM|ASPCR_CAD|ASPCR_CIO3);
#endif
portfff6 = 0x66f0;  /* IOSR: reset all bits */
}


/**************************************************************************
 * sign_on
 * This function displays the sign on message.
 *
 **************************************************************************/
void sign_on(void)
{
#if(MAIN)

#if(SIGN_ON_FLAG_Versa_Filter)
disp_text("Versa-Filter2-20", 1, -1);
wait(1000000);  /* wait 1 sec. */
#endif

#if(SIGN_ON_FLAG_AccuQuest)
disp_text("   AccuQuest    ", 1, -1);
wait(1500000);  /* wait 1.5 sec. */

disp_text("  AQC5501-001   ", 1, -1);
wait(1500000);  /* wait 1.5 sec. */
#endif

/*disp_text("Firmware:  V", 1, 0);
disp_num(VERSION, 13, 4, 2);
wait(1000000);  /* wait 1 sec. */

/*disp_text("Serial No:", 1, 0);
disp_num(serial_number, 11, 6 ,0);
wait(1000000);  /* wait 1 sec. */

#if(1)  /* fix: set to 1 */
disp_text("Testing         ", 1, 0);
wait(100000);   /* wait .1 sec. */
disp_text(".", 9, 0);
wait(100000);   /* wait .1 sec. */
disp_text(".", 11, 0);
wait(100000);   /* wait .1 sec. */
disp_text(".", 13, 0);
wait(100000);   /* wait .1 sec. */
disp_text("OK", 15, 0);
wait(400000);   /* wait .4 sec. */
#endif

update_disp_left();     /* display parameter state (done in recall() below) */
update_disp_right(1);


#else   /* if not MAIN */
disp_text("Programming Mode", 1, -1);
wait(1000000);  /* wait 1 sec. */

disp_text("Firmware:  V", 1, 0);
disp_num(VERSION, 13, 4, 2);
wait(1000000);  /* wait 1 sec. */

disp_text("Serial No:", 1, 0);
disp_num(serial_number, 11, 6 ,0);
wait(1000000);  /* wait 1 sec. */ 

disp_text("Ready to program", 1, 0);
disp_text("", 16, 1);   /* turn on cusor in position 16 */
#endif
}


/**************************************************************************
 * parse_command
 * This function reads the serial_in_buf[] from the last read entry to the
 * write_ptr. The characters in the buffer are used to update the command
 * parser state and read the incomming command strings. When a valid command
 * is parsed it is executed.
 *
 **************************************************************************/
void parse_command(void)
{
int i, j, itemp, iflag, cursortemp, execute_flag=0, program_flag=0, prog_err_flag=0;
int param_ptr_temp, index_ab_temp, flag_options_temp, change_value_flag;
long param_value;
char *value_ptr, *cptr;
char ctemp, craw;
char carray[17];
unsigned utemp;

/*portfff5 &= ~0x0200;  /* suspend delta interupts while updating display (disp_xxxx below) */
cursortemp = cursor_pos;    /* save current cursor position */

while(write_ptr!=read_ptr){
  craw = serial_in_buf[read_ptr++]; /* get current character */
  ctemp = tolower(craw);            /* convert to lower case */
  read_ptr &= SERIAL_BUF_LEN-1; /* make shure that read_ptr was incremented modulo SERIAL_BUF_LEN */
/* disp_text("                ", 1, -1); */
/* disp_text(&ctemp, 1, -1); */
#if(1)  /* fix: set to 1 */
  switch(p_state){
  case 0:   /* Start of parse: */
    if(ctemp=='a') p_state = 1;
    sn_ok_flag = 0;
    break;
  case 1:   /* Received: a */
    if(ctemp=='t') p_state = 2;
    else if(ctemp=='a') p_state = 1;
    else p_state = 0;
    break;
  case 2:   /* Received: at */
    if(ctemp=='a') p_state = 3;
    else if(ctemp=='s') p_state = 5;
    else if(ctemp==' ') p_state = 2;
    else p_state = 0;
    break;
  case 3:   /* Received: at a */
    if(ctemp=='t') p_state = 2;
    else if(ctemp=='l') p_state = 4;
    else p_state = 0;
    break;
  case 4:   /* Received: at al */
    if(ctemp=='l'){
        sn_ok_flag = 1;
        p_state = 20;
      }
    else p_state = 0;
    break;
  case 5:   /* Received: at s */
    if(ctemp=='n') p_state = 6;
    else p_state = 0;
    break;
  case 6:   /* Received: at sn */
    if(ctemp==':') p_state = 7;
    else p_state = 0;
    break;
  case 7:   /* Received: at sn: */
    if(isdigit(ctemp)){
      p_state = 8;
      p_long = (long)(ctemp&0x0f);  /* convert character to number */
    }
    else if(ctemp==' ') p_state = 7;
    else p_state = 0;
    break;
  case 8:   /* Received: at sn:# */
    if(isdigit(ctemp)){
      p_long = 10*p_long + (long)(ctemp&0x0f);  /* convert character to number; next digit */
      p_state = 8;
    }
    else if(ctemp==' ') p_state = 8;
    else if(ctemp==','){
      if(p_long==serial_number){
        sn_ok_flag = 1;
        p_state = 20;
      }
      else{
        p_long = 0;
        p_state = 8;
      }
    }
    else{
      /* Modulo decrement read_ptr because current char could be the first char of command: */
      if((read_ptr--)==0){
        read_ptr = SERIAL_BUF_LEN-1;
      }
      if(p_long==serial_number){
        sn_ok_flag = 1;
      }
      else{
        sn_ok_flag = 0;
      }
      p_state = 20;
    }
    break;

  case 20:  /* Received valid header and serial number (eg. at sn:132001) */
    if(isalpha(ctemp)){ /* if ctemp is a letter */
      for(i=0;i<17;i++){
        parameter_str[i] = value_str[i] = '\0'; /* clear command strings */
      }
      parameter_str[0] = ctemp; /* load first char of parameter */
      p_state = 21;
    }
    else if(ctemp=='\r') p_state = 0;   /* carrage return */
    else p_state = 20;
    break;
  case 21:  /* Received first letter of parameter, now get rest... */
    if(ctemp=='\r'){    /* carrage return */
      execute_flag = 1;
      p_state = 0;
    }
    else if(ctemp==':'){    /* if parameter reception complete */
      p_state = 22;
      if(strncmp(parameter_str, "program",7)==0){;  /* if parameter_str == "program" */
        func_addr_a = (unsigned)&no_func_a;  /* set the functions to none to get maximum CPU time */
        func_addr_b = (unsigned)&no_func_b;  /* set the functions to none to get maximum CPU time */
        assembly_flag &= ~3;    /* turn off the VU Meter */
        /* auto_vu_count = (int)params[2][0];   /* restart counter; set to 0 or 1 depending on RevertToLevels */
        p_state = 40;
      }
    }
    else{
      i =  strlen(parameter_str);
      if(i<16){
        parameter_str[i] = ctemp;   /* load next available location with char */
      }
      else{
        p_state = 0;
      }    
    }
    break;
  case 22:  /* Received all of parameter (eg. at sn:132001 param:),
               now get value with UPPER or lower cases...              */
    if(ctemp=='\r'){    /* carrage return */
      execute_flag = 1;
      p_state = 0;
    }
    else if(ctemp==':'){    /* second ':', final params are comming (must be the 'func:UserFIR: # # ...') */
      func_addr_temp_a = func_addr_a;   /* save the current A function */
      func_addr_temp_b = func_addr_b;   /* save the current B function */
      func_addr_a = (unsigned)&no_func_a;   /* set the functions to none to get maximum CPU time */
      func_addr_b = (unsigned)&no_func_b;
      data_count = 0;   /* clear coef. order counter */
      p_state = 30;
    }
    else{
      i =  strlen(value_str);
      if(i<16){
        value_str[i] = craw;    /* load next available location with U/l char */
      }
      else{
        p_state = 0;
      }
    }
    break;

  /* Read user FIR parameters: */
  case 30:  /* Received 'param:value:' (likely: 'func:UserFIR:'), now get numbers */
    p_state = 31;
    if(ctemp==' ') p_state = 30;
    else if(ctemp=='-'){
      sign_mult = -1;
      p_long = 0;
    }
    else if(isdigit(ctemp)){
      sign_mult = +1;
      p_long = (long)(ctemp&0x0f);  /* convert character to number */
    }
    else if(ctemp=='\r') goto p_cont1;
    else p_state = 0;
    switch(parameter_str[0]){   /* look at first letter of parameter_str[] */
    case 'a':   /* 'afunc' likely */
      index_ab_p = 0;
      break;
    case 'b':   /* 'bfunc' likely */
      index_ab_p = 1;
      break;
    case 'f':   /* 'func' likely */
      index_ab_p = 2;
      break;
    default:
      p_state = 0;
      break;
    }   /* end switch(parameter_str[0]) */
    break;
  case 31:  /* Received 'func:UserFIR: # -' or 'func:UserFIR: # #' */
    if(isdigit(ctemp)){
      p_long = 10*p_long + (long)(ctemp&0x0f);  /* convert character to number; next digit */
    }
    else if((ctemp==' ')||(ctemp=='\r')){   /* coef. ready */
      p_state = 30;
      p_long = sign_mult*p_long;        /* change sign if nessasary */
      if(p_long>32767){                 /* hard limit values */
        p_long = 32767;
      }
      if(p_long<-32768){
        p_long = -32768;
      }
      utemp = NPARAMSTRUCT + (data_count>>1);   /* offset plus data_count divided by 2 */
      if((data_count++)&0x0001){        /* if data count odd: 1, 3, 5,... */
        params[utemp][index_ab_p] |= (p_long<<16);  /* load odd coef. into MS Byte of array */
      }
      else{ /* data count even: 0, 2, 4,... */
        params[utemp][index_ab_p] = p_long&0x0000ffff;      /* load even data into array */
      }
      if(params[6][0]==2){  /* if Ch A Only */
        itemp = 256;        /* allow long filters on Ch A */
      }
      else{
        itemp = 128;  
      }
      if(data_count>itemp){
        data_count = itemp;
        goto p_cont1;   /* got maximum number of filter taps */
      }
    }
    else p_state = 0;
    if(ctemp=='\r'){    /* if ctemp carrage return, then we are done getting FIR coefs */
p_cont1:
      if(data_count<3) data_count = 3;      /* 3 is the minimum filter order */
      params[40][index_ab_p] = data_count;  /* save order of the filter that was just loaded */
      params[41][index_ab_p] = 1L;  /* set coef. pointer to 1 when changing order */
      func_addr_a = func_addr_temp_a; /* reset the function */
      func_addr_b = func_addr_temp_b; /* reset the function */
      execute_flag = 1;
      p_state = 0;
    }
    break;

  /* Read FLASH programming parameters: */
  case 40:  /* Received "program:" string */
    if(ctemp==' ') p_state = 40;
    else if(ctemp=='s') p_state = 41;
    else p_state = 0;
    break;
  case 41:  /* Received "program: s". */
    if(ctemp==' ') p_state = 41;
    else if(ctemp==':') p_state = 42;
    else p_state = 0;
    break;
  case 42:  /* Received "program: s:" */
    if(isdigit(ctemp)){
      p_uint1 = (unsigned)(ctemp&0x0f); /* convert character to number */
      p_state = 43;
    }
    else if(ctemp==' ') p_state = 42;
    else p_state = 0;
    break;
  case 43:  /* Received "program: s:#" */
    if(isdigit(ctemp)){
      p_uint1 = 10*p_uint1 + (unsigned)(ctemp&0x0f);    /* convert character to number; next digit */
      p_state = 43;
    }
    else if(ctemp==' ') p_state = 43;
    else if(ctemp=='l') p_state = 44;
    else p_state = 0;
    break;
  case 44:  /* Received "program: s:# l". */
    if(ctemp==' ') p_state = 44;
    else if(ctemp==':') p_state = 45;
    else p_state = 0;
    break;
  case 45:  /* Received "program: s:# l:" */
    if(isdigit(ctemp)){
      p_uint2 = (unsigned)(ctemp&0x0f); /* convert character to number */
      p_state = 46;
    }
    else if(ctemp==' ') p_state = 45;
    else p_state = 0;
    break;
  case 46:  /* Received "program: s:# l:#" */
    if(isdigit(ctemp)){
      p_uint2 = 10*p_uint2 + (unsigned)(ctemp&0x0f);    /* convert character to number; next digit */
      p_state = 46;
    }
    else if(ctemp==' ') p_state = 46;
    else if(ctemp=='d') p_state = 47;
    else p_state = 0;
    break;
  case 47:  /* Received "program: s:# l:# d". */
    if(ctemp==' ') p_state = 47;
    else if(ctemp==':'){
      /* Check start and stop parameters */
      /* (must start at beginning of a sector and be one sector long max.) */
      if(((p_uint1==0x0000)||(p_uint1==0x2000)||(p_uint1==0x4000)||(p_uint1==0x6000)||
      (p_uint1==0x8000)||(p_uint1==0xa000)||(p_uint1==0xc000)||(p_uint1==0xe000))&&(p_uint2<=0x2000)&&
      (p_uint2>0)){
        if(sn_ok_flag){     /* fix: don't need because we will not get this far with invalid SN */
          disp_text("Recvd    0 Words", 1, -1);
        }
        data_count = 0;
        check_sum = 0;
        p_state = 48;
      }
      else{
        prog_err_flag = 1;
        p_state = 0;
      }
    }
    else p_state = 0;
    break;
  case 48:  /* Programming parameters valid, now load flash_data array..." */
    if(data_count<(p_uint2<<1)){
      utemp = data_count>>1;    /* divide data_count by 2 */
      if(data_count&0x0001){    /* if byte count odd: 1, 3, 5,... */
        flash_data[utemp] = flash_data[utemp] | craw;   /* load odd data into array */
        if(sn_ok_flag){     /* fix: don't need because we will not get this far with invalid SN */
          if(((utemp+1)&0x002f) == 0){
            disp_num((long)(utemp) + 1, 6, 5 ,0);   /* periodicaly update "Recvd     Words" */
          }
        }
      }
      else{ /* byte count even: 0, 2, 4,... */
        flash_data[utemp] = (craw<<8);  /* load even data into MS Byte of array */
      }
      check_sum += craw;                /* compute running sum */
      data_count++;
    }
    if(data_count==(p_uint2<<1)){
      if(sn_ok_flag){
        disp_num((long)((data_count-1)>>1) + 1, 6, 5 ,0);   /* update final "Recvd     Words" */
        wait(1000000);  /* wait 1 sec. */
      }
      data_count = 0;
      p_sum = 0;
      p_state = 49;
    }
    break;
  case 49:  /* Received FLASH data, now get transmitted check sum, and verify... */
    if(data_count<8){   /* receiving check sum */
      if(isdigit(ctemp)){
        p_sum = 10*p_sum + (long)(ctemp&0x0f);  /* convert character to number; next digit */
      }
      else if(ctemp==' ') p_state = 49;
      else{
        prog_err_flag = 1;
        p_state = 0;    /* space or digit expected */
      }
    }
    if((data_count++) == 7){    /* full check sum received, verify: */
      if(p_sum==check_sum){ /* received check sum == calculated check sum */
        program_flag = 1;   /* set flag to program FLASH */
      }
      else{ /* bad check sum */
        prog_err_flag = 1;
      }
      p_state = 0;
    }
    break;

  default:
    break;
  } /* end switch(p_state) */


/* Execute serial Command section: */

/*disp_text("                ", 1, -1); /* for testing only */
/*disp_text(parameter_str, 1, -1);
disp_text(value_str, 9, -1); */

#if(MAIN)

  if(sn_ok_flag && execute_flag){     /* Execute command in parameter_str and value_str (if valid) */
    if(strncmp(parameter_str, "reset", 5)==0){
      initialize(); /* initialize DSP hardware */
      /* sign_on(); /* display the sign-on message */
    }
    else if(strncmp(parameter_str, "display", 7)==0){
      if(value_str[0]!='\0'){   /* if value_str not empty display contents */
        disp_text("                ", 1, -1);
        disp_text(value_str, 1, -1);
        flash_cursor_flag = 0;  /* stop flashing currsor */
/*assembly_flag &= ~3;  /* turn off the VU Meter */
/*auto_vu_count = (int)params[2][0];    /* restart counter; set to 0 or 1 depending on RevertToLevels */
      }
      else{                     /* else go back to normal display */
        update_disp_left();     /* update display to reflect parameter settings */
        update_disp_right(1);
        params_changed = 1;     /* set change flag to update assembly_flag to turn on VU meters if nessasary */
        flash_cursor_flag = 1;  /* resume flashing currsor */
      }
    }
    else if(!quietsn_flag && strncmp(parameter_str, "sendsn", 6)==0){
      portfff5 &= ~0x0080;  /* suspend async. receive ints. so this module
                             will not hear itself or other modules talking.
                             This avoids serial receive buffer overflows. */
      wait(SENDSN_WAIT*(long)rand());   /* wait a random ammout of time, rand() is 0 to 32767 */
      xmit(serial_str);     /* send serial number with check sums out the RS-232 port */
    }
    else if(strncmp(parameter_str, "quietsn", 7)==0){
      quietsn_flag = 1;     /* don't send serial number on sucsesive "sendsn" commands */
    }
    else if(strncmp(parameter_str, "echo", 4)==0){
      disp_text("                ", 1, -1);
      disp_text(value_str, 1, -1);
      flash_cursor_flag = 0;    /* stop flashing currsor */
      xmit(value_str);          /* send value_str out the RS-232 port */
    }

    else{   /* standard serial command sent */
      /* Do parameter search here: */
      param_ptr_temp = param_struct_search(&itemp); /* search param_struct[i].text for parameter_str[] */
      if(!itemp){   /* if no match */
        goto parse_continue;    /* error in parameter_str[] */
      }
      flag_options_temp = 0;
      if(itemp==3){         /* if 1-offset: " APgain"=="APgain" */
        index_ab_temp = 2;  /* common section parameter found */
      }
      else if(itemp==1){    /* if 0-offset: "Store"=="Store" */
        index_ab_temp = 0;  /* options section parameter found */
        flag_options_temp = 1;
      }
      else if(itemp==2){    /* if 0-offset after first: " APgain"=="bAPgain" */
        if(parameter_str[0]=='a'){
          index_ab_temp = 0;
        }
        else if(parameter_str[0]=='b'){
          index_ab_temp = 1;
        }
        else{
          goto parse_continue;  /* error in parameter_str[] */
        }
      }
      
      /* Test if param_ptr_temp, index_ab_temp, and flag_options_temp are valid for
         the current module func and mode settings: */
      if(!flag_options_temp){   /* if received command is not in options section, do more tests: */
        itemp = (int)params[0][index_ab_temp];  /* get function code for received memory area */
        /* If received command does not correspond to current function, then error: */
        if(   (params[6][0]==0 && index_ab_temp!=2)     /* if Mode==A&B Common and no common param */
           || (params[6][0]!=0 && index_ab_temp==2) ){  /* if Mode!=A&B Common and common param */
           goto parse_continue; /* error in parameter_str[] */
        }     
        /* if received command not related to current function */
        if(param_ptr_temp&&(param_ptr_temp<param_ptr_start[itemp] || param_ptr_temp>param_ptr_end[itemp])){
           goto parse_continue; /* error in parameter_str[] */
        }     
      }
      /* A valid match to parameter was found. Now now get value: */
      value_ptr = &value_str[0];    /* point to value string */                                                   
      while(*value_ptr==' ') value_ptr++;   /* skip leading spaces value_str[] */
      param_value = 0;
      change_value_flag = (*value_ptr!='\0');   /* if value_str is not a null, change param value below */
      utemp = param_struct[param_ptr_temp].flag;    /* get the flag bits */  
      if(utemp>>15){    /* display type: "text text" */
        itemp = utemp&0xff; /* get nnnn nnnn */
/*        if(itemp==0) change_value_flag = 1;   /* if null_text label case, always set change flag */
/*   if(itemp==0) params_changed = 1;   /* if null_text label case, always update display */
        /* Search over the nnnn nnnn text label entries of the parameter match: */
        for(i=0;i<itemp;i++){   /* loop is skipped in null_text label case */
          param_value = -1;
          cptr = (char*)(*(char*)(param_struct[param_ptr_temp].label + i)); /* get pointer to beginning of ith parameter text */
          while(*cptr==' ') cptr++; /* skip leading spaces of label */
          if(string_compare(cptr, value_ptr)){  /* if match (a null matches anything) */
            param_value = (long)i;  /* return the param_value that points to the matching text */
            break;
          }
        }
        if(param_value<0){  /* if no match */
          goto parse_continue;  /* error in parameter_str[] */
        }
      }
      else{     /* display type: "text int" or "text float" */
        /* Convert value_str[] to a number: */
        if(iflag = (*value_ptr=='-')) value_ptr++;  /* if first char. '-', set minus sign flag  */
                                                    /* and skip character */
        itemp = (int)(0x0007&utemp); /* get fff, number of fractional digits */
        i = 0;
        do{     /* extract the number scaled by the number of fractional digits */
          ctemp = *value_ptr++; /* get character */
          j = isdigit(ctemp);
          if(i==0 && !j){   /* the first non-number (usualy a ".") starts the read counter */
            i = 1;
          }
          else{
            if(j){
              param_value = 10*param_value + (long)(ctemp&0x0f);
            }
            else{
              param_value = 10*param_value;
            }
            if(i) i++;
          }
        } while(i<=itemp);
        
        if(iflag){                      /* if negative flag */
          param_value = -param_value;   /* negate */
        }

        /* Check limits on numerical parameter value: */
        params_changed_copy = 1;            /* flag to update_dsp() to */
        update_dsp(param_ptr_temp, index_ab);   /* update min_value and max_value only */
        if(param_value>max_value){          /* hard limit values */
          param_value = max_value;
        }
        if(param_value<min_value){
          param_value = min_value;
        }
      }
      
      flag_options = flag_options_temp; /* load flag_options with return value of search */
      param_ptr = param_ptr_temp;       /* load param_ptr */
      index_ab = index_ab_temp;         /* load index_ab */

      
      if(change_value_flag){    /* update params[][] with param_value */
        params[param_ptr][index_ab] = param_value;  /* load new parameter value */
        params_changed = 3;     /* flag main loop to update DSP (because we changed parameter values) */
      }
      else{ /* no value was specified, so x-mit current value out the RS-232 port */
        param_value = params[param_ptr][index_ab];  /* get current param value */
        if(utemp>>15){  /* display type: "text text" */
          if(param_ptr_temp==1){        /* If "Levels" parameter, write current hold values: */
            /* xmit(num2string((long)(scale_k_a*(float)in_a_hold), 0, &i, (char*)&carray));
            xmit(num2string((long)(scale_k_b*(float)in_b_hold), 0, &i, (char*)&carray));
            xmit(num2string((long)(scale_k_a*(float)out_a_hold), 0, &i, (char*)&carray));
            xmit(num2string((long)(scale_k_b*(float)out_b_hold), 0, &i, (char*)&carray)); */
          }
          else if(param_ptr_temp==12){  /* If "Firmware" parameter, write version: */
            xmit(num2string(VERSION, 2, &i, (char*)&carray));
          }
          else if(param_ptr_temp==13){  /* If "Serial No" parameter, write SN: */
            xmit(num2string(serial_number, 0, &i, (char*)&carray));
          }
          else{     /* some other "text text" parameter */
            /* transmit current parameter text out the RS-232 port: */
            xmit((char*)(*(char*)(param_struct[param_ptr_temp].label + param_value)));
          }
        }
        else{       /* display type: "text int" or "text float" */
          itemp = (int)(0x0007&utemp); /* get fff, number of fractional digits */
          /* transmit current parameter value out the RS-232 port: */
          xmit(num2string(param_value, itemp, &i, (char*)&carray));
        }
        params_changed = 1;     /* flag main loop to update only min. and max. */
      }
      update_disp_left();           /* update display to reflect parameter settings */
      update_disp_right(1);
      flash_cursor_flag = 1;    /* resume flashing currsor */
/*assembly_flag &= ~3;  /* turn off the VU Meter */
/*auto_vu_count = (int)params[2][0];    /* restart counter; set to 0 or 1 depending on RevertToLevels */
    }   /* end parameter search */
    
    
parse_continue:
    execute_flag = 0;
  }
#endif  /* #if(MAIN) */


  if(sn_ok_flag && prog_err_flag){
    disp_text("PARAMETER ERROR ", 1, -1);
    wait(1000000);  /* wait 1 sec. */
    disp_text("WRITE ABORTED!  ", 1, -1);
    wait(2000000);  /* wait 2 sec. */
    update_disp_left();     /* update display to reflect parameter settings */
    update_disp_right(1);
    params_changed = 2;     /* flag main loop to update DSP (because we changed func_addr_a and _b above) */
    prog_err_flag = 0;
/*assembly_flag &= ~3;  /* turn off the VU Meter */
/*auto_vu_count = (int)params[2][0];    /* restart counter; set to 0 or 1 depending on RevertToLevels */
  }

  if(sn_ok_flag && program_flag){     /* Program FLASH */
/*auto_vu_count = (int)params[2][0];    /* restart counter; set to 0 or 1 depending on RevertToLevels */
/*    disp_text("                ", 1, -1);
    disp_text("start=",-1);
    disp_num(p_uint1,7,2,0);
    disp_text("len=", 10, -1);
    disp_num(p_uint2,14,2,0); */
    
    /* Check for valid serial number in flash_data: */
    if((p_uint1==0)||(p_uint1==32768)){ /* if programming sector 0 or 4 */
      j = 0;
      itemp = 0;
      for(i=SERIAL_LOC+2;i<SERIAL_LOC+7;i++){
        utemp = flash_data[i];
        carray[j++] = (char)((utemp&0xff00)>>8);    /* get upper byte from flash_data[] */
        carray[j++] = (char)(utemp&0x00ff);         /* get lower byte from flash_data[] */
        itemp += (utemp!=0x3030);                   /* scan for ASCII "00" in flash_data[] */
      }                                             /* itemp==0 if serial number was all zeros */
      
      if((itemp==0)||(!valid_serial(carray))){  /* if invalid serial number, substitute current: */
        j = 0;
        for(i=SERIAL_LOC+2;i<SERIAL_LOC+7;i++){
          itemp = (int)serial_str[j++]<<8;              /* get even serial number char */
          flash_data[i] = itemp|((int)serial_str[j++]); /* append odd serial number char */
        }
      }
    }
    
/*assembly_flag &= ~3;  /* turn off the VU Meter */
/*auto_vu_count = (int)params[2][0];    /* restart counter; set to 0 or 1 depending on RevertToLevels */

    disp_text("Programming...  ", 1, -1);
    flash_locked = 0;   /* unlock flash */
    itemp = prog_flash(p_uint1, p_uint2, flash_data, 1, "inUpdate");
    if(itemp==0){   /* if no error */
      disp_text("Sector   Prog OK", 1, -1);
      disp_num(p_uint1/8192, 7, 2 ,0);  /* update final Sector # */
/*    disp_text("Cycle power!    ", 1, -1); /* fix: send this message from PC after all sectors programmed */
/*    wait(1000000);    /* wait 1 sec. */

/*    update_disp_left();       /* update display to reflect parameter settings */
/*    update_disp_right(1);
    params_changed = 2;     /* flag main loop to update DSP (because we changed func_addr_a and _b above) */
      program_flag = 0;
    }
  }
#endif
}   /* end while(write_ptr!=read_ptr) */

/*disp_text("", cursortemp, 0); /* put cursor back */

if(cursor_pos!=cursortemp){     /* if cursor position was modified (by any disp_ above) put into pos. 1 */
  disp_text("", 1, 0);          /* put cursor in position 1 */
}

/*portfff5 |= 0x0200;   /* re-enable delta interupts */

}


/**************************************************************************
 * txrxint_c (delta interrupt service routine)
 * This is the c-code part of the interrupt service routine that handles
 * the user input actions on both the RS-232 and switch inputs.
 * 
 * This interrupt is initially handled in assembly so that the
 * sample interrupts can be quickly turned back on so no
 * samples will be missed.
 *
 **************************************************************************/
void txrxint_c(void)
{
int i, j, temp_pos, type, nstart, n_offset, nlength, nfrac, nlabel;
int io;
int sum2, io0_sum, io1_sum, io2_sum;
unsigned count_start;
long jj;
char carray[8];

/***** Get RS-232 Action: *************************************************/
if(iosr_copy&0x4000){   /* an "a" Detect Complete interrupt occurred */
  portfff5 &= ~ASPCR_CAD;   /* reset the "calibrate "a" detect" (CAD) bit in the ASPCR*/
  portfff6 = 0x4000;    /* reset the ""a" detect complete" (ADC) bit in the IOSR */
}

/* if(portfff6&0x2600){ /* a break, framing error, or receive overrun detected */
/*  disp_text("OVERRUN ERROR!  ", 1, -1);
  wait(1000000);
  disp_num(portfff6,0,16,0);
  wait(1000000);
}   /* for testing */

if(iosr_copy&0x0100){   /* a serial Data Ready interrupt occurred */
  assembly_flag &= ~3;  /* Turn off the VU Meter to help avoid loosing data
                           If the sample int. is bussy (full order filters, VU-meter, wtnoise on)
                           then serial data will be missed if a new character is received too soon.
                           38.4Kbaud seems to be the maximum in this case.                     */

  serial_in_buf[write_ptr++] = portfff4;    /* read the serial data ADTR (also resets the DR bit in the IOSR) */
  
  write_ptr &= SERIAL_BUF_LEN-1;    /* make shure that write_ptr was incremented mod SERIAL_BUF_LEN */

  auto_vu_count = (int)params[2][0];    /* restart counter; set to 0 or 1 depending on RevertToLevels */

    
  if(write_ptr==read_ptr || portfff6&0x2600){   /* Buffer was just overwritten or a break, framing error,
                                                   or receive overrun detected */
/*  if(write_ptr==read_ptr){    /* Buffer was just overwritten (FIX: garentee no overflow?) */
    serial_error_flag = 1;  /* flag serial comm. error for main() */
  }

  return;   /* get out as soon as possible (when full DSP utilization, we have been missing chars) */

}


#if(MAIN)
/***** Get Switch Action: *************************************************/
if(iosr_copy&0x00f0){   /* a delta (I/O pin state change) inturrupt occurred */
  /***** Set appropriate SW Action flags (sw_pressed, sw_released, cw, ccw): */
  sum2=0, io0_sum=0, io1_sum=0, io2_sum=0;  /* init. some vars. */

  io = iosr_copy;
  count_start = portfffa;       /* grab current timer value (for debounce interval timing) */
  while(delta_t(count_start)<SW_DEBOUNCE){  /* accumulate sw states over debounce interval (~1000uS) */
  /* while(sum<100) */
    io0_sum += 0x0001&io;       /* do runing sum of IO0 */
    io1_sum += 0x0001&(io>>1);  /* do runing sum of IO1 */
    io2_sum += 0x0001&(io>>2);  /* do runing sum of IO2 */
    io = portfff6;              /* read iosr */
    sum2++;
  }
  sum2 = sum2>>1;   /* divide sum by 2 */

  /* Update Switch and Rotary-Encoder Action parameters: */
  sw_down_old = sw_down;            /* keep sw_down history */
  sw_down = io2_sum<sum2;           /* get inverse of IO2, switch input bit */
  if(sw_down&&(!sw_down_old)){  /* determine if sw pressed or released based on states */
    sw_pressed = 1;
  }
  if((!sw_down)&&sw_down_old){
    sw_released = 1;
  }

#if(ENCODER_TYPE==0)    /* Panasonic */
  if(0x0010&iosr_copy){             /* quick return if only input 0 changed */
   return;
  }

  gray_code_old = gray_code; 
  gray_code = ((io1_sum>sum2)<<1)|(io0_sum>sum2);   /* get code for Rotary Encoder (0,1,3,2) */
  switch((gray_code_old<<2)|gray_code){
  case 3:
  case 12:
  case 7:
  case 8:
    ccw = 1;
    break;
  case 6:
  case 9:
  case 2:
  case 13:
    cw = 1;
    break;
  }
#else   /* Switch Channel */
  gray_code_old = gray_code; 
  gray_code = ((io1_sum>sum2)<<1)|(io0_sum>sum2);   /* get code for Rotary Encoder (0,1,3,2) */
  switch((gray_code_old<<2)|gray_code){
  case 7:
  case 8:
    cw = 1;
    break;
  case 4:
  case 11:
    ccw = 1;
    break;
  }
#endif

  /***** Done seting SW Action flags (sw_down, sw_pressed, sw_released, cw, ccw) *******/


  if(sw_pressed||cw||ccw){  /* if SW Action calls for a click sound: */
    port0_copy |= 0x0040;   /* Raise the IO10 (speaker bus) pin to click the speaker */
    port0 = port0_copy;
    auto_vu_count = (int)params[2][0];  /* restart counter; set to 0 or 1 depending on RevertToLevels */
  }

  /****** Update display and parameters in response to SW actions: *************/
  
  flash_cursor_flag = 1;    /* resume flashing currsor */
  
  nstart = (0x000f&(param_struct[param_ptr].flag>>10)) + 1; /* get nstart */
  type = param_struct[param_ptr].flag>>15;
  if(type){ /* display type: "text text" */
    n_offset = 0;
    nlength = 1;
    nfrac = 0;
  }
  else{     /* display type: "text int" or "text float" */
    n_offset = (0x0007&(param_struct[param_ptr].flag>>7));  /* get the cursor offset */
    nlength = 0x000f&(param_struct[param_ptr].flag>>3);
    nfrac = 0x0007&param_struct[param_ptr].flag;
  }
  
  if(sw_pressed&&(cursor_pos!=1)&&(0x4000&param_struct[param_ptr].flag)){ /* if press action param. */
    press_flag = 1; /* flag that knob was pressed on a press action param. */
    assembly_flag &= ~3;    /* turn off the VU Meter */
    disp_text("Press to        ", 1, -1);   /* write LCD and turn off cursor */
    /* Get current function text up to the ":" */
    for(i=0;i<7;i++){
      carray[i] = *(param_struct[param_ptr].text + i);
      if(carray[i]==':'){
        break;
      }
    }
    carray[i] = 0;  /* put null at end */
    disp_text(carray, 10, 0);   /* write function string to LCD */
    disp_text("",1, 0);         /* position cursor to left */
  }
  else if(press_flag){  /* if previosly flaged a press action */
    if(sw_pressed){
      params_changed = 3;   /* flag update_dsp() to carry out press action (code 3) */
      press_flag = 0;
    }
    else if(cw||ccw){
      disp_text("Cancelled       ", 1, 0);  /* write LCD */
      wait(750000);             /* wait .75 sec. */
      update_disp_left();       /* display parameter state */
      update_disp_right(1);
      press_flag = 0;
      /* make shure that the first knob turn after this msg. increments the display: */
      gray_code = 0x0003&portfff6;  /* put rotary encoder bits into gray_code so first sw action is taken */
      portfff6 = 0x00f0;            /* clear pending delta interrupts */
    }
  }
  else if(sw_pressed||(sw_down&&cw)){
    /* Increment cursor_pos to next valid position: */
    temp_pos = cursor_pos;
    if(temp_pos==1){
      temp_pos = nstart + n_offset;
    }
    else{
      ++temp_pos;
      if(temp_pos>=(nstart+nlength)){
        temp_pos = 1;
      }
      else if(nfrac&&(temp_pos==(nstart+nlength-nfrac-1))){ /* float with cursor on decimal point */
        ++temp_pos;
      }
    }
    disp_text("",temp_pos, +1); /* position cursor and turn on */
  } /* end if(sw_pressed||(sw_down&&cw)) */
  else if(sw_down&&ccw){
    /* Decrement cursor_pos to previous valid position: */
    temp_pos = cursor_pos;
    if(temp_pos==1){
      temp_pos = nstart + nlength - 1;
    }
    else{
      --temp_pos;
      if(temp_pos<nstart){
        temp_pos = 1;
      }
      else if(nfrac&&(temp_pos==(nstart+nlength-nfrac-1))){ /* float with cursor on decimal point */
        --temp_pos;
      }
    }
    disp_text("", temp_pos, +1);    /* position cursor and turn on */
  } /* end else if(sw_down&&ccw) */
  else if((cw||ccw)&&((assembly_flag&3)==3)){   /* if SW turned with auto VU mode active */
    assembly_flag &= ~3;    /* turn off the VU Meter */
    /* auto_vu_count = (int)params[2][0];   /* restart counter; set to 0 or 1 depending on RevertToLevels */
    update_disp_left();     /* write parameter text to LCD left */
    update_disp_right(cursor_pos_vu);   /* write parameter value to LCD right and reposition cursor */
    params_changed = 1; /* set change flag to update only min_value and max_value vars. */
  }
  else if((cw||ccw)&&(!down_turn_flag)){
    assembly_flag &= ~3;    /* turn off the VU Meter */
    /* auto_vu_count = (int)params[2][0];   /* restart counter; set to 0 or 1 depending on RevertToLevels */
    if(cursor_pos==1){  /* currsor on left */
      i = inc_dec_param_ptr();  /* increment/decrement param_ptr and update index_ab */
      update_disp_left();   /* write parameter text to LCD left */
      update_disp_right(1); /* write parameter value to LCD right */
      params_changed = 1;   /* set change flag to update only min_value and max_value vars. */
      if(i){    /* beep if a boudary was crossed in menu */
        beep(30, 550);
        wait(200000);   /* wait for human to respond to beep */
        /* make shure that the first knob turn after a beep increments the display: */
        gray_code = 0x0003&portfff6;    /* put rotary encoder bits into gray_code so first sw action is taken */
        portfff6 = 0x00f0;              /* clear pending delta interrupts */
      }
    }
    else{       /* currsor on right (somewhere), inc/dec parameter value: */
      if(type){ /* display type: "text text" */ 
        nlabel = 0x00ff&param_struct[param_ptr].flag;       /* get number of labels */
        i = (int)params[param_ptr][index_ab];   /* get current parameter value for A, B, or Common */
        inc_dec_mod(&i, nlabel);        /* and increment mod nlabel */
        params[param_ptr][index_ab] = (long)i;
      }     /* end if (type==1) */
      else{ /* type==0 */
        if(nfrac==0){   /* display type: "text int" */
          i = nstart + nlength - cursor_pos - 1;
        }
        else{           /* display type: "text float" */
          if(cursor_pos>=(nstart + nlength - nfrac)){   /* cursor to right of decimal point */
            i = nstart + nlength - cursor_pos - 1;
          }
          else{
            i = nstart + nlength - cursor_pos - 2;
          }
        }
        add_sub_pow(&params[param_ptr][index_ab], i);   /* add/sub 10^i to/from current parameter value */
      } /* end else type==0 */
      update_disp_right(cursor_pos);    /* write parameter value to LCD right */
      params_changed = 2;               /* set change flag to update all */
    } /* end else cursor on right (somewhere) */
  } /* end else if((cw||ccw)&&(!down_turn_flag)) */
  
  down_turn_flag = sw_down&&(cw||ccw);  /* set flag if knob was turned while pressed */
  
  /* Clear all SW Action flags: */
  sw_pressed = 0;
  sw_released = 0;
  cw = 0;
  ccw = 0;

  port0_copy &= ~0x0040;    /* Lower the IO10 (speaker bus) pin */
  port0 = port0_copy;

}   /* end if delta inturrupt */
#endif


}   /* return to asmembly and return from interrupt */


/**************************************************************************
 * reset_lcd
 *   This function resets the LCD, truns off the cursor and sends it home
 * and loads the CG RAM with bar graph characters.
 *
 **************************************************************************/
void reset_lcd(void)
{
int i, j;

/* Perform SW reset of display (incase hardware reset failed): */
write_lcd_nibble(0x3, 4100);    /* send 4 bit nibble to LCD instruction reg. */
write_lcd_nibble(0x3, 100);     /* send 4 bit nibble to LCD instruction reg. */
write_lcd_nibble(0x3, 100);     /* send 4 bit nibble to LCD instruction reg. */
write_lcd_nibble(0x2, 100);     /* send 4 bit nibble to LCD instruction reg. */

/* Set interface to 4 bits and put the display in known state: */
write_lcd_inst(0x28, 40);       /* Function Set: 4 bit data length, 5x7 dots */
write_lcd_inst(0x0c, 40);       /* Display On, Cursor Off */
cursor_flag = 0;                /* Clear cursor_flag */
flash_cursor_flag = 1;          /* normaly flash cursor */
write_lcd_inst(0x01, 1640);     /* Clear Display */
write_lcd_inst(0x06, 40);       /* Entry Mode Set: Increment cursor, no shift */
write_lcd_inst(0x02, 1640);     /* Cursor Home */

/* Load thermometer bar code graph into Character Generator RAM: */
write_lcd_inst(0x40, 40);   /* Set Character Generator Address to beginning */
for(i=7;i>=0;i--){
  for(j=1;j<9;j++){
    if(j<=i){
      write_lcd_data(0x00, 40); /* bar off */
    }
    else{
      write_lcd_data(0x0e, 40); /* bar on */
    }
  }
}

write_lcd_inst(0x80, 40);   /* Set Display Data Address to beginning (home) */
cursor_pos = 1;     /* Set global variable to current cursor position */

}

/**************************************************************************
 * disp_text
 *   This function writes a text string to the LCD.
 *
 *   *text      -   Text string to write to LCD display
 *                  (if null string -> cursor put into position)
 *   position   -   1 to 16 - Position of first character
 *                  Final cursor position: one character to the right of the
 *                  end of the string.
 *                  (out-of-limits not checked)
 *  cursor_on   -   Turns on/off curssor:
 *                  +1 - turns on cursor
 *                  -1 - turns off cursor
 *                   0 - cursor visibility not affected
 *
 * Possible speedup: Only update the part of the display that changed.
 *
 *************************************************************************/
void disp_text(char *text, int position, int cursor_on)
{
unsigned port_temp;

port_temp = portfff5;   /* save state of portfff5 */
portfff5 &= ~0x0200;    /* Suspend delta interrupts while updating display
                           This is because the delta int. also calls disp_text() and wait(). */

/* Turn on/off cursor: */
if((cursor_on>0)&&(!cursor_flag)){
  write_lcd_inst(0x0e, 40);     /* turn on cursor */
  cursor_flag = 1;              /* flag cursor on */
}
if((cursor_on<0)&&(cursor_flag)){
  write_lcd_inst(0x0c, 40);     /* turn off cursor */
  cursor_flag = 0;              /* flag cursor off */
}

if(position!=cursor_pos){   /* Put cursor in "position" if not there */
  if(position<9){
    write_lcd_inst((0x80 - 1) + position, 40);
    }
  else{
    write_lcd_inst((0xc0 - 9) + position, 40);
  }
  cursor_pos = position;
}
/* Write characters: */
while(*text!='\0'){ /* loop while not end of string (not null character) */
  write_lcd_data(*text++, 40);
  cursor_pos++;
  if(cursor_pos==9){
    write_lcd_inst(0xc0, 40);/* put the cursor in location 8 */
  }
}
portfff5 = port_temp;   /* restore state of portfff5 (could unmask delta interrupts) */
}


 /**************************************************************************
 * disp_num
 *   This function writes a number to the LCD. The number is a long and
 * can appear as an integer: e.g. 123 or as a float: e.g. 32.45
 *
 *   num        -   number to write to LCD display
 *   position   -   1 to 16 - Position of first character (#, -, or space)
 *                  Final cursor position: one character to the right of the
 *                  end of the string
 *                  (out-of-limits not checked)
 *   width      -   Total field width of displed number
 *                  e.g. for num = -26745 (and nfrac=0)
 *                  width=6 -> "-26745"
 *                        7 -> " -26745"
 *                        5 -> "*****"
 *  nfrac       -   number of nfractional digits, number of digits to the right of the
 *                  decimal point.
 *                  e.g. for num = -26745 (and width=7)
 *                  nfrac=0 ->  " -26745"
 *                  nfrac=1 ->  "-2674.5"
 *                  nfrac=5 ->  "-.26745"
 *                  nfrac=6 ->  "-.02674" ?
 *
 **************************************************************************/
void disp_num(long num, int position, int width, int nfrac)
{

char tempc[17];
char *text;
int i, strlength;

text = num2string(num, nfrac, &strlength, (char*)&tempc);   /* convert number to string */

/* Check for field overflow and write "***...**" if overfull: */
if(strlength>width){
  text = &tempc[16];    /* point to last charracter in array */
  for(i=0;i<width;i++){
    *--text = '*';      /* fill with *'s if field overflowed */
  }
}
/* Add leading spaces if necesary: */
while(width>strlength++){
  *--text = ' ';
}      

/* Display text at *bufptr */
disp_text(text, position, 0);

}


 /**************************************************************************
 * num2string
 *   This function converts a number to a string. The number is a long and
 * is converted as an integer: e.g. 123 or as a float: e.g. 32.45
 *
 *  num         -   number to write to convert
 *  nfrac       -   number of nfractional digits, number of digits to the right of the
 *                  decimal point.
 *                  e.g. for num = -26745
 *                  nfrac=0 ->  "-26745"
 *                  nfrac=1 ->  "-2674.5"
 *                  nfrac=5 ->  "-.26745"
 *  *strlength  -   returns length of resulting string (not counting terminating null)
 *  *str17      -   pointer to begining of string array buffer
 *                  (Important: length must be 17 elements)
 *
 **************************************************************************/
char* num2string(long num, int nfrac, int *strlength, char *str17)
{
char *bufptr;
int neg = num < 0;
long unum = neg ? -num : num;   /* take absolute value */

bufptr = (str17+16);    /* point to end of charracter array */
*bufptr = 0;            /* put null character at end of charracter array */
*strlength = 0;         /* start string length counter */
/* Convert integer and load array: */
do{
  *--bufptr = (unum % 10) + '0';    /* next LS digit, add to code for 0 and store in array */
  (*strlength)++;           /* inc. strlength */
  if(*strlength==nfrac){
    *--bufptr = '.';        /* insert decimal point if fraction */
    (*strlength)++;         /* inc. strlength */
    nfrac = 0;              /* flag fact that decimal point has been inserted */
  }
} while((unum/=10)||nfrac);

/* Insert minus sign if negative: */
if(neg){
  *--bufptr = '-';
  (*strlength)++;           /* inc. strlength */
}

return bufptr;
}

/**************************************************************************
 * write_lcd_inst
 *   This function writes a byte into the LCD's instruction register (RS low)
 *
 *   byte       -   input instruction byte
 *   delay_usec -   wait time after instruction write (in micro-seconds)
 *                  valid range: 10 to 32,767 usec
 *                               0 - for minimal delay
 *
 *   This function does not efect the 2 MSBs of port0 (IO10 and IO11).
 **************************************************************************/
void write_lcd_inst(int byte, int wait_usec)
{
int i, iend, nibh, nibl;

nibh = (byte&0xf0)>>4;  /* get high and low nibbles */
nibl = byte&0x0f;

port0_copy = port0_copy&(~0x10);        /* Lower the RS pin */
port0 = port0_copy;


port0_copy = (0xd0&port0_copy)|0x20|nibh; /* Raise E pin on LCD and write low */
port0 = port0_copy;

port0_copy = (0xd0&port0_copy)|nibh;    /* Lower E pin on LCD and hold low */
port0 = port0_copy;


port0_copy = (0xd0&port0_copy)|0x20|nibl; /* Raise E pin on LCD and write high */
port0 = port0_copy;

port0_copy = (0xd0&port0_copy)|nibl;    /* Lower E pin on LCD and hold high */
port0 = port0_copy;

wait((long)wait_usec);                      /* wait here for LCD not bussy */
}

/**************************************************************************
 * write_lcd_data
 *   This function writes a byte into the LCD's data register (RS high)
 *
 *   byte       -   input data byte
 *   wait_usec -    wait time after data write (in micro-seconds)
 *                  valid range: 10 to 32,767 usec
 *                               0 - for minimal delay
 *
 *   This function does not efect the 2 MSBs of port0 (IO10 and IO11).
 **************************************************************************/
void write_lcd_data(int byte, int wait_usec)
{
int i, iend, nibh, nibl;

nibh = (byte&0xf0)>>4;  /* get high and low nibbles */
nibl = byte&0x0f;

port0_copy = port0_copy|0x10;       /* Raise the RS pin */
port0 = port0_copy;


port0_copy = (0xd0&port0_copy)|0x20|nibh; /* Raise E pin on LCD and write low */
port0 = port0_copy;

port0_copy = (0xd0&port0_copy)|nibh;    /* Lower E pin on LCD and hold low */
port0 = port0_copy;


port0_copy = (0xd0&port0_copy)|0x20|nibl; /* Raise E pin on LCD and write high */
port0 = port0_copy;

port0_copy = (0xd0&port0_copy)|nibl;    /* Lower E pin on LCD and hold high */
port0 = port0_copy;

wait((long)wait_usec);                      /* wait here for LCD not bussy */
}

/**************************************************************************
 * write_lcd_nibble
 *   This function writes a 4-bit nibble into the LCD's instruction register
 *
 *   nibble     -   input instruction nibble
 *   delay_usec -   wait time after instruction write (in micro-seconds)
 *                  valid range: 10 to 32,767 usec
 *                               0 - for minimal delay
 *
 *   This function does not efect the 2 MSBs of port0 (IO10 and IO11).
 **************************************************************************/
void write_lcd_nibble(int nibble, int wait_usec)
{
int i, iend;

port0_copy = port0_copy&(~0x10);        /* Lower the RS pin */
port0 = port0_copy;

port0_copy = (0xd0&port0_copy)|0x20|nibble; /* Raise E pin on LCD and write */
port0 = port0_copy;
port0_copy = (0xd0&port0_copy)|nibble;  /* Lower E pin on LCD and hold */
port0 = port0_copy;

wait((long)wait_usec);                      /* wait here for LCD not bussy */
}



/**************************************************************************
 * delta_t
 * Returns time in ~microseconds since count_start.
 * In this case a ~microsecond is about 1.058 true microseconds.
 * Returns 0 to 32767. Rolls over if more that 32767us has ellapsed.
 *
 **************************************************************************/
int delta_t(unsigned count_start)
{
int itemp;

itemp = (count_start>>1) - (portfffa>>1);       /* compute difference in ~microseconds */
                                                /* if itemp pos. => timer didn't wrap around */
                                                /* if itemp neg. => timer wraped, correct below: */
return ((itemp>=0) ? itemp:(itemp + 0x8000));   /* compute and return ellapsed time */

}


/**************************************************************************
 * wait
 * Wait for wait_usec ~microseconds. In this case a ~microsecond is about
 * 1.0579 true microseconds. Call overhead and ISR's could slightly
 * extend the wait time.
 *
 *                  valid range: ~20 to 500,000,000 usec
 *                               0 - for minimal delay 
 *
 **************************************************************************/
void wait(long wait_usec)
{
int i, wait_course, wait_fine;
unsigned count_start;

/* Get course and fine time variables: */
wait_course = (int) (wait_usec>>14);        /* get course wait (multiples of 16384) */
wait_fine = (int) (wait_usec&0x00003fff);   /* get fine wait time */

count_start = portfffa;     /* grab current timer value (for interval timing) */
while(delta_t(count_start)<wait_fine);

if(wait_course==0) return;

/* Wait for an additional (wait_course*16384us): */
for(i=0;i<wait_course;i++){
  count_start = portfffa;       /* grab current timer value (for interval timing) */
  while(delta_t(count_start)<16384);
}
  

}

/**************************************************************************
 * wait_n_samples
 * Wait for n sampling intervals 
 *
 **************************************************************************/
void wait_n_samples(int n)
{
wait(n*1000000/((long)fsample));    /* wait for ~four sampling intervals for CODEC updates */

}


/**************************************************************************
 * prog_flash
 * IMPORTANT: set func_addr_a and _b to no_func_a and _b before calling
 * (because of added wait states below)!
 * This function programs the Am29F010 FLASH memory. 32K Bytes of FLASH memory
 * is mapped to the global address space of the DSP starting at address 0x8000.
 * One of four possible blocks of FLASH are mapped to the DSP depending on the
 * IO2 and CLIPA pin levels. 32K Bytes represents two independently erasable FLASH
 * sectors.
 *
 * This function stores 16 bit words in two consecutive FLASH locations
 * with high byte stored first.
 *
 * The FLASH is logically organized into words as follows:
 * Address range        IO2 CLIPA   Use         Sector
 * 0x0000 to 0x1fff     1   0   -   CODE (Main) sector 0 (8K words)(see p. 4-16 Users Guide for required format)
 * 0x2000 to 0x3fff             -   CODE (Main) sector 1 (8K words)
 *
 * 0x4000 to 0x5fff     1   1   -   CODE (Main) sector 2 (8K words)
 * 0x6000 to 0x7fff             -   DATA        sector 3 (8K words)
 *
 * 0x8000 to 0x9fff     0   0   -   CODE (Backup) sector 4 (8K words) - Boots when button down
 * 0xa000 to 0xbfff             -   CODE (Backup) sector 5 (8K words)
 *
 * 0xc000 to 0xdfff     0   1   -   CODE (Backup) sector 7 (8K words)
 * 0xe000 to 0xffff             -   SPARE       sector 8 (8K words)
 *
 *  start   -   Starting word location of FLASH storage to program (0 to 0xffff)
 *  length  -   Length in words to program (1 to 0x2000)
 *              (don't span sectors with input range)
 *  datawords - Pointer to data array
 *  erase_flag- 1 - erases (sets to 0xffff's) entire sector before programming
 *              0 - programs without erassing (only use if known to be errased)
 *  error_text- Text pointer to indicate the location that prog_flash was called from.
 *              should be 8 characters. eg. "in Store"
 *
 * Returns:     0 - programmed OK
 *              1 - programming error
 *
 *
 *  Error codes:0 - programmed OK
 *              1 - parameter error (or flash_locked = 1)
 *              2 - erase timeout error
 *              3 - erase verify error
 *              4 - programming 0->1 error, must erase first
 *              5 - programming timeout error
 *              6 - programming DATA section error: can't set CLIPA bit
 *                  (or FLASH was incorrectly programmed preveiously)
 *
 **************************************************************************/
prog_flash(unsigned start, unsigned length, unsigned *datawords, int erase_flag, char *error_text)
{
unsigned i, j, status1, status2, word, byte, xbyte, sector_start, port_temp;
char carray[10];
long error_code=0;

if(flash_locked){   /* return */
  error_code = 1;
  goto prog_error;
}

port_temp = portfff5;   /* save state of portfff5 */
portfff5 &= ~0x0200;    /* mask delta interrupts (if not) */
portfff5 |= 0x0004;     /* make IO2 an output (if not) */

if(start<0x4000){       /* set: IO2 = 1, CLIPA = 0 */
  portfff6 = (portfff6&0x000f)|0x0004;  /* and set IO2 */
  out_atten &= ~0x000c; /* clear CLIP LEDs (if not) */
}
else if(start<0x8000){  /* set: IO2 = 1, CLIPA = 1 */
  start -= 0x4000;
  portfff6 = (portfff6&0x000f)|0x0004;  /* and set IO2 */
  out_atten |= 0x000c;  /* set CLIP LEDs */
}
else if(start<0xc000){  /* set: IO2 = 0, CLIPA = 0 */
  start -= 0x8000;
  portfff6 &= 0x000b;   /* and clear IO2 */
  out_atten &= ~0x000c; /* clear CLIP LEDs (if not) */
}
else{                   /* set: IO2 = 0, CLIPA = 1 */
  start -= 0xc000;
  portfff6 &= 0x000b;   /* and clear IO2 */
  out_atten |= 0x000c;  /* set CLIP LEDs */
}

wait_n_samples(20);     /* wait for ~20 sampling intervals for CLIP signals to propagate */
                        /* wait for I02 state to settle (4 RC's = 4*10K*0.01uF) */
                        
if(((start<0x2000)&&((start+length)>0x2000))||((start>=0x2000)&&((start+length)>0x4000))){
  error_code = 1;   /* parameter error */
  goto prog_error;
}

*greg_ptr = 0x0080; /* map flash memory to global data space: 0x8000 to 0xffff */ 
portfffc = (IO_WAITS*0x0200 + FLASH_WAITS*0x0040 + 0*0x0008 + 0);   /* Increase # wait states in data mem. */
/* Warning increasing wait states can keep the ISR from completing */

*((unsigned int *)(0x8000+0x5555)) = 0x00aa;    /* put FLASH into normal read state */
*((unsigned int *)(0x8000+0x2aaa)) = 0x0055;
*((unsigned int *)(0x8000+0x5555)) = 0x00f0;

if(out_atten&0x0008){   /* If CLIPA should be set: */
  /* Verify that CLIPA signal was actualy set by making sure that the FLASH serial number */
  /* is not present in FLASH memory space. This avoids accidently overwriting CODE memory. */

  /* Read serial number section from FLASH: */
  j = 0;
  for(i=(2*SERIAL_LOC)+4;i<(2*SERIAL_LOC)+4+10;i++){
    carray[j++] = (*((unsigned int *)(0x8000+i)))&0x00ff;   /* get bytes from FLASH */
  }
  if(valid_serial(carray)){ /* error if valid serial number found here */
    error_code = 6;
    goto prog_error;
  }
}

if(erase_flag){ /* Erase FLASH sector(s): */
  if(start<0x2000){
    sector_start = 0x0000;
  }
  else{
    sector_start = 0x4000;
  }
  
  /* Erase sector 0 or 1 (depending on sector_start): */
  *((unsigned int *)(0x8000+0x5555)) = 0x00aa;  /* write unlock code to FLASH */
  *((unsigned int *)(0x8000+0x2aaa)) = 0x0055;
  *((unsigned int *)(0x8000+0x5555)) = 0x0080;
  *((unsigned int *)(0x8000+0x5555)) = 0x00aa;
  *((unsigned int *)(0x8000+0x2aaa)) = 0x0055;
  *((unsigned int *)(0x8000+sector_start)) = 0x0030;    /* write command to erase sector 0 */
  wait(100);    /* wait 100 uS */
  do{
    status1 = *((unsigned int *)(0x8000+sector_start)); /* read two consecutive status bytes */
    status2 = *((unsigned int *)(0x8000+sector_start));
    if(((status1&0x0040)==(status2&0x0040))||(status1&0x0080)){ /* no change in DQ6: done toggling */
      goto pf_1;                                                /* or DQ7==1 */
    }
  }
  while(!(status1&0x00020));    /* loop unless timeout (DQ5==1) */
  status1 = *((unsigned int *)(0x8000+sector_start));   /* read two consecutive status bytes */
  if(status1&0x0080){   /* DQ7==1 */
    goto pf_1;
  }
  error_code = 2;   /* erase timeout error */
  goto prog_error;
 pf_1:
  *((unsigned int *)(0x8000+0x5555)) = 0x00aa;  /* put FLASH into normal read state */
  *((unsigned int *)(0x8000+0x2aaa)) = 0x0055;
  *((unsigned int *)(0x8000+0x5555)) = 0x00f0;
  /* Varify FLASH erased: */
  for(i=sector_start;i<(0x4000+sector_start);i++){
    if(((*((unsigned int *)(0x8000+i)))&0x00ff)!=0x00ff){
      error_code = 3;   /* not erased error */
      goto prog_error;
    }
  }
}

/* Program FLASH: */
j = 0;
start = (start<<1);             /* set start to point to the starting word in FLASH */
length = start + (length<<1);   /* set length to last FLASH address + 1 */
for(i=start;i<length;i++){
  if(i&0x0001){ /* i odd */
    byte = word&0x00ff;         /* get LS byte */
  }
  else{     /* i even */
    word = datawords[j++];      /* get current word */
    byte = word>>8;             /* get MS byte */
  }
  *((unsigned int *)(0x8000+0x5555)) = 0x00aa;  /* put FLASH into normal read state */
  *((unsigned int *)(0x8000+0x2aaa)) = 0x0055;
  *((unsigned int *)(0x8000+0x5555)) = 0x00f0;
  xbyte = (*((unsigned int *)(0x8000+i))^byte)&0x00ff;  /* current FLASH byte XOR with desired data */
  if(xbyte){        /* data is different: must reprogram byte */
    if(xbyte&byte){ /* error: can't program 0 bit to 1 bit */
      error_code = 4;   /* programming 0 to 1 error */
      goto prog_error;
    }
    *((unsigned int *)(0x8000+0x5555)) = 0x00aa;    /* write unlock code to FLASH */
    *((unsigned int *)(0x8000+0x2aaa)) = 0x0055;
    *((unsigned int *)(0x8000+0x5555)) = 0x00a0;
    *((unsigned int *)(0x8000+i)) = byte;   /* program FLASH byte */
    do{ /* !data polling */
      status1 = *((unsigned int *)(0x8000+i));
      status2 = *((unsigned int *)(0x8000+i));
      if(((status1&0x0040)==(status2&0x0040))||((status1&0x0080)==(byte&0x0080))){  /* no change in DQ6 */
        goto pf_3;                                                                  /* or DQ7==DATA7 */
      }
    }
    while(!(status1&0x00020));  /* loop unless timeout (DQ5==1) */
    status1 = *((unsigned int *)(0x8000+i));
    if((status1&0x0080)==(byte&0x0080)){        /* done */
      goto pf_3;
    }
    error_code = 5; /* programming timeout error */
    goto prog_error;
  }
  pf_3:
  continue;
}

prog_out:
portfffc = (IO_WAITS*0x0200 + 0*0x0040 + 0*0x0008 + 0); /* back to 0 wait states in data mem. */

portfff5 &= ~0x0004;    /* make IO2 an input */
wait(400);              /* wait for I02 to go high (4 RC's = 4*10K*0.01uF) */
portfff6 = 0x00f0;      /* clear pending delta interrupts */
portfff5 = port_temp;   /* restore state of portfff5 (could unmask delta interrupts) */
out_atten &= ~0x000c;   /* clear CLIP LEDs (if not) */
flash_locked=1;         /* lockout accidental FLASH programming */

if(error_code){
  return 1;             /* error */
}
else{
  return 0;             /* success */
}


prog_error:
*((unsigned int *)(0x8000+0x5555)) = 0x00aa;    /* put FLASH into normal read state */
*((unsigned int *)(0x8000+0x2aaa)) = 0x0055;
*((unsigned int *)(0x8000+0x5555)) = 0x00f0;

/*portfff5 &= ~0x0200;  /* suspend delta interrupts while updating display */
disp_text("ProgERR", 1, -1);
disp_num(error_code, 8, 1, 0);
disp_text(error_text, 9, -1);
/*portfff5 |= 0x0200;   /* re-enable delta interupts */

wait(2000000);
goto prog_out;

}


/**************************************************************************
 * read_flash
 * IMPORTANT: set func_addr_a and _b to no_func_a and _b before calling
 * (because of added wait states below)!
 * This function reads the Am29F010 FLASH memory. 32K Bytes of FLASH memory
 * is mapped to the global address space of the DSP starting at address 0x8000.
 * This function retrieves 16 bit words from two consecutive FLASH locations
 * with high byte stored first.
 *
 *  start   -   Starting word location of FLASH storage to retreive (0 to 0xffff)
 *  length  -   Length in words to retreive (1 to 0x2000)
 *              (don't span from 0x3fff-0x4000 (or similar) with input range)
 *  datawords - Pointer to data array
 *
 **************************************************************************/
void read_flash(unsigned start, unsigned length, unsigned *datawords)
{
unsigned i, j, port_temp;

port_temp = portfff5;   /* save state of portfff5 */
portfff5 &= ~0x0200;    /* mask delta interrupts (if not) */
portfff5 |= 0x0004;     /* make IO2 an output (if not) */

if(start<0x4000){       /* set: IO2 = 1, CLIPA = 0 */
  portfff6 = (portfff6&0x000f)|0x0004;  /* and set IO2 */
  out_atten &= ~0x000c; /* clear CLIP LEDs (if not) */
}
else if(start<0x8000){  /* set: IO2 = 1, CLIPA = 1 */
  start -= 0x4000;
  portfff6 = (portfff6&0x000f)|0x0004;  /* and set IO2 */
  out_atten |= 0x000c;  /* set CLIP LEDs */
}
else if(start<0xc000){  /* set: IO2 = 0, CLIPA = 0 */
  start -= 0x8000;
  portfff6 &= 0x000b;   /* and clear IO2 */
  out_atten &= ~0x000c; /* clear CLIP LEDs (if not) */
}
else{                   /* set: IO2 = 0, CLIPA = 1 */
  start -= 0xc000;
  portfff6 &= 0x000b;   /* and clear IO2 */
  out_atten |= 0x000c;  /* set CLIP LEDs */
}

wait_n_samples(20);     /* wait for ~20 sampling intervals for CLIP signals to propagate */
                        /* wait for I02 state to settle (4 RC's = 4*10K*0.01uF) */
                        
*greg_ptr = 0x0080; /* map flash memory to global data space: 0x8000 to 0xffff */ 
portfffc = (IO_WAITS*0x0200 + FLASH_WAITS*0x0040 + 0*0x0008 + 0);   /* Increase # wait states in data mem. */
/* Warning increasing wait states can keep the ISR from completing */

*((unsigned int *)(0x8000+0x5555)) = 0x00aa;    /* put FLASH into normal read state (if should already be there) */
*((unsigned int *)(0x8000+0x2aaa)) = 0x0055;
*((unsigned int *)(0x8000+0x5555)) = 0x00f0;

/* Read FLASH: */
j = 0;
start = (start<<1);             /* set start to point to the starting byte in FLASH */
length = start + (length<<1);   /* set length to last FLASH address + 1 */
for(i=start;i<length;i+=2){
  datawords[j++] = ((*((unsigned int *)(0x8000+i)))<<8) | ((*((unsigned int *)(0x8001+i))&0x00ff)); /* build word */
}

portfffc = (IO_WAITS*0x0200 + 0*0x0040 + 0*0x0008 + 0); /* back to 0 wait states in data mem. */

portfff5 &= ~0x0004;    /* make IO2 an input */
wait(400);              /* wait for I02 to go high (4 RC's = 4*10K*0.01uF) */
portfff6 = 0x00f0;      /* clear pending delta interrupts */
portfff5 = port_temp;   /* restore state of portfff5 (could unmask delta interrupts) */
out_atten &= ~0x000c;   /* clear CLIP LEDs (if not) */

}


/**************************************************************************
 * valid_serial
 * This function returns a 1 if the serial number string is valid
 * and a 0 if not valid.
 *
 * A valid serial number consists of 6 digits followed by 4 check digits:
 *
 *      0123456789
 *      ######oess  -   SN with check sums
 *
 *      ######  -   6 digit SN eg. 132001
 *      o       -   check sum of # # #  (odd digits) modulo 10
 *      e       -   check sum of  # # # (even digits) modulo 10
 *      ss      -   check sum of ######oe
 *
 **************************************************************************/
int valid_serial(char *carray)
{
int i, code, num[10], sumo=0, sume=0, sumss=0;

for(i=0;i<10;i++){
  code = carray[i]; /* get */
  num[i] = code - 0x30; /* convert to decimal and save */
  if((code<0x30)||(code>0x39)){
    return 0;   /* error, not ASCII digit 0 to 9 */
  }
}

sumo = num[0]+num[2]+num[4];
/* sumo = sumo % 10;            /* sum = sum modulo 10 */
while(sumo>=10){
  sumo -= 10;
}

sume = num[1]+num[3]+num[5];
/* sume = sume % 10;            /* sum = sum modulo 10 */
while(sume>=10){
  sume -= 10;
}

sumss = num[0]+num[1]+num[2]+num[3]+num[4]+num[5]+sumo+sume;

if((sumo!=num[6])||(sume!=num[7])||(sumss!=(10*num[8]+num[9]))){
  return 0;     /* checksum error */
}

return 1;       /* checksum OK */
}


/**************************************************************************
 * update_disp_left
 * Updates (writes) the left side of the LCD after striping "#" and "."
 * from the .text string.
 **************************************************************************/
void update_disp_left(void)
{
#if(MAIN)

#define BUFL 17

int i, pos, pos2;
char ctemp, ctemp1[BUFL], ctemp2[BUFL];
char *c1ptr, *c2ptr;

c1ptr = &ctemp1[0]; /* set pointers */
c2ptr = &ctemp2[0];
  
pos = 0;
do{
  ctemp = (*(c1ptr++) = *(param_struct[param_ptr].text + (pos++))); /* copy chars */
} while((ctemp!='#')&&(ctemp!='\0'));

if(ctemp=='\0'){
  disp_text(ctemp1, 1, -1); /* Found no #'s. Write left side LCD only (no chars on right) */
}
else{   /* '#' found */
  *(c1ptr-1) = '\0';    /* replace the '#' in *(c1ptr) with NULL */
  pos2 = 0;
  do{
    ctemp = *(param_struct[param_ptr].text + (pos++));  /* copy chars */
    if((ctemp!='#')&&(ctemp!='_')&&(ctemp!='.')&&(ctemp!='\0')&&(pos2==0)){
      pos2 = pos;
    }
    if(pos2){
      *(c2ptr++) = ctemp;   /* copy chars */
    }
  } while(ctemp!='\0');
  
  if(pos2!=0){
    disp_text(ctemp2, pos2, -1);    /* write right side of LCD with the text right of #.# */
  }
  disp_text(ctemp1, 1, -1);         /* write left side LCD (order puts cursor_pos in place for #.# */
}

if(flag_options==0){    /* if not in options section */
  if(index_ab==0){
    disp_text("a", 1, 0);   /* Write the A function indicator character */
  }
  else if(index_ab==1){
    disp_text("b", 1, 0);   /* Write the B function indicator character */
  }
  else if(index_ab==2){
    disp_text(" ", 1, 0);   /* Write the Common function indicator character */
  }
}

#endif
}

/**************************************************************************
 * update_disp_right
 * Updates (writes) the right side of the LCD with text or a number
 * depending on type.
 * Cursor is put back into pos and turned on prior to retrun.
 **************************************************************************/
void update_disp_right(int pos)
{
#if(MAIN)

int type, nstart, nlength, nfrac;

type = param_struct[param_ptr].flag>>15;
nstart = (0x000f&(param_struct[param_ptr].flag>>10)) + 1;

if(type){   /* display type: "text text" */
  disp_text((char*)(*(char*)(param_struct[param_ptr].label + params[param_ptr][index_ab])), nstart, 0);
}
else{       /* display type: "text int" or "text float" */
  nlength = 0x000f&(param_struct[param_ptr].flag>>3);
  nfrac = 0x0007&param_struct[param_ptr].flag;
  disp_num(params[param_ptr][index_ab], nstart, nlength, nfrac);
}

disp_text("", pos , +1);    /* put cursor back into position and turn on */

#endif
}


/**************************************************************************
 * beep
 * Beep the speaker.
 *
 **************************************************************************/
void beep(unsigned duration, unsigned period)
{
int i;

for(i=0;i<duration;i++){
  port0_copy |= 0x0040;
  port0 = port0_copy;
  wait(period);
  port0_copy &= ~0x0040;
  port0 = port0_copy;
  wait(period);
}

}



















/*========= Required only by MAIN ========================================*/
#if(MAIN)



/**************************************************************************
 * store_all
 * This subroutine stores the current setting to all flash memory locations.
 *
 **************************************************************************/
void store_all(void)
{

int itemp;

record[0] = 0xffff;
flash_locked = 0;   /* unlock flash */
itemp = prog_flash(0x6000, 1, record, 1, "in Erase");   /* erase FLASH */
if(itemp==0){   /* if no error */
  for(itemp=LAST_MEM_LOC;itemp>=0;itemp--){
    params[10][0] = itemp;  /* load store location value */
    params_changed_copy = 3;
    store();                /* store current settings */
  }  
}
}

/**************************************************************************
 * xmit
 * This subroutine transimits the text string out of the RS-232 port
 * and follows with a charage return.
 *
 **************************************************************************/
void xmit(char *text)
{

portfff5 &= ~0x0080;    /* suspend async. receive ints. so this module
                           will not hear itself talking. This avoids serial
                           receive buffer overflows. */

while(*text!='\0'){
  while(!(portfff6&0x1000));/* hold here until ADTR and AXSR are empty */
  portfff4 = *text++;       /* send character out the RS-232 port */
}
portfff4 = '\r';    /* send CR last */
while(!(portfff6&0x1000));/* hold here until ADTR and AXSR are empty (CR sent) */

portfff6 = 0x6700;      /* reset any async. serial port interrupt indicator bits */
portfff5 |= 0x0080;     /* re-enable receive ints. */

}


/**************************************************************************
 * param_struct_search()
 * This function searches param_struct[i].text for a parameter_str[] 
 * match as follows:
 *
 * match_type = 0   -   no match found
 *              1   -   0-offset match
 *              2   -   0-offset match except with space in first .text
 *                          " APgain"=="bAPgain"
 *              3   -   1-offset match with space in first .text
 *                          " APgain"=="APgain"
 *
 **************************************************************************/
int param_struct_search(int *match_type)
{
int i;
char *cptr;

for(i=0;i<NPARAMSTRUCT;i++){            /* Search param_struct[i].text: */

  cptr = param_struct[i].text;  /* get pointer value to beginning of ith parameter text */

  if(string_compare(cptr, &parameter_str[0])){  /* if 0-offset match */
    *match_type = 1;
    return(i);
  }
  else if(*cptr++==' '){        /* first character is a space */
    if(string_compare(cptr, &parameter_str[0])){    /* if 1-offset match */
      *match_type = 3;
      return(i);
    }
    if(string_compare(cptr, &parameter_str[1])){    /* if 0-offset match (skipping first character) */
      *match_type = 2;
      return(i);
    }
  }
}

*match_type = 0;    /* no match found */
return(0);
}

/**************************************************************************
 * string_compare()
 * This function first converts both string to lower case and then compares
 * the strings up to end of the shortest string. Returns 1 if equal and
 * 0 if not equal. A null string matches anything.
 **************************************************************************/
int string_compare(char *s1, char *s2)
{
char ctemp1, ctemp2;
  
for(;;){
  ctemp1 = tolower(*s1++);
  ctemp2 = tolower(*s2++);
  if((!ctemp1)||(!ctemp2)) return(1);
  if(ctemp1 != ctemp2) return(0);
}
}


/**************************************************************************
 * inc_dec_param_ptr
 * Increment/Decrement param_ptr and set index_ab based on cw
 *
 * If Mode:A&B Common then the sequence is:
 *  Section index_ab    flag_options    param_ptr (name)        return 1 for beep
 *
 *  Common  2           0               0   (FUNC)              beep
 *                                      param_ptr_start[func_c]
 *                                      . . .
 *                                      param_ptr_end[func_c]   
 *  Options 0           1               OPTIONS_START           beep
 *                                      . . .
 *                                      OPTIONS_END
 *
 * If Mode:A&B Separate then the sequence is:
 *  Section index_ab    flag_options    param_ptr (name)        return 1 for beep
 *
 *  A       0           0               0   (FUNC)              beep
 *                                      param_ptr_start[func_a]
 *                                      . . .
 *                                      param_ptr_end[func_a]   
 *  B       1           0               0   (FUNC)              beep
 *                                      param_ptr_start[func_b]
 *                                      . . .
 *                                      param_ptr_end[func_b]
 *  Options 0           1               OPTIONS_START           beep
 *                                      . . .
 *                                      OPTIONS_END
 *
 * If Mode: Ch A Only then the sequence is:
 *  Section index_ab    flag_options    param_ptr (name)        return 1 for beep
 *
 *  A       0           0               0   (FUNC)              beep
 *                                      param_ptr_start[func_a]
 *                                      . . .
 *                                      param_ptr_end[func_a]   
 *  Options 0           1               OPTIONS_START           beep
 *                                      . . .
 *                                      OPTIONS_END
 *
 * Returns: 0 - if no boundary crossed
 *          1 - if boundary crossed: A <=> B <=> Options, or Common <=> Options ...
 **************************************************************************/
int inc_dec_param_ptr(void)
{
int func_a, func_b, func_c;

func_a = (int)params[0][0]; /* get A function index */
func_b = (int)params[0][1]; /* get B function index */
func_c = (int)params[0][2]; /* get Common function index */

switch((params[6][0]<<2)+(cw<<1)+flag_options){ /* compute a code for the switch */
case 0:     /* A&BCommon, ccw, flag_options=0 */
  if(param_ptr==0){
    param_ptr = OPTIONS_END;
    index_ab = 0;
    flag_options = 1;
  }
  else if(param_ptr==param_ptr_start[func_c]){
    param_ptr = 0;
    return 1;
  }
  else{
    param_ptr--;
  }
  break;

case 1:     /* A&BCommon, ccw, flag_options=1 */
  if(param_ptr==OPTIONS_START){
    param_ptr = param_ptr_end[func_c];
    index_ab = 2;
    flag_options = 0;
  }
  else{
    param_ptr--;
    if(param_ptr==OPTIONS_START){
      return 1;
    }
  }
  break;

case 2:     /* A&BCommon, cw, flag_options=0 */
  if(param_ptr==0){
    param_ptr = param_ptr_start[func_c];
  }
  else if(param_ptr==param_ptr_end[func_c]){
    param_ptr = OPTIONS_START;
    index_ab = 0;
    flag_options = 1;
    return 1;
  }
  else{
    param_ptr++;
  }
  break;

case 3:     /* A&BCommon, cw, flag_options=1 */
  if(param_ptr==OPTIONS_END){
    param_ptr = 0;
    index_ab = 2;
    flag_options = 0;
    return 1;
  }
  else{
    param_ptr++;
  }
  break;

case 4:     /* A&BSeparate, ccw, flag_options=0 */
  if(index_ab){
    if(param_ptr==0){
      param_ptr = param_ptr_end[func_a];
      index_ab = 0;
    }
    else if(param_ptr==param_ptr_start[func_b]){
      param_ptr = 0;
      return 1;
    }
    else{
      param_ptr--;
    }
  }
  else{ /* index_ab==0 */
    if(param_ptr==0){
      param_ptr = OPTIONS_END;
      index_ab = 0;
      flag_options = 1;
    }
    else if(param_ptr==param_ptr_start[func_a]){
      param_ptr = 0;
      return 1;
    }
    else{
      param_ptr--;
    }
  }
  break;

case 5:     /* A&BSeparate, ccw, flag_options=1 */
  if(param_ptr==OPTIONS_START){
    param_ptr = param_ptr_end[func_b];
    index_ab = 1;
    flag_options = 0;
  }
  else{
    param_ptr--;
    if(param_ptr==OPTIONS_START){
      return 1;
    }
  }
  break;

case 6:     /* A&BSeparate, cw, flag_options=0 */
  if(index_ab){
    if(param_ptr==0){
      param_ptr = param_ptr_start[func_b];
    }
    else if(param_ptr==param_ptr_end[func_b]){
      param_ptr = OPTIONS_START;
      index_ab = 0;
      flag_options = 1;
      return 1;
    }
    else{
      param_ptr++;
    }
  }
  else{ /* index_ab==0 */
    if(param_ptr==0){
      param_ptr = param_ptr_start[func_a];
    }
    else if(param_ptr==param_ptr_end[func_a]){
      param_ptr = 0;
      index_ab = 1;
      return 1;
    }
    else{
      param_ptr++;
    }
  }
  break;

case 7:     /* A&BSeparate, cw, flag_options=1 */
  if(param_ptr==OPTIONS_END){
    param_ptr = 0;
    flag_options = 0;
    return 1;
  }
  else{
    param_ptr++;
  }
  break;

case 8:     /* Ch A Only, ccw, flag_options=0 */
  if(param_ptr==0){
    param_ptr = OPTIONS_END;
    flag_options = 1;
  }
  else if(param_ptr==param_ptr_start[func_a]){
    param_ptr = 0;
    return 1;
  }
  else{
    param_ptr--;
  }
  break;

case 9:     /* Ch A Only, ccw, flag_options=1 */
  if(param_ptr==OPTIONS_START){
    param_ptr = param_ptr_end[func_a];
    flag_options = 0;
  }
  else{
    param_ptr--;
    if(param_ptr==OPTIONS_START){
      return 1;
    }
  }
  break;

case 10:    /* Ch A Only, cw, flag_options=0 */
  if(param_ptr==0){
    param_ptr = param_ptr_start[func_a];
  }
  else if(param_ptr==param_ptr_end[func_a]){
    param_ptr = OPTIONS_START;
    flag_options = 1;
    return 1;
  }
  else{
    param_ptr++;
  }
  break;

case 11:    /* Ch A Only, cw, flag_options=1 */
  if(param_ptr==OPTIONS_END){
    param_ptr = 0;
    flag_options = 0;
    return 1;
  }
  else{
    param_ptr++;
  }
  break;

default:
  break;
}   /* end switch */

return 0;
}


/**************************************************************************
 * inc_dec_mod (fix: posible shorter to use: "*ptr = ++(*ptr) % mod" for mod)
 * Increment/Decrement modulo mod based on cw
 **************************************************************************/
void inc_dec_mod(int *ptr, int mod)
{
if(mod){
  if(cw){
    ++(*ptr);
    if((*ptr)>=mod) *ptr = 0;
  }
  else{
    --(*ptr);
    if((*ptr)<0) *ptr = mod - 1;
  }
}
else{
  *ptr = 0;     /* if mod==0, return 0 */
}
}


/**************************************************************************
 * add_sub
 * Add/Subtract 10^pow to long based on cw (where: 0<=pow<=9)
 **************************************************************************/
void add_sub_pow(long *ptr, int pow)
{
long delta;

switch(pow){
case 0:
  delta = 1;
  break;
case 1:
  delta = 10;
  break;
case 2:
  delta = 100;
  break;
case 3:
  delta = 1000;
  break;
case 4:
  delta = 10000;
  break;
case 5:
  delta = 100000;
  break;
case 6:
  delta = 1000000;
  break;
case 7:
  delta = 10000000;
  break;
case 8:
  delta = 100000000;
  break;
case 9:
  delta = 1000000000;
  break;
default:
  delta = 1;
  break;
}

if(cw){
  (*ptr) += delta;
}
  else{
  (*ptr) -= delta;
}

if((*ptr)>max_value){
  (*ptr) = max_value;
}

if((*ptr)<min_value){
  (*ptr) = min_value;
}

}


/**************************************************************************
 * update_dsp
 * This function updates the DSP's function to reflect current parameter
 * values.
 *
 **************************************************************************/
void update_dsp(int param_ptr_tmp, int index_ab_tmp)
{
long ltemp;
int itemp, iorder, cursor_temp;
float f1, f2, fcenter, fwidth, ftemp;

/* Update the DSP's function: */
switch(param_ptr_tmp){
case 0: /* FUNC: */
  if(params_changed_copy==1) break; /* update only min_value and max_value */
  gain(index_ab_tmp);   /* Calculate setting of gain constant and output attenuator based on current func and index_ab_tmp */
  update_dsp(param_ptr_start[(int)params[0][index_ab_tmp]],index_ab_tmp);   /* Initialize the current function with recursive call */
  break;

case 1: /* Levels:In Out */
  assembly_flag |= 1;   /* flag the VU Meter to turn on */
  break;

case 2: /* RevertToLevels: */
  auto_vu_count = (int)params[2][0];    /* restart counter; set to 0 or 1 depending on RevertToLevels */
  break;

case 3: /* FullScalIn: */
  min_value = 1;    /* set min and max value to bound paramter */
  max_value = max_in_level;
  if(params_changed_copy==1) break; /* update only min_value and max_value */
  set_all_gains();      /* set optimal CODEC gains and attenuations */
  break;

case 4: /* SampleRate: */
  if(params_changed_copy==1) break; /* update only min_value and max_value */
  set_fsample();        /* set CODEC sampling rate */
  init_freq_params();   /* pull up factory default frequencies */
  /* no "break;"           fall into case 6: below to init functions: */

case 6: /* Mode: */
  if(params_changed_copy==1) break; /* update only min_value and max_value */
  /* Set the current function: */
  if(params[6][0]==0){  /* Mode:A&B Common */
    gain(2);    /* Calculate setting of gain constants and output attenuator based on current func */
    update_dsp(param_ptr_start[(int)params[0][2]],2);   /* Initialize the current function with recursive call */
  }
  else if(params[6][0]==1){ /* Mode:A&B Separate */
    if(params[17][0] > 128) params[17][0] = 127;    /* reset filter order to default if greater that 128 */
    if(params[20][0] > 128) params[20][0] = 127;
    if(params[26][0] > 128) params[26][0] = 127;
    if(params[32][0] > 128) params[32][0] = 127;
    if(params[40][0] > 128) params[40][0] = 127;
    gain(0);    /* Calculate setting of gain constant and output attenuator based on current func */
    update_dsp(param_ptr_start[(int)params[0][0]],0);   /* Initialize the current function with recursive call */
    gain(1);    /* Calculate setting of gain constant and output attenuator based on current func */
    update_dsp(param_ptr_start[(int)params[0][1]],1);   /* Initialize the current function with recursive call */
  }
  else{                 /* Mode:Ch A Only */
    func_addr_b = (unsigned)&no_func_b; /* set Ch B to no_func so Ch A can run long filter */
    gain(0);    /* Calculate setting of gain constant and output attenuator based on current func */
    update_dsp(param_ptr_start[(int)params[0][0]],0);   /* Initialize the current function with recursive call */
  }
  break;

case 5: /* InputSrc: */
  if(params_changed_copy==1) break; /* update only min_value and max_value */
  assembly_flag = (int)params[5][0] ? (assembly_flag|8):(assembly_flag&(~8));   /* set/clear the white noise flag for assembly code */
  break;

case 7: /* Cascade Ch A&B */
  if(params_changed_copy==1) break; /* update only min_value and max_value */
  assembly_flag = (int)params[7][0] ? (assembly_flag|4):(assembly_flag&(~4));   /* set/clear the cascade flag for assembly code */
  break;

case 8: /* Master Mode */
  if(params_changed_copy==1) break; /* update only min_value and max_value */

  break;

case 9: /* Erase Mem: */
  if(params_changed_copy==3){
    initialize();   /* initialize DSP hardware */
    wait(500000);   /* wait to display "FUNC:AllPass" */
    store_all();    /* store current settings to all flash memory locations */
    update_disp_left();     /* display parameter state */
    update_disp_right(1);
  }
  break;

case 10:    /* Store: */
  store();  /* store settings if params_changed_copy==3 */
  break;

case 11:    /* Recall: */
  recall(); /* recall settings if params_changed_copy==3 */
  break;

case 12:    /* Firmware:  V */
/*  portfff5 &= ~0x0200;    /* suspend delta interupts while updating display */
/*  if(params_changed_copy==1){ */
    disp_num(VERSION, 13, 4, 2);
    disp_text("", 1, 0);    /* position cursor home */
/*  }
  else if(params_changed_copy==2){
    disp_text("", 13, 0);   /* position cursor to 13 */
/*  } *
/*  portfff5 |= 0x0200;     /* re-enable delta interupts */
  break;

case 13:    /* Serial No: */
/*  portfff5 &= ~0x0200;    /* suspend delta interupts while updating display */
/*  if(params_changed_copy==1){ */
    disp_num(serial_number, 11, 6, 0);
    disp_text("", 1, 0);    /* position cursor home */
/*  }
  else if(params_changed_copy==2){
    disp_text("", 11, 0);   /* position cursor to 11 */
/*  } */
/*  portfff5 |= 0x0200;     /* re-enable delta interupts */
  break;

case 14:    /* NFgain: */
  min_value = -GAIN_MAX;    /* set min and max value to bound paramter */
  max_value = GAIN_MAX;
  if(params_changed_copy==1) break; /* update only min_value and max_value */
  gain(index_ab_tmp);
  /* Set function address to no function: */
  itemp = index_ab_tmp + 1; /* itemp: 1-A, 2-B, 3-Common */
  if(itemp&1){
    func_addr_a = (unsigned)&no_func_a; /* set no function */
  }
  if(itemp&2){
    func_addr_b = (unsigned)&no_func_b; /* set no function */
  }
  break;
case 15:    /* APgain: */
  min_value = -GAIN_MAX;    /* set min and max value to bound paramter */
  max_value = GAIN_MAX;
  if(params_changed_copy==1) break; /* update only min_value and max_value */
  gain(index_ab_tmp);
  /* Set function address to allpass: */
  itemp = index_ab_tmp + 1; /* itemp: 1-A, 2-B, 3-Common */
  if(itemp&1){
    func_addr_a = (unsigned)&allpass_func_a; /* send input to output */
  }
  if(itemp&2){
    func_addr_b = (unsigned)&allpass_func_b; /* send input to output */
  }
  break;

case 18:    /* LPgain: */
case 21:    /* HPgain: */
case 27:    /* BPgain: */
case 33:    /* BSgain: */
case 36:    /* Ngain: */
case 39:    /* INgain: */
case 42:    /* UFgain: */
case 45:    /* Samp: */
  min_value = -GAIN_MAX;    /* set min and max value to bound paramter */
  max_value = GAIN_MAX;
  if(params_changed_copy==1) break; /* update only min_value and max_value */
  gain(index_ab_tmp);
  break;

case 17:    /* LPorder: */
case 20:    /* HPorder: */
  min_value = (long)(3);    /* set min and max value to bound paramter */
  if(params[6][0]==2){      /* if Ch A Only */
    max_value = (long)(256);    /* allow long filters on Ch A */
  }
  else{
    max_value = (long)(128);  
  }
  if(params_changed_copy==1) return;    /* update only min_value and max_value */
  f1 = (float)params[param_ptr_tmp-1][index_ab_tmp];    /* get current fcut */
  iorder = (int)params[param_ptr_tmp][index_ab_tmp];    /* get current order */
  goto compute_filt;

case 26:    /* BPorder: */
case 32:    /* BSorder: */
  min_value = (long)(3);    /* set min and max value to bound paramter */
  if(params[6][0]==2){      /* if Ch A Only */
    max_value = (long)(256);    /* allow long filters on Ch A */
  }
  else{
    max_value = (long)(128);  
  }
  if(params_changed_copy==1) return;    /* update only min_value and max_value */
  f1 = (float)params[param_ptr_tmp-4][index_ab_tmp];    /* get current f1 */
  f2 = (float)params[param_ptr_tmp-3][index_ab_tmp];    /* get current f2 */
  iorder = (int)params[param_ptr_tmp][index_ab_tmp];    /* get current order */
  goto compute_filt;

case 16:    /* LPfcut: */
case 19:    /* HPfcut: */
  min_value = (long)(FCUT_MIN*fsample); /* set min and max value to bound paramter */
  max_value = (long)(FCUT_MAX*fsample);
  if(params_changed_copy==1) return;    /* update only min_value and max_value */
  f1 = (float)params[param_ptr_tmp][index_ab_tmp];  /* get current fcut */
  iorder = (int)params[param_ptr_tmp+1][index_ab_tmp];  /* get current order */
  goto compute_filt;

case 22:    /* BPf1: */
case 28:    /* BSf1: */
  min_value = (long)(F1_MIN*fsample);   /* set min and max value to bound paramter */
  max_value = (long)(F1_MAX*fsample);
  if(params_changed_copy==1) return;    /* update only min_value and max_value */
  f1 = (float)params[param_ptr_tmp][index_ab_tmp];  /* get current f1 */
  f2 = (float)params[param_ptr_tmp+1][index_ab_tmp];    /* get current f2 */
  if((f2-f1)<FWIDTH_MIN*fsample){   /* "push" f2 up to maintain minimum width */
    f2 = f1 + FWIDTH_MIN*fsample;
    params[param_ptr_tmp+1][index_ab_tmp] = (long)f2;   /* save modified f2 */
  }
  params[param_ptr_tmp+2][index_ab_tmp] = (long)((f1+f2)/2.0);  /* save updated fcenter */
  params[param_ptr_tmp+3][index_ab_tmp] = (long)(f2-f1);        /* save updated fwidth */
  iorder = (int)params[param_ptr_tmp+4][index_ab_tmp];  /* get current order */
  goto compute_filt;

case 23:    /* BPf2: */
case 29:    /* BSf2: */
  min_value = (long)(F2_MIN*fsample);   /* set min and max value to bound paramter */
  max_value = (long)(F2_MAX*fsample);
  if(params_changed_copy==1) return;    /* update only min_value and max_value */
  f1 = (float)params[param_ptr_tmp-1][index_ab_tmp];    /* get current f1 */
  f2 = (float)params[param_ptr_tmp][index_ab_tmp];  /* get current f2 */
  if((f2-f1)<FWIDTH_MIN*fsample){   /* "push" f1 down to maintain minimum width */
    f1 = f2 - FWIDTH_MIN*fsample;
    params[param_ptr_tmp-1][index_ab_tmp] = (long)f1;   /* save modified f1 */
  }
  params[param_ptr_tmp+1][index_ab_tmp] = (long)((f1+f2)/2.0);  /* save updated fcenter */
  params[param_ptr_tmp+2][index_ab_tmp] = (long)(f2-f1);        /* save updated fwidth */
  iorder = (int)params[param_ptr_tmp+3][index_ab_tmp];  /* get current order */
  goto compute_filt;

case 24:    /* BPfcntr: */
case 30:    /* BSfcntr: */
  fwidth = (float)params[param_ptr_tmp+1][index_ab_tmp];    /* get fwidth */
  min_value = (long)((F1_MIN*fsample + fwidth/2.0) + 0.5);  /* set min and max value to bound paramter */
  max_value = (long)((F2_MAX*fsample - fwidth/2.0) + 0.5);
  if(params_changed_copy==1) return;    /* update only min_value and max_value */
  fcenter = (float)params[param_ptr_tmp][index_ab_tmp]; /* get fcenter */
  ftemp = fwidth/2.0;
  f1 = fcenter - ftemp;
  f2 = fcenter + ftemp;
/*    if(f1<F1_MIN*fsample){    /* set f1 to minimum */
/*      f1 = F1_MIN*fsample;
    }
    if(f2>F2_MAX*fsample){  /* set f2 to maxmum */
/*      f2 = F2_MAX*fsample;
    } */
  params[param_ptr_tmp-2][index_ab_tmp] = (long)f1;     /* save modified f1 */
  params[param_ptr_tmp-1][index_ab_tmp] = (long)f2;     /* save modified f2 */
  params[param_ptr_tmp+1][index_ab_tmp] = (long)(f2 - f1);  /* save modified fwidth */
  iorder = (int)params[param_ptr_tmp+2][index_ab_tmp];  /* get current order */
  goto compute_filt;

case 25:    /* BPfwdth: */
case 31:    /* BSfwdth: */
  min_value = (long)(FWIDTH_MIN*fsample);   /* set min and max value to bound paramter */
  max_value = (long)((F2_MAX - F1_MIN)*fsample + 0.5);
  if(params_changed_copy==1) return;    /* update only min_value and max_value */
  fcenter = (float)params[param_ptr_tmp-1][index_ab_tmp];   /* get fcenter */
  fwidth = (float)params[param_ptr_tmp][index_ab_tmp];      /* get fwidth */
  ftemp = fwidth/2.0;                                          
  f1 = fcenter - ftemp;
  f2 = fcenter + ftemp;
  if(f1<F1_MIN*fsample){    /* set f1 to minimum */
    f1 = F1_MIN*fsample;
    f2 = f1 + fwidth;
  }
  if(F2_MAX*fsample<f2){    /* set f2 to maxmum */
    f2 = F2_MAX*fsample;
    f1 = f2 - fwidth;
  }
  params[param_ptr_tmp-3][index_ab_tmp] = (long)f1;     /* save modified f1 */
  params[param_ptr_tmp-2][index_ab_tmp] = (long)f2;     /* save modified f2 */
  params[param_ptr_tmp-1][index_ab_tmp] = (long)((f1 + f2)/2.0);    /* save modified fcenter */
  iorder = (int)params[param_ptr_tmp+1][index_ab_tmp];  /* get current order */
 compute_filt:
  compute_fir(f1, f2, iorder, index_ab_tmp);    /* Compute and load FIR filter coefficients for LP, HP, BP or BS */
  break;

case 34:    /* Nfnotch: */
case 37:    /* INfcntr: */
  min_value = (long)(FNFNOTCH_MIN*fsample); /* set min and max value to bound paramter */
  max_value = (long)(FNFNOTCH_MAX*fsample);
  if(params_changed_copy==1) return;    /* update only min_value and max_value */
  f1 = (float)params[param_ptr_tmp][index_ab_tmp];      /* get current fnotch */
  f2 = (float)params[param_ptr_tmp+1][index_ab_tmp];    /* get current fwdth */
  goto compute_n;
  
case 35:    /* Nfwidth: */
case 38:    /* INfwdth: */
  min_value = (long)(FNFWIDTH_MIN*fsample); /* set min and max value to bound paramter */
  max_value = (long)(FNFWIDTH_MAX*fsample);
  if(params_changed_copy==1) return;    /* update only min_value and max_value */
  f1 = (float)params[param_ptr_tmp-1][index_ab_tmp];    /* get current fnotch */
  f2 = (float)params[param_ptr_tmp][index_ab_tmp];      /* get current fwdth */
 compute_n:
  compute_notch(f1, f2, index_ab_tmp);  /* compute and load notch coefficients */
  break;

case 40:    /* UForder: */
  min_value = (long)(3);    /* set min and max value to bound parameter */
  if(params[6][0]==2){      /* if Ch A Only */
    max_value = (long)(256);    /* allow long filters on Ch A */
  }
  else{
    max_value = (long)(128);  
  }
  if(params_changed_copy==1) return;    /* update only min_value and max_value */
  params[param_ptr_tmp+1][index_ab_tmp] = 1L;   /* set coef. poiner to 1 when changing order */
  iorder = (int)params[param_ptr_tmp][index_ab_tmp];    /* get current order */
  goto load_user;

case 41:    /* UFtap: */
  iorder = (int)params[param_ptr_tmp-1][index_ab_tmp];  /* get current order */
  min_value = 1L;   /* set min and max value to bound paramter */
  max_value = (long)iorder; /* limmit max to current order */

  cursor_temp = cursor_pos;                 /* save cursor position */

  itemp = (int)params[param_ptr_tmp][index_ab_tmp] - 1; /* get current coef. tap index */
  ltemp = params[NPARAMSTRUCT + (itemp>>1)][index_ab_tmp];      /* get pair of coefs */
  if(itemp&1){
    disp_num((long)(ltemp>>16), 11, 6, 0);  /* disp odd (1,3,...) coefs */
  }
  else{
    disp_num((long)((int)(ltemp&0x0000ffff)), 11, 6, 0);    /* disp even (0,2,...) coefs */
  }

  disp_text("", cursor_temp, 0);    /* put cursor back */

  if(params_changed_copy==1) return;    /* update only min_value and max_value */
  
 load_user:  
  load_userfir(iorder, index_ab_tmp);   /* Load User FIR filter coefficients */
  break;

default:
  break;
}   /* end switch(param_ptr_tmp) */
    
}


/**************************************************************************
 * compute_fir
 * This function computes and loads FIR coefficients for LP, HP, BP and BS
 * filters based on the currently selected function in params[0][index_ab_tmp].
 *
 **************************************************************************/
void compute_fir(float f1, float f2, int iorder, int index_ab_tmp)
{
int i, itemp, max_flag, iorderm1, iorderm1d2, iorderd2;
float ftemp1, ftemp2, d2fsf1, d2fsf2, coef_max;
float coefs[128];

iorderm1 = iorder-1;
iorderm1d2 = iorderm1>>1;
iorderd2 = iorder>>1;

/* Compute modified-Blackman-window (coefs are from filt.m 'aopt'), if nessesary: */
if(iorder!=iorder_old){
  ftemp1 = (float)(iorder-1);
  for(i=0;i<=iorderm1d2;i++){   /* loop over half of the window (including center if odd) */
    ftemp2 = (PIT2*(float)(i))/ftemp1;
    window[i] = 0.48216433063585 - 0.48550251793519*cos(ftemp2) + 0.03233315142896*cos(2*ftemp2);
  }
  iorder_old = iorder;
}

/* Compute FIR filter coefficients: */
ftemp1 = 2.0/fsample;   /* compute some temp vars once */
d2fsf1 = ftemp1*f1;
d2fsf2 = ftemp1*f2;
switch((int)params[0][index_ab_tmp]){   /* switch on index of currently selected function */
case 2: /* LowPass */
  for(i=0;i<iorderd2;i++){  /* loop over half of filter (less center if odd) */
    ftemp1 = PI*((float)(i) - ((float)iorderm1)/2.0);   /* fix: can speed up by avoiding /2.0 */
    coefs[i] = window[i]*sin(d2fsf1*ftemp1)/ftemp1;
  }
  if(iorder&1){     /* if iorder odd, compute center coef[] */
    coefs[i] = window[i]*d2fsf1;    /* update center coef */
  }
  break;
    
case 3: /* HighPass */
  for(i=0;i<iorderd2;i++){  /* loop over half of filter (less center if odd) */
    ftemp1 = PI*((float)(i) - ((float)iorderm1)/2.0);
    coefs[i] = window[i]*(sin(ftemp1) - sin(d2fsf1*ftemp1))/ftemp1;
  }
  if(iorder&1){     /* if iorder odd, compute center coef[] */
    coefs[i] = window[i]*(1.0 - d2fsf1);    /* update center coef for HP filter */
  }
  break;
    
case 4: /* BandPass */
  for(i=0;i<iorderd2;i++){  /* loop over half of filter (less center if odd) */
    ftemp1 = PI*((float)(i) - ((float)iorderm1)/2.0);
    coefs[i] = window[i]*(sin(d2fsf2*ftemp1) - sin(d2fsf1*ftemp1))/ftemp1;
  }
  if(iorder&1){     /* if iorder odd, compute center coef[] */
    coefs[i] = window[i]*(d2fsf2 - d2fsf1); /* update center coef for BP filter */
  }
  break;
    
case 5: /* BandStop */
  for(i=0;i<iorderd2;i++){  /* loop over half of filter (less center if odd) */
    ftemp1 = PI*((float)(i) - ((float)iorderm1)/2.0);
    coefs[i] = window[i]*(sin(ftemp1) + sin(d2fsf1*ftemp1) - sin(d2fsf2*ftemp1))/ftemp1;
  }
  if(iorder&1){     /* if iorder odd, compute center coef[] */
    coefs[i] = window[i]*(1.0 + d2fsf1 - d2fsf2);   /* update center coef for BS filter */
  }
  break;

default:
  break;
}   /* end switch */

/* Determine quantization scale factor: */
coef_max = coefs[iorderm1d2];   /* get center coef (maximum coef) */
max_flag = (0.4999<coef_max);   /* store coef_max>0.4999 flag */
ftemp1 = max_flag ? 32768.0:65536.0;    /* save quantization scale factor */

/* Load the scale, quantize, and load filter coefs: */
out_gain |= 0x0400;     /* mute the outputs */

#if(0)
switch(index_ab_tmp){   /* switch on index_ab_tmp */
case 0: /* Ch A */
  func_addr_a = (unsigned)&no_func_a; /* get more CPU time to write the fir_coef[] */
  data_ptr_a = 0x03ff - iorderm1;   /* point to first used Ch A filter state data (in B1: 300h-3ffh) */
  orderm2_a = iorderm1-1;               /* load assembly language constant */
  asm(" clrc    CNF     ; map internal memory block B0 into Data space so we can write it");
  for(i=0;i<=iorderm1d2;i++){
    fir_coef[i] = fir_coef[iorderm1-i] = (int)(ftemp1*coefs[i] + 0.5);  /* write coefs to first and second half of filter A locations */
  }
  asm(" setc    CNF     ; map internal memory block B0 into Program space");
  func_addr_a = (unsigned)(max_flag ? &fir_15_a:&fir_16_a); /* set function A */
  break;
  
case 1: /* Ch B */
  func_addr_b = (unsigned)&no_func_b; /* get more CPU time to write the fir_coef[] */
  data_ptr_b = 0x037f - iorderm1;   /* point to first used Ch B filter state data (in B1: 300h-37fh) */
  orderm2_b = iorderm1-1;               /* load assembly language constant */
  asm(" clrc    CNF     ; map internal memory block B0 into Data space so we can write it");
  for(i=0;i<=iorderm1d2;i++){
    fir_coef[i+128] = fir_coef[iorderm1+128-i] = (int)(ftemp1*coefs[i] + 0.5);  /* write coefs to first and second half of filter A locations */
  }
  asm(" setc    CNF     ; map internal memory block B0 into Program space");
  func_addr_b = (unsigned)(max_flag ? &fir_15_b:&fir_16_b); /* set function B */
  break;
  
case 2: /* Ch A&B */
  func_addr_a = (unsigned)&no_func_a; /* get more CPU time to write the fir_coef[] */
  func_addr_b = (unsigned)&no_func_b; /* get more CPU time to write the fir_coef[] */
  data_ptr_a = 0x03ff - iorderm1;   /* point to first used Ch A filter state data (in B1: 300h-3ffh) */
  data_ptr_b = 0x037f - iorderm1;   /* point to first used Ch B filter state data (in B1: 300h-37fh) */
  orderm2_a = orderm2_b = iorderm1-1;   /* load assembly language constant */
  asm(" clrc    CNF     ; map internal memory block B0 into Data space so we can write it");
  for(i=0;i<=iorderm1d2;i++){
    fir_coef[i] = fir_coef[iorderm1-i] = (int)(ftemp1*coefs[i] + 0.5);  /* write coefs to first and second half of filter A locations */
    fir_coef[i+128] = fir_coef[iorderm1+128-i] = (int)(ftemp1*coefs[i] + 0.5);  /* write coefs to first and second half of filter A locations */
  }
  asm(" setc    CNF     ; map internal memory block B0 into Program space");
  func_addr_a = (unsigned)(max_flag ? &fir_15_a:&fir_16_a); /* set function A */
  func_addr_b = (unsigned)(max_flag ? &fir_15_b:&fir_16_b); /* set function B */
  break;

default:
  break;
}   /* end switch */
#endif

#if(1)
itemp = index_ab_tmp + 1;   /* itemp: 1-A, 2-B, 3-Common */
if(itemp&1){
  func_addr_a = (unsigned)&no_func_a; /* get more CPU time to write the fir_coef[] */
  data_ptr_a = 0x03ff - iorderm1;   /* point to first used Ch A filter state data (in B1: 300h-3ffh) */
  orderm2_a = iorderm1-1;               /* load assembly language constant */
  asm(" clrc    CNF     ; map internal memory block B0 into Data space so we can write it");
  for(i=0;i<=iorderm1d2;i++){
    fir_coef[i] = fir_coef[iorderm1-i] = (int)(ftemp1*coefs[i] + 0.5);  /* write coefs to first and second half of filter A locations */
  }
  asm(" setc    CNF     ; map internal memory block B0 into Program space");
  func_addr_a = (unsigned)(max_flag ? &fir_15_a:&fir_16_a); /* set function A */
}

if(itemp&2){
  func_addr_b = (unsigned)&no_func_b; /* get more CPU time to write the fir_coef[] */
  data_ptr_b = 0x037f - iorderm1;   /* point to first used Ch B filter state data (in B1: 300h-37fh) */
  orderm2_b = iorderm1-1;               /* load assembly language constant */
  asm(" clrc    CNF     ; map internal memory block B0 into Data space so we can write it");
  for(i=0;i<=iorderm1d2;i++){
    fir_coef[i+128] = fir_coef[iorderm1+128-i] = (int)(ftemp1*coefs[i] + 0.5);  /* write coefs to first and second half of filter A locations */
  }
  asm(" setc    CNF     ; map internal memory block B0 into Program space");
  func_addr_b = (unsigned)(max_flag ? &fir_15_b:&fir_16_b); /* set function B */
}
#endif

wait_n_samples(iorder); /* wait for ~order sampling intervals for transient to propagate */
out_gain &= ~0x0400;        /* un-mute the outputs */

}


/**************************************************************************
 * load_userfir
 * This function computes and loads FIR coefficients for user specifed
 * filters based on the currently selected function in params[0][index_ab_tmp].
 *
 **************************************************************************/
void load_userfir(int iorder, int index_ab_tmp)
{
int i, j, itemp, iorderm1, iorderm1d2;
long ltemp;

iorderm1 = iorder-1;
iorderm1d2 = iorderm1>>1;

out_gain |= 0x0400;     /* mute the outputs */
  
itemp = index_ab_tmp + 1;   /* itemp: 1-A, 2-B, 3-Common */
if(itemp&1){
  func_addr_a = (unsigned)&no_func_a; /* get more CPU time to write the fir_coef[] */
  data_ptr_a = 0x03ff - iorderm1;   /* point to first used Ch A filter state data (in B1: 300h-3ffh) */
  orderm2_a = iorderm1-1;               /* load assembly language constant */
  asm(" clrc    CNF     ; map internal memory block B0 into Data space so we can write it");
  j = 0;    /* starting position for filter A */
  for(i=0;i<=iorderm1d2;i++){
    ltemp = params[NPARAMSTRUCT + i][index_ab_tmp]; /* get pair of coefs */
    fir_coef[j++] = (int)(ltemp&0x0000ffff);        /* write even coefs to filter A locations */
    fir_coef[j++] = (int)(((unsigned long)ltemp)>>16);  /* write odd coefs to filter A locations */
  }
  asm(" setc    CNF     ; map internal memory block B0 into Program space");
  func_addr_a = (unsigned)&fir_15_a; /* set function A */
}

if(itemp&2){
  func_addr_b = (unsigned)&no_func_b; /* get more CPU time to write the fir_coef[] */
  data_ptr_b = 0x037f - iorderm1;   /* point to first used Ch B filter state data (in B1: 300h-37fh) */
  orderm2_b = iorderm1-1;               /* load assembly language constant */
  asm(" clrc    CNF     ; map internal memory block B0 into Data space so we can write it");
  j = 128;  /* starting position for filter B */
  for(i=0;i<=iorderm1d2;i++){
    ltemp = params[NPARAMSTRUCT + i][index_ab_tmp]; /* get pair of coefs */
    fir_coef[j++] = (int)(ltemp&0x0000ffff);        /* write even coefs to filter A locations */
    fir_coef[j++] = (int)(((unsigned long)ltemp)>>16);  /* write odd coefs to filter A locations */
  }
  asm(" setc    CNF     ; map internal memory block B0 into Program space");
  func_addr_b = (unsigned)&fir_15_b; /* set function B */
}

wait_n_samples(iorder); /* wait for ~order sampling intervals for transient to propagate */
out_gain &= ~0x0400;        /* un-mute the outputs */

}


/**************************************************************************
 * compute_notch
 * This function computes and loads IIR coefficients for the Notch and Inverse Notch
 * filters based on the currently selected function in params[0][index_ab_tmp].
 *
 **************************************************************************/
void compute_notch(float fn, float fw, int index_ab_tmp)
{
int* iptr;
int itemp;
float t1, t2, t3, k1, k2, c1, c2, d1, d2, g1, g2;

/* k1 = -cos(2*PI*fn/fsample);  /* compute k1 from fnotch */
k1 = -sin(2*PI*fn/fsample + PID2);  /* compute k1 from fnotch (use sin() to save memory) */
t1 = PI*fw/fsample;
/* t2 = cos(t1); */
t2 = sin(t1 + PID2);
t3 = sin(t1);
k2 = (t2-t3)/(t2+t3);       /* compute k2 from fwidth */
/* c1 = sqrt(1.0 - k1*k1);      /* compute normalization c1 */
/* c2 = sqrt(1.0 - k2*k2);      /* compute normalization c2 */

/* c1 = .5930;
c2 = .0196;
d1 = (1.0 - k1*k1)/c1;
d2 = (1.0 - k2*k2)/c2; */

d1 = -1;
d2 = -1;
c1 = (1.0 - k1*k1)/d1;
c2 = (1.0 - k2*k2)/d2;

/* Compute ideal notch filter coefficients: */
switch((int)params[0][index_ab_tmp]){   /* switch on index of currently selected function */
case 6: /* Notch */
  /* gain params (g1 and g2) must be in range [-4,4) */
  g1 = 0.5;
  g2 = 0.0;
  break;
  
case 7: /* Inverse Notch */
  /* gain params (g1 and g2) must be in range [-4,4) */
  g1 = 0.0;
  g2 = 0.5;
  break;

default:
  break;
}   /* end switch */


/* Load quantized coefs: */
out_gain |= 0x0400;     /* mute the outputs */

itemp = index_ab_tmp + 1;   /* itemp: 1-A, 2-B, 3-Common */
if(itemp&1){
  func_addr_a = (unsigned)&no_func_a; /* get more CPU time to write the coefdata[] */
  data_ptr_a = 0x03ff;  /* point to first used Ch A filter state data */
  /* Write coefs to filter A locations: */
  iptr = (int*)&coefdata[0x80]; /* get address of first coef to write */
  *iptr++ = (int)(32768*c2 + 0.5);  /* c2 */
  *iptr++ = (int)(32768*k2 + 0.5);  /* k2 */
  *iptr++ = (int)(32768*d2 + 0.5);  /* d2 */
  *iptr++ = (int)(32768*k2 + 0.5);  /* k2 (copy) */
  *iptr++ = (int)(32768*c1 + 0.5);  /* c1 */
  *iptr++ = (int)(32768*k1 + 0.5);  /* k1 */
  *iptr++ = (int)(32768*d1 + 0.5);  /* d1 */
  *iptr++ = (int)(32768*k1 + 0.5);  /* k1 (copy) */
  *iptr++ = (int)(8192*g1 + 0.5);   /* g1 */
  *iptr++ = (int)(8192*g2 + 0.5);   /* g2 */
  func_addr_a = (unsigned)&notch_a; /* set function A */
}

if(itemp&2){
  func_addr_b = (unsigned)&no_func_b; /* get more CPU time to write the coefdata[] */
  data_ptr_b = 0x037f;  /* point to first used Ch B filter state data */
  /* Write coefs to filter B locations: */
  iptr = (int*)&coefdata[0];    /* get address of first coef to write */
  *iptr++ = (int)(32768*c2 + 0.5);  /* c2 */
  *iptr++ = (int)(32768*k2 + 0.5);  /* k2 */
  *iptr++ = (int)(32768*d2 + 0.5);  /* d2 */
  *iptr++ = (int)(32768*k2 + 0.5);  /* k2 (copy) */
  *iptr++ = (int)(32768*c1 + 0.5);  /* c1 */
  *iptr++ = (int)(32768*k1 + 0.5);  /* k1 */
  *iptr++ = (int)(32768*d1 + 0.5);  /* d1 */
  *iptr++ = (int)(32768*k1 + 0.5);  /* k1 (copy) */
  *iptr++ = (int)(8192*g1 + 0.5);   /* g1 */
  *iptr++ = (int)(8192*g2 + 0.5);   /* g2 */
  func_addr_b = (unsigned)&notch_b; /* set function B */
}
  
/* wait_n_samples(???);     /* fix: wait for ??? sampling intervals for transient to propagate */
out_gain &= ~0x0400;        /* un-mute the outputs */

}


/**************************************************************************
 * set_all_gains
 * Sets all optimal CODEC gains and attenuations for the current function.
 *
 **************************************************************************/
void set_all_gains(void)
{
float cl, ftemp;

/* Compute the optimal input gain for the CODEC and the scale factor for the VU Meters: */                  
/* Channel A: */
cl = (float)params[3][0];   /* get FullScalIn value */
for(gopt_a=1;gopt_a<16;gopt_a++){
  ftemp = (16.0*in_cal_levels[gopt_a]*cl)/max_in_level; /* calulate eqs. 8 & 9 in Gain cal. notes */
  if(2.0<=ftemp){
    break;  /* break out of for loop */
  }
}
gopt_a--;
out_gain = (out_gain&(~0x000f))|gopt_a;     /* update the CODEC gain A */
scale_k_a = max_in_level/(8.0*in_cal_levels[gopt_a]*cl);    /* compute VU Meter scale factor */

/* Channel B: */
/* cl = (float)params[3][0];    /* get FullScalIn value */
for(gopt_b=1;gopt_b<16;gopt_b++){
  ftemp = (16.0*in_cal_levels[gopt_b]*cl)/max_in_level; /* calulate eqs. 8 & 9 in Gain cal. notes */
  if(2.0<=ftemp){
    break;  /* break out of for loop */
  }
}
gopt_b--;
out_gain = (out_gain&(~0x00f0))|(gopt_b<<4);        /* update the CODEC gain B */
scale_k_b = max_in_level/(8.0*in_cal_levels[gopt_b]*cl);    /* compute VU Meter scale factor */

/* Set the gains for Ch A and Ch B: */
if(params[6][0]){   /* Mode:A&B Separate or Mode: Ch A Only */
  gain(0);  /* Calculate setting of gain constant and output attenuator for Ch A */
  gain(1);  /* Calculate setting of gain constant and output attenuator for Ch B */
}
else{   /* Mode:A&B Common */
  gain(2);  /* Calculate setting of gain constant and output attenuator for Ch A and B */
}

}


/**************************************************************************
 * gain
 * Calculate setting of gain constant and output attenuator setting based
 * on the current function setting and index_ab_tmp.
 *
 **************************************************************************/
void gain(int index_ab_tmp)
{
int itemp;
float tgain, tlin, ftemp;

/* set gain fixfix: if 256*ftemp > 32767 we must limit max_value*/
tgain = 0.01*(float)params[param_ptr_end[(int)params[0][index_ab_tmp]]][index_ab_tmp];  /* get gain parameter for the current func */

itemp = index_ab_tmp + 1;   /* itemp: 1-A, 2-B, 3-Common */
/* Set gain for Ch A, Ch B or both: */
if(itemp&1){
  tlin = 16.0*in_cal_levels[gopt_a]/max_in_level;   /* compute Lin(gopt_a) for calulation below */
  ftemp = tgain/(tlin*scale_k_a);   /* calulate eq. 13 in Gain cal. notes */
  for(aopt_a=1;aopt_a<32;aopt_a++){
    if(out_cal[aopt_a]<=fabs(ftemp)){
      break;    /* break out of for loop */
    }
  }
  aopt_a--;
  out_atten = (out_atten&(~0x01f0))|(aopt_a<<4);    /* update the CODEC attenuation A */
  ftemp = tgain/(out_cal[aopt_a]*tlin);         /* compute output gain factor */
  t_reg_scale_a = (int)(256.0*ftemp + 0.5);         /* convert to scaled integer for use in assembly */
}
if(itemp&2){
  tlin = 16.0*in_cal_levels[gopt_b]/max_in_level;   /* compute Lin(gopt_b) for calulation below */
  ftemp = tgain/(tlin*scale_k_b);   /* calulate eq. 13 in Gain cal. notes */
  for(aopt_b=1;aopt_b<32;aopt_b++){
    if(out_cal[aopt_b]<=fabs(ftemp)){
      break;    /* break out of for loop */
    }
  }
  aopt_b--;
  out_atten = (out_atten&(~0x3e00))|(aopt_b<<9);    /* update the CODEC attenuation B */
  ftemp = tgain/(out_cal[aopt_b]*tlin);         /* compute output gain factor */
  t_reg_scale_b = (int)(256.0*ftemp + 0.5);         /* convert to scaled integer for use in assembly */
}

/* out_atten = (out_atten&(~0x3ff0))|(aopt_a<<4)|(aopt_b<<9);   /* update the CODEC attenuations */


}


/**************************************************************************
 * vu_update
 * This function updates the VU level meter. This function is called every
 * 5ms when displaying the VU meters. This fact is used to decay the
 * levels between the peaks.
 *
 **************************************************************************/
void vu_update(void)
{
int temp_pos;
unsigned vu_temp;
char carray[3];

carray[2] = 0;

vu_temp = convert_to_vu(scale_k_a, in_a_hold);  /* Compute VU level for in_a */
if(vu_temp>=in_a_vu_level){ /* get peak level for VU Meter */
  in_a_vu_level = vu_temp;
}

vu_temp = convert_to_vu(scale_k_b, in_b_hold);  /* Compute VU level for in_b */
if(vu_temp>=in_b_vu_level){ /* get peak level for VU Meter */
  in_b_vu_level = vu_temp;
}

vu_temp = convert_to_vu(scale_k_a, out_a_hold); /* Compute VU level for out_a */
if(vu_temp>=out_a_vu_level){    /* get peak level for VU Meter */
  out_a_vu_level = vu_temp;
}

vu_temp = convert_to_vu(scale_k_b, out_b_hold); /* Compute VU level for out_b */
if(vu_temp>=out_b_vu_level){    /* get peak level for VU Meter */
  out_b_vu_level = vu_temp;
}

/* Write the VU levels to LCD: */
temp_pos = cursor_pos;                  /* save cursor position */
carray[0] = vu_chars[in_a_vu_level];    /* get VU character */
carray[1] = vu_chars[in_b_vu_level];    /* get VU character */
disp_text(carray, 10, -1);

carray[0] = vu_chars[out_a_vu_level];   /* get VU character */
carray[1] = vu_chars[out_b_vu_level];   /* get VU character */
disp_text(carray, 15, -1);

disp_text("", temp_pos, 0); /* put cursor back */


/* Decay all the VU Meters together: */
if((vu_counter++)>=VU_DECAY){
  vu_counter = 0;
  in_a_hold = 0;
  in_b_hold = 0;
  out_a_hold = 0;
  out_b_hold = 0;
  if(in_a_vu_level>0){
    in_a_vu_level--;
  }
  if(in_b_vu_level>0){
    in_b_vu_level--;
  }
  if(out_a_vu_level>0){
    out_a_vu_level--;
  }
  if(out_b_vu_level>0){
    out_b_vu_level--;
  }
}

}


/**************************************************************************
 * convert_to_vu
 * This function converts to a log scale for the VU level meter.
 *
 **************************************************************************/
unsigned convert_to_vu(float scale, int signal)
{

float ftemp;
int itemp;

ftemp = scale*(float)(signal);  /* fix: + 1.0 scale the peak held value */
if(32767.0<ftemp){  /*if ftemp>0x7fff then clip */
  ftemp = 32767.0;
}

#if(0)  /* 3 dB per division */
itemp = (int)ftemp;
if(itemp==32767) return 12;     /* use number larger than 9 to force "X" to hold longer */
else if(itemp>=23170) return 8; /* -3dB down, 32768*(1/sqrt(2)) */
else if(itemp>=16384) return 7; /* -6dB down, 32768*(1/sqrt(2*2)) */
else if(itemp>=11585) return 6; /* -9dB down, 32768*(1/sqrt(2*2*2)) */
else if(itemp>=8192) return 5;
else if(itemp>=5793) return 4;
else if(itemp>=4096) return 3;
else if(itemp>=2896) return 2;
else if(itemp>=2048) return 1;
else return 0;

#else   /* 6 dB per division */
itemp = (int)ftemp;
if(itemp==32767) return 12;     /* use number larger than 9 to force "X" to hold longer */
else if(itemp>=16384) return 8; /* -6dB down, 32768*(1/sqrt(4)) */
else if(itemp>=8192) return 7;  /* -12dB down, 32768*(1/sqrt(4*4)) */
else if(itemp>=4096) return 6;  /* -18dB down, 32768*(1/sqrt(4*4*4)) */
else if(itemp>=2048) return 5;
else if(itemp>=1024) return 4;
else if(itemp>=512) return 3;
else if(itemp>=256) return 2;
else if(itemp>=128) return 1;
else return 0;

#endif
}


/**************************************************************************
 * led_update
 * This function updates the overload LEDs. This function is called every
 * 5ms. This fact is used to reset the LEDs after some hold time.
 *
 **************************************************************************/
void led_update(void)
{


/* set overload LEDs to reflect the status of the overload bits: */
out_atten = ((in_error_stick & 0x0100)>>5)|(out_atten & (~0x0008));
out_atten = ((in_error_stick & 0x0200)>>7)|(out_atten & (~0x0004));
/* led_counter = 0; */

if((led_counter++)>=OVERFLOW_STICK){
  led_counter = 0;
  in_error_stick = in_error_stick & (~0x0100);  /* unstick overload A bit */
  in_error_stick = in_error_stick & (~0x0200);  /* unstick overload B bit */
}


}


/**************************************************************************
 * store
 * This function stores the filter module's settings in FLASH memory sector 3.
 *
 * Flash memory format:
 *  0x6000  First record:   next_loc -  pointer to next records start
 *  0x6001                  prev_loc -  pointer to previous record start (0 for first record)
 *  0x6002                  Firmware Version
 *  0x6003                  Serial Number (low byte)
 *  0x6004                  Serial Number (high byte)
 *  0x6005                  loc_code -  memory location code (0 to 9)
 *  0x6006 ...              Module state variables: params[][]
 *
 *          Second record: ...
 *
 **************************************************************************/
void store(void)
{
int i, j, itemp;
int retrieved_flag[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  /* 10 memory locations max. */
unsigned next_loc, prev_loc=0, loc_code, ptr, ptr_m1=0;
unsigned current_loc;
unsigned fd_ptr, fd_ptr_m1=0, nrecord;
unsigned* uptr;                                                 

min_value = 0;  /* set min and max value to bound parameter */
max_value = LAST_MEM_LOC;

if(params_changed_copy!=3){ /* return if no press action is flaged */
  return;
}

nrecord = RECORD_LENGTH;

current_loc = (unsigned)params[10][0];

func_addr_temp_a = func_addr_a; /* save the current A function */
func_addr_temp_b = func_addr_b; /* save the current B function */
func_addr_a = (unsigned)&no_func_a; /* reduce ISR overhead, required for read_flash() and prog_flash() */
func_addr_b = (unsigned)&no_func_b;

portfff5 &= ~0x0200;    /* suspend delta interrupts while storing */
disp_num(current_loc, 1, 8, 0); /* write: "9 Stored" */
disp_text(" Stored ", 9, 0);
beep(75, 400);                  /* Beep speaker */

/* search for end of record data in FLASH */
ptr = 0x6000;   /* point to first record */
read_flash(ptr, 5, record);     /* read part of record header */
next_loc = record[0];
while(next_loc!=0xffff){
  /* Check for valid version and SN in each record: */
  if(record_bad()){ /* if error in record */
    goto store_error;
  }
  ptr_m1 = ptr;                 /* save pointer history */
  ptr = next_loc;               /* point to next location */
  read_flash(ptr, 5, record);   /* read part of record header */
  next_loc = record[0];
}
/* ptr now points to first blank location, ptr_m1 points to previous location. */

if(nrecord>(0x7fff - ptr)){ /* if current record too big to fit into remaining FLASH */
/* if(nrecord>(0x6000+2000 - ptr)){ /* fix: if current record too big to fit into remaining FLASH */
/* disp_text("Refreshing FLASH", 1, 0); /* write LCD */
/* wait(500000);            /* wait */
  /* Refresh FLASH by reading latest contents and re-saving: */
  fd_ptr = 0;           /* point to first location in flash_data[] array */
  prev_loc = ptr_m1;    /* point to last record in FLASH */
  retrieved_flag[current_loc] = 1;  /* flag current location, because we will store it later */
  do{   /* search backwords for valid records */
    do{
      ptr = prev_loc;               /* point to previous location */
      read_flash(ptr, 6, record);   /* read previous header */
      next_loc = record[0];
      prev_loc = record[1];
      loc_code = record[5];
      if(record_bad()){ /* test for error in record */
       store_error:
        disp_text("Error-MemCorrupt", 1, -1);   /* write LCD */
        wait(1500000);
        goto store_out;
      }
    } while((prev_loc!=0)&&(retrieved_flag[loc_code]==1)); /* loop while retreiving duplicated records */
    
    if(retrieved_flag[loc_code]==0){    /* if at a valid record */
      /* Read record (at ptr) and append to flash_data[]: */
      /* leave ptr at first blank location and ptr_m1 to previous location */
      read_flash(ptr, nrecord, &flash_data[fd_ptr]);    /* read FLASH */
      flash_data[fd_ptr] = fd_ptr + nrecord + 0x6000;   /* next_loc update*/
      flash_data[fd_ptr+1] = fd_ptr_m1 + 0x6000;        /* prev_loc update*/
      fd_ptr_m1 = fd_ptr;               /* store pointer history */
      fd_ptr += nrecord;                /* compute the next flash_data[] pointer */
      retrieved_flag[loc_code] = 1;     /* flag loc_code record as retreved */
    }
  } while(prev_loc!=0); /* loop until begining of FLASH */
  
  /* Erase FLASH and save all memory records from flash_data[]: */
  flash_data[1] = 0;    /* force 0 for first prev_loc */
  flash_locked = 0;     /* unlock flash */
  itemp = prog_flash(0x6000, fd_ptr, flash_data, 1, "inStore1");    /* erase and program FLASH with flash_data[] */
  if(itemp){
    goto store_out;
  }
  ptr = fd_ptr + 0x6000;                    /* compute pointer for below */
  /* ptr_m1 = ptr - RECORD_LENGTH;      This give a compiler ERROR!!!!!! */
  ptr_m1 = ptr - nrecord;
}

/* Store current state to location ptr: */
record[0] = ptr + nrecord;                  /* next_loc value for new record */
record[1] = ptr_m1;                         /* prev_loc value for new record */
record[2] = VERSION;                        /* Version for new record */
record[3] = *((unsigned*)(&serial_number));     /* first word of SN */
record[4] = *((unsigned*)(&serial_number)+1);   /* second word of SN */
record[5] = current_loc;                    /* loc_code for new record */

/* Sneek other state variables into spare locations of params[][] if nessary: */
/*params[1][1] =  */

/* Save params[][] into record[]: */
j=6;
uptr = (unsigned*)(&params[0][0]);  /* point to beginning of params[i][] */
for(i=0;i<NPARAMS;i++){
  record[j++] = *uptr++;                    /* read first word from params[i][0] */
  record[j++] = *uptr++;                    /* read second word from params[i][0] */
  record[j++] = *uptr++;                    /* read first word from params[i][1] */
  record[j++] = *uptr++;                    /* read second word from params[i][1] */
  record[j++] = *uptr++;                    /* read first word from params[i][2] */
  record[j++] = *uptr++;                    /* read second word from params[i][2] */
}
/* record[j++] =    /* more data... */

flash_locked = 0;   /* unlock flash */
prog_flash(ptr, nrecord, record, 0, "inStore2");    /* program FLASH, don't erase */


/* read_flash(0x6000, 0x2000, flash_data);  /* fix: read sector 3  for inspection */

store_out:
func_addr_a = func_addr_temp_a; /* reset the function */
func_addr_b = func_addr_temp_b; /* reset the function */
wait(500000);           /* wait(500000) for human to read display */
update_disp_left();     /* display parameter state */
update_disp_right(1);
portfff5 |= 0x0200;     /* re-enable delta interupts */
sw_down = 0;            /* this is required to get the curssor flashing again */
}


/**************************************************************************
 * recall
 * This function recalls the filter module's settings from FLASH memory sector 3.
 *
 **************************************************************************/
void recall(void)
{

int i, j;
unsigned next_loc, prev_loc=0, loc_code, ptr, ptr_m1=0, current_loc;
unsigned desired_loc_ptr=0, nrecord;
unsigned* uptr;

min_value = 0;  /* set min and max value to bound parameter */
max_value = LAST_MEM_LOC;

if(params_changed_copy!=3){ /* return if no press action is flaged */
  return;
}

nrecord = RECORD_LENGTH;

current_loc = (unsigned)params[11][0];

func_addr_a = (unsigned)&no_func_a; /* reduce ISR overhead, required for read_flash() and prog_flash() */
func_addr_b = (unsigned)&no_func_b;

portfff5 &= ~0x0200;    /* suspend delta interupts while recalling */
disp_num(current_loc, 1, 7, 0); /* write: "9 Recalled" */
disp_text(" Recalled", 8, 0);   /* write LCD */
beep(75, 400);                  /* Beep speaker */

/* search for end of record data in FLASH */
ptr = 0x6000;   /* point to first record */
read_flash(ptr, 6, record); /* read part of header */
next_loc = record[0];
loc_code = record[5];
while(next_loc!=0xffff){
  if(record_bad()){ /* Error: memory not properly erased or is corrupted */
    goto recall_error;
  }
  if(loc_code==current_loc){    /* loc_code == desired location */
    desired_loc_ptr = ptr;      /* point to a valid memory location */
  }
  ptr_m1 = ptr;     /* save pointer history */
  ptr = next_loc;   /* point to next location */
  read_flash(ptr, 6, record);   /* read part of header */
  next_loc = record[0];
  loc_code = record[5];
}
/* ptr now points to first blank location, ptr_m1 points to previous location. */

if(desired_loc_ptr==0){ /* desired location not in memory, so don't recall */
 recall_error:
/*  disp_text("Recall Error    ", 1, -1);   /* write LCD */
  disp_text("Location Blank! ", 1, -1); /* write LCD */
  wait(1000000);
  goto recall_out;
}

/* Read FLASH record[] */
read_flash(desired_loc_ptr, nrecord, record);   /* read FLASH */

/* Load into params[][] from record[]: */
j=6;
uptr = (unsigned*)(&params[0][0]);  /* point to beginning of params[i][] */
for(i=0;i<NPARAMS;i++){
  *uptr++ = record[j++];                    /* write first word to params[i][0] */
  *uptr++ = record[j++];                    /* write second word to params[i][0] */
  *uptr++ = record[j++];                    /* write first word to params[i][1] */
  *uptr++ = record[j++];                    /* write second word to params[i][1] */
  *uptr++ = record[j++];                    /* write first word to params[i][2] */
  *uptr++ = record[j++];                    /* write second word to params[i][2] */
}
/* = record[j++] /* more data... */


recall_out:

set_fsample();  /* if nessasary, set the sampling rate (freqs should not be a out of bounds!) */
auto_vu_count = (int)params[2][0];  /* restart counter; set to 0 or 1 depending on RevertToLevels */
assembly_flag = (int)params[5][0] ? (assembly_flag|8):(assembly_flag&(~8)); /* set/clear the white noise flag for assembly code */
assembly_flag = (int)params[7][0] ? (assembly_flag|4):(assembly_flag&(~4)); /* set/clear the cascade flag for assembly code */

/* reset VU variables: (keeps VUs from bouncing back down when recalling. Optional) */
in_a_hold = 0;
in_b_hold = 0;
out_a_hold = 0;
out_b_hold = 0;
in_a_vu_level = 0;
in_b_vu_level = 0;
out_a_vu_level = 0;
out_b_vu_level = 0;

/* Display top level function: */
param_ptr = 0;      /* point function parameter */
flag_options = 0;   /* don't point to options section */
index_ab = params[6][0] ? 0:2;  /* if Mode:A&B Common (params[6][0]==0) then index common params, else index A params */   

set_all_gains();    /* set optimal CODEC gains and attenuations */


/* Set the current function: */
if(params[6][0]==0){    /* Mode:A&B Common */
  update_dsp(param_ptr_start[(int)params[0][2]],2); /* Initialize the current function */
}
else if(params[6][0]==1){ /* Mode:A&B Separate */
  update_dsp(param_ptr_start[(int)params[0][0]],0); /* Initialize the current function */
  update_dsp(param_ptr_start[(int)params[0][1]],1); /* Initialize the current function */
}
else{                   /* Mode:Ch A Only */
  func_addr_b = (unsigned)&no_func_b;   /* set Ch B to no_func so Ch A can run long filter */
  update_dsp(param_ptr_start[(int)params[0][0]],0); /* Initialize the current function */
}
min_value = 0;  /* reset min and max value to bound parameter because it was changed in update_dsp() */
max_value = LAST_MEM_LOC;

wait(600000);           /* wait(600000) for human to read display (longer than store) */

/* params_changed = 2;      /* flag main loop to update DSP */
update_disp_left();     /* display parameter state */
update_disp_right(1);
portfff5 |= 0x0200;     /* re-enable delta interupts */
sw_down = 0;            /* this is required to get the curssor flashing again */
}


/**************************************************************************
 * record_bad
 * Checks Vesion and serial number in first 5 locations of record[].
 * Returns 0 if record OK
 *         1 if record bad
 *
 **************************************************************************/
int record_bad(void)
{

return ( (record[2]!=VERSION)||(record[3]!=*((unsigned*)(&serial_number)))||
         (record[4]!=*((unsigned*)(&serial_number)+1))                       );
}


#endif /* if(main) */
/*==========================================================================*/











































































/* unused code scraps: */
#if(0)

/**************************************************************************
 * this ISR was too slow at re-enabling interrupts
 *
 **************************************************************************/
interrupt void txrxint_c_old(void)
{
int i,j,temp;
unsigned long jj;

/* asm("    setc    XF  ; turn on xf LED "); */

portfff6 = 0x00f0;  /* Reset change detect bits (clears condition that caused interrupt) */

asm("   clrc    INTM        ; Re enable interrupts so sampling ISR works");
/* handle masking of another txrxint here... */

/* read_IOSR(temp); */
temp = portfff6;
temp = temp;

for(jj=1;jj<3.00000;jj++){}


*ifr_ptr = CLR_TXRXINT; /* Clear TXRXINT flag bit to avoid double interrupts*/
/*  asm("   clrc    XF  ; turn off xf LED "); */
}

/**************************************************************************
 *
 *
 **************************************************************************/
interrupt void int1_c(void)
{
#define N 75
#define ORDER 50
int i,j,sum;
int array1[200], coeff[200], output[100];

/* asm("    setc    XF  ; turn on xf LED ");
asm("   clrc    XF  ; turn off xf LED ");
asm("   setc    XF  ; turn on xf LED "); */

/* for(i=1;i<10;i++){
    sum+=array1[i]*coeff[i];
} */

/* for(i=0;i<N-ORDER;i++){
    sum=0;
    for(j=1;j<ORDER;j++){
        sum+=array1[i+j]*coeff[j];
    }
    output[i]=sum>>15;
} */


/*  *ifr_ptr = 0x0001;  /* Clear INT1 flag bit (not required) */
/*  asm("   clrc    XF  ; turn off xf LED "); */
}

 /* old disp_text
 *   *text      -   Text string to write to LCD display
 *                  (if null string -> only cursor repositioned)
 *   position   -   1 to 16 - Position of first character
 *                  0       - Text written to current cursor position (fast)
 *                  If sign positive: cursor is made visible (if not)
 *                  If sign negative: cursor is made invisible (if not)
 *                  Cursor is put into "position" on exit
 *                  (out-of-limits not checked)
 *
 **************************************************************************/
void disp_text(char *text, int position)
{
int abs_pos;

if(position){
  /* Put cursor in "position" */
  abs_pos = abs(position);  /* take absolute value */
  if(abs_pos<9){
    write_lcd_inst((0x80 - 1) + abs_pos, 40);
    }
  else{
    write_lcd_inst((0xc0 - 9) + abs_pos, 40);
  }
  cursor_pos = abs_pos;

  /* Write characters: */
  while(*text != 0){    /* loop while not end of string (not null character) */
    write_lcd_data(*text++, 40);
    cursor_pos++;
    if(cursor_pos==9){
      write_lcd_inst(0xc0, 40);/* put the cursor in location 8 */
    }
  }
  
  /* Put cursor back into "position" */
  if(abs_pos<9){
    write_lcd_inst((0x80 - 1) + abs_pos, 40);
    }
  else{
    write_lcd_inst((0xc0 - 9) + abs_pos, 40);
  }
  cursor_pos = abs_pos;

  /* Turn on/off cursor: */
  if(position>0){
    write_lcd_inst(0x0e, 40);   /* turn on cursor if "position" positive */
    }
  else{
    write_lcd_inst(0x0c, 40);   /* turn off cursor if "position" negative */
  }
  
}
else{   /* position = 0 */

  /* Write characters to current position: */
  while(*text != 0){    /* loop while not end of string (not null character) */
    write_lcd_data(*text++, 40);
    cursor_pos++;
    if(cursor_pos==9){
      write_lcd_inst(0xc0, 40);/* put the cursor in location 8 */
    }
  }
  
  /* Put cursor back into "position" */
  abs_pos = abs(position);  /* take absolute value */
  if(abs_pos<9){
    write_lcd_inst((0x80 - 1) + abs_pos, 40);
    }
  else{
    write_lcd_inst((0xc0 - 9) + abs_pos, 40);
  }
  cursor_pos = abs_pos;
}
}


/* old dsip_int: */
void disp_int(int inum, int position, int width)
{
#define BUFLEN 17

char tempc[BUFLEN];
char *bufptr;
int i, wcnt = width, neg = inum < 0;
int unum = neg ? -inum : inum;  /* take absolute value */

/* Convert integer and load array: */
*(bufptr = &tempc[BUFLEN - 1]) = 0; /* put null character at end of charracter array */
do{
  *--bufptr = (unum % 10) + '0';    /* next LS digit, add to code for 0 and store in array */
  wcnt--;
} while(unum /= 10);

/* Insert minus sign if negative: */
if(neg){
  *--bufptr = '-';
  wcnt--;
}
/* Check for field overflow and flag: */
if(wcnt<0){
  bufptr = &tempc[BUFLEN - 1];  /* point to last charracter in array */
  for(i=0;i<width;i++){
    *--bufptr = '*';            /* fill with *'s if field overflowed */
  }
}
/* Add leading spaces: */
while((wcnt--)>0){
  *--bufptr = ' ';
}

/* Display text at *bufptr */
disp_text(bufptr, position);
}


/**************************************************************************
 * disp_float
 *   This function writes a formated number to the LCD.
 *
 *   num        -   Number to write to LCD display
 *   position   -   1 to 16 - Position of first character (#, -, or space)
 *                  0       - Number written to current cursor position (fast)
 *                  If sign positive: cursor is made visible
 *                  If sign negative: cursor is made invisible
 *                  Cursor is put into "position" on exit
 *                  (out-of-limits not checked)
 *   width      -   Total field width of display
 *   nfrac      -   Number of digits in fractional portion
 *                  w, f = width, fraction e.g. if num = -4.8567
 *                                              6, 2 -> " -4.85"
 *                                              4, 1 -> "-4.8"
 *                                              4, 0 -> "  -4"
 *                                              4, 2 -> "****"
 *
 **************************************************************************/
void disp_float(float num, int position, int width, int nfrac)
{
int i, abs_pos, imult;
char *text, *tempstr1, *tempstr2;
char numstr[17];
long inum;
float fnum;

imult = 1;       
for(i=0;i<nfrac;i++){
  imult *= 10;
}

/* get the integer and fractional portion of num: */
inum = (long) num;      /* get the interger part of the number */
fnum = num - inum;      /* get fractional part (same sign as num) */
fnum = fabs(fnum);      /* take absolute value of fractional part */

ltoa((long)(imult*fnum),numstr);    /* shift fractional portion and convert to text */
tempstr1 = ".";
strcat(tempstr1, numstr);
ltoa(inum,numstr);
strcat(numstr, tempstr1);

disp_text(numstr, position);

}

/**************************************************************************
 * ideal_fir
 * This function computes ideal truncated FIR filter coefficients for
 * a multi-band filter.
 *
 **************************************************************************/
void ideal_fir(float *coefs, int ntaps, float *gains, float *freqs, int nbands)
{
int n, k;
float ssum, ftemp, d2fs, sincx;

gains[nbands] = 0.0;    /* append zero at end of gains */
d2fs = 2.0/fsample;

for(n=0;n<ntaps;n++){
  ssum = 0.0;
  for(k=0;k<nbands;k++){    /* loop over filter bands */
    ftemp = PI*((float)n - ((float)(ntaps-1))/2.0);
    if(ftemp){
      sincx = sin(d2fs*freqs[k]*ftemp)/ftemp;
    }
    else{
      sincx = d2fs*freqs[k];
    }
    ssum += (gains[k] - gains[k+1])*sincx;
  }
  coefs[n] = ssum;
}

}

/**************************************************************************
 **************************************************************************/
void get_serial_old(char *sn)
{
*sn = '0';
*(sn+1) = '1';
*(sn+2) = '2';
*(sn+3) = '3';
*(sn+4) = '4';
*(sn+5) = '5';
*(sn+6) = '6';
*(sn+7) = '7';
*(sn+8) = '8';
*(sn+9) = '9';
*(sn+10) = '0';
*(sn+11) = '1';

}


/**************************************************************************
 * prog_test
 * a temparary prog.: remove!
 **************************************************************************/
void prog_test(void)    /* execute flash programming tests */
{
unsigned sumjunk, startadr;
int i,j, temp;


sumjunk = 0;
for(j=0;j<8;j++){
  startadr = j*0x2000;
  for(i=0;i<0x2000;i++){
    flash_data[i] = i+startadr;
  }

  disp_text("programming...  ", -1);
  flash_locked = 0; /* unlock flash */
  temp = prog_flash(startadr, 0x2000, flash_data, 1);
  if(temp==0){
    disp_text("done programming", -1);
  }
  else{
    disp_text("Prog. error:    ", -1);
    disp_num(temp,14,1,0);
  }


  for(i=0;i<0x2000;i++){
    flash_data[i] = 3;
  }

  if(1){
    read_flash(startadr, 0x2000, flash_data);
    for(i=0;i<0x2000;i++){
      sumjunk += (flash_data[i]!=i+startadr);
      if(flash_data[i]!=i+startadr){
        sumjunk = sumjunk;
      }
    }
    sumjunk = sumjunk;
  }
}  /* end for */

while(1){
  sumjunk = sumjunk;
}
}

/* Compute VU level for in_b: */
ftemp = scale_k_b*(float)(in_b_hold) + 1.0; /* scale the peak held value */
if(ftemp>=32768.0){ /*if ftemp>=0x8000 then clip */
  ftemp = 32768.0;
}

vu_temp = 0;
while(((unsigned)(ftemp))>0){   /* Take coarse log(). This can be speeded up by ftemp = ftemp/((.7071068^15)) before loop */
  ftemp = ftemp*0.7071068;
  vu_temp++;    /* count number of times we can mult by 1/sqrt(2) before ftemp<1 (3dB per division) */
}

if(vu_temp>in_b_vu_level){  /* get peak level for VU Meter */
  in_b_vu_level = vu_temp;
  vu_ctr_in_b = 0;
  if(in_b_vu_level>30){ /* if overload: hold 'X' on display for 4 times VU_DECAY */
    vu_ctr_in_b = -3*VU_DECAY;
  }
}
else if(vu_ctr_in_b>=VU_DECAY){ /* decay the VU Meter*/
  vu_ctr_in_b = 0;
  in_b_hold = 0;
  if(in_b_vu_level>0){
    in_b_vu_level--;
  }
}

in_b_code = in_b_vu_level - 22; /* compute ASCII code for VU Meter: in_b_code=8 for 0dB (scale_k*hold = 32768) */
if(in_b_code<0){
  in_b_code = 0;
}
else if(in_b_code>=9){
  in_b_code = 9;
}


/* Compute VU level for out_b: */
ftemp = scale_k_b*(float)(out_b_hold) + 1.0;    /* scale the peak held value */
if(ftemp>=32768.0){ /*if ftemp>=0x8000 then clip */
  ftemp = 32768.0;
}

vu_temp = 0;
while(((unsigned)(ftemp))>0){   /* Take coarse log(). This can be speeded up by ftemp = ftemp/((.7071068^15)) before loop */
  ftemp = ftemp*0.7071068;
  vu_temp++;    /* count number of times we can mult by 1/sqrt(2) before ftemp<1 (3dB per division) */
}

if(vu_temp>out_b_vu_level){ /* get peak level for VU Meter */
  out_b_vu_level = vu_temp;
  vu_ctr_out_b = 0;
  if(out_b_vu_level>30){    /* if overload: hold 'X' on display for 4 times VU_DECAY */
    vu_ctr_out_b = -3*VU_DECAY;
  }
}
else if(vu_ctr_out_b>=VU_DECAY){    /* decay the VU Meter*/
  vu_ctr_out_b = 0;
  out_b_hold = 0;
  if(out_b_vu_level>0){
    out_b_vu_level--;
  }
}

out_b_code = out_b_vu_level - 22;   /* compute ASCII code for VU Meter: out_b_code=8 for 0dB (scale_k*hold = 32768) */
if(out_b_code<0){
  out_b_code = 0;
}
else if(out_b_code>=9){
  out_b_code = 9;
}

/**************************************************************************
 * time_startold
 * Restart timer so subsequent calls to time() returns number of micro-seconds
 * since time_start was called.
 *
 **************************************************************************/
void time_startold(void)
{

/* Start the timer free running for wait() and delta_t() usage: */
portfff9 = 0xffff;      /* (PRD) set reload value */
portfff8 = 0x03fc;      /* (TCR) set prescaler for divide by 13 (CLKOUT1/13 = ~1.8904MHz) and reload timer */
portfff8 = 0x03cc;      /* (TCR) start timer */
                        /* TIM counter (portfffa) now counts down from 0xffff and repeats */

portfff9 = 0x0000;      /* (PRD) set timer reload value to 0 so counter counts */
                        /* down to 0 and stays there */
}

/**************************************************************************
 * timeold
 * Return time in ~microseconds since time_start() was called.
 * In this case a ~microsecond is about 1.058 true microseconds.
 * Returns 0 to 32767. Returns 32767 if more that 32767uS has ellapsed.
 *
 **************************************************************************/
int timeold(void)
{
int tim;

tim = portfffa;     /* get current running timer value */
/* return (unsigned)(PSCPERIOD*((float)(0xffff-tim)));  /* compute and return ellapsed time */
return ((0xffff-tim)>>1);   /* compute and return ellapsed time */

}


/**************************************************************************
 * press_interface()
 * Respond to press/hold command by writing: "hold to ..." to LCD
 **************************************************************************/
void press_interface()
{
int i=0;
char ctemp;
char carray[]="Hold to         ";

if(press_hold_flag==1){
  /* get current function text up to the ":" and write display */
  while(((ctemp = (*(params[func_index][param_index][0].text + i)))!=':')&&(ctemp!='\0')&&(i<8)){
    carray[(i++)+8] = ctemp;
  }
  disp_text(carray, 1, 0);  /* write LCD */
}
else if(press_hold_flag<HOLD_TIME){
  if(0x0004&portfff6){  /* if knob is up */
    disp_text("Aborted!        ", 1, 0);    /* write LCD */
    wait(1000000);          /* wait 1 sec. */
    update_disp_left();     /* display parameter state */
    update_disp_right(1);
    press_hold_flag = 0;
    return;
  }
  
  
  
}
else{   /* press_hold_flag >= HOLD_TIME => press/hold OK */
  params_changed = 2;   /* flag update_dsp() to carry out "press" action */
  /* down_turn_flag = 1;    /* flag cursor not to jump back to right after release */
  press_hold_flag = 0;
  return;
}
press_hold_flag++;
}



/* Read last code sector in flash and load into code memory: */
#define C_LOADER    0           /* 0-for degugging, 1-for c-boot loader code activation */
#define CODE_START  8192+8064-2 /* (16254=0x3f7e) starting memory location for c-boot loader */
#define CODE_ORIGIN 0x40        /* code origin in the linker command file: Filtlink.cmd */
#define CODE_LENGTH 0x5400      /* code length in the linker command file: Filtlink.cmd */
#define NWORDS      (0x40 + 0x5400 - (8192+8064-2)) /* number of words to transfer */

#if(C_LOADER)
/* read_flash(0x4000, NWORDS, flash_data);  /* read data from sector 2 (last code sector) */
read_flash(0x2000, 0x2000, flash_data); /* read data from sector 2 (last code sector) */
uptr = (unsigned*)CODE_START;   /* point to beginning of params[i][] */
for(utemp=0;utemp<NWORDS;utemp++){
  *uptr++ = flash_data[utemp];          /* write code word to memory */
}
utemp = utemp;
#endif



  for(itemp=0;itemp<CURSOR_PERIOD;itemp++){
    count_start = portfffa;     /* grab current timer value (for loop interval timing) */
    if(write_ptr!=read_ptr){    /* something in serial_in_buf[] */
      parse_command();          /* check if command complete, parse and execute */
      count_start = portfffa;   /* grab current timer value to avoid overflow (resets interval) */
    }

#if(MAIN)
    if(params_changed){
      portfff5 &= ~0x0200;  /* suspend delta interupts so params_changed flag is not modifed here: */
      params_changed_copy = params_changed; /* make working copy for update_dsp() */
      params_changed = 0;   /* reset change flag */
      portfff5 |= 0x0200;   /* re-enable delta interupts */
      update_dsp(param_ptr, index_ab);  /* update the DSP's function to reflect current parameters */
      count_start = portfffa;   /* grab current timer value to avoid overflow (resets interval) */
    }
    
    led_update();           /* set overload LEDs to reflect the status of the overload bits */
    if(assembly_flag&3){
      portfff5 &= ~0x0200;  /* suspend delta interupts while updating display in VU flag */
      vu_update();          /* update VU meters (this is usually called every 5ms) */
      portfff5 |= 0x0200;   /* re-enable delta interupts */
    }
#endif

    while(delta_t(count_start)<30000);  /* 5000 wait until loop time interval is over */
  }



/**************************************************************************
 * compute_notch
 * This function computes and loads IIR coefficients for the Notch and Inverse Notch
 * filters based on the currently selected function in params[0][index_ab_tmp].
 *
 **************************************************************************/
void compute_notch_old(float fn, float fw, int index_ab_tmp)
{
int* iptr;
float r, costheta2, kin, kout, a11, a21, b01, b11, b21, a12, a22, b02, b12, b22;

r = 1 - 2*PI*fw/fsample;            /* estimate the radius of pole from fwidth */
costheta2 = 2*cos(2*PI*fn/fsample); /* temp var = 2cos(theta) where: theta = 2*pi*fn/fsample */

/* Compute ideal notch filter coefficients: */
/* Note: all a's and b's must be in range: [-2.0, 2.0) before quantization. */
switch((int)params[0][index_ab_tmp]){   /* switch on index of currently selected function */
case 5: /* Notch */
  kin = 0.75;   /* kin magnitude <= 1.0 */
  kout = 1.1;   /* kout magnitude < 2.0 */

/* Realize the zeros first (avoids overflow, but adds noise near fnotch): */
  a21 = 0.0;
  a11 = 0.0;
  b21 = kin;
  b11 =-kin*costheta2;
  b01 = kin;

  a22 = r*r;
  a12 =-r*costheta2;
  b22 = 0.0;
  b12 = 0.0;
  b02 = kout;

  break;
  
case 6: /* Inverse Notch */

  break;

default:
  break;
}   /* end switch */


/* Load quantized coefs: */
out_gain |= 0x0400;     /* mute the outputs */
switch(index_ab_tmp){
case 0: /* Ch. A */
  func_addr_a = (unsigned)&no_func_a; /* get more CPU time to write the coefdata[] */
  data_ptr_a = 0x03ff;  /* point to first used Ch A filter state data */
  /* Write coefs to filter A locations (section 2 is just an all pass): */
  iptr = (int*)&coefdata[0x80]; /* get address of first coef to write */
  *iptr++ = (int)(16384.0*a21); /* a21 */
  *iptr++ = (int)(16384.0*a11); /* a11 */
  *iptr++ = (int)(16384.0*b21); /* b21 */
  *iptr++ = (int)(16384.0*b11); /* b11 */
  *iptr++ = (int)(16384.0*b01); /* b01 */
  
  *iptr++ = (int)(16384.0*a22); /* a22 */
  *iptr++ = (int)(16384.0*a12); /* a12 */
  *iptr++ = (int)(16384.0*b22); /* b22 */
  *iptr++ = (int)(16384.0*b12); /* b12 */
  *iptr++ = (int)(16384.0*b02); /* b02 */
  
  func_addr_a = (unsigned)&iir_4_a; /* set function A */
  break;

case 1: /* Ch. B */
  func_addr_b = (unsigned)&no_func_b; /* get more CPU time to write the coefdata[] */
  data_ptr_b = 0x037f;  /* point to first used Ch B filter state data */
  
  /* Write coefs to filter B locations (section 2 is just an all pass): */
  iptr = (int*)&coefdata[0];    /* get address of first coef to write */
  *iptr++ = (int)(16384.0*a21); /* a21 */
  *iptr++ = (int)(16384.0*a11); /* a11 */
  *iptr++ = (int)(16384.0*b21); /* b21 */
  *iptr++ = (int)(16384.0*b11); /* b11 */
  *iptr++ = (int)(16384.0*b01); /* b01 */
  
  *iptr++ = (int)(16384.0*a22); /* a22 */
  *iptr++ = (int)(16384.0*a12); /* a12 */
  *iptr++ = (int)(16384.0*b22); /* b22 */
  *iptr++ = (int)(16384.0*b12); /* b12 */
  *iptr++ = (int)(16384.0*b02); /* b02 */
  
  func_addr_b = (unsigned)&iir_4_b; /* set function B */
  break;

case 2: /* Ch. A&B */
  func_addr_a = (unsigned)&no_func_a; /* get more CPU time to write the coefdata[] */
  func_addr_b = (unsigned)&no_func_b; /* get more CPU time to write the coefdata[] */
  data_ptr_a = 0x03ff;  /* point to first used Ch A filter state data */
  data_ptr_b = 0x037f;  /* point to first used Ch B filter state data */
  
  /* Write coefs to filter A locations (section 2 is just an all pass): */
  iptr = (int*)&coefdata[0x80]; /* get address of first coef to write */
  *iptr++ = (int)(16384.0*a21); /* a21 */
  *iptr++ = (int)(16384.0*a11); /* a11 */
  *iptr++ = (int)(16384.0*b21); /* b21 */
  *iptr++ = (int)(16384.0*b11); /* b11 */
  *iptr++ = (int)(16384.0*b01); /* b01 */
  
  *iptr++ = (int)(16384.0*a22); /* a22 */
  *iptr++ = (int)(16384.0*a12); /* a12 */
  *iptr++ = (int)(16384.0*b22); /* b22 */
  *iptr++ = (int)(16384.0*b12); /* b12 */
  *iptr++ = (int)(16384.0*b02); /* b02 */
  
  /* Write coefs to filter B locations (section 2 is just an all pass): */
  iptr = (int*)&coefdata[0];    /* get address of first coef to write */
  *iptr++ = (int)(16384.0*a21); /* a21 */
  *iptr++ = (int)(16384.0*a11); /* a11 */
  *iptr++ = (int)(16384.0*b21); /* b21 */
  *iptr++ = (int)(16384.0*b11); /* b11 */
  *iptr++ = (int)(16384.0*b01); /* b01 */
  
  *iptr++ = (int)(16384.0*a22); /* a22 */
  *iptr++ = (int)(16384.0*a12); /* a12 */
  *iptr++ = (int)(16384.0*b22); /* b22 */
  *iptr++ = (int)(16384.0*b12); /* b12 */
  *iptr++ = (int)(16384.0*b02); /* b02 */

  func_addr_a = (unsigned)&iir_4_a; /* set function A */
  func_addr_b = (unsigned)&iir_4_b; /* set function B */
  break;

default:
  break;
}   /* end switch(index_ab_tmp) */

  
/* wait_n_samples(???);     /* wait for ??? sampling intervals for transient to propagate */
out_gain &= ~0x0400;        /* un-mute the outputs */


}


/**************************************************************************
 * init_func
 * Initializes the current functions' gains and coefs when called with index_ab.
 *
 **************************************************************************/
void init_func(int index_ab_tmp)
{
int itemp, itemp2;

/* Set the gain for the currently selected function: */
gain(index_ab_tmp); /* Calculate setting of gain constant and output attenuator based on current func and index_ab_tmp */
itemp = (int)params[0][index_ab_tmp];   /* get index of currently selected function */
if(itemp==0){   /* current function is AllPass */
  itemp2 = index_ab_tmp + 1;    /* itemp: 1-A, 2-B, 3-Common */
  if(itemp2&1){
    func_addr_a = (unsigned)&allpass_func_a; /* send input to output */
  }
  if(itemp2&2){
    func_addr_b = (unsigned)&allpass_func_b; /* send input to output */
  }
}
else if(itemp<=4){  /* currrent function is an FIR or IIR filter */
  set_filter(param_ptr_start[itemp], index_ab_tmp); /* calculate any slave params and compute the filter coefs and load */
}
else if(itemp<=6){  /* current function is a Notch or Inverse Notch filter */
  set_notch(param_ptr_start[itemp], index_ab_tmp);  /* compute the filter coefs and load */
}

}


/**************************************************************************
 * compute_iir
 * This function computes and loads IIR coefficients for LP, HP, BP and BS
 * filters based on the currently selected function in params[0][index_ab_tmp].
 *
 * Variables set in the header file "filt.h":
 *
 *  PROTO_CUTOFF    1.3089969390e-001   - Prototype cutoff frequency in radians
 *  NPZ_IIR 48          - # of entries in the pz_iir array for each filter type
 *  NK_IIR  8           - # of entries in the k_iir array for each filter type
 *  fcomplex pz_iir[]   - complex array of poles and zeros for digital prototyp filters:
 *                        access with [NPZ_IIR*itype + order_ptr[iorder-1] + offset]
 *              Butter  ChebI   ChebII  Ellip
 *  order = 1   p1      p1      p1      p1
 *              z1      z1      z1      z1
 *  order = 2   p1      p1      p1      p1
 *              p2      p2      p2      p2
 *              z1      z1      z1      z1
 *              z2      z2      z2      z2
 *  order = 3   p1      p1      p1      p1
 *              p2      p2      p2      p2
 *              p3      p3      p3      p3
 *              z1      z1      z1      z1
 *              z2      z2      z2      z2
 *              z3      z3      z3      z3
 * ... for orders 4,6,8
 *
 *  float k_iir[]       - gain constant multipliers for IIR filter
 *                        access with [NK_IIR*itype + iorder - 1]
 *
 *  int order_ptr[]     - filter order offset
 *
 * The IIR filters are realized with a 'scaled' normalized lattice filter
 * structure:
 *
 *                 ci
 *  >-------------->-- sum ---------->
 *            |         |
 *            |         |
 *           \|/ki     /|\-ki
 *            |         |                . . . to section 1
 *       -1   |    di   |      -1
 *   -- z -- sum --<---------- z  ---<
 *  |                       |
 *  |                       |
 * \|/v(i+1)               \|/ vi
 *  |                       |
 *  |                       |
 *   --------------------- sum ------>
 *
 **************************************************************************/
void compute_iir(float f1, float f2, int index_ab_tmp, int itype)
{
int i,j, itemp;
int *iptr1, *iptr2;
float omega1, omega2, alpha, kk, t1, t2, t3, t4, k_iir_now;
float apoly[9], bpoly[9], Cmm1[9], Amm1[9], k[8], v[9], c[8], d[8];
fcomplex ctemp1, ctemp2, z0, pz1, trad, pz_iir_now[16], c1[9], c2[9];
int iorder, iorderd2;

omega1 = 2*PI*f1/fsample;   /* desired cutoff frequencies in radians */
omega2 = 2*PI*f2/fsample;
iorder = 4;             /* filter order of prototype */
iorderd2 = iorder;

/* Compute IIR filter coefficients: */
switch((int)params[0][index_ab_tmp]){   /* switch on index of currently selected function */
case 1: /* LowPass */
  /* LP to LP transform: */
  alpha = sin((PROTO_CUTOFF-omega1)/2)/sin((PROTO_CUTOFF+omega1)/2);
  itemp = NPZ_IIR*itype + order_ptr[iorder-1];  /* point to first pole in pole/zero list */
  for(i=0;i<(iorder<<1);i++){   /* loop twice the filter order times */
    /* Map poles/zeros with: pz2 = (alpha+pz1)./(1+alpha*pz1) */
    pz_iir_now[i] = Cdiv(Cadd(Complex(alpha,0.0),pz_iir[i+itemp]),Cadd(Complex(1.0,0),RCmul(alpha,pz_iir[i+itemp])));
  }
  z0 = Complex(1.0,0);  /* z-plane location that D.C. is mapped into */
  break;
    
case 2: /* HighPass */
  /* LP to HP transform: */
  alpha = -cos((PROTO_CUTOFF+omega1)/2)/cos((PROTO_CUTOFF-omega1)/2);
  itemp = NPZ_IIR*itype + order_ptr[iorder-1];  /* point to first pole in pole/zero list */
  for(i=0;i<(iorder<<1);i++){   /* loop twice the filter order times */
    /* Map poles/zeros with: pz2 = -(alpha+pz1)./(1+alpha*pz1) */
    pz_iir_now[i] = Cdiv(Csub(Complex(-alpha,0.0),pz_iir[i+itemp]),Cadd(Complex(1.0,0),RCmul(alpha,pz_iir[i+itemp])));
  }
  z0 = Complex(-1.0,0); /* z-plane location that D.C. is mapped into */
  break;
    
case 3: /* BandPass */
  /* LP to BP transform: */
  iorderd2 = iorder>>1; /* iorderd2 = iorder/2 */
  alpha = cos((omega2+omega1)/2)/cos((omega2-omega1)/2);
  kk = tan(PROTO_CUTOFF/2)/tan((omega2-omega1)/2);
  t1 = 2*alpha*kk/(kk+1);
  t2 = (kk-1)/(kk+1);
  t3 = t1*t1 - 4*t2;
  t4 = 2*t1*t1 - 4*t2*t2 - 4;
  itemp = NPZ_IIR*itype + order_ptr[iorderd2-1];    /* point to first pole in pole/zero list */
  for(i=0;i<iorder;i++){    /* loop twice the filter order times */
    /* Map poles/zeros with:
    trad = ((t1^2-4*t2)*pz1.^2 + (2*t1^2-4-4*t2^2)*pz1 + (t1^2 - 4*t2)).^(0.5); % map poles
    pz2 =       (t1 + t1*pz1 + trad)./(2 + 2*t2*pz1);
    pz2 = [pz2; (t1 + t1*pz1 - trad)./(2 + 2*t2*pz1)];      */
    pz1 = pz_iir[i+itemp];
    trad = Csqrt(Cadd(Cadd(RCmul(t3,Cmul(pz1,pz1)),RCmul(t4,pz1)),Complex(t3,0)));
    pz_iir_now[2*i] = Cdiv(Cadd(Cadd(Complex(t1,0),RCmul(t1,pz1)),trad),Cadd(Complex(2,0),RCmul(2*t2,pz1)));
    pz_iir_now[2*i+1] = Cdiv(Csub(Cadd(Complex(t1,0),RCmul(t1,pz1)),trad),Cadd(Complex(2,0),RCmul(2*t2,pz1)));
  }
  
  trad = Complex(2*t3 + t4, 0);
  z0 = Cdiv(Cadd(Complex(2*t1,0),Csqrt(trad)),Complex(2 + 2*t2, 0));
  break;
  
case 4: /* BandStop */
  /* LP to BS transform: */
  iorderd2 = iorder>>1; /* iorderd2 = iorder/2 */
  alpha = cos((omega2+omega1)/2)/cos((omega2-omega1)/2);
  kk = tan(PROTO_CUTOFF/2)*tan((omega2-omega1)/2);
  t1 = 2*alpha/(kk+1);
  t2 = (1-kk)/(kk+1);
  t3 = t1*t1 - 4*t2;
  t4 = 2*t1*t1 - 4*t2*t2 - 4;
  itemp = NPZ_IIR*itype + order_ptr[iorderd2-1];    /* point to first pole in pole/zero list */
  for(i=0;i<iorder;i++){    /* loop twice the filter order times */
    /* Map poles/zeros with:
    trad = ((t1^2-4*t2)*pz1.^2 - (2*t1^2-4-4*t2^2)*pz1 + (t1^2 - 4*t2)).^(0.5); % map poles
    pz2 =       (-t1 + t1*pz1 + trad)./(-2 + 2*t2*pz1);
    pz2 = [pz2; (-t1 + t1*pz1 - trad)./(-2 + 2*t2*pz1)];        */
    pz1 = pz_iir[i+itemp];
    trad = Csqrt(Cadd(Csub(RCmul(t3,Cmul(pz1,pz1)),RCmul(t4,pz1)),Complex(t3,0)));
    pz_iir_now[2*i] = Cdiv(Cadd(Csub(RCmul(t1,pz1),Complex(t1,0)),trad),Csub(RCmul(2*t2,pz1),Complex(2,0)));
    pz_iir_now[2*i+1] = Cdiv(Csub(Csub(RCmul(t1,pz1),Complex(t1,0)),trad),Csub(RCmul(2*t2,pz1),Complex(2,0)));
  }
  z0 = Complex(1.0,0);
 break;
  
default:
  break;
}   /* end switch */

/* Compute new gain constant: k2 = k1*((prod(z1-1)/(prod(p1-1))*(prod(p2-z0))/prod(z2-z0))) */
ctemp1 = Complex(k_iir[NK_IIR*itype + iorderd2 - 1],0); /* init. ctemp1 = k1 */
for(i=0;i<iorderd2;i++){    /* loop half the filter order number of times */
  ctemp1 = Cmul(ctemp1,Cdiv(Csub(pz_iir[i+itemp+iorderd2],Complex(1.0,0)),Csub(pz_iir[i+itemp],Complex(1.0,0))));
}
for(i=0;i<iorder;i++){  /* loop filter order number of times */
  ctemp1 = Cmul(ctemp1,Cdiv(Csub(pz_iir_now[i],z0),Csub(pz_iir_now[i+iorder],z0)));
}
k_iir_now = ctemp1.r;   /* take real part (imag. should be near zero) */

/* Expand poles and zeros into polynomial form (sames as MATLABs poly()): */
c1[0] = Complex(1.0,0);         /* init temp complex array 2 */
c2[0] = Complex(k_iir_now,0);   /* init temp complex array 1 */
for(i=1;i<=iorder;i++){
  c1[i] = c2[i] = Complex(0,0);
}
for(i=0;i<iorder;i++){  /* loop the filter order number of times */
  ctemp1 = pz_iir_now[i];
  ctemp2 = pz_iir_now[i+iorder];
  for(j=i;j>=0;j--){
    c1[j+1] = Csub(c1[j+1],Cmul(ctemp1,c1[j]));
    c2[j+1] = Csub(c2[j+1],Cmul(ctemp2,c2[j]));
  }
}
for(i=0;i<=iorder;i++){
  apoly[i] = c1[i].r;   /* get real part of c1[] */
  bpoly[i] = c2[i].r;   /* get real part of c2[] */
}

/* Convert to lattice form (same as MATLABs: [k,v] = tf2latc(bpoly,apoly) ): */
itemp = iorder;
for(i=(iorder-1);i>=0;i--){
  t1 = k[i] = apoly[itemp]; /* get lattice coefficients */
  t2 = v[i+1] = bpoly[itemp];
  for(j=0;j<=itemp;j++){
    Cmm1[j] = bpoly[j] - t2*apoly[itemp-j];
    Amm1[j] = (apoly[j] - t1*apoly[itemp-j])/(1 - t1*t1);
  }
  itemp--;
  for(j=0;j<=itemp;j++){
    apoly[j] = Amm1[j];
    bpoly[j] = Cmm1[j];
  }
}  
v[0] = bpoly[0];    /* grab last ladder coeff */

/* Normalize and scale based on the reflection coefs: */
t1 = 1.0;   /* Add-hawk scaling factor to limit signal growth = ((order*2).^(-1/(order*2))) */
for(i=0;i<iorder;i++){
  t2 = k[i];
  c[i] = t1*sqrt(1.0 - t2*t2);
  d[i] = (1.0 - t2*t2)/c[i];
}

/* Scale the MA taps based on c[]: */
t1 = 1.0;
for(i=(iorder-1);i>=0;i--){
  t1 *= c[i];
  v[i] /= t1;
}

/* Load quantized coefs: */
out_gain |= 0x0400;     /* mute the outputs */

itemp = index_ab_tmp + 1;   /* itemp: 1-A, 2-B, 3-Common */
if(itemp&1){
  func_addr_a = (unsigned)&no_func_a; /* get more CPU time to write the coefdata[] */
  data_ptr_a = 0x03ff;  /* point to first used Ch A filter state data */
  /* Write coefs to filter A locations: */
  iptr1 = (int*)(0x037f + (iorder<<2)); /* point to first lattice coef address */
  iptr2 = (int*)(0x0380 + (iorder<<2)); /* point to first MA coef address */
  for(i=0;i<iorder;i++){
    *iptr1-- = (int)(32768*k[i] + 0.5); /* ki */
    *iptr1-- = (int)(32768*d[i] + 0.5); /* di */
    *iptr1-- = (int)(32768*k[i] + 0.5); /* ki (copy) */
    *iptr1-- = (int)(32768*c[i] + 0.5); /* ci */
    *iptr2++ = (int)(32768*v[i] + 0.5); /* vi (MA taps) */
  }
  *iptr2 = (int)(32768*v[iorder] + 0.5);    /* vi (last MA tap) */
  func_addr_a = (unsigned)&lattice_4_a; /* set function A */
}

if(itemp&2){
  func_addr_b = (unsigned)&no_func_b; /* get more CPU time to write the coefdata[] */
  data_ptr_b = 0x037f;  /* point to first used Ch B filter state data */
  /* Write coefs to filter B locations: */
  iptr1 = (int*)(0x02ff + (iorder<<2)); /* point to first lattice coef address */
  iptr2 = (int*)(0x0300 + (iorder<<2)); /* point to first MA coef address */
  for(i=0;i<iorder;i++){
    *iptr1-- = (int)(32768*k[i] + 0.5); /* ki */
    *iptr1-- = (int)(32768*d[i] + 0.5); /* di */
    *iptr1-- = (int)(32768*k[i] + 0.5); /* ki (copy) */
    *iptr1-- = (int)(32768*c[i] + 0.5); /* ci */
    *iptr2++ = (int)(32768*v[i] + 0.5); /* vi (MA taps) */
  }
  *iptr2 = (int)(32768*v[iorder] + 0.5);    /* vi (last MA tap) */
  func_addr_b = (unsigned)&lattice_4_b; /* set function B */
}
  
/* wait_n_samples(???);     /* fix: wait for ??? sampling intervals for transient to propagate */
out_gain &= ~0x0400;        /* un-mute the outputs */

}


/**************************************************************************
 * complex.c
 * This is the complex arithmetic library from "Numerical Recipes in C"
 * pp. 948-950
 *
 **************************************************************************/
fcomplex Cadd(fcomplex a, fcomplex b)
{
    fcomplex c;
    c.r=a.r+b.r;
    c.i=a.i+b.i;
    return c;
}

fcomplex Csub(fcomplex a, fcomplex b)
{
    fcomplex c;
    c.r=a.r-b.r;
    c.i=a.i-b.i;
    return c;
}


fcomplex Cmul(fcomplex a, fcomplex b)
{
    fcomplex c;
    c.r=a.r*b.r-a.i*b.i;
    c.i=a.i*b.r+a.r*b.i;
    return c;
}

fcomplex Complex(float re, float im)
{
    fcomplex c;
    c.r=re;
    c.i=im;
    return c;
}

fcomplex Conjg(fcomplex z)
{
    fcomplex c;
    c.r=z.r;
    c.i = -z.i;
    return c;
}

fcomplex Cdiv(fcomplex a, fcomplex b)
{
    fcomplex c;
    float r,den;
    if (fabs(b.i) <= fabs(b.r)) {
        r=b.i/b.r;
        den=b.r+r*b.i;
        c.r=(a.r+r*a.i)/den;
        c.i=(a.i-r*a.r)/den;
    } else {
        r=b.r/b.i;
        den=b.i+r*b.r;
        c.r=(a.r*r+a.i)/den;
        c.i=(a.i*r-a.r)/den;
    }
    return c;
}

/*
float Cabs(fcomplex z)
{
    float x,y,ans,temp;
    x=fabs(z.r);
    y=fabs(z.i);
    if (x == 0.0)
        ans=y;
    else if (y == 0.0)
        ans=x;
    else if (x > y) {
        temp=y/x;
        ans=x*sqrt(1.0+temp*temp);
    } else {
        temp=x/y;
        ans=y*sqrt(1.0+temp*temp);

}
    return ans;
}
*/

fcomplex Csqrt(fcomplex z)
{
    fcomplex c;
    float x,y,w,r;
    if ((z.r == 0.0) && (z.i == 0.0)) {
        c.r=0.0;
        c.i=0.0;
        return c;
    } else {
        x=fabs(z.r);
        y=fabs(z.i);
        if (x >= y) {
            r=y/x;
            w=sqrt(x)*sqrt(0.5*(1.0+sqrt(1.0+r*r)));
        } else {
            r=x/y;
            w=sqrt(y)*sqrt(0.5*(r+sqrt(1.0+r*r)));
        }
        if (z.r >= 0.0) {
            c.r=w;
            c.i=z.i/(2.0*w);
        } else {
            c.i=(z.i >= 0) ? w : -w;
            c.r=z.i/(2.0*c.i);
        }
        return c;
    }

}

fcomplex RCmul(float x, fcomplex a)
{
    fcomplex c;
    c.r=x*a.r;
    c.i=x*a.i;
    return c;
}



#endif
