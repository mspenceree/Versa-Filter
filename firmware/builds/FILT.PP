#line 135 "c203.h"
unsigned int ioData;

 
#line 14 "c:\c2xxti\stdlib.h"
typedef unsigned size_t;
#line 19
typedef char wchar_t;





int           atoi(const char *_st);
long          atol(const char *_st);
double        atof(const char *_st);

double        strtod (const char *_st, char **_endptr);
long          strtol (const char *_st, char **_endptr, int _base);
unsigned long strtoul(const char *_st, char **_endptr, int _base);

typedef struct _div_t  { int  quot, rem; } div_t;
typedef struct _ldiv_t { long quot, rem; } ldiv_t;
div_t         div(int _numer, int _denom);
ldiv_t        ldiv(long _numer, long _denom);


int           rand(void);
void          srand(unsigned _seed);

void         *calloc(size_t _num, size_t _size);
void          free(void *_ptr);
void         *malloc(size_t _size);
void          minit(void);
void         *realloc(void *_ptr, size_t _size);



void          abort(void);
void          exit(int _status);
int           atexit(void (*_func)(void));

int           abs(int _i);
long          labs(long _l);

void qsort(void *_base,
           size_t _nmemb, 
           size_t _size,
           int (*_compar)(const void *, const void *));

void *bsearch(const void *_key,
	      const void *_abase,
	      size_t     _nmemb,
	      size_t     _size,
	      int (*_compar)(const void *, const void *));

char *getenv(const char *_name);

int ti_sprintf(char *s, const char *format, ...);

#line 19 "c:\c2xxti\math.h"
         double asin(double _x);
         double acos(double _x);
         double atan(double _x);
         double atan2(double _y, double _x);
 double ceil(double _x);
         double cos(double _x);
         double cosh(double _x);
         double exp(double _x);
         double fabs(double _x);
 double floor(double _x);
 double fmod(double _x, double _y);
         double frexp(double _x, int *_exp);
         double ldexp(double _x, int _exp);
         double log(double _x);
         double log10(double _x);
         double modf(double _x, double *_iptr);
         double pow(double _x, double _y);
         double sin(double _x);
         double sinh(double _x);
         double sqrt(double _x);
         double tan(double _x);
         double tanh(double _x);

#line 23 "c:\c2xxti\string.h"
 void  *memchr  (const void *_s,        int   _c,  size_t _n);
 int    memcmp  (const void *_s1, const void *_s2, size_t _n);
 void  *memcpy  (      void *_s1, const void *_s2, size_t _n);
         void  *memmove (      void *_s1, const void *_s2, size_t _n);
 void  *memset  (      void *_s,        int   _c,  size_t _n);

 char  *strcat  (      char *_s1, const char *_s2);
 char  *strchr  (const char *_s,        int   _c);
 int    strcmp  (const char *_s1, const char *_s2);
         int    strcoll (const char *_s1, const char *_s2);
 char  *strcpy  (      char *_s1, const char *_s2);
         size_t strcspn (const char *_s1, const char *_s2);
         char  *strerror(      int   _errno);
 size_t strlen  (const char *_s1);
         char  *strncat (      char *_s1, const char *_s2, size_t _n);
         int    strncmp (const char *_s1, const char *_s2, size_t _n);
         char  *strncpy (      char *_s1, const char *_s2, size_t _n);
         char  *strpbrk (const char *_s1, const char *_s2);
 char  *strrchr (const char *_s1,       int   _c);
         size_t strspn  (const char *_s1, const char *_s2);
         char  *strstr  (const char *_s1, const char *_s2);
         char  *strtok  (      char *_s1, const char *_s2);
         size_t strxfrm (      char *_s1, const char *_s2, size_t _n);

#line 16 "c:\c2xxti\ctype.h"
extern unsigned char _ctypes_[];

#line 27
 int isalnum(int _c);
 int isalpha(int _c);
 int iscntrl(int _c);
 int isdigit(int _c);
 int isgraph(int _c);
 int islower(int _c);
 int isprint(int _c);
 int ispunct(int _c);
 int isspace(int _c);
 int isupper(int _c);
 int isxdigit(int _c);
 int isascii(int _c);

 int toupper(int _c);
 int tolower(int _c);
 int toascii(int _c);

 
 
 
#line 2 "filt.h"
float in_cal_levels[]={
       1.0187500000e-001,
       1.2115047484e-001,
       1.4407300667e-001,
       1.7133264461e-001,
       2.0375000000e-001,
       2.4230094968e-001,
       2.8814601333e-001,
       3.4266528922e-001,
       4.0750000000e-001,
       4.8460189936e-001,
       5.7629202667e-001,
       6.8533057843e-001,
       8.1500000000e-001,
       9.6920379873e-001,
       1.1525840533e+000,
       1.3706611569e+000,
};
float out_cal[]={
       1.1999000000e+001,
       1.0089916087e+001,
       8.4845742675e+000,
       7.1346480865e+000,
       5.9995000000e+000,
       5.0449580433e+000,
       4.2422871337e+000,
       3.5673240432e+000,
       2.9997500000e+000,
       2.5224790217e+000,
       2.1211435669e+000,
       1.7836620216e+000,
       1.4998750000e+000,
       1.2612395108e+000,
       1.0605717834e+000,
       8.9183101081e-001,
       7.4993750000e-001,
       6.3061975541e-001,
       5.3028589172e-001,
       4.4591550540e-001,
       3.7496875000e-001,
       3.1530987771e-001,
       2.6514294586e-001,
       2.2295775270e-001,
       1.8748437500e-001,
       1.5765493885e-001,
       1.3257147293e-001,
       1.1147887635e-001,
       9.3742187500e-002,
       7.8827469427e-002,
       6.6285736465e-002,
       5.5739438175e-002,
};
#line 89 "FILT.C"
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
 
	""
	};
 
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

 
#line 171
struct pstruct {
  char *text;
  unsigned int flag;
  unsigned int label;
  };

struct pstruct param_struct[]={
 	{" FUNC:",			0, (unsigned int)func_text},
 
  
 	{"Levels-In  Out  ",0, (unsigned int)null_text},
 	{"RevertToLevels:",	0, (unsigned int)revertolevels_text},
 	{"FullScalIn:#_Vpp",0, 0},
 	{"SampleRate:",		0, (unsigned int)samplerate_text},
 	{"InputSrc:",		0, (unsigned int)inputsrc_text},
 	{"Mode:",			0, (unsigned int)mode_text},
 	{"Cascade Ch A&B:",	0, (unsigned int)cascade_ch_a_b_text},
 	{"Master Mode:",	0, (unsigned int)master_mode_text},
 
 	{"Initialize:",		0x4000,  (unsigned int)initialize_text},
 	{"Store:  # press ",0x4000, 0},
 	{"Recall: # press ",0x4000, 0},
 	{"Firmware:  V",	0, (unsigned int)null_text},
 	{"Serial No:",		0, (unsigned int)null_text},

  
 	{" NFgain:###_.##x",0, 0},

 	{" APgain:###_.##x",0, 0},

 	{" LPfcut: #####Hz",0, 0},
 	{" LPorder:    ###",0, 0},
 	{" LPgain:###_.##x",0, 0},

 	{" HPfcut: #####Hz",0, 0},
 	{" HPorder:    ###",0, 0},
 	{" HPgain:###_.##x",0, 0},

 	{" BPf1:   #####Hz",0, 0},
 	{" BPf2:   #####Hz",0, 0},
 	{" BPfcntr:#####Hz",0, 0},
 	{" BPfwdth:#####Hz",0, 0},
 	{" BPorder:    ###",0, 0},
 	{" BPgain:###_.##x",0, 0},

 	{" BSf1:   #####Hz",0, 0},
 	{" BSf2:   #####Hz",0, 0},
 	{" BSfcntr:#####Hz",0, 0},
 	{" BSfwdth:#####Hz",0, 0},
 	{" BSorder:    ###",0, 0},
 	{" BSgain:###_.##x",0, 0},

 	{" Nfnotch:#####Hz",0, 0},
 	{" Nfwidth:#####Hz",0, 0},
 	{" Ngain: ###_.##x",0, 0},

 	{" INfcntr:#####Hz",0, 0},
 	{" INfwdth:#####Hz",0, 0},
 	{" INgain:###_.##x",0, 0},

 	{" UForder:    ###",0, 0},
 	{" UFtap:##_      ",0, 0},	 
 	{" UFgain:###_.##x",0, 0},

 	{" Sfreq:#####.#Hz",0, 0},
 	{" Sphase:###.#deg",0, 0},
 	{" Samp:   ###_.##",0, 0},

			};
 

 
int param_ptr_start[]={14, 15, 16, 19, 22, 28, 34, 37, 40, 43};
int param_ptr_end[]=  {14, 15, 18, 21, 27, 33, 36, 39, 42, 45};	 

#line 254
long params[(sizeof param_struct)/(sizeof param_struct[0]) + 128][3];


 
#line 280
unsigned int port0_copy;
unsigned flash_data[0x2000];	 
int sw_down, sw_down_old, sw_pressed, sw_released;  
int gray_code, gray_code_old, cw, ccw, testcount;
int flash_locked;	 
int cursor_pos, cursor_pos_vu, cursor_flag, flash_cursor_flag, down_turn_flag, press_flag;
int param_ptr, write_ptr, read_ptr, p_state, flag_options;
int index_ab;		     
#line 292
int params_changed; 	 
int params_changed_copy;
int led_counter, vu_counter;
int max_in_level;	 
long min_value, max_value;	 
long p_long, p_sum, serial_number;	 
unsigned p_uint1, p_uint2;
int sn_ok_flag, quietsn_flag;
int serial_error_flag;
unsigned data_count;
int sign_mult, index_ab_p;
long check_sum;
unsigned func_addr_temp_a, func_addr_temp_b;
float fsample;	 
char serial_str[11];	 
char serial_in_buf[128];	 
char parameter_str[17], value_str[17];	 
char vu_chars[]={' ', 8, 9, 10, 11, 12, 13, 14, 15, 'X', 'X', 'X', 'X'};	 
unsigned gopt_a, gopt_b, aopt_a, aopt_b;	 
unsigned in_a_vu_level, in_b_vu_level, out_a_vu_level, out_b_vu_level;
float scale_k_a, scale_k_b;
int auto_vu_count;
int iorder_old;
float window[128];		 


unsigned record[6 + (sizeof param_struct)/(sizeof param_struct[0]) + 128 + (sizeof param_struct)/(sizeof param_struct[0]) + 128 + (sizeof param_struct)/(sizeof param_struct[0]) + 128 + (sizeof param_struct)/(sizeof param_struct[0]) + 128 + (sizeof param_struct)/(sizeof param_struct[0]) + 128 + (sizeof param_struct)/(sizeof param_struct[0]) + 128];

 



volatile unsigned int *imr_ptr, *greg_ptr, *ifr_ptr;

 
extern int k7f00h, kf80fh, kfff0h;
extern int in_a, in_b, out_a, out_b, t_reg_scale_a, t_reg_scale_b, assembly_flag;
extern int in_a_hold, in_b_hold, out_a_hold, out_b_hold;
extern unsigned in_error, in_error_stick, in_digital, out_gain, out_atten, iosr_copy;
extern unsigned func_addr_a, func_addr_b, coef_ptr_a, coef_ptr_b, data_ptr_a, data_ptr_b;
extern unsigned orderm2_a, orderm2_b;
extern int fir_coef[256], coefdata[256];

 
ioport	unsigned	port0;		 
ioport	unsigned	portffe8;	 
ioport	unsigned	portffec;	 
ioport	unsigned	portfff0;	 
ioport	unsigned	portfff1;	 
ioport	unsigned	portfff4;	 
ioport	unsigned	portfff5;	 
ioport	unsigned	portfff6;	 
ioport	unsigned	portfff7;	 
ioport	unsigned	portfff8;	 
ioport	unsigned	portfff9;	 
ioport	unsigned	portfffa;	 
ioport	unsigned	portfffc;	 

 
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
 
#line 431
void main(void) 
{
unsigned count_start;
unsigned ptr;
int itemp;
float ftemp;

 
#line 443
initialize(); 	 

 


#line 453
ptr = 0x6000;	 
read_flash(ptr, 5, record);		 
itemp = 0;
if(record[0]==0xffff){
  store_all();	 
}
while(record[0]!=0xffff){
   
  if(record_bad()){
    store_all();	 
    break;
  }
  ptr = record[0];				 
  read_flash(ptr, 5, record);	 
}


 
params[11][0] = 0;	 
params_changed_copy = 3;
recall();			 


#line 490
sign_on();	 


#line 537
while(1){	 
			 
  if(cursor_flag&&(!sw_down)){
 
    disp_text("", cursor_pos, -1);	 
 
    if(auto_vu_count){	
      if((auto_vu_count++)>10){
        cursor_pos_vu = cursor_pos;	                         
 
        disp_text(param_struct[1].text, 1, 0);	 
        disp_text("", cursor_pos_vu, 0);	 
 
        assembly_flag |= 3;		 
        auto_vu_count = (int)params[2][0];	 
      }
    }
  }
  else if(press_flag==0){	 
 
    disp_text("", cursor_pos, flash_cursor_flag);	 
 
  }
  
  for(itemp=0;itemp<50;itemp++){		 
    count_start = portfffa;		 
    while(delta_t(count_start)<5000){	 

      if(write_ptr!=read_ptr){	 
        parse_command();		 
        count_start = portfffa;	 
      }

      if(serial_error_flag==1){	 
        serial_error_flag = 0;	 
 
 
        disp_text("RS-232 Error!   ", 1, -1);
		disp_text("", 1, 0);	 
        beep(200, 400);
 
 
        reset_port();
      }



      if(params_changed){
        portfff5 &= ~0x0200; 	 
        params_changed_copy = params_changed;	 
        params_changed = 0;		 
        portfff5 |= 0x0200; 	 
        update_dsp(param_ptr, index_ab);	 
        count_start = portfffa;	 
      }



    }	 


    led_update(); 			 

    portfff5 &= ~0x0200; 	 
    if(assembly_flag&3){
      vu_update();			 
    }
    portfff5 |= 0x0200; 	 

    rand();			 





  }	 


   
   
  asm("	nop			");


}   

}  
 


 
#line 629
void initialize(void)
{
int i, j, nstart, n_offset, nlength, nfrac, temp;
char ctemp;
unsigned flag_temp, cursor_temp, utemp;
unsigned* uptr;                                                 


asm("	setc	INTM	; disable all interrupts	");
asm("	setc	SXM		; set sign extension bit (sign-extension on)");
asm("	clrc	CNF		; map internal memory block B0 into Data space");
asm("	clrc	OVM		; clear overflow mode bit => normal overflow (assumed by c)");
asm("	spm		0		; set product shift mode to No Shift (assumed by c)");

 
sw_down=0, sw_pressed=0, sw_released=0;  
cw=0, ccw=0, testcount=0;
down_turn_flag=0, press_flag=0;
param_ptr=0, write_ptr=0, read_ptr=0, p_state=0, flag_options=0;
serial_error_flag = 0;	 
led_counter=0, vu_counter=0;
in_a_vu_level=0, in_b_vu_level=0, out_a_vu_level=0, out_b_vu_level=0;
quietsn_flag = 0;
iorder_old=0;
flash_locked=1;	 
flash_cursor_flag = 1;	 
params_changed=2;	 
max_in_level=20;	 
min_value=(-2147483647-1), max_value=2147483647;	 

init_params();		 
set_fsample();		 
init_freq_params();


 
imr_ptr = (volatile unsigned int *)4;	 
greg_ptr = (volatile unsigned int *)5;	 
ifr_ptr = (volatile unsigned int *)6;	 

*imr_ptr = 0;		 
*ifr_ptr = 0xffff;	 
*greg_ptr = 0x0080;	  
portffe8 = 1;		 
portffec = 0x01c;	 
 
portfffc = (1*0x0200 + 0*0x0040 + 0*0x0008 + 0);

 
k7f00h = 0x7f00;
kf80fh = 0xf80f;
kfff0h = 0xfff0;
 
out_gain = 0*0x0400 + 0*0x0010 + 0;
 
out_atten = 0*0x0200 + 0*0x0010;
 
in_a_hold = in_b_hold = out_a_hold = out_b_hold = 0;
assembly_flag = 0;

 
portfff9 = 0xffff;		 
portfff8 = 0x03fc;		 
portfff8 = 0x03cc;		 
						 

func_addr_a = (unsigned)&no_func_a;   
func_addr_b = (unsigned)&no_func_b;   

coef_ptr_a = 0x0380;	 
coef_ptr_b = 0x0300;	 

 
 


 
get_serial(&serial_str);
 
 
 
serial_number = 0;
for(i=0;i<6;i++){
  serial_number = 10*serial_number + (serial_str[i]&0x0f);
}

 
port0_copy = 0x0080;	 
port0 = port0_copy;
reset_lcd();

 

reset_port();

 
*ifr_ptr = 0xffff;	 
portfff1 = 0x4300;	 
portfff1 = 0x4330;	 
*imr_ptr = 0x0008|0x0020;	 

auto_vu_count = (int)params[2][0];	 
index_ab = params[6][0] ? 0:2;	    





for(i=0;i<(sizeof param_struct)/(sizeof param_struct[0]);i++){
 
  nstart = 15;
  nfrac = 0;
  n_offset = 0;
  for(j=0;j<=16;j++){
    ctemp = *(param_struct[i].text + j);
    if(nstart==15){
      if((ctemp=='#')||(ctemp=='_')){
        nstart = j;
        param_struct[i].flag &= 0x7fff;	 
      }
      else if(ctemp=='\0'){		 
        nstart = j;
        param_struct[i].flag |= 0x8000;	 
        break;
      }
    }
    else{	 
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
    nfrac = nstart + nlength - nfrac - 1;	 
  }
  if(0x8000&param_struct[i].flag){	 
    j = -1;
    while(*(char*)(*(char*)(param_struct[i].label + (++j)))!='\0'){}  
    param_struct[i].flag = (0xc000&param_struct[i].flag)|(nstart<<10)|j;  
  }
  else{								 
    param_struct[i].flag = (0xc000&param_struct[i].flag)|(nstart<<10)|(n_offset<<7)|(nlength<<3)|nfrac;  
  }
}	 


 
 
#line 794
set_all_gains();	 



asm("	clrc	INTM		; Enable interrupts");
wait_n_samples(10);	 
in_error_stick = 0;	 
gray_code = 0x0003&portfff6;	 

srand(in_a + serial_number%32768);	 



update_disp_left();		 
update_disp_right(1);

}  


 
#line 817
void init_params(void)
{
int j;
unsigned* uptr;

 
uptr = (unsigned*)(&params[0][0]);
for(j=0;j<(sizeof param_struct)/(sizeof param_struct[0]) + 128 + (sizeof param_struct)/(sizeof param_struct[0]) + 128 + (sizeof param_struct)/(sizeof param_struct[0]) + 128 + (sizeof param_struct)/(sizeof param_struct[0]) + 128 + (sizeof param_struct)/(sizeof param_struct[0]) + 128 + (sizeof param_struct)/(sizeof param_struct[0]) + 128;j++){
  *uptr++ = 0;
}

params[0][0] = params[0][1] = params[0][2] = 1;	 
params[3][0] = 20;	 
params[4][0] = 1;	 

params[15][0] = params[18][0] = params[21][0] = params[27][0] = params[33][0] = params[36][0] = params[39][0] = params[42][0] = params[45][0] =
params[15][1] = params[18][1] = params[21][1] = params[27][1] = params[33][1] = params[36][1] = params[39][1] = params[42][1] = params[45][1] =
params[15][2] = params[18][2] = params[21][2] = params[27][2] = params[33][2] = params[36][2] = params[39][2] = params[42][2] = params[45][2] = 100;  

params[17][0] = params[20][0] = params[26][0] = params[32][0] = params[40][0] = 
params[17][1] = params[20][1] = params[26][1] = params[32][1] = params[40][1] = 
params[17][2] = params[20][2] = params[26][2] = params[32][2] = params[40][2] = 127;  

params[41][0] = params[41][1] = params[41][2] = 1;	 
}


 
#line 849
void init_freq_params(void)
{

params[16][0] = params[19][0] = params[22][0] = params[25][0] = params[28][0] = params[31][0] =
params[16][1] = params[19][1] = params[22][1] = params[25][1] = params[28][1] = params[31][1] =
params[16][2] = params[19][2] = params[22][2] = params[25][2] = params[28][2] = params[31][2] = 

params[34][0] = params[37][0] =
params[34][1] = params[37][1] =
params[34][2] = params[37][2] = 1000;  

params[23][0] = params[29][0] =
params[23][1] = params[29][1] =
params[23][2] = params[29][2] = 2000;	 

params[24][0] = params[30][0] =
params[24][1] = params[30][1] =
params[24][2] = params[30][2] = 1500;	 

params[35][0] = params[38][0] =
params[35][1] = params[38][1] =
params[35][2] = params[38][2] = 1000;  

params[43][0] =
params[43][1] =
params[43][2] = 10000; 	 

}


 
#line 884
void set_fsample(void)
{
long ltemp;

 
if(params[4][0]){	 
  asm(" 	clrc	XF		; Set the CODEC sampling rate to 48 Ksps");
  fsample = 48000.0;		 
}   
else{
  asm("		setc	XF		; Set the CODEC sampling rate to 8 Ksps");
  fsample = 8000.0;			 
}


 
 
}


 
#line 910
void reset_port(void)
{
portfff5 = 0;		 
portfff6 = 0x66f0;	 
portfff5 = 0x2000;
portfff5 = 0;

portfff5 = (0x2000|0x0200|0x0080|0x0020|0x0008);



portfff6 = 0x66f0;	 
}


 
#line 930
void sign_on(void)
{




disp_text("Versa-Filter2-20", 1, -1);
wait(1000000);	 
#line 956
disp_text("Testing         ", 1, 0);
wait(100000);	 
disp_text(".", 9, 0);
wait(100000);	 
disp_text(".", 11, 0);
wait(100000);	 
disp_text(".", 13, 0);
wait(100000);	 
disp_text("OK", 15, 0);
wait(400000);	 



update_disp_left();		 
update_disp_right(1);


#line 987
}


 
#line 998
void parse_command(void)
{
int i, j, itemp, iflag, cursortemp, execute_flag=0, program_flag=0, prog_err_flag=0;
int param_ptr_temp, index_ab_temp, flag_options_temp, change_value_flag;
long param_value;
char *value_ptr, *cptr;
char ctemp, craw;
char carray[17];
unsigned utemp;

 
cursortemp = cursor_pos;	 

while(write_ptr!=read_ptr){
  craw = serial_in_buf[read_ptr++];	 
  ctemp = tolower(craw);			 
  read_ptr &= 128-1;	 
 
 

  switch(p_state){
  case 0:	 
    if(ctemp=='a') p_state = 1;
    sn_ok_flag = 0;
    break;
  case 1:	 
    if(ctemp=='t') p_state = 2;
    else if(ctemp=='a') p_state = 1;
    else p_state = 0;
    break;
  case 2:	 
    if(ctemp=='a') p_state = 3;
    else if(ctemp=='s') p_state = 5;
    else if(ctemp==' ') p_state = 2;
    else p_state = 0;
    break;
  case 3:	 
    if(ctemp=='t') p_state = 2;
    else if(ctemp=='l') p_state = 4;
    else p_state = 0;
    break;
  case 4:	 
    if(ctemp=='l'){
        sn_ok_flag = 1;
        p_state = 20;
      }
    else p_state = 0;
    break;
  case 5:	 
    if(ctemp=='n') p_state = 6;
    else p_state = 0;
    break;
  case 6:	 
    if(ctemp==':') p_state = 7;
    else p_state = 0;
    break;
  case 7:	 
    if(isdigit(ctemp)){
      p_state = 8;
      p_long = (long)(ctemp&0x0f);	 
    }
    else if(ctemp==' ') p_state = 7;
    else p_state = 0;
    break;
  case 8:	 
    if(isdigit(ctemp)){
      p_long = 10*p_long + (long)(ctemp&0x0f);	 
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
       
      if((read_ptr--)==0){
        read_ptr = 128-1;
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

  case 20:	 
    if(isalpha(ctemp)){	 
      for(i=0;i<17;i++){
        parameter_str[i] = value_str[i] = '\0';	 
      }
      parameter_str[0] = ctemp;	 
      p_state = 21;
    }
    else if(ctemp=='\r') p_state = 0;	 
    else p_state = 20;
    break;
  case 21:	 
    if(ctemp=='\r'){	 
      execute_flag = 1;
      p_state = 0;
    }
    else if(ctemp==':'){	 
      p_state = 22;
      if(strncmp(parameter_str, "program",7)==0){;	 
        func_addr_a = (unsigned)&no_func_a;   
        func_addr_b = (unsigned)&no_func_b;   
        assembly_flag &= ~3;	 
         
        p_state = 40;
      }
    }
    else{
      i =  strlen(parameter_str);
      if(i<16){
        parameter_str[i] = ctemp;	 
      }
      else{
        p_state = 0;
      }    
    }
    break;
  case 22:	 

    if(ctemp=='\r'){	 
      execute_flag = 1;
      p_state = 0;
    }
    else if(ctemp==':'){	 
      func_addr_temp_a = func_addr_a;	 
      func_addr_temp_b = func_addr_b;	 
      func_addr_a = (unsigned)&no_func_a;	 
      func_addr_b = (unsigned)&no_func_b;
      data_count = 0;	 
      p_state = 30;
    }
    else{
      i =  strlen(value_str);
      if(i<16){
        value_str[i] = craw;	 
      }
      else{
        p_state = 0;
      }
    }
    break;

   
  case 30:	 
    p_state = 31;
    if(ctemp==' ') p_state = 30;
    else if(ctemp=='-'){
      sign_mult = -1;
      p_long = 0;
    }
    else if(isdigit(ctemp)){
      sign_mult = +1;
      p_long = (long)(ctemp&0x0f);	 
    }
    else if(ctemp=='\r') goto p_cont1;
    else p_state = 0;
    switch(parameter_str[0]){	 
    case 'a':	 
      index_ab_p = 0;
      break;
    case 'b':	 
      index_ab_p = 1;
      break;
    case 'f':	 
      index_ab_p = 2;
      break;
    default:
      p_state = 0;
      break;
    }	 
    break;
  case 31:	 
    if(isdigit(ctemp)){
      p_long = 10*p_long + (long)(ctemp&0x0f);	 
    }
    else if((ctemp==' ')||(ctemp=='\r')){ 	 
      p_state = 30;
      p_long = sign_mult*p_long;		 
      if(p_long>32767){					 
        p_long = 32767;
      }
      if(p_long<-32768){
        p_long = -32768;
      }
      utemp = (sizeof param_struct)/(sizeof param_struct[0]) + (data_count>>1);	 
      if((data_count++)&0x0001){		 
        params[utemp][index_ab_p] |= (p_long<<16);	 
      }
      else{	 
        params[utemp][index_ab_p] = p_long&0x0000ffff;		 
      }
      if(params[6][0]==2){	 
        itemp = 256;		 
      }
      else{
        itemp = 128;  
      }
      if(data_count>itemp){
        data_count = itemp;
        goto p_cont1;	 
      }
    }
    else p_state = 0;
    if(ctemp=='\r'){	 
p_cont1:
      if(data_count<3) data_count = 3;		 
      params[40][index_ab_p] = data_count;	 
      params[41][index_ab_p] = 1L;	 
      func_addr_a = func_addr_temp_a;  
      func_addr_b = func_addr_temp_b;  
      execute_flag = 1;
      p_state = 0;
    }
    break;

   
  case 40:	 
    if(ctemp==' ') p_state = 40;
    else if(ctemp=='s') p_state = 41;
    else p_state = 0;
    break;
  case 41:	 
    if(ctemp==' ') p_state = 41;
    else if(ctemp==':') p_state = 42;
    else p_state = 0;
    break;
  case 42:	 
    if(isdigit(ctemp)){
      p_uint1 = (unsigned)(ctemp&0x0f);	 
      p_state = 43;
    }
    else if(ctemp==' ') p_state = 42;
    else p_state = 0;
    break;
  case 43:	 
    if(isdigit(ctemp)){
      p_uint1 = 10*p_uint1 + (unsigned)(ctemp&0x0f);	 
      p_state = 43;
    }
    else if(ctemp==' ') p_state = 43;
    else if(ctemp=='l') p_state = 44;
    else p_state = 0;
    break;
  case 44:	 
    if(ctemp==' ') p_state = 44;
    else if(ctemp==':') p_state = 45;
    else p_state = 0;
    break;
  case 45:	 
    if(isdigit(ctemp)){
      p_uint2 = (unsigned)(ctemp&0x0f);	 
      p_state = 46;
    }
    else if(ctemp==' ') p_state = 45;
    else p_state = 0;
    break;
  case 46:	 
    if(isdigit(ctemp)){
      p_uint2 = 10*p_uint2 + (unsigned)(ctemp&0x0f);	 
      p_state = 46;
    }
    else if(ctemp==' ') p_state = 46;
    else if(ctemp=='d') p_state = 47;
    else p_state = 0;
    break;
  case 47:	 
    if(ctemp==' ') p_state = 47;
    else if(ctemp==':'){
       
       
      if(((p_uint1==0x0000)||(p_uint1==0x2000)||(p_uint1==0x4000)||(p_uint1==0x6000)||
      (p_uint1==0x8000)||(p_uint1==0xa000)||(p_uint1==0xc000)||(p_uint1==0xe000))&&(p_uint2<=0x2000)&&
      (p_uint2>0)){
        if(sn_ok_flag){		 
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
  case 48:	 
    if(data_count<(p_uint2<<1)){
      utemp = data_count>>1;	 
      if(data_count&0x0001){	 
        flash_data[utemp] = flash_data[utemp] | craw;	 
        if(sn_ok_flag){		 
          if(((utemp+1)&0x002f) == 0){
            disp_num((long)(utemp) + 1, 6, 5 ,0);	 
          }
        }
      }
      else{	 
        flash_data[utemp] = (craw<<8);	 
      }
      check_sum += craw;				 
      data_count++;
    }
    if(data_count==(p_uint2<<1)){
      if(sn_ok_flag){
        disp_num((long)((data_count-1)>>1) + 1, 6, 5 ,0);	 
        wait(1000000);	 
      }
      data_count = 0;
      p_sum = 0;
      p_state = 49;
    }
    break;
  case 49:	 
    if(data_count<8){	 
      if(isdigit(ctemp)){
        p_sum = 10*p_sum + (long)(ctemp&0x0f);	 
      }
      else if(ctemp==' ') p_state = 49;
      else{
        prog_err_flag = 1;
        p_state = 0;	 
      }
    }
    if((data_count++) == 7){	 
      if(p_sum==check_sum){	 
        program_flag = 1;	 
      }
      else{	 
        prog_err_flag = 1;
      }
      p_state = 0;
    }
    break;

  default:
  	break;
  }	 


 

 
 
#line 1361
  if(sn_ok_flag && execute_flag){	   
    if(strncmp(parameter_str, "reset", 5)==0){
      initialize();  
       
    }
    else if(strncmp(parameter_str, "display", 7)==0){
      if(value_str[0]!='\0'){	 
        disp_text("                ", 1, -1);
        disp_text(value_str, 1, -1);
        flash_cursor_flag = 0;	 
 
 
      }
      else{						 
        update_disp_left();		 
        update_disp_right(1);
        params_changed = 1;		 
        flash_cursor_flag = 1;	 
      }
    }
    else if(!quietsn_flag && strncmp(parameter_str, "sendsn", 6)==0){
      portfff5 &= ~0x0080;	 


      wait(22*(long)rand());	 
      xmit(serial_str);		 
    }
    else if(strncmp(parameter_str, "quietsn", 7)==0){
      quietsn_flag = 1;		 
    }
    else if(strncmp(parameter_str, "echo", 4)==0){
      disp_text("                ", 1, -1);
      disp_text(value_str, 1, -1);
      flash_cursor_flag = 0;	 
      xmit(value_str);			 
    }

    else{	 
       
      param_ptr_temp = param_struct_search(&itemp);	 
      if(!itemp){	 
        goto parse_continue;	 
      }
      flag_options_temp = 0;
      if(itemp==3){ 	 	 
        index_ab_temp = 2;	 
      }
      else if(itemp==1){ 	 
        index_ab_temp = 0;	 
        flag_options_temp = 1;
      }
      else if(itemp==2){ 	 
        if(parameter_str[0]=='a'){
          index_ab_temp = 0;
        }
        else if(parameter_str[0]=='b'){
          index_ab_temp = 1;
        }
        else{
          goto parse_continue;	 
        }
      }
      
       

      if(!flag_options_temp){	 
        itemp = (int)params[0][index_ab_temp];	 
         
        if(   (params[6][0]==0 && index_ab_temp!=2)		 
           || (params[6][0]!=0 && index_ab_temp==2)	){	 
           goto parse_continue;	 
        }     
         
        if(param_ptr_temp&&(param_ptr_temp<param_ptr_start[itemp] || param_ptr_temp>param_ptr_end[itemp])){
           goto parse_continue;	 
        }     
      }
       
      value_ptr = &value_str[0];	                                                    
      while(*value_ptr==' ') value_ptr++;	 
      param_value = 0;
      change_value_flag = (*value_ptr!='\0');	 
      utemp = param_struct[param_ptr_temp].flag;	   
      if(utemp>>15){	 
        itemp = utemp&0xff;	 
 
 
         
        for(i=0;i<itemp;i++){	 
          param_value = -1;
          cptr = (char*)(*(char*)(param_struct[param_ptr_temp].label + i));	 
          while(*cptr==' ') cptr++;	 
          if(string_compare(cptr, value_ptr)){	 
            param_value = (long)i;	 
            break;
          }
        }
        if(param_value<0){	 
          goto parse_continue;	 
        }
      }
      else{		 
         
        if(iflag = (*value_ptr=='-')) value_ptr++;	 
        											 
        itemp = (int)(0x0007&utemp);  
        i = 0;
        do{		 
          ctemp = *value_ptr++;	 
          j = isdigit(ctemp);
          if(i==0 && !j){	 
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
        
        if(iflag){ 						 
          param_value = -param_value;	 
        }

         
        params_changed_copy = 1;			 
        update_dsp(param_ptr_temp, index_ab);	 
        if(param_value>max_value){			 
          param_value = max_value;
        }
        if(param_value<min_value){
          param_value = min_value;
        }
      }
      
      flag_options = flag_options_temp;	 
      param_ptr = param_ptr_temp;		 
      index_ab = index_ab_temp;			 

      
      if(change_value_flag){	 
        params[param_ptr][index_ab] = param_value;	 
        params_changed = 3;		 
      }
      else{	 
        param_value = params[param_ptr][index_ab];	 
        if(utemp>>15){	 
          if(param_ptr_temp==1){		 
             



          }
          else if(param_ptr_temp==12){	 
            xmit(num2string(220, 2, &i, (char*)&carray));
          }
          else if(param_ptr_temp==13){	 
            xmit(num2string(serial_number, 0, &i, (char*)&carray));
          }
          else{		 
             
            xmit((char*)(*(char*)(param_struct[param_ptr_temp].label + param_value)));
          }
        }
        else{		 
          itemp = (int)(0x0007&utemp);  
           
          xmit(num2string(param_value, itemp, &i, (char*)&carray));
        }
        params_changed = 1;		 
      }
      update_disp_left();			 
      update_disp_right(1);
      flash_cursor_flag = 1;	 
 
 
    }	 
    
    
parse_continue:
    execute_flag = 0;
  }






  if(sn_ok_flag && prog_err_flag){
    disp_text("PARAMETER ERROR ", 1, -1);
    wait(1000000);	 
    disp_text("WRITE ABORTED!  ", 1, -1);
    wait(2000000);	 
    update_disp_left();		 
    update_disp_right(1);
    params_changed = 2;		 
    prog_err_flag = 0;
 
 
  }

  if(sn_ok_flag && program_flag){	   
 
 
#line 1572
    if((p_uint1==0)||(p_uint1==32768)){	 
      j = 0;
      itemp = 0;
      for(i=0x0040+2;i<0x0040+7;i++){
        utemp = flash_data[i];
        carray[j++] = (char)((utemp&0xff00)>>8);	 
        carray[j++] = (char)(utemp&0x00ff);			 
        itemp += (utemp!=0x3030);					 
      }												 
      
      if((itemp==0)||(!valid_serial(carray))){	 
        j = 0;
        for(i=0x0040+2;i<0x0040+7;i++){
          itemp = (int)serial_str[j++]<<8;				 
          flash_data[i] = itemp|((int)serial_str[j++]);  
        }
      }
    }
    
 
 

    disp_text("Programming...  ", 1, -1);
    flash_locked = 0;	 
    itemp = prog_flash(p_uint1, p_uint2, flash_data, 1, "inUpdate");
    if(itemp==0){	 
      disp_text("Sector   Prog OK", 1, -1);
      disp_num(p_uint1/8192, 7, 2 ,0);	 
 
 

 
 

      program_flag = 0;
    }
  }

}	 

 

if(cursor_pos!=cursortemp){ 	 
  disp_text("", 1, 0);			 
}

 

}


 
#line 1633
void txrxint_c(void)
{
int i, j, temp_pos, type, nstart, n_offset, nlength, nfrac, nlabel;
int io;
int sum2, io0_sum, io1_sum, io2_sum;
unsigned count_start;
long jj;
char carray[8];

 
if(iosr_copy&0x4000){	 
  portfff5 &= ~0x0020;	 
  portfff6 = 0x4000;	 
}

 
 
#line 1655
if(iosr_copy&0x0100){	 
  assembly_flag &= ~3;	 
#line 1661
  serial_in_buf[write_ptr++] = portfff4;	 
  
  write_ptr &= 128-1;	 

  auto_vu_count = (int)params[2][0];	 

    
  if(write_ptr==read_ptr || portfff6&0x2600){	 



    serial_error_flag = 1;	 
  }

  return;	 

}





if(iosr_copy&0x00f0){	 
   
  sum2=0, io0_sum=0, io1_sum=0, io2_sum=0;	 

  io = iosr_copy;
  count_start = portfffa;		 
  while(delta_t(count_start)<500){	 
   
    io0_sum += 0x0001&io;		 
    io1_sum += 0x0001&(io>>1);	 
    io2_sum += 0x0001&(io>>2);	 
    io = portfff6;				 
    sum2++;
  }
  sum2 = sum2>>1;	 

   
  sw_down_old = sw_down;			 
  sw_down = io2_sum<sum2;			 
  if(sw_down&&(!sw_down_old)){ 	 
    sw_pressed = 1;
  }
  if((!sw_down)&&sw_down_old){
    sw_released = 1;
  }

#line 1729
  gray_code_old = gray_code; 
  gray_code = ((io1_sum>sum2)<<1)|(io0_sum>sum2);	 
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
#line 1746
  if(sw_pressed||cw||ccw){	 
    port0_copy |= 0x0040;	 
    port0 = port0_copy;
    auto_vu_count = (int)params[2][0];	 
  }

   
  
  flash_cursor_flag = 1;	 
  
  nstart = (0x000f&(param_struct[param_ptr].flag>>10)) + 1;	 
  type = param_struct[param_ptr].flag>>15;
  if(type){	 
    n_offset = 0;
    nlength = 1;
    nfrac = 0;
  }
  else{		 
    n_offset = (0x0007&(param_struct[param_ptr].flag>>7));	 
    nlength = 0x000f&(param_struct[param_ptr].flag>>3);
    nfrac = 0x0007&param_struct[param_ptr].flag;
  }
  
  if(sw_pressed&&(cursor_pos!=1)&&(0x4000&param_struct[param_ptr].flag)){  
    press_flag = 1;	 
    assembly_flag &= ~3;	 
    disp_text("Press to        ", 1, -1);	 
     
    for(i=0;i<7;i++){
      carray[i] = *(param_struct[param_ptr].text + i);
      if(carray[i]==':'){
        break;
      }
    }
    carray[i] = 0;	 
    disp_text(carray, 10, 0);	 
    disp_text("",1, 0);			 
  }
  else if(press_flag){	 
    if(sw_pressed){
      params_changed = 3;	 
      press_flag = 0;
    }
    else if(cw||ccw){
      disp_text("Cancelled       ", 1, 0);	 
      wait(750000);				 
      update_disp_left();		 
      update_disp_right(1);
      press_flag = 0;
       
      gray_code = 0x0003&portfff6;	 
      portfff6 = 0x00f0;			 
    }
  }
  else if(sw_pressed||(sw_down&&cw)){
     
    temp_pos = cursor_pos;
    if(temp_pos==1){
      temp_pos = nstart + n_offset;
    }
    else{
      ++temp_pos;
      if(temp_pos>=(nstart+nlength)){
        temp_pos = 1;
      }
      else if(nfrac&&(temp_pos==(nstart+nlength-nfrac-1))){	 
        ++temp_pos;
      }
    }
    disp_text("",temp_pos, +1);	 
  }	 
  else if(sw_down&&ccw){
     
    temp_pos = cursor_pos;
    if(temp_pos==1){
      temp_pos = nstart + nlength - 1;
    }
    else{
      --temp_pos;
      if(temp_pos<nstart){
        temp_pos = 1;
      }
      else if(nfrac&&(temp_pos==(nstart+nlength-nfrac-1))){	 
        --temp_pos;
      }
    }
    disp_text("", temp_pos, +1);	 
  }	 
  else if((cw||ccw)&&((assembly_flag&3)==3)){	 
    assembly_flag &= ~3;	 
     
    update_disp_left();		 
    update_disp_right(cursor_pos_vu);	 
    params_changed = 1;	 
  }
  else if((cw||ccw)&&(!down_turn_flag)){
    assembly_flag &= ~3;	 
     
    if(cursor_pos==1){	 
      i = inc_dec_param_ptr();	 
      update_disp_left();	 
      update_disp_right(1);	 
      params_changed = 1;	 
      if(i){   	 
        beep(30, 550);
        wait(200000);	 
         
        gray_code = 0x0003&portfff6;	 
        portfff6 = 0x00f0;				 
      }
    }
    else{		 
      if(type){	  
        nlabel = 0x00ff&param_struct[param_ptr].flag;		 
        i = (int)params[param_ptr][index_ab];	 
        inc_dec_mod(&i, nlabel);		 
        params[param_ptr][index_ab] = (long)i;
      }		 
      else{	 
        if(nfrac==0){	 
          i = nstart + nlength - cursor_pos - 1;
        }
        else{			 
          if(cursor_pos>=(nstart + nlength - nfrac)){	 
            i = nstart + nlength - cursor_pos - 1;
          }
          else{
            i = nstart + nlength - cursor_pos - 2;
          }
        }
        add_sub_pow(&params[param_ptr][index_ab], i);	 
      }	 
      update_disp_right(cursor_pos);	 
      params_changed = 2;				 
    }  
  }	 
  
  down_turn_flag = sw_down&&(cw||ccw);	 
  
   
  sw_pressed = 0;
  sw_released = 0;
  cw = 0;
  ccw = 0;

  port0_copy &= ~0x0040;	 
  port0 = port0_copy;

}	 






} 	 


 
#line 1907
void reset_lcd(void)
{
int i, j;

 
write_lcd_nibble(0x3, 4100);	 
write_lcd_nibble(0x3, 100);		 
write_lcd_nibble(0x3, 100);		 
write_lcd_nibble(0x2, 100);		 

 
write_lcd_inst(0x28, 40);		 
write_lcd_inst(0x0c, 40);		 
cursor_flag = 0;				 
flash_cursor_flag = 1;			 
write_lcd_inst(0x01, 1640);		 
write_lcd_inst(0x06, 40);		 
write_lcd_inst(0x02, 1640);		 

 
write_lcd_inst(0x40, 40);	 
for(i=7;i>=0;i--){
  for(j=1;j<9;j++){
    if(j<=i){
      write_lcd_data(0x00, 40);	 
    }
    else{
      write_lcd_data(0x0e, 40);	 
    }
  }
}

write_lcd_inst(0x80, 40);	 
cursor_pos = 1;		 

}

 
#line 1962
void disp_text(char *text, int position, int cursor_on)
{
unsigned port_temp;

port_temp = portfff5;	 
portfff5 &= ~0x0200; 	 






if((cursor_on>0)&&(!cursor_flag)){
  write_lcd_inst(0x0e, 40);		 
  cursor_flag = 1;				 
}
if((cursor_on<0)&&(cursor_flag)){
  write_lcd_inst(0x0c, 40);		 
  cursor_flag = 0;				 
}

if(position!=cursor_pos){	 
  if(position<9){
    write_lcd_inst((0x80 - 1) + position, 40);
    }
  else{
    write_lcd_inst((0xc0 - 9) + position, 40);
  }
  cursor_pos = position;
}
 
while(*text!='\0'){	 
  write_lcd_data(*text++, 40);
  cursor_pos++;
  if(cursor_pos==9){
    write_lcd_inst(0xc0, 40); 
  }
}
portfff5 = port_temp;	 
}


  
#line 2025
void disp_num(long num, int position, int width, int nfrac)
{

char tempc[17];
char *text;
int i, strlength;

text = num2string(num, nfrac, &strlength, (char*)&tempc);	 

 
if(strlength>width){
  text = &tempc[16];	 
  for(i=0;i<width;i++){
    *--text = '*';		 
  }
}
 
while(width>strlength++){
  *--text = ' ';
}      

 
disp_text(text, position, 0);

}


  
#line 2069
char* num2string(long num, int nfrac, int *strlength, char *str17)
{
char *bufptr;
int neg = num < 0;
long unum = neg ? -num : num;	 

bufptr = (str17+16);	 
*bufptr = 0;			 
*strlength = 0;			 
 
do{
  *--bufptr = (unum % 10) + '0';	 
  (*strlength)++;			 
  if(*strlength==nfrac){
    *--bufptr = '.';		 
    (*strlength)++;			 
    nfrac = 0;				 
  }
} while((unum/=10)||nfrac);

 
if(neg){
  *--bufptr = '-';
  (*strlength)++;			 
}

return bufptr;
}

 
#line 2109
void write_lcd_inst(int byte, int wait_usec)
{
int i, iend, nibh, nibl;

nibh = (byte&0xf0)>>4;	 
nibl = byte&0x0f;

port0_copy = port0_copy&(~0x10);		 
port0 = port0_copy;


port0_copy = (0xd0&port0_copy)|0x20|nibh;  
port0 = port0_copy;

port0_copy = (0xd0&port0_copy)|nibh;	 
port0 = port0_copy;


port0_copy = (0xd0&port0_copy)|0x20|nibl;  
port0 = port0_copy;

port0_copy = (0xd0&port0_copy)|nibl;	 
port0 = port0_copy;

wait((long)wait_usec);						 
}

 
#line 2147
void write_lcd_data(int byte, int wait_usec)
{
int i, iend, nibh, nibl;

nibh = (byte&0xf0)>>4;	 
nibl = byte&0x0f;

port0_copy = port0_copy|0x10;		 
port0 = port0_copy;


port0_copy = (0xd0&port0_copy)|0x20|nibh;  
port0 = port0_copy;

port0_copy = (0xd0&port0_copy)|nibh;	 
port0 = port0_copy;


port0_copy = (0xd0&port0_copy)|0x20|nibl;  
port0 = port0_copy;

port0_copy = (0xd0&port0_copy)|nibl;	 
port0 = port0_copy;

wait((long)wait_usec);						 
}

 
#line 2185
void write_lcd_nibble(int nibble, int wait_usec)
{
int i, iend;

port0_copy = port0_copy&(~0x10);		 
port0 = port0_copy;

port0_copy = (0xd0&port0_copy)|0x20|nibble;  
port0 = port0_copy;
port0_copy = (0xd0&port0_copy)|nibble;	 
port0 = port0_copy;

wait((long)wait_usec);						 
}



 
#line 2209
int delta_t(unsigned count_start)
{
int itemp;

itemp = (count_start>>1) - (portfffa>>1);		 
												 
												 
return ((itemp>=0) ? itemp:(itemp + 0x8000));	 

}


 
#line 2231
void wait(long wait_usec)
{
int i, wait_course, wait_fine;
unsigned count_start;

 
wait_course = (int) (wait_usec>>14);		 
wait_fine = (int) (wait_usec&0x00003fff);    

count_start = portfffa;		 
while(delta_t(count_start)<wait_fine);

if(wait_course==0) return;

 
for(i=0;i<wait_course;i++){
  count_start = portfffa;		 
  while(delta_t(count_start)<16384);
}
  

}

 
#line 2259
void wait_n_samples(int n)
{
wait(n*1000000/((long)fsample));	 

}


 
#line 2316
prog_flash(unsigned start, unsigned length, unsigned *datawords, int erase_flag, char *error_text)
{
unsigned i, j, status1, status2, word, byte, xbyte, sector_start, port_temp;
char carray[10];
long error_code=0;

if(flash_locked){	 
  error_code = 1;
  goto prog_error;
}

port_temp = portfff5;	 
portfff5 &= ~0x0200;	 
portfff5 |= 0x0004;		 

if(start<0x4000){		 
  portfff6 = (portfff6&0x000f)|0x0004;	 
  out_atten &= ~0x000c;	 
}
else if(start<0x8000){	 
  start -= 0x4000;
  portfff6 = (portfff6&0x000f)|0x0004;	 
  out_atten |= 0x000c;	 
}
else if(start<0xc000){	 
  start -= 0x8000;
  portfff6 &= 0x000b;	 
  out_atten &= ~0x000c;	 
}
else{					 
  start -= 0xc000;
  portfff6 &= 0x000b;	 
  out_atten |= 0x000c;	 
}

wait_n_samples(20);		 
                         
                        
if(((start<0x2000)&&((start+length)>0x2000))||((start>=0x2000)&&((start+length)>0x4000))){
  error_code = 1;	 
  goto prog_error;
}

*greg_ptr = 0x0080;	  
portfffc = (1*0x0200 + 4*0x0040 + 0*0x0008 + 0);	 
 

*((unsigned int *)(0x8000+0x5555)) = 0x00aa;	 
*((unsigned int *)(0x8000+0x2aaa)) = 0x0055;
*((unsigned int *)(0x8000+0x5555)) = 0x00f0;

if(out_atten&0x0008){	 
   
   

   
  j = 0;
  for(i=(2*0x0040)+4;i<(2*0x0040)+4+10;i++){
    carray[j++] = (*((unsigned int *)(0x8000+i)))&0x00ff;	 
  }
  if(valid_serial(carray)){	 
    error_code = 6;
    goto prog_error;
  }
}

if(erase_flag){	 
  if(start<0x2000){
    sector_start = 0x0000;
  }
  else{
    sector_start = 0x4000;
  }
  
   
  *((unsigned int *)(0x8000+0x5555)) = 0x00aa;	 
  *((unsigned int *)(0x8000+0x2aaa)) = 0x0055;
  *((unsigned int *)(0x8000+0x5555)) = 0x0080;
  *((unsigned int *)(0x8000+0x5555)) = 0x00aa;
  *((unsigned int *)(0x8000+0x2aaa)) = 0x0055;
  *((unsigned int *)(0x8000+sector_start)) = 0x0030;	 
  wait(100);	 
  do{
    status1 = *((unsigned int *)(0x8000+sector_start));	 
    status2 = *((unsigned int *)(0x8000+sector_start));
    if(((status1&0x0040)==(status2&0x0040))||(status1&0x0080)){	 
      goto pf_1;												 
    }
  }
  while(!(status1&0x00020));	 
  status1 = *((unsigned int *)(0x8000+sector_start));	 
  if(status1&0x0080){	 
    goto pf_1;
  }
  error_code = 2;	 
  goto prog_error;
 pf_1:
  *((unsigned int *)(0x8000+0x5555)) = 0x00aa;	 
  *((unsigned int *)(0x8000+0x2aaa)) = 0x0055;
  *((unsigned int *)(0x8000+0x5555)) = 0x00f0;
   
  for(i=sector_start;i<(0x4000+sector_start);i++){
    if(((*((unsigned int *)(0x8000+i)))&0x00ff)!=0x00ff){
      error_code = 3;	 
      goto prog_error;
    }
  }
}

 
j = 0;
start = (start<<1);				 
length = start + (length<<1);	 
for(i=start;i<length;i++){
  if(i&0x0001){	 
    byte = word&0x00ff;			 
  }
  else{		 
    word = datawords[j++];		 
    byte = word>>8;				 
  }
  *((unsigned int *)(0x8000+0x5555)) = 0x00aa;	 
  *((unsigned int *)(0x8000+0x2aaa)) = 0x0055;
  *((unsigned int *)(0x8000+0x5555)) = 0x00f0;
  xbyte = (*((unsigned int *)(0x8000+i))^byte)&0x00ff;	 
  if(xbyte){		 
    if(xbyte&byte){	 
      error_code = 4;	 
      goto prog_error;
    }
    *((unsigned int *)(0x8000+0x5555)) = 0x00aa;	 
    *((unsigned int *)(0x8000+0x2aaa)) = 0x0055;
    *((unsigned int *)(0x8000+0x5555)) = 0x00a0;
    *((unsigned int *)(0x8000+i)) = byte;	 
    do{	 
      status1 = *((unsigned int *)(0x8000+i));
      status2 = *((unsigned int *)(0x8000+i));
      if(((status1&0x0040)==(status2&0x0040))||((status1&0x0080)==(byte&0x0080))){	 
        goto pf_3;																	 
      }
    }
    while(!(status1&0x00020));	 
    status1 = *((unsigned int *)(0x8000+i));
    if((status1&0x0080)==(byte&0x0080)){		 
      goto pf_3;
    }
    error_code = 5;	 
    goto prog_error;
  }
  pf_3:
  continue;
}

prog_out:
portfffc = (1*0x0200 + 0*0x0040 + 0*0x0008 + 0);	 

portfff5 &= ~0x0004;	 
wait(400);				 
portfff6 = 0x00f0;		 
portfff5 = port_temp;	 
out_atten &= ~0x000c;	 
flash_locked=1;			 

if(error_code){
  return 1;				 
}
else{
  return 0;				 
}


prog_error:
*((unsigned int *)(0x8000+0x5555)) = 0x00aa;	 
*((unsigned int *)(0x8000+0x2aaa)) = 0x0055;
*((unsigned int *)(0x8000+0x5555)) = 0x00f0;

 
disp_text("ProgERR", 1, -1);
disp_num(error_code, 8, 1, 0);
disp_text(error_text, 9, -1);
 

wait(2000000);
goto prog_out;

}


 
#line 2519
void read_flash(unsigned start, unsigned length, unsigned *datawords)
{
unsigned i, j, port_temp;

port_temp = portfff5;	 
portfff5 &= ~0x0200;	 
portfff5 |= 0x0004;		 

if(start<0x4000){		 
  portfff6 = (portfff6&0x000f)|0x0004;	 
  out_atten &= ~0x000c;	 
}
else if(start<0x8000){	 
  start -= 0x4000;
  portfff6 = (portfff6&0x000f)|0x0004;	 
  out_atten |= 0x000c;	 
}
else if(start<0xc000){	 
  start -= 0x8000;
  portfff6 &= 0x000b;	 
  out_atten &= ~0x000c;	 
}
else{					 
  start -= 0xc000;
  portfff6 &= 0x000b;	 
  out_atten |= 0x000c;	 
}

wait_n_samples(20);		 
						 
                        
*greg_ptr = 0x0080;	  
portfffc = (1*0x0200 + 4*0x0040 + 0*0x0008 + 0);	 
 

*((unsigned int *)(0x8000+0x5555)) = 0x00aa;	 
*((unsigned int *)(0x8000+0x2aaa)) = 0x0055;
*((unsigned int *)(0x8000+0x5555)) = 0x00f0;

 
j = 0;
start = (start<<1);				 
length = start + (length<<1);	 
for(i=start;i<length;i+=2){
  datawords[j++] = ((*((unsigned int *)(0x8000+i)))<<8) | ((*((unsigned int *)(0x8001+i))&0x00ff));	 
}

portfffc = (1*0x0200 + 0*0x0040 + 0*0x0008 + 0);	 

portfff5 &= ~0x0004;	 
wait(400);				 
portfff6 = 0x00f0;		 
portfff5 = port_temp;	 
out_atten &= ~0x000c;	 

}


 
#line 2593
int valid_serial(char *carray)
{
int i, code, num[10], sumo=0, sume=0, sumss=0;

for(i=0;i<10;i++){
  code = carray[i];	 
  num[i] = code - 0x30;	 
  if((code<0x30)||(code>0x39)){
    return 0;	 
  }
}

sumo = num[0]+num[2]+num[4];
 
while(sumo>=10){
  sumo -= 10;
}

sume = num[1]+num[3]+num[5];
 
while(sume>=10){
  sume -= 10;
}

sumss = num[0]+num[1]+num[2]+num[3]+num[4]+num[5]+sumo+sume;

if((sumo!=num[6])||(sume!=num[7])||(sumss!=(10*num[8]+num[9]))){
  return 0;		 
}

return 1;		 
}


 
#line 2632
void update_disp_left(void)
{
#line 2638
int i, pos, pos2;
char ctemp, ctemp1[17], ctemp2[17];
char *c1ptr, *c2ptr;

c1ptr = &ctemp1[0];	 
c2ptr = &ctemp2[0];
  
pos = 0;
do{
  ctemp = (*(c1ptr++) = *(param_struct[param_ptr].text + (pos++)));	 
} while((ctemp!='#')&&(ctemp!='\0'));

if(ctemp=='\0'){
  disp_text(ctemp1, 1, -1);	 
}
else{	 
  *(c1ptr-1) = '\0';	 
  pos2 = 0;
  do{
    ctemp = *(param_struct[param_ptr].text + (pos++));	 
    if((ctemp!='#')&&(ctemp!='_')&&(ctemp!='.')&&(ctemp!='\0')&&(pos2==0)){
      pos2 = pos;
    }
    if(pos2){
      *(c2ptr++) = ctemp;	 
    }
  } while(ctemp!='\0');
  
  if(pos2!=0){
    disp_text(ctemp2, pos2, -1);	 
  }
  disp_text(ctemp1, 1, -1);			 
}

if(flag_options==0){	 
  if(index_ab==0){
    disp_text("a", 1, 0);	 
  }
  else if(index_ab==1){
    disp_text("b", 1, 0);	 
  }
  else if(index_ab==2){
    disp_text(" ", 1, 0);	 
  }
}


}

 
#line 2693
void update_disp_right(int pos)
{



int type, nstart, nlength, nfrac;

type = param_struct[param_ptr].flag>>15;
nstart = (0x000f&(param_struct[param_ptr].flag>>10)) + 1;

if(type){	 
  disp_text((char*)(*(char*)(param_struct[param_ptr].label + params[param_ptr][index_ab])), nstart, 0);
}
else{		 
  nlength = 0x000f&(param_struct[param_ptr].flag>>3);
  nfrac = 0x0007&param_struct[param_ptr].flag;
  disp_num(params[param_ptr][index_ab], nstart, nlength, nfrac);
}

disp_text("", pos , +1);	 


}


 
#line 2722
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



















 
#line 2765
void store_all(void)
{

int itemp;

record[0] = 0xffff;
flash_locked = 0;	 
itemp = prog_flash(0x6000, 1, record, 1, "in Erase");	 
if(itemp==0){	 
  for(itemp=4;itemp>=0;itemp--){
    params[10][0] = itemp;	 
    params_changed_copy = 3;
    store();				 
  }  
}
}

 
#line 2788
void xmit(char *text)
{

portfff5 &= ~0x0080;	 





while(*text!='\0'){
  while(!(portfff6&0x1000)); 
  portfff4 = *text++;		 
}
portfff4 = '\r';	 
while(!(portfff6&0x1000)); 

portfff6 = 0x6700;		 
portfff5 |= 0x0080;		 

}


 
#line 2821
int param_struct_search(int *match_type)
{
int i;
char *cptr;

for(i=0;i<(sizeof param_struct)/(sizeof param_struct[0]);i++){			 

  cptr = param_struct[i].text;	 

  if(string_compare(cptr, &parameter_str[0])){	 
    *match_type = 1;
    return(i);
  }
  else if(*cptr++==' '){		 
    if(string_compare(cptr, &parameter_str[0])){	 
      *match_type = 3;
      return(i);
    }
    if(string_compare(cptr, &parameter_str[1])){	 
      *match_type = 2;
      return(i);
    }
  }
}

*match_type = 0;	 
return(0);
}

 
#line 2856
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


 
#line 2913
int inc_dec_param_ptr(void)
{
int func_a, func_b, func_c;

func_a = (int)params[0][0];	 
func_b = (int)params[0][1];	 
func_c = (int)params[0][2];	 

switch((params[6][0]<<2)+(cw<<1)+flag_options){	 
case 0:		 
  if(param_ptr==0){
    param_ptr = 13;
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

case 1:		 
  if(param_ptr==1){
    param_ptr = param_ptr_end[func_c];
    index_ab = 2;
    flag_options = 0;
  }
  else{
    param_ptr--;
    if(param_ptr==1){
      return 1;
    }
  }
  break;

case 2:		 
  if(param_ptr==0){
    param_ptr = param_ptr_start[func_c];
  }
  else if(param_ptr==param_ptr_end[func_c]){
    param_ptr = 1;
    index_ab = 0;
    flag_options = 1;
    return 1;
  }
  else{
    param_ptr++;
  }
  break;

case 3:		 
  if(param_ptr==13){
    param_ptr = 0;
    index_ab = 2;
    flag_options = 0;
    return 1;
  }
  else{
    param_ptr++;
  }
  break;

case 4:		 
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
  else{	 
    if(param_ptr==0){
      param_ptr = 13;
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

case 5:		 
  if(param_ptr==1){
    param_ptr = param_ptr_end[func_b];
    index_ab = 1;
    flag_options = 0;
  }
  else{
    param_ptr--;
    if(param_ptr==1){
      return 1;
    }
  }
  break;

case 6:		 
  if(index_ab){
    if(param_ptr==0){
      param_ptr = param_ptr_start[func_b];
    }
    else if(param_ptr==param_ptr_end[func_b]){
      param_ptr = 1;
      index_ab = 0;
      flag_options = 1;
      return 1;
    }
    else{
      param_ptr++;
    }
  }
  else{	 
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

case 7:		 
  if(param_ptr==13){
    param_ptr = 0;
    flag_options = 0;
    return 1;
  }
  else{
    param_ptr++;
  }
  break;

case 8:		 
  if(param_ptr==0){
    param_ptr = 13;
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

case 9:		 
  if(param_ptr==1){
    param_ptr = param_ptr_end[func_a];
    flag_options = 0;
  }
  else{
    param_ptr--;
    if(param_ptr==1){
      return 1;
    }
  }
  break;

case 10:	 
  if(param_ptr==0){
    param_ptr = param_ptr_start[func_a];
  }
  else if(param_ptr==param_ptr_end[func_a]){
    param_ptr = 1;
    flag_options = 1;
    return 1;
  }
  else{
    param_ptr++;
  }
  break;

case 11:	 
  if(param_ptr==13){
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
}	 

return 0;
}


 



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
  *ptr = 0;		 
}
}


 



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


 
#line 3213
void update_dsp(int param_ptr_tmp, int index_ab_tmp)
{
long ltemp;
int itemp, iorder, cursor_temp;
float f1, f2, fcenter, fwidth, ftemp;

 
switch(param_ptr_tmp){
case 0:	 
  if(params_changed_copy==1) break;	 
  gain(index_ab_tmp);	 
  update_dsp(param_ptr_start[(int)params[0][index_ab_tmp]],index_ab_tmp);	 
  break;

case 1:	 
  assembly_flag |= 1;	 
  break;

case 2:	 
  auto_vu_count = (int)params[2][0];	 
  break;

case 3:  
  min_value = 1;	 
  max_value = max_in_level;
  if(params_changed_copy==1) break;	 
  set_all_gains();		 
  break;

case 4:	 
  if(params_changed_copy==1) break;	 
  set_fsample();		 
  init_freq_params();	 
   

case 6:	 
  if(params_changed_copy==1) break;	 
   
  if(params[6][0]==0){	 
    gain(2);	 
    update_dsp(param_ptr_start[(int)params[0][2]],2);	 
  }
  else if(params[6][0]==1){  
    if(params[17][0] > 128) params[17][0] = 127;	 
    if(params[20][0] > 128) params[20][0] = 127;
    if(params[26][0] > 128) params[26][0] = 127;
    if(params[32][0] > 128) params[32][0] = 127;
    if(params[40][0] > 128) params[40][0] = 127;
    gain(0);	 
    update_dsp(param_ptr_start[(int)params[0][0]],0);	 
    gain(1);	 
    update_dsp(param_ptr_start[(int)params[0][1]],1);	 
  }
  else{					 
    func_addr_b = (unsigned)&no_func_b;	 
    gain(0);	 
    update_dsp(param_ptr_start[(int)params[0][0]],0);	 
  }
  break;

case 5:	 
  if(params_changed_copy==1) break;	 
  assembly_flag = (int)params[5][0] ? (assembly_flag|8):(assembly_flag&(~8));	 
  break;

case 7:	 
  if(params_changed_copy==1) break;	 
  assembly_flag = (int)params[7][0] ? (assembly_flag|4):(assembly_flag&(~4));	 
  break;

case 8:	 
  if(params_changed_copy==1) break;	 

  break;

case 9:	 
  if(params_changed_copy==3){
    initialize(); 	 
    wait(500000);	 
    store_all();	 
    update_disp_left();		 
    update_disp_right(1);
  }
  break;

case 10:	 
  store();	 
  break;

case 11:	 
  recall();	 
  break;

case 12:	 
 
 
    disp_num(220, 13, 4, 2);
    disp_text("", 1, 0);	 
 
#line 3316
  break;

case 13:	 
 
 
    disp_num(serial_number, 11, 6, 0);
    disp_text("", 1, 0);	 
 
#line 3328
  break;

case 14:	 
  min_value = -10000;	 
  max_value = 10000;
  if(params_changed_copy==1) break;	 
  gain(index_ab_tmp);
   
  itemp = index_ab_tmp + 1;	 
  if(itemp&1){
    func_addr_a = (unsigned)&no_func_a;  
  }
  if(itemp&2){
    func_addr_b = (unsigned)&no_func_b;  
  }
  break;
case 15:	 
  min_value = -10000;	 
  max_value = 10000;
  if(params_changed_copy==1) break;	 
  gain(index_ab_tmp);
   
  itemp = index_ab_tmp + 1;	 
  if(itemp&1){
    func_addr_a = (unsigned)&allpass_func_a;  
  }
  if(itemp&2){
    func_addr_b = (unsigned)&allpass_func_b;  
  }
  break;

case 18:	 
case 21:	 
case 27:	 
case 33:	 
case 36:	 
case 39:	 
case 42:	 
case 45:	 
  min_value = -10000;	 
  max_value = 10000;
  if(params_changed_copy==1) break;	 
  gain(index_ab_tmp);
  break;

case 17:	 
case 20:	 
  min_value = (long)(3);	 
  if(params[6][0]==2){		 
    max_value = (long)(256);	 
  }
  else{
    max_value = (long)(128);  
  }
  if(params_changed_copy==1) return;	 
  f1 = (float)params[param_ptr_tmp-1][index_ab_tmp];	 
  iorder = (int)params[param_ptr_tmp][index_ab_tmp];	 
  goto compute_filt;

case 26:	 
case 32:	 
  min_value = (long)(3);	 
  if(params[6][0]==2){		 
    max_value = (long)(256);	 
  }
  else{
    max_value = (long)(128);  
  }
  if(params_changed_copy==1) return;	 
  f1 = (float)params[param_ptr_tmp-4][index_ab_tmp];	 
  f2 = (float)params[param_ptr_tmp-3][index_ab_tmp];	 
  iorder = (int)params[param_ptr_tmp][index_ab_tmp];	 
  goto compute_filt;

case 16:	 
case 19:	 
  min_value = (long)(200.0/48000.0*fsample);	 
  max_value = (long)(20000.0/48000.0*fsample);
  if(params_changed_copy==1) return;	 
  f1 = (float)params[param_ptr_tmp][index_ab_tmp];	 
  iorder = (int)params[param_ptr_tmp+1][index_ab_tmp];	 
  goto compute_filt;

case 22:	 
case 28:	 
  min_value = (long)(200.0/48000.0*fsample);	 
  max_value = (long)(19600.0/48000.0*fsample);
  if(params_changed_copy==1) return;	 
  f1 = (float)params[param_ptr_tmp][index_ab_tmp];	 
  f2 = (float)params[param_ptr_tmp+1][index_ab_tmp];	 
  if((f2-f1)<400.0/48000.0*fsample){	 
    f2 = f1 + 400.0/48000.0*fsample;
    params[param_ptr_tmp+1][index_ab_tmp] = (long)f2;	 
  }
  params[param_ptr_tmp+2][index_ab_tmp] = (long)((f1+f2)/2.0);	 
  params[param_ptr_tmp+3][index_ab_tmp] = (long)(f2-f1);		 
  iorder = (int)params[param_ptr_tmp+4][index_ab_tmp];	 
  goto compute_filt;

case 23:	 
case 29:	 
  min_value = (long)(600.0/48000.0*fsample);	 
  max_value = (long)(20000.0/48000.0*fsample);
  if(params_changed_copy==1) return;	 
  f1 = (float)params[param_ptr_tmp-1][index_ab_tmp];	 
  f2 = (float)params[param_ptr_tmp][index_ab_tmp];	 
  if((f2-f1)<400.0/48000.0*fsample){	 
    f1 = f2 - 400.0/48000.0*fsample;
    params[param_ptr_tmp-1][index_ab_tmp] = (long)f1;	 
  }
  params[param_ptr_tmp+1][index_ab_tmp] = (long)((f1+f2)/2.0);	 
  params[param_ptr_tmp+2][index_ab_tmp] = (long)(f2-f1);		 
  iorder = (int)params[param_ptr_tmp+3][index_ab_tmp];	 
  goto compute_filt;

case 24:	 
case 30:	 
  fwidth = (float)params[param_ptr_tmp+1][index_ab_tmp];	 
  min_value = (long)((200.0/48000.0*fsample + fwidth/2.0) + 0.5);	 
  max_value = (long)((20000.0/48000.0*fsample - fwidth/2.0) + 0.5);
  if(params_changed_copy==1) return;	 
  fcenter = (float)params[param_ptr_tmp][index_ab_tmp];	 
  ftemp = fwidth/2.0;
  f1 = fcenter - ftemp;
  f2 = fcenter + ftemp;
 
 
#line 3459
  params[param_ptr_tmp-2][index_ab_tmp] = (long)f1;		 
  params[param_ptr_tmp-1][index_ab_tmp] = (long)f2;		 
  params[param_ptr_tmp+1][index_ab_tmp] = (long)(f2 - f1);	 
  iorder = (int)params[param_ptr_tmp+2][index_ab_tmp];	 
  goto compute_filt;

case 25:	 
case 31:	 
  min_value = (long)(400.0/48000.0*fsample);	 
  max_value = (long)((20000.0/48000.0 - 200.0/48000.0)*fsample + 0.5);
  if(params_changed_copy==1) return;	 
  fcenter = (float)params[param_ptr_tmp-1][index_ab_tmp];	 
  fwidth = (float)params[param_ptr_tmp][index_ab_tmp];		 
  ftemp = fwidth/2.0;                                          
  f1 = fcenter - ftemp;
  f2 = fcenter + ftemp;
  if(f1<200.0/48000.0*fsample){	 
    f1 = 200.0/48000.0*fsample;
    f2 = f1 + fwidth;
  }
  if(20000.0/48000.0*fsample<f2){	 
    f2 = 20000.0/48000.0*fsample;
    f1 = f2 - fwidth;
  }
  params[param_ptr_tmp-3][index_ab_tmp] = (long)f1;		 
  params[param_ptr_tmp-2][index_ab_tmp] = (long)f2;		 
  params[param_ptr_tmp-1][index_ab_tmp] = (long)((f1 + f2)/2.0);	 
  iorder = (int)params[param_ptr_tmp+1][index_ab_tmp];	 
 compute_filt:
  compute_fir(f1, f2, iorder, index_ab_tmp);	 
  break;

case 34:	 
case 37:	 
  min_value = (long)(200.0/48000.0*fsample);	 
  max_value = (long)(20000.0/48000.0*fsample);
  if(params_changed_copy==1) return;	 
  f1 = (float)params[param_ptr_tmp][index_ab_tmp];		 
  f2 = (float)params[param_ptr_tmp+1][index_ab_tmp];	 
  goto compute_n;
  
case 35:	 
case 38:	 
  min_value = (long)(10.0/48000.0*fsample);	 
  max_value = (long)(10000.0/48000.0*fsample);
  if(params_changed_copy==1) return;	 
  f1 = (float)params[param_ptr_tmp-1][index_ab_tmp];	 
  f2 = (float)params[param_ptr_tmp][index_ab_tmp];		 
 compute_n:
  compute_notch(f1, f2, index_ab_tmp);	 
  break;

case 40:	 
  min_value = (long)(3);	 
  if(params[6][0]==2){		 
    max_value = (long)(256);	 
  }
  else{
    max_value = (long)(128);  
  }
  if(params_changed_copy==1) return;	 
  params[param_ptr_tmp+1][index_ab_tmp] = 1L;	 
  iorder = (int)params[param_ptr_tmp][index_ab_tmp];	 
  goto load_user;

case 41:	 
  iorder = (int)params[param_ptr_tmp-1][index_ab_tmp];	 
  min_value = 1L;	 
  max_value = (long)iorder;	 

  cursor_temp = cursor_pos;					 

  itemp = (int)params[param_ptr_tmp][index_ab_tmp] - 1;	 
  ltemp = params[(sizeof param_struct)/(sizeof param_struct[0]) + (itemp>>1)][index_ab_tmp];		 
  if(itemp&1){
    disp_num((long)(ltemp>>16), 11, 6, 0);	 
  }
  else{
    disp_num((long)((int)(ltemp&0x0000ffff)), 11, 6, 0);	 
  }

  disp_text("", cursor_temp, 0);	 

  if(params_changed_copy==1) return;	 
  
 load_user:  
  load_userfir(iorder, index_ab_tmp);	 
  break;

default:
  break;
}	 
    
}


 
#line 3561
void compute_fir(float f1, float f2, int iorder, int index_ab_tmp)
{
int i, itemp, max_flag, iorderm1, iorderm1d2, iorderd2;
float ftemp1, ftemp2, d2fsf1, d2fsf2, coef_max;
float coefs[128];

iorderm1 = iorder-1;
iorderm1d2 = iorderm1>>1;
iorderd2 = iorder>>1;

 
if(iorder!=iorder_old){
  ftemp1 = (float)(iorder-1);
  for(i=0;i<=iorderm1d2;i++){	 
    ftemp2 = (6.28318530717959*(float)(i))/ftemp1;
    window[i] = 0.48216433063585 - 0.48550251793519*cos(ftemp2) + 0.03233315142896*cos(2*ftemp2);
  }
  iorder_old = iorder;
}

 
ftemp1 = 2.0/fsample;	 
d2fsf1 = ftemp1*f1;
d2fsf2 = ftemp1*f2;
switch((int)params[0][index_ab_tmp]){	 
case 2:	 
  for(i=0;i<iorderd2;i++){	 
    ftemp1 = 3.14159265359*((float)(i) - ((float)iorderm1)/2.0);	 
    coefs[i] = window[i]*sin(d2fsf1*ftemp1)/ftemp1;
  }
  if(iorder&1){		 
    coefs[i] = window[i]*d2fsf1;	 
  }
  break;
    
case 3:	 
  for(i=0;i<iorderd2;i++){	 
    ftemp1 = 3.14159265359*((float)(i) - ((float)iorderm1)/2.0);
    coefs[i] = window[i]*(sin(ftemp1) - sin(d2fsf1*ftemp1))/ftemp1;
  }
  if(iorder&1){		 
    coefs[i] = window[i]*(1.0 - d2fsf1);	 
  }
  break;
    
case 4:	 
  for(i=0;i<iorderd2;i++){	 
    ftemp1 = 3.14159265359*((float)(i) - ((float)iorderm1)/2.0);
    coefs[i] = window[i]*(sin(d2fsf2*ftemp1) - sin(d2fsf1*ftemp1))/ftemp1;
  }
  if(iorder&1){		 
    coefs[i] = window[i]*(d2fsf2 - d2fsf1);	 
  }
  break;
    
case 5:	 
  for(i=0;i<iorderd2;i++){	 
    ftemp1 = 3.14159265359*((float)(i) - ((float)iorderm1)/2.0);
    coefs[i] = window[i]*(sin(ftemp1) + sin(d2fsf1*ftemp1) - sin(d2fsf2*ftemp1))/ftemp1;
  }
  if(iorder&1){		 
    coefs[i] = window[i]*(1.0 + d2fsf1 - d2fsf2);	 
  }
  break;

default:
  break;
}	 

 
coef_max = coefs[iorderm1d2];	 
max_flag = (0.4999<coef_max);	 
ftemp1 = max_flag ? 32768.0:65536.0;	 

 
out_gain |= 0x0400;		 

#line 3686
itemp = index_ab_tmp + 1;	 
if(itemp&1){
  func_addr_a = (unsigned)&no_func_a;  
  data_ptr_a = 0x03ff - iorderm1;	 
  orderm2_a = iorderm1-1;				 
  asm("	clrc	CNF		; map internal memory block B0 into Data space so we can write it");
  for(i=0;i<=iorderm1d2;i++){
    fir_coef[i] = fir_coef[iorderm1-i] = (int)(ftemp1*coefs[i] + 0.5);	 
  }
  asm("	setc	CNF		; map internal memory block B0 into Program space");
  func_addr_a = (unsigned)(max_flag ? &fir_15_a:&fir_16_a);  
}

if(itemp&2){
  func_addr_b = (unsigned)&no_func_b;  
  data_ptr_b = 0x037f - iorderm1;	 
  orderm2_b = iorderm1-1;				 
  asm("	clrc	CNF		; map internal memory block B0 into Data space so we can write it");
  for(i=0;i<=iorderm1d2;i++){
    fir_coef[i+128] = fir_coef[iorderm1+128-i] = (int)(ftemp1*coefs[i] + 0.5);	 
  }
  asm("	setc	CNF		; map internal memory block B0 into Program space");
  func_addr_b = (unsigned)(max_flag ? &fir_15_b:&fir_16_b);  
}



wait_n_samples(iorder);	 
out_gain &= ~0x0400;		 

}


 
#line 3724
void load_userfir(int iorder, int index_ab_tmp)
{
int i, j, itemp, iorderm1, iorderm1d2;
long ltemp;

iorderm1 = iorder-1;
iorderm1d2 = iorderm1>>1;

out_gain |= 0x0400;		 
  
itemp = index_ab_tmp + 1;	 
if(itemp&1){
  func_addr_a = (unsigned)&no_func_a;  
  data_ptr_a = 0x03ff - iorderm1;	 
  orderm2_a = iorderm1-1;				 
  asm("	clrc	CNF		; map internal memory block B0 into Data space so we can write it");
  j = 0;	 
  for(i=0;i<=iorderm1d2;i++){
    ltemp = params[(sizeof param_struct)/(sizeof param_struct[0]) + i][index_ab_tmp];	 
    fir_coef[j++] = (int)(ltemp&0x0000ffff);		 
    fir_coef[j++] = (int)(((unsigned long)ltemp)>>16);	 
  }
  asm("	setc	CNF		; map internal memory block B0 into Program space");
  func_addr_a = (unsigned)&fir_15_a;  
}

if(itemp&2){
  func_addr_b = (unsigned)&no_func_b;  
  data_ptr_b = 0x037f - iorderm1;	 
  orderm2_b = iorderm1-1;				 
  asm("	clrc	CNF		; map internal memory block B0 into Data space so we can write it");
  j = 128;	 
  for(i=0;i<=iorderm1d2;i++){
    ltemp = params[(sizeof param_struct)/(sizeof param_struct[0]) + i][index_ab_tmp];	 
    fir_coef[j++] = (int)(ltemp&0x0000ffff);		 
    fir_coef[j++] = (int)(((unsigned long)ltemp)>>16);	 
  }
  asm("	setc	CNF		; map internal memory block B0 into Program space");
  func_addr_b = (unsigned)&fir_15_b;  
}

wait_n_samples(iorder);	 
out_gain &= ~0x0400;		 

}


 
#line 3777
void compute_notch(float fn, float fw, int index_ab_tmp)
{
int* iptr;
int itemp;
float t1, t2, t3, k1, k2, c1, c2, d1, d2, g1, g2;

 
k1 = -sin(2*3.14159265359*fn/fsample + 1.5707963268);	 
t1 = 3.14159265359*fw/fsample;
 
t2 = sin(t1 + 1.5707963268);
t3 = sin(t1);
k2 = (t2-t3)/(t2+t3);		 
 
 

 
#line 3798
d1 = -1;
d2 = -1;
c1 = (1.0 - k1*k1)/d1;
c2 = (1.0 - k2*k2)/d2;

 
switch((int)params[0][index_ab_tmp]){	 
case 6:	 
   
  g1 = 0.5;
  g2 = 0.0;
  break;
  
case 7:	 
   
  g1 = 0.0;
  g2 = 0.5;
  break;

default:
  break;
}	 


 
out_gain |= 0x0400;		 

itemp = index_ab_tmp + 1;	 
if(itemp&1){
  func_addr_a = (unsigned)&no_func_a;  
  data_ptr_a = 0x03ff;	 
   
  iptr = (int*)&coefdata[0x80];	 
  *iptr++ = (int)(32768*c2 + 0.5);	 
  *iptr++ = (int)(32768*k2 + 0.5);	 
  *iptr++ = (int)(32768*d2 + 0.5);	 
  *iptr++ = (int)(32768*k2 + 0.5);	 
  *iptr++ = (int)(32768*c1 + 0.5);	 
  *iptr++ = (int)(32768*k1 + 0.5);	 
  *iptr++ = (int)(32768*d1 + 0.5);	 
  *iptr++ = (int)(32768*k1 + 0.5);	 
  *iptr++ = (int)(8192*g1 + 0.5);	 
  *iptr++ = (int)(8192*g2 + 0.5);	 
  func_addr_a = (unsigned)&notch_a;  
}

if(itemp&2){
  func_addr_b = (unsigned)&no_func_b;  
  data_ptr_b = 0x037f;	 
   
  iptr = (int*)&coefdata[0];	 
  *iptr++ = (int)(32768*c2 + 0.5);	 
  *iptr++ = (int)(32768*k2 + 0.5);	 
  *iptr++ = (int)(32768*d2 + 0.5);	 
  *iptr++ = (int)(32768*k2 + 0.5);	 
  *iptr++ = (int)(32768*c1 + 0.5);	 
  *iptr++ = (int)(32768*k1 + 0.5);	 
  *iptr++ = (int)(32768*d1 + 0.5);	 
  *iptr++ = (int)(32768*k1 + 0.5);	 
  *iptr++ = (int)(8192*g1 + 0.5);	 
  *iptr++ = (int)(8192*g2 + 0.5);	 
  func_addr_b = (unsigned)&notch_b;  
}
  
 
out_gain &= ~0x0400;		 

}


 
#line 3873
void set_all_gains(void)
{
float cl, ftemp;

 					
 
cl = (float)params[3][0];	 
for(gopt_a=1;gopt_a<16;gopt_a++){
  ftemp = (16.0*in_cal_levels[gopt_a]*cl)/max_in_level;	 
  if(2.0<=ftemp){
    break;	 
  }
}
gopt_a--;
out_gain = (out_gain&(~0x000f))|gopt_a;		 
scale_k_a = max_in_level/(8.0*in_cal_levels[gopt_a]*cl);	 

 
 
for(gopt_b=1;gopt_b<16;gopt_b++){
  ftemp = (16.0*in_cal_levels[gopt_b]*cl)/max_in_level;	 
  if(2.0<=ftemp){
    break;	 
  }
}
gopt_b--;
out_gain = (out_gain&(~0x00f0))|(gopt_b<<4);		 
scale_k_b = max_in_level/(8.0*in_cal_levels[gopt_b]*cl);	 

 
if(params[6][0]){	 
  gain(0);	 
  gain(1);	 
}
else{	 
  gain(2);	 
}

}


 
#line 3920
void gain(int index_ab_tmp)
{
int itemp;
float tgain, tlin, ftemp;

 
tgain = 0.01*(float)params[param_ptr_end[(int)params[0][index_ab_tmp]]][index_ab_tmp];	 

itemp = index_ab_tmp + 1;	 
 
if(itemp&1){
  tlin = 16.0*in_cal_levels[gopt_a]/max_in_level;	 
  ftemp = tgain/(tlin*scale_k_a);	 
  for(aopt_a=1;aopt_a<32;aopt_a++){
    if(out_cal[aopt_a]<=fabs(ftemp)){
      break;	 
    }
  }
  aopt_a--;
  out_atten = (out_atten&(~0x01f0))|(aopt_a<<4);	 
  ftemp = tgain/(out_cal[aopt_a]*tlin);			 
  t_reg_scale_a = (int)(256.0*ftemp + 0.5);			 
}
if(itemp&2){
  tlin = 16.0*in_cal_levels[gopt_b]/max_in_level;	 
  ftemp = tgain/(tlin*scale_k_b);	 
  for(aopt_b=1;aopt_b<32;aopt_b++){
    if(out_cal[aopt_b]<=fabs(ftemp)){
      break;	 
    }
  }
  aopt_b--;
  out_atten = (out_atten&(~0x3e00))|(aopt_b<<9);	 
  ftemp = tgain/(out_cal[aopt_b]*tlin);			 
  t_reg_scale_b = (int)(256.0*ftemp + 0.5);			 
}

 


}


 
#line 3970
void vu_update(void)
{
int temp_pos;
unsigned vu_temp;
char carray[3];

carray[2] = 0;

vu_temp = convert_to_vu(scale_k_a, in_a_hold);	 
if(vu_temp>=in_a_vu_level){	 
  in_a_vu_level = vu_temp;
}

vu_temp = convert_to_vu(scale_k_b, in_b_hold);	 
if(vu_temp>=in_b_vu_level){	 
  in_b_vu_level = vu_temp;
}

vu_temp = convert_to_vu(scale_k_a, out_a_hold);	 
if(vu_temp>=out_a_vu_level){	 
  out_a_vu_level = vu_temp;
}

vu_temp = convert_to_vu(scale_k_b, out_b_hold);	 
if(vu_temp>=out_b_vu_level){	 
  out_b_vu_level = vu_temp;
}

 
temp_pos = cursor_pos;					 
carray[0] = vu_chars[in_a_vu_level];	 
carray[1] = vu_chars[in_b_vu_level];	 
disp_text(carray, 10, -1);

carray[0] = vu_chars[out_a_vu_level];	 
carray[1] = vu_chars[out_b_vu_level];	 
disp_text(carray, 15, -1);

disp_text("", temp_pos, 0);	 


 
if((vu_counter++)>=2){
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


 
#line 4040
unsigned convert_to_vu(float scale, int signal)
{

float ftemp;
int itemp;

ftemp = scale*(float)(signal);	 
if(32767.0<ftemp){	 
  ftemp = 32767.0;
}

#line 4065
itemp = (int)ftemp;
if(itemp==32767) return 12;		 
else if(itemp>=16384) return 8;	 
else if(itemp>=8192) return 7;	 
else if(itemp>=4096) return 6;	 
else if(itemp>=2048) return 5;
else if(itemp>=1024) return 4;
else if(itemp>=512) return 3;
else if(itemp>=256) return 2;
else if(itemp>=128) return 1;
else return 0;


}


 
#line 4087
void led_update(void)
{


 
out_atten = ((in_error_stick & 0x0100)>>5)|(out_atten & (~0x0008));
out_atten = ((in_error_stick & 0x0200)>>7)|(out_atten & (~0x0004));
 

if((led_counter++)>=20){
  led_counter = 0;
  in_error_stick = in_error_stick & (~0x0100);	 
  in_error_stick = in_error_stick & (~0x0200);	 
}


}


 
#line 4122
void store(void)
{
int i, j, itemp;
int retrieved_flag[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};	 
unsigned next_loc, prev_loc=0, loc_code, ptr, ptr_m1=0;
unsigned current_loc;
unsigned fd_ptr, fd_ptr_m1=0, nrecord;
unsigned* uptr;                                                 

min_value = 0;	 
max_value = 4;

if(params_changed_copy!=3){	 
  return;
}

nrecord = 6 + (sizeof param_struct)/(sizeof param_struct[0]) + 128 + (sizeof param_struct)/(sizeof param_struct[0]) + 128 + (sizeof param_struct)/(sizeof param_struct[0]) + 128 + (sizeof param_struct)/(sizeof param_struct[0]) + 128 + (sizeof param_struct)/(sizeof param_struct[0]) + 128 + (sizeof param_struct)/(sizeof param_struct[0]) + 128;

current_loc = (unsigned)params[10][0];

func_addr_temp_a = func_addr_a;	 
func_addr_temp_b = func_addr_b;	 
func_addr_a = (unsigned)&no_func_a;	 
func_addr_b = (unsigned)&no_func_b;

portfff5 &= ~0x0200; 	 
disp_num(current_loc, 1, 8, 0);	 
disp_text(" Stored ", 9, 0);
beep(75, 400);					 

 
ptr = 0x6000;	 
read_flash(ptr, 5, record);		 
next_loc = record[0];
while(next_loc!=0xffff){
   
  if(record_bad()){	 
    goto store_error;
  }
  ptr_m1 = ptr;					 
  ptr = next_loc;				 
  read_flash(ptr, 5, record);	 
  next_loc = record[0];
}
 

if(nrecord>(0x7fff - ptr)){	 
 
 
 
   
  fd_ptr = 0;			 
  prev_loc = ptr_m1;	 
  retrieved_flag[current_loc] = 1;	 
  do{	 
    do{
      ptr = prev_loc;				 
      read_flash(ptr, 6, record);	 
      next_loc = record[0];
      prev_loc = record[1];
      loc_code = record[5];
      if(record_bad()){	 
       store_error:
        disp_text("Error-MemCorrupt", 1, -1);	 
        wait(1500000);
        goto store_out;
      }
    } while((prev_loc!=0)&&(retrieved_flag[loc_code]==1));  
    
    if(retrieved_flag[loc_code]==0){	 
       
       
      read_flash(ptr, nrecord, &flash_data[fd_ptr]);	 
      flash_data[fd_ptr] = fd_ptr + nrecord + 0x6000;	 
      flash_data[fd_ptr+1] = fd_ptr_m1 + 0x6000;		 
      fd_ptr_m1 = fd_ptr;				 
      fd_ptr += nrecord;				 
      retrieved_flag[loc_code] = 1;		 
    }
  } while(prev_loc!=0);	 
  
   
  flash_data[1] = 0;	 
  flash_locked = 0;		 
  itemp = prog_flash(0x6000, fd_ptr, flash_data, 1, "inStore1");	 
  if(itemp){
    goto store_out;
  }
  ptr = fd_ptr + 0x6000; 					 
   
  ptr_m1 = ptr - nrecord;
}

 
record[0] = ptr + nrecord;					 
record[1] = ptr_m1;							 
record[2] = 220;						 
record[3] = *((unsigned*)(&serial_number));		 
record[4] = *((unsigned*)(&serial_number)+1);	 
record[5] = current_loc;					 

 
 

 
j=6;
uptr = (unsigned*)(&params[0][0]);	 
for(i=0;i<(sizeof param_struct)/(sizeof param_struct[0]) + 128;i++){
  record[j++] = *uptr++;					 
  record[j++] = *uptr++;					 
  record[j++] = *uptr++;					 
  record[j++] = *uptr++;					 
  record[j++] = *uptr++;					 
  record[j++] = *uptr++;					 
}
 

flash_locked = 0;	 
prog_flash(ptr, nrecord, record, 0, "inStore2");	 


 

store_out:
func_addr_a = func_addr_temp_a;  
func_addr_b = func_addr_temp_b;  
wait(500000);			 
update_disp_left();		 
update_disp_right(1);
portfff5 |= 0x0200; 	 
sw_down = 0;			 
}


 
#line 4261
void recall(void)
{

int i, j;
unsigned next_loc, prev_loc=0, loc_code, ptr, ptr_m1=0, current_loc;
unsigned desired_loc_ptr=0, nrecord;
unsigned* uptr;

min_value = 0;	 
max_value = 4;

if(params_changed_copy!=3){	 
  return;
}

nrecord = 6 + (sizeof param_struct)/(sizeof param_struct[0]) + 128 + (sizeof param_struct)/(sizeof param_struct[0]) + 128 + (sizeof param_struct)/(sizeof param_struct[0]) + 128 + (sizeof param_struct)/(sizeof param_struct[0]) + 128 + (sizeof param_struct)/(sizeof param_struct[0]) + 128 + (sizeof param_struct)/(sizeof param_struct[0]) + 128;

current_loc = (unsigned)params[11][0];

func_addr_a = (unsigned)&no_func_a;	 
func_addr_b = (unsigned)&no_func_b;

portfff5 &= ~0x0200; 	 
disp_num(current_loc, 1, 7, 0);	 
disp_text(" Recalled", 8, 0);	 
beep(75, 400);					 

 
ptr = 0x6000;	 
read_flash(ptr, 6, record);	 
next_loc = record[0];
loc_code = record[5];
while(next_loc!=0xffff){
  if(record_bad()){	 
    goto recall_error;
  }
  if(loc_code==current_loc){	 
    desired_loc_ptr = ptr;		 
  }
  ptr_m1 = ptr;		 
  ptr = next_loc;	 
  read_flash(ptr, 6, record);	 
  next_loc = record[0];
  loc_code = record[5];
}
 

if(desired_loc_ptr==0){	 
 recall_error:
 
  disp_text("Location Blank! ", 1, -1);	 
  wait(1000000);
  goto recall_out;
}

 
read_flash(desired_loc_ptr, nrecord, record);	 

 
j=6;
uptr = (unsigned*)(&params[0][0]);	 
for(i=0;i<(sizeof param_struct)/(sizeof param_struct[0]) + 128;i++){
  *uptr++ = record[j++];					 
  *uptr++ = record[j++];					 
  *uptr++ = record[j++];					 
  *uptr++ = record[j++];					 
  *uptr++ = record[j++];					 
  *uptr++ = record[j++];					 
}
 


recall_out:

set_fsample();	 
auto_vu_count = (int)params[2][0];	 
assembly_flag = (int)params[5][0] ? (assembly_flag|8):(assembly_flag&(~8));	 
assembly_flag = (int)params[7][0] ? (assembly_flag|4):(assembly_flag&(~4));	 

 
in_a_hold = 0;
in_b_hold = 0;
out_a_hold = 0;
out_b_hold = 0;
in_a_vu_level = 0;
in_b_vu_level = 0;
out_a_vu_level = 0;
out_b_vu_level = 0;

 
param_ptr = 0;		 
flag_options = 0;	 
index_ab = params[6][0] ? 0:2;	    

set_all_gains();	 


 
if(params[6][0]==0){	 
  update_dsp(param_ptr_start[(int)params[0][2]],2);	 
}
else if(params[6][0]==1){  
  update_dsp(param_ptr_start[(int)params[0][0]],0);	 
  update_dsp(param_ptr_start[(int)params[0][1]],1);	 
}
else{					 
  func_addr_b = (unsigned)&no_func_b;	 
  update_dsp(param_ptr_start[(int)params[0][0]],0);	 
}
min_value = 0;	 
max_value = 4;

wait(600000);			 

 
update_disp_left();		 
update_disp_right(1);
portfff5 |= 0x0200; 	 
sw_down = 0;			 
}


 
#line 4390
int record_bad(void)
{

return ( (record[2]!=220)||(record[3]!=*((unsigned*)(&serial_number)))||
         (record[4]!=*((unsigned*)(&serial_number)+1))                       );
}


