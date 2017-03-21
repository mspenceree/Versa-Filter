*******************************************************
* TMS320C2x/C2xx/C5x ANSI C Codegen Version 7.00       
*******************************************************
;	C:\TIC2XX\C2000\CGTOOLS\BIN\DSPAC.EXE -v2xx -l -x0 -ic:\c2xxti FILT.C C:\WINDOWS\TEMP\FILT.if 
;	dspopt NOT RUN
;	C:\TIC2XX\C2000\CGTOOLS\BIN\DSPCG.EXE -o -v2xx -b C:\WINDOWS\TEMP\FILT.if FILT.asm C:\WINDOWS\TEMP\FILT.tmp 
	.port
	.bss	_params_changed_copy,1,1
	.bss	_cursor_flag,1
	.bss	_sw_down,1
	.bss	_cursor_pos,1
	.bss	_auto_vu_count,1
	.bss	_cursor_pos_vu,1
	.bss	_press_flag,1
	.bss	_flash_cursor_flag,1
	.bss	_write_ptr,1
	.bss	_read_ptr,1
	.bss	_serial_error_flag,1
	.bss	_params_changed,1
	.bss	_param_ptr,1
	.bss	_index_ab,1
	.bss	_sw_pressed,1
	.bss	_sw_released,1
	.bss	_cw,1
	.bss	_ccw,1
	.bss	_testcount,1
	.bss	_down_turn_flag,1
	.bss	_p_state,1
	.bss	_flag_options,1
	.bss	_led_counter,1
	.bss	_vu_counter,1
	.bss	_in_a_vu_level,1
	.bss	_in_b_vu_level,1
	.bss	_out_a_vu_level,1
	.bss	_out_b_vu_level,1
	.bss	_quietsn_flag,1
	.bss	_iorder_old,1
	.bss	_flash_locked,1
	.bss	_max_in_level,1
	.bss	_imr_ptr,1
	.bss	_greg_ptr,1
	.bss	_ifr_ptr,1
	.bss	_port0_copy,1
	.bss	_gray_code,1
	.bss	_sn_ok_flag,1
	.bss	_func_addr_temp_a,1
	.bss	_func_addr_temp_b,1
	.bss	_data_count,1
	.bss	_sign_mult,1
	.bss	_index_ab_p,1
	.bss	_p_uint1,1
	.bss	_p_uint2,1
	.bss	_sw_down_old,1
	.bss	_gray_code_old,1
	.bss	_gopt_a,1
	.bss	_gopt_b,1
	.bss	_aopt_a,1
	.bss	_aopt_b,1
	.bss	_null_text,1
	.bss	_ioData,1
	.bss	_min_value,2,1
	.bss	_max_value,2,1
	.bss	_serial_number,2,1
	.bss	_fsample,2,1
	.bss	_p_long,2,1
	.bss	_check_sum,2,1
	.bss	_p_sum,2,1
	.bss	_scale_k_a,2,1
	.bss	_scale_k_b,2,1
	.bss	_initialize_text,2
	.bss	_revertolevels_text,3
	.bss	_samplerate_text,3
	.bss	_master_mode_text,3
	.bss	_cascade_ch_a_b_text,3
	.bss	_inputsrc_text,3
	.bss	_mode_text,4
	.bss	_param_ptr_start,10
	.bss	_param_ptr_end,10
	.bss	_func_text,10
	.bss	_serial_str,11
	.bss	_vu_chars,13
	.bss	_parameter_str,17
	.bss	_value_str,17
	.bss	_in_cal_levels,32
	.bss	_out_cal,64
	.bss	_serial_in_buf,128
	.bss	_param_struct,138
	.bss	_window,256
	.bss	_params,1044
	.bss	_record,1050
	.bss	_flash_data,8192
	.file	"FILT.C"
	.file	"c203.h"
	.globl	_ioData
	.file	"c:\c2xxti\stdlib.h"
	.sym	_size_t,0,14,13,16
	.sym	_wchar_t,0,2,13,16
	.globl	_atoi
	.globl	_atol
	.globl	_atof
	.globl	_strtod
	.globl	_strtol
	.globl	_strtoul

	.stag	__div_t,32
	.member	_quot,0,4,8,16
	.member	_rem,16,4,8,16
	.eos
	.sym	_div_t,0,8,13,32,__div_t

	.stag	__ldiv_t,64
	.member	_quot,0,5,8,32
	.member	_rem,32,5,8,32
	.eos
	.sym	_ldiv_t,0,8,13,64,__ldiv_t
	.globl	_div
	.globl	_ldiv
	.globl	_rand
	.globl	_srand
	.globl	_calloc
	.globl	_free
	.globl	_malloc
	.globl	_minit
	.globl	_realloc
	.globl	_abort
	.globl	_exit
	.globl	_atexit
	.globl	_abs
	.globl	_labs
	.globl	_qsort
	.globl	_bsearch
	.globl	_getenv
	.globl	_ti_sprintf
	.file	"c:\c2xxti\math.h"
	.globl	_asin
	.globl	_acos
	.globl	_atan
	.globl	_atan2
	.globl	_ceil
	.globl	_cos
	.globl	_cosh
	.globl	_exp
	.globl	_fabs
	.globl	_floor
	.globl	_fmod
	.globl	_frexp
	.globl	_ldexp
	.globl	_log
	.globl	_log10
	.globl	_modf
	.globl	_pow
	.globl	_sin
	.globl	_sinh
	.globl	_sqrt
	.globl	_tan
	.globl	_tanh
	.file	"c:\c2xxti\string.h"
	.globl	_memchr
	.globl	_memcmp
	.globl	_memcpy
	.globl	_memmove
	.globl	_memset
	.globl	_strcat
	.globl	_strchr
	.globl	_strcmp
	.globl	_strcoll
	.globl	_strcpy
	.globl	_strcspn
	.globl	_strerror
	.globl	_strlen
	.globl	_strncat
	.globl	_strncmp
	.globl	_strncpy
	.globl	_strpbrk
	.globl	_strrchr
	.globl	_strspn
	.globl	_strstr
	.globl	_strtok
	.globl	_strxfrm
	.file	"c:\c2xxti\ctype.h"
	.globl	__ctypes_
	.globl	_isalnum
	.globl	_isalpha
	.globl	_iscntrl
	.globl	_isdigit
	.globl	_isgraph
	.globl	_islower
	.globl	_isprint
	.globl	_ispunct
	.globl	_isspace
	.globl	_isupper
	.globl	_isxdigit
	.globl	_isascii
	.globl	_toupper
	.globl	_tolower
	.globl	_toascii
	.file	"filt.h"

	.sect	".cinit"
	.word	IS1,_in_cal_levels
	.float	1.01875e-1
	.float	1.2115047484e-1
	.float	1.4407300667e-1
	.float	1.7133264461e-1
	.float	2.0375e-1
	.float	2.4230094968e-1
	.float	2.8814601333e-1
	.float	3.4266528922e-1
	.float	4.075e-1
	.float	4.8460189936e-1
	.float	5.7629202667e-1
	.float	6.8533057843e-1
	.float	8.15e-1
	.float	9.6920379873e-1
	.float	1.1525840533
	.float	1.3706611569
IS1	.set	32

	.sym	_in_cal_levels,_in_cal_levels,54,2,512,,16
	.globl	_in_cal_levels
	.word	IS2,_out_cal
	.float	1.1999e1
	.float	1.0089916087e1
	.float	8.4845742675
	.float	7.1346480865
	.float	5.9995
	.float	5.0449580433
	.float	4.2422871337
	.float	3.5673240432
	.float	2.99975
	.float	2.5224790217
	.float	2.1211435669
	.float	1.7836620216
	.float	1.499875
	.float	1.2612395108
	.float	1.0605717834
	.float	8.9183101081e-1
	.float	7.499375e-1
	.float	6.3061975541e-1
	.float	5.3028589172e-1
	.float	4.459155054e-1
	.float	3.7496875e-1
	.float	3.1530987771e-1
	.float	2.6514294586e-1
	.float	2.229577527e-1
	.float	1.87484375e-1
	.float	1.5765493885e-1
	.float	1.3257147293e-1
	.float	1.1147887635e-1
	.float	9.37421875e-2
	.float	7.8827469427e-2
	.float	6.6285736465e-2
	.float	5.5739438175e-2
IS2	.set	64

	.sym	_out_cal,_out_cal,54,2,1024,,32
	.globl	_out_cal
	.file	"FILT.C"
	.word	IS3,_func_text
	.word	SL0
	.word	SL1
	.word	SL2
	.word	SL3
	.word	SL4
	.word	SL5
	.word	SL6
	.word	SL7
	.word	SL8
	.word	SL9
IS3	.set	10

	.sym	_func_text,_func_text,114,2,160,,10
	.globl	_func_text
	.word	IS4,_revertolevels_text
	.word	SL10
	.word	SL11
	.word	SL9
IS4	.set	3

	.sym	_revertolevels_text,_revertolevels_text,114,2,48,,3
	.globl	_revertolevels_text
	.word	IS5,_samplerate_text
	.word	SL12
	.word	SL13
	.word	SL9
IS5	.set	3

	.sym	_samplerate_text,_samplerate_text,114,2,48,,3
	.globl	_samplerate_text
	.word	IS6,_inputsrc_text
	.word	SL14
	.word	SL15
	.word	SL9
IS6	.set	3

	.sym	_inputsrc_text,_inputsrc_text,114,2,48,,3
	.globl	_inputsrc_text
	.word	IS7,_mode_text
	.word	SL16
	.word	SL17
	.word	SL18
	.word	SL9
IS7	.set	4

	.sym	_mode_text,_mode_text,114,2,64,,4
	.globl	_mode_text
	.word	IS8,_cascade_ch_a_b_text
	.word	SL10
	.word	SL11
	.word	SL9
IS8	.set	3

	.sym	_cascade_ch_a_b_text,_cascade_ch_a_b_text,114,2,48,,3
	.globl	_cascade_ch_a_b_text
	.word	IS9,_master_mode_text
	.word	SL19
	.word	SL20
	.word	SL9
IS9	.set	3

	.sym	_master_mode_text,_master_mode_text,114,2,48,,3
	.globl	_master_mode_text
	.word	IS10,_initialize_text
	.word	SL21
	.word	SL9
IS10	.set	2

	.sym	_initialize_text,_initialize_text,114,2,32,,2
	.globl	_initialize_text
	.word	IS11,_null_text
	.word	SL9
IS11	.set	1

	.sym	_null_text,_null_text,114,2,16,,1
	.globl	_null_text

	.stag	_pstruct,48
	.member	_text,0,18,8,16
	.member	_flag,16,14,8,16
	.member	_label,32,14,8,16
	.eos
	.word	IS12,_param_struct
	.word	SL22
	.word	0
	.word	_func_text+0
	.word	SL23
	.word	0
	.word	_null_text+0
	.word	SL24
	.word	0
	.word	_revertolevels_text+0
	.word	SL25
	.word	0
	.word	0
	.word	SL26
	.word	0
	.word	_samplerate_text+0
	.word	SL27
	.word	0
	.word	_inputsrc_text+0
	.word	SL28
	.word	0
	.word	_mode_text+0
	.word	SL29
	.word	0
	.word	_cascade_ch_a_b_text+0
	.word	SL30
	.word	0
	.word	_master_mode_text+0
	.word	SL31
	.word	16384
	.word	_initialize_text+0
	.word	SL32
	.word	16384
	.word	0
	.word	SL33
	.word	16384
	.word	0
	.word	SL34
	.word	0
	.word	_null_text+0
	.word	SL35
	.word	0
	.word	_null_text+0
	.word	SL36
	.word	0
	.word	0
	.word	SL37
	.word	0
	.word	0
	.word	SL38
	.word	0
	.word	0
	.word	SL39
	.word	0
	.word	0
	.word	SL40
	.word	0
	.word	0
	.word	SL41
	.word	0
	.word	0
	.word	SL42
	.word	0
	.word	0
	.word	SL43
	.word	0
	.word	0
	.word	SL44
	.word	0
	.word	0
	.word	SL45
	.word	0
	.word	0
	.word	SL46
	.word	0
	.word	0
	.word	SL47
	.word	0
	.word	0
	.word	SL48
	.word	0
	.word	0
	.word	SL49
	.word	0
	.word	0
	.word	SL50
	.word	0
	.word	0
	.word	SL51
	.word	0
	.word	0
	.word	SL52
	.word	0
	.word	0
	.word	SL53
	.word	0
	.word	0
	.word	SL54
	.word	0
	.word	0
	.word	SL55
	.word	0
	.word	0
	.word	SL56
	.word	0
	.word	0
	.word	SL57
	.word	0
	.word	0
	.word	SL58
	.word	0
	.word	0
	.word	SL59
	.word	0
	.word	0
	.word	SL60
	.word	0
	.word	0
	.word	SL61
	.word	0
	.word	0
	.word	SL62
	.word	0
	.word	0
	.word	SL63
	.word	0
	.word	0
	.word	SL64
	.word	0
	.word	0
	.word	SL65
	.word	0
	.word	0
	.word	SL66
	.word	0
	.word	0
	.word	SL67
	.word	0
	.word	0
IS12	.set	138

	.sym	_param_struct,_param_struct,56,2,2208,_pstruct,46
	.globl	_param_struct
	.word	IS13,_param_ptr_start
	.word	14
	.word	15
	.word	16
	.word	19
	.word	22
	.word	28
	.word	34
	.word	37
	.word	40
	.word	43
IS13	.set	10

	.sym	_param_ptr_start,_param_ptr_start,52,2,160,,10
	.globl	_param_ptr_start
	.word	IS14,_param_ptr_end
	.word	14
	.word	15
	.word	18
	.word	21
	.word	27
	.word	33
	.word	36
	.word	39
	.word	42
	.word	45
IS14	.set	10

	.sym	_param_ptr_end,_param_ptr_end,52,2,160,,10
	.globl	_param_ptr_end
	.globl	_params
	.globl	_port0_copy
	.globl	_flash_data
	.globl	_sw_down
	.globl	_sw_down_old
	.globl	_sw_pressed
	.globl	_sw_released
	.globl	_gray_code
	.globl	_gray_code_old
	.globl	_cw
	.globl	_ccw
	.globl	_testcount
	.globl	_flash_locked
	.globl	_cursor_pos
	.globl	_cursor_pos_vu
	.globl	_cursor_flag
	.globl	_flash_cursor_flag
	.globl	_down_turn_flag
	.globl	_press_flag
	.globl	_param_ptr
	.globl	_write_ptr
	.globl	_read_ptr
	.globl	_p_state
	.globl	_flag_options
	.globl	_index_ab
	.globl	_params_changed
	.globl	_params_changed_copy
	.globl	_led_counter
	.globl	_vu_counter
	.globl	_max_in_level
	.globl	_min_value
	.globl	_max_value
	.globl	_p_long
	.globl	_p_sum
	.globl	_serial_number
	.globl	_p_uint1
	.globl	_p_uint2
	.globl	_sn_ok_flag
	.globl	_quietsn_flag
	.globl	_serial_error_flag
	.globl	_data_count
	.globl	_sign_mult
	.globl	_index_ab_p
	.globl	_check_sum
	.globl	_func_addr_temp_a
	.globl	_func_addr_temp_b
	.globl	_fsample
	.globl	_serial_str
	.globl	_serial_in_buf
	.globl	_parameter_str
	.globl	_value_str
	.word	IS15,_vu_chars
	.word	32
	.word	8
	.word	9
	.word	10
	.word	11
	.word	12
	.word	13
	.word	14
	.word	15
	.word	88
	.word	88
	.word	88
	.word	88
IS15	.set	13

	.sym	_vu_chars,_vu_chars,50,2,208,,13
	.globl	_vu_chars
	.globl	_gopt_a
	.globl	_gopt_b
	.globl	_aopt_a
	.globl	_aopt_b
	.globl	_in_a_vu_level
	.globl	_in_b_vu_level
	.globl	_out_a_vu_level
	.globl	_out_b_vu_level
	.globl	_scale_k_a
	.globl	_scale_k_b
	.globl	_auto_vu_count
	.globl	_iorder_old
	.globl	_window
	.globl	_record
	.globl	_imr_ptr
	.globl	_greg_ptr
	.globl	_ifr_ptr
	.globl	_k7f00h
	.globl	_kf80fh
	.globl	_kfff0h
	.globl	_in_a
	.globl	_in_b
	.globl	_out_a
	.globl	_out_b
	.globl	_t_reg_scale_a
	.globl	_t_reg_scale_b
	.globl	_assembly_flag
	.globl	_in_a_hold
	.globl	_in_b_hold
	.globl	_out_a_hold
	.globl	_out_b_hold
	.globl	_in_error
	.globl	_in_error_stick
	.globl	_in_digital
	.globl	_out_gain
	.globl	_out_atten
	.globl	_iosr_copy
	.globl	_func_addr_a
	.globl	_func_addr_b
	.globl	_coef_ptr_a
	.globl	_coef_ptr_b
	.globl	_data_ptr_a
	.globl	_data_ptr_b
	.globl	_orderm2_a
	.globl	_orderm2_b
	.globl	_fir_coef
	.globl	_coefdata
	.globl	_port0
	.globl	_portffe8
	.globl	_portffec
	.globl	_portfff0
	.globl	_portfff1
	.globl	_portfff4
	.globl	_portfff5
	.globl	_portfff6
	.globl	_portfff7
	.globl	_portfff8
	.globl	_portfff9
	.globl	_portfffa
	.globl	_portfffc
	.globl	_no_func_a
	.globl	_no_func_b
	.globl	_allpass_func_a
	.globl	_allpass_func_b
	.globl	_iir_4_a
	.globl	_iir_4_b
	.globl	_fir_15_a
	.globl	_fir_15_b
	.globl	_fir_16_a
	.globl	_fir_16_b
	.globl	_lattice_2_a
	.globl	_lattice_2_b
	.globl	_lattice_4_a
	.globl	_lattice_4_b
	.globl	_lattice_8_a
	.globl	_lattice_8_b
	.globl	_notch_a
	.globl	_notch_b
	.globl	_get_serial
	.globl	_txrxint_c
	.globl	_inc_dec_param_ptr
	.globl	_inc_dec_mod
	.globl	_add_sub_pow
	.globl	_initialize
	.globl	_set_fsample
	.globl	_init_params
	.globl	_init_freq_params
	.globl	_reset_port
	.globl	_sign_on
	.globl	_reset_lcd
	.globl	_update_disp_left
	.globl	_update_disp_right
	.globl	_disp_text
	.globl	_disp_num
	.globl	_num2string
	.globl	_write_lcd_inst
	.globl	_write_lcd_data
	.globl	_write_lcd_nibble
	.globl	_delta_t
	.globl	_wait
	.globl	_wait_n_samples
	.globl	_prog_flash
	.globl	_read_flash
	.globl	_compute_fir
	.globl	_load_userfir
	.globl	_compute_notch
	.globl	_xmit
	.globl	_parse_command
	.globl	_update_dsp
	.globl	_set_all_gains
	.globl	_gain
	.globl	_vu_update
	.globl	_convert_to_vu
	.globl	_led_update
	.globl	_store
	.globl	_recall
	.globl	_beep
	.globl	_record_bad
	.globl	_store_all
	.text

	.sym	_main,_main,32,2,0
	.globl	_main

	.func	431
******************************************************
* FUNCTION DEF : _main
******************************************************
_main:
	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,6
	LAR	AR0,*0+

	.sym	_count_start,1,14,1,16
	.sym	_ptr,2,14,1,16
	.sym	_itemp,3,4,1,16
	.sym	_ftemp,4,6,1,32
	.line	13
	CALL	_initialize
	.line	23
	MAR	* ,AR2
	LARK	AR2,2
	MAR	*0+
	SPLK	#24576,* ,AR1
	.line	24
	LALK	_record+0
	SACL	*+
	LACK	5
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	_read_flash
	SBRK	3
	.line	25
	LACK	0
	MAR	* ,AR2
	LARK	AR2,3
	MAR	*0+
	SACL	* 
	.line	26
	LDPK	_record
	ZALS	_record
	RSXM
	SUBK	-1
	BNZ	L1
	.line	27
	CALL	_store_all,AR1
L1:
	.line	29
	LDPK	_record
	ZALS	_record
	RSXM
	SUBK	-1
	BZ	L3
	MAR	* ,AR1
L2:
	.line	31
	CALL	_record_bad,AR1
	BZ	L4
	.line	32
	CALL	_store_all
	.line	33
	B	L3
L4:
	.line	35
	MAR	* ,AR2
	LARK	AR2,2
	MAR	*0+
	BLKD	#_record,* ,AR1
	.line	36
	LALK	_record+0
	SACL	*+
	LACK	5
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	_read_flash
	SBRK	3
	.line	29
	LDPK	_record
	ZALS	_record
	RSXM
	SUBK	-1
	BNZ	L2
L3:
	.line	41
	LACK	0
	LDPK	_params+66
	SACL	_params+66
	SACH	_params+67
	.line	42
	LACK	3
	LDPK	_params_changed_copy
	SACL	_params_changed_copy
	.line	43
	CALL	_recall,AR1
	.line	60
	CALL	_sign_on
L5:
	.line	109
	LDPK	_cursor_flag
	LAC	_cursor_flag
	BZ	L7
	LAC	_sw_down
	BNZ	L7
	.line	111
	MAR	* ,AR1
	LACK	-1
	SACL	*+
	BLKD	_cursor_pos+0,*+
	LRLK	AR3,SL9
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
	.line	113
	LDPK	_auto_vu_count
	LAC	_auto_vu_count
	BZ	L10
	.line	114
	SSXM
	LAC	_auto_vu_count
	ADDK	1
	SACL	_auto_vu_count
	SUBK	1
	SUBK	10
	BLEZ	L10
	.line	115
	BLKD	#_cursor_pos,_cursor_pos_vu
	.line	117
	LACK	0
	SACL	*+
	LACK	1
	SACL	*+,AR3
	LRLK	AR3,_param_struct+3
	LAC	* ,AR1
	SACL	*+
	CALL	_disp_text
	SBRK	3
	.line	118
	LACK	0
	SACL	*+
	BLKD	_cursor_pos_vu+0,*+
	LRLK	AR3,SL9
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
	.line	120
	LACK	3
	LDPK	_assembly_flag
	OR	_assembly_flag
	SACL	_assembly_flag
	.line	121
	LDPK	_params+12
	BLDD	_params+12,#_auto_vu_count
	B	L10
L7:
	.line	125
	LAC	_press_flag
	BNZ	L10
	.line	127
	MAR	* ,AR1
	BLKD	_flash_cursor_flag+0,*+
	BLKD	_cursor_pos+0,*+
	LRLK	AR3,SL9
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
L10:
	.line	131
	LACK	0
	MAR	* ,AR2
	LARK	AR2,3
	MAR	*0+
	SACL	* 
	SSXM
	LAC	* 
	SUBK	50
	BGEZ	L13
	SBRK	2
L12:
	.line	132
	IN	* ,0fffah
	.line	133
	LAC	* ,AR1
	SACL	*+
	CALL	_delta_t
	MAR	*-
	SUBK	5000
	BGEZ	L15
L14:
	.line	135
	LDPK	_write_ptr
	LAC	_write_ptr
	SUB	_read_ptr
	BZ	L16
	.line	136
	CALL	_parse_command,AR1
	.line	137
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	IN	* ,0fffah
L16:
	.line	140
	LDPK	_serial_error_flag
	LAC	_serial_error_flag
	SUBK	1
	BNZ	L17
	.line	141
	LACK	0
	SACL	_serial_error_flag
	.line	144
	MAR	* ,AR1
	LACK	-1
	SACL	*+
	LACK	1
	SACL	*+
	LRLK	AR3,SL68
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
	.line	145
	LACK	0
	SACL	*+
	LACK	1
	SACL	*+
	LRLK	AR3,SL9
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
	.line	146
	LACK	400
	SACL	*+
	LACK	200
	SACL	*+
	CALL	_beep
	SBRK	2
	.line	149
	CALL	_reset_port
L17:
	.line	154
	LDPK	_params_changed
	LAC	_params_changed
	BZ	L18
	.line	155
	MAR	* ,AR0
	IN	* ,0fff5h
	LAC	* 
	ANDK	65023
	SACL	* 
	OUT	* ,0fff5h
	.line	156
	BLKD	#_params_changed,_params_changed_copy
	.line	157
	LACK	0
	SACL	_params_changed
	.line	158
	IN	* ,0fff5h
	LAC	* 
	ORK	512
	SACL	* 
	OUT	* ,0fff5h,AR1
	.line	159
	BLKD	_index_ab+0,*+
	BLKD	_param_ptr+0,*+
	CALL	_update_dsp
	SBRK	2
	.line	160
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	IN	* ,0fffah
L18:
	.line	133
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	LAC	* ,AR1
	SACL	*+
	CALL	_delta_t
	MAR	*-
	SUBK	5000
	BLZ	L14
L15:
	.line	167
	CALL	_led_update
	.line	169
	MAR	* ,AR0
	IN	* ,0fff5h
	LAC	* 
	ANDK	65023
	SACL	* 
	OUT	* ,0fff5h
	.line	170
	LACK	3
	LDPK	_assembly_flag
	AND	_assembly_flag
	BZ	L19
	.line	171
	CALL	_vu_update,AR1
L19:
	.line	173
	MAR	* ,AR0
	IN	* ,0fff5h
	LAC	* 
	ORK	512
	SACL	* 
	OUT	* ,0fff5h,AR1
	.line	175
	CALL	_rand
	.line	131
	MAR	* ,AR2
	LARK	AR2,3
	MAR	*0+
	LAC	* 
	ADDK	1
	SACL	* 
	SSXM
	LAC	* 
	SUBK	50
	SBRK	2
	BLZ	L12
L13:
	.line	184
	nop			
	.line	187
	B	L5
	.endfunc	619,000000000H,6

	.sym	_initialize,_initialize,32,2,0
	.globl	_initialize

	.func	629
******************************************************
* FUNCTION DEF : _initialize
******************************************************
_initialize:
	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,15
	LAR	AR0,*0+

	.line	9
	setc	INTM	; disable all interrupts	
	.line	10
	setc	SXM		; set sign extension bit (sign-extension on)
	.line	11
	clrc	CNF		; map internal memory block B0 into Data space
	.line	12
	clrc	OVM		; clear overflow mode bit => normal overflow (assumed by c)
	.line	13
	spm		0		; set product shift mode to No Shift (assumed by c)
	.sym	_i,1,4,1,16
	.sym	_j,2,4,1,16
	.sym	_nstart,3,4,1,16
	.sym	_n_offset,4,4,1,16
	.sym	_nlength,5,4,1,16
	.sym	_nfrac,6,4,1,16
	.sym	_temp,7,4,1,16
	.sym	_ctemp,8,2,1,16
	.sym	_flag_temp,9,14,1,16
	.sym	_cursor_temp,10,14,1,16
	.sym	_utemp,11,14,1,16
	.sym	_uptr,12,30,1,16
	.line	16
	LACK	0
	LDPK	_sw_down
	SACL	_sw_down
	SACL	_sw_pressed
	SACL	_sw_released
	.line	17
	SACL	_cw
	SACL	_ccw
	SACL	_testcount
	.line	18
	SACL	_down_turn_flag
	SACL	_press_flag
	.line	19
	SACL	_param_ptr
	SACL	_write_ptr
	SACL	_read_ptr
	SACL	_p_state
	SACL	_flag_options
	.line	20
	SACL	_serial_error_flag
	.line	21
	SACL	_led_counter
	SACL	_vu_counter
	.line	22
	SACL	_in_a_vu_level
	SACL	_in_b_vu_level
	SACL	_out_a_vu_level
	SACL	_out_b_vu_level
	.line	23
	SACL	_quietsn_flag
	.line	24
	SACL	_iorder_old
	.line	25
	LACK	1
	SACL	_flash_locked
	.line	26
	SACL	_flash_cursor_flag
	.line	27
	LACK	2
	SACL	_params_changed
	.line	28
	LACK	20
	SACL	_max_in_level
	.line	29
	LALK	32768,15
	SFL
	SACL	_min_value
	SACH	_min_value+1
	RSXM
	LALK	65535,15
	ORK	32767
	SACL	_max_value
	SACH	_max_value+1
	.line	31
	CALL	_init_params
	.line	32
	CALL	_set_fsample
	.line	33
	CALL	_init_freq_params
	.line	37
	LACK	4
	LDPK	_imr_ptr
	SACL	_imr_ptr
	.line	38
	LACK	5
	SACL	_greg_ptr
	.line	39
	LACK	6
	SACL	_ifr_ptr
	.line	41
	LAR	AR3,_imr_ptr
	LACK	0
	MAR	* ,AR3
	SACL	* ,AR4
	.line	42
	LAR	AR4,_ifr_ptr
	SPLK	#65535,* ,AR5
	.line	43
	LAR	AR5,_greg_ptr
	LACK	128
	SACL	* ,AR0
	.line	44
	LACK	1
	SACL	* 
	OUT	* ,0ffe8h
	.line	45
	LACK	28
	SACL	* 
	OUT	* ,0ffech
	.line	47
	LACK	512
	SACL	* 
	OUT	* ,0fffch
	.line	50
	LDPK	_k7f00h
	SPLK	#32512,_k7f00h
	.line	51
	LDPK	_kf80fh
	SPLK	#-2033,_kf80fh
	.line	52
	LDPK	_kfff0h
	SPLK	#-16,_kfff0h
	.line	54
	LACK	0
	LDPK	_out_gain
	SACL	_out_gain
	.line	56
	LDPK	_out_atten
	SACL	_out_atten
	.line	58
	LDPK	_out_b_hold
	SACL	_out_b_hold
	LDPK	_out_a_hold
	SACL	_out_a_hold
	LDPK	_in_b_hold
	SACL	_in_b_hold
	LDPK	_in_a_hold
	SACL	_in_a_hold
	.line	59
	LDPK	_assembly_flag
	SACL	_assembly_flag
	.line	62
	LACK	65535
	SACL	* 
	OUT	* ,0fff9h
	.line	63
	LACK	1020
	SACL	* 
	OUT	* ,0fff8h
	.line	64
	LACK	972
	SACL	* 
	OUT	* ,0fff8h,AR1
	.line	67
	LALK	_no_func_a+0
	LDPK	_func_addr_a
	SACL	_func_addr_a
	.line	68
	LALK	_no_func_b+0
	LDPK	_func_addr_b
	SACL	_func_addr_b
	.line	70
	LDPK	_coef_ptr_a
	SPLK	#896,_coef_ptr_a
	.line	71
	LDPK	_coef_ptr_b
	SPLK	#768,_coef_ptr_b
	.line	78
	LALK	_serial_str+0
	SACL	*+
	CALL	_get_serial
	MAR	*-,AR2
	.line	82
	LACK	0
	LDPK	_serial_number
	SACL	_serial_number
	SACH	_serial_number+1
	.line	83
	LARK	AR2,1
	MAR	*0+
	SACL	* 
	SSXM
	LAC	* 
	SUBK	6
	BGEZ	L21
	ADRK	12
	MAR	* ,AR1
L20:
	.line	84
	LACK	10
	SACL	*+
	SACH	*+
	SACL	*+
	LDPK	_serial_number
	ZALS	_serial_number
	ADDH	_serial_number+1
	MAR	*-
	CALL	L$$MPY
	MAR	* ,AR2
	SACL	*+
	SACH	* 
	SBRK	13
	LAC	* ,AR0
	ADLK	_serial_str+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LACK	15
	AND	* ,AR1
	SACL	* 
	SSXM
	LAC	* ,AR2
	ADRK	12
	ADDS	*+
	ADDH	* 
	SACL	_serial_number
	SACH	_serial_number+1
	.line	83
	SBRK	13
	LAC	* 
	ADDK	1
	SACL	* 
	LAC	* 
	SUBK	6
	ADRK	12
	MAR	* ,AR1
	BLZ	L20
L21:
	.line	88
	LACK	128
	SACL	_port0_copy
	.line	89
	MAR	* ,AR0
	SACL	* 
	OUT	* ,00h,AR1
	.line	90
	CALL	_reset_lcd
	.line	94
	CALL	_reset_port
	.line	97
	LDPK	_ifr_ptr
	LAR	AR3,_ifr_ptr
	MAR	* ,AR3
	SPLK	#65535,* ,AR0
	.line	98
	LACK	17152
	SACL	* 
	OUT	* ,0fff1h
	.line	99
	LACK	17200
	SACL	* 
	OUT	* ,0fff1h,AR4
	.line	100
	LAR	AR4,_imr_ptr
	LACK	40
	SACL	* 
	.line	102
	BLKD	#_params+12,_auto_vu_count
	.line	103
	LDPK	_params+36
	ZALS	_params+36
	ADDH	_params+37
	BZ	LL4
	LACK	0
	B	LL5
LL4:
	LACK	2
LL5:
	LDPK	_index_ab
	SACL	_index_ab
	.line	108
	LACK	0
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	SACL	* 
	ZALS	* 
	SUBK	46
	BGEZ	L23
	ADRK	2
L22:
	.line	110
	LACK	15
	SACL	* 
	.line	111
	LACK	0
	ADRK	3
	SACL	* 
	.line	112
	SBRK	2
	SACL	* 
	.line	113
	SBRK	2
	SACL	* 
	SSXM
	LAC	* 
	SUBK	16
	BGZ	L25
	MAR	*-
L24:
	.line	114
	LT	*+,AR0
	MPYK	3
	PAC
	ADLK	_param_struct+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR2
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR2
	ADRK	6
	SACL	* 
	.line	115
	SBRK	5
	LAC	* 
	SUBK	15
	BNZ	L26
	.line	116
	ADRK	5
	LAC	* 
	SUBK	35
	BZ	LL6
	LAC	* 
	SUBK	95
	BNZ	L27
LL6:
	.line	117
	SBRK	6
	LAC	*+
	SACL	* 
	.line	118
	SBRK	2
	LT	* ,AR0
	MPYK	3
	PAC
	ADLK	_param_struct+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	MAR	*+
	LACK	32767
	AND	* 
	SACL	* 
	B	L30
L27:
	.line	120
	LAC	* 
	BNZ	L30
	.line	121
	SBRK	6
	LAC	*+
	SACL	* 
	.line	122
	SBRK	2
	LT	* ,AR0
	MPYK	3
	PAC
	ADLK	_param_struct+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	MAR	*+
	LALK	1,15
	OR	* 
	SACL	* 
	.line	123
	B	L25
L26:
	.line	127
	ADRK	5
	LAC	* 
	SUBK	95
	BNZ	L31
	.line	128
	SBRK	6
	LAC	* 
	ADRK	2
	SACL	* 
L31:
	.line	130
	LARK	AR2,8
	MAR	*0+
	LAC	* 
	SUBK	46
	BNZ	L32
	.line	131
	SBRK	6
	LAC	* 
	ADRK	4
	SACL	* 
	B	L30
L32:
	.line	133
	LAC	* 
	SUBK	35
	BZ	L30
	LAC	* 
	SUBK	95
	BZ	L30
	LAC	* 
	SUBK	46
	BZ	L30
	.line	134
	SBRK	6
	LAC	*+
	SUB	* 
	ADRK	2
	SACL	* 
	.line	135
	B	L25
L30:
	.line	113
	MAR	* ,AR2
	LARK	AR2,2
	MAR	*0+
	LAC	* 
	ADDK	1
	SACL	* 
	SSXM
	LAC	*-
	SUBK	16
	BLEZ	L24
L25:
	.line	139
	MAR	* ,AR2
	LARK	AR2,4
	MAR	*0+
	LAC	* 
	BZ	L35
	.line	140
	MAR	*-
	SUB	*+
	SACL	* 
L35:
	.line	142
	ADRK	2
	LAC	* 
	BZ	L36
	.line	143
	SBRK	3
	LAC	* 
	SUBK	1
	ADRK	2
	ADD	*+
	SUB	* 
	SACL	* 
L36:
	.line	145
	SBRK	5
	LT	* ,AR0
	MPYK	3
	PAC
	ADLK	_param_struct+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	MAR	*+
	BIT	* ,0
	BBZ	L37
	.line	146
	MAR	* ,AR2
	MAR	*+
	SPLK	#-1,* 
L38:
	.line	147
	LAC	* 
	ADDK	1
	SACL	*-,AR1
	SACL	* ,AR2
	LT	* ,AR0
	MPYK	3
	PAC
	ADLK	_param_struct+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	ADRK	2
	LAC	* ,AR1
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	LAR	AR3,* 
	LAC	* ,AR2
	MAR	*+
	BNZ	L38
	.line	148
	MAR	*+
	LAC	* ,10,AR1
	SACL	*+,AR2
	SBRK	2
	LT	*+,AR0
	MPYK	3
	PAC
	ADLK	_param_struct+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LACK	49152
	MAR	*+
	AND	* ,AR1
	MAR	*-
	OR	* 
	SACL	* 
	LAC	* ,AR2
	OR	*-,AR1
	SACL	* ,AR2
	LT	* ,AR0
	MPYK	3
	PAC
	ADLK	_param_struct+0,0
	SACL	* 
	LAR	AR3,* ,AR1
	LAC	* ,AR3
	MAR	*+
	SACL	* 
	B	L39
L37:
	.line	151
	MAR	* ,AR2
	ADRK	2
	LAC	* ,10,AR1
	SACL	*+,AR2
	SBRK	2
	LT	* ,AR0
	MPYK	3
	PAC
	ADLK	_param_struct+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LACK	49152
	MAR	*+
	AND	* ,AR1
	MAR	*-
	OR	* 
	SACL	*+,AR2
	ADRK	3
	LAC	*+,7,AR1
	MAR	*-
	OR	* 
	SACL	* ,AR2
	LAC	*+,3,AR1
	OR	* 
	SACL	* 
	LAC	* ,AR2
	OR	* ,AR1
	SACL	*+,AR2
	SBRK	5
	LT	* ,AR0
	MPYK	3
	PAC
	ADLK	_param_struct+0,0
	SACL	* 
	LAR	AR3,* ,AR1
	MAR	*-
	LAC	* ,AR3
	MAR	*+
	SACL	* 
L39:
	.line	108
	MAR	* ,AR2
	LAC	* 
	ADDK	1
	SACL	* 
	ZALS	* 
	SUBK	46
	ADRK	2
	BLZ	L22
L23:
	.line	166
	CALL	_set_all_gains,AR1
	.line	169
	clrc	INTM		; Enable interrupts
	.line	170
	LACK	10
	SACL	*+
	CALL	_wait_n_samples
	MAR	*-,AR0
	.line	171
	LACK	0
	LDPK	_in_error_stick
	SACL	_in_error_stick
	.line	172
	IN	* ,0fff6h
	LAC	* ,AR1
	ANDK	3
	LDPK	_gray_code
	SACL	_gray_code
	.line	174
	LACK	32767
	AND	_serial_number
	SACL	* 
	LDPK	_in_a
	LAC	_in_a
	ADD	* 
	SACL	*+
	CALL	_srand
	MAR	*-
	.line	177
	CALL	_update_disp_left
	.line	178
	LACK	1
	SACL	*+
	CALL	_update_disp_right
	MAR	*-
EPI0_2:
	.line	180
	SBRK	16
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	808,000000000H,15

	.sym	_init_params,_init_params,32,2,0
	.globl	_init_params

	.func	817
******************************************************
* FUNCTION DEF : _init_params
******************************************************
_init_params:
	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,3
	LAR	AR0,*0+,AR2

	.sym	_j,1,4,1,16
	.sym	_uptr,2,30,1,16
	.line	7
	LALK	_params+0
	LARK	AR2,2
	MAR	*0+
	SACL	*-
	.line	8
	LACK	0
	SACL	* 
	ZALS	* 
	SUBK	1044
	BGEZ	L41
	MAR	*+
L40:
	.line	9
	LAR	AR3,* ,AR3
	LACK	0
	SACL	*+,AR2
	SAR	AR3,*-
	.line	8
	LAC	* 
	ADDK	1
	SACL	* 
	ZALS	*+
	SUBK	1044
	BLZ	L40
L41:
	.line	12
	LACK	1
	LDPK	_params+4
	SACL	_params+4
	SACH	_params+5
	SACL	_params+2
	SACH	_params+3
	SACL	_params
	SACH	_params+1
	.line	13
	LACK	20
	SACL	_params+18
	SACH	_params+19
	.line	14
	LACK	1
	SACL	_params+24
	SACH	_params+25
	.line	16
	LACK	100
	LDPK	_params+274
	SACL	_params+274
	SACH	_params+275
	SACL	_params+256
	SACH	_params+257
	SACL	_params+238
	SACH	_params+239
	LDPK	_params+220
	SACL	_params+220
	SACH	_params+221
	SACL	_params+202
	SACH	_params+203
	SACL	_params+166
	SACH	_params+167
	SACL	_params+130
	SACH	_params+131
	SACL	_params+112
	SACH	_params+113
	LDPK	_params+94
	SACL	_params+94
	SACH	_params+95
	LDPK	_params+272
	SACL	_params+272
	SACH	_params+273
	SACL	_params+254
	SACH	_params+255
	SACL	_params+236
	SACH	_params+237
	LDPK	_params+218
	SACL	_params+218
	SACH	_params+219
	SACL	_params+200
	SACH	_params+201
	SACL	_params+164
	SACH	_params+165
	SACL	_params+128
	SACH	_params+129
	SACL	_params+110
	SACH	_params+111
	LDPK	_params+92
	SACL	_params+92
	SACH	_params+93
	LDPK	_params+270
	SACL	_params+270
	SACH	_params+271
	SACL	_params+252
	SACH	_params+253
	SACL	_params+234
	SACH	_params+235
	LDPK	_params+216
	SACL	_params+216
	SACH	_params+217
	SACL	_params+198
	SACH	_params+199
	SACL	_params+162
	SACH	_params+163
	SACL	_params+126
	SACH	_params+127
	SACL	_params+108
	SACH	_params+109
	LDPK	_params+90
	SACL	_params+90
	SACH	_params+91
	.line	20
	LACK	127
	LDPK	_params+244
	SACL	_params+244
	SACH	_params+245
	LDPK	_params+196
	SACL	_params+196
	SACH	_params+197
	SACL	_params+160
	SACH	_params+161
	SACL	_params+124
	SACH	_params+125
	SACL	_params+106
	SACH	_params+107
	LDPK	_params+242
	SACL	_params+242
	SACH	_params+243
	LDPK	_params+194
	SACL	_params+194
	SACH	_params+195
	SACL	_params+158
	SACH	_params+159
	SACL	_params+122
	SACH	_params+123
	SACL	_params+104
	SACH	_params+105
	LDPK	_params+240
	SACL	_params+240
	SACH	_params+241
	LDPK	_params+192
	SACL	_params+192
	SACH	_params+193
	SACL	_params+156
	SACH	_params+157
	SACL	_params+120
	SACH	_params+121
	SACL	_params+102
	SACH	_params+103
	.line	24
	LACK	1
	LDPK	_params+250
	SACL	_params+250
	SACH	_params+251
	SACL	_params+248
	SACH	_params+249
	SACL	_params+246
	SACH	_params+247
EPI0_3:
	.line	25
	MAR	* ,AR1
	SBRK	4
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	841,000000000H,3

	.sym	_init_freq_params,_init_freq_params,32,2,0
	.globl	_init_freq_params

	.func	849
******************************************************
* FUNCTION DEF : _init_freq_params
******************************************************
_init_freq_params:
	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,1
	LAR	AR0,*0+

	.line	4
	LACK	1000
	LDPK	_params+226
	SACL	_params+226
	SACH	_params+227
	LDPK	_params+208
	SACL	_params+208
	SACH	_params+209
	SACL	_params+224
	SACH	_params+225
	SACL	_params+206
	SACH	_params+207
	SACL	_params+222
	SACH	_params+223
	SACL	_params+204
	SACH	_params+205
	SACL	_params+190
	SACH	_params+191
	SACL	_params+172
	SACH	_params+173
	SACL	_params+154
	SACH	_params+155
	SACL	_params+136
	SACH	_params+137
	SACL	_params+118
	SACH	_params+119
	SACL	_params+100
	SACH	_params+101
	SACL	_params+188
	SACH	_params+189
	SACL	_params+170
	SACH	_params+171
	SACL	_params+152
	SACH	_params+153
	SACL	_params+134
	SACH	_params+135
	SACL	_params+116
	SACH	_params+117
	SACL	_params+98
	SACH	_params+99
	SACL	_params+186
	SACH	_params+187
	SACL	_params+168
	SACH	_params+169
	SACL	_params+150
	SACH	_params+151
	SACL	_params+132
	SACH	_params+133
	SACL	_params+114
	SACH	_params+115
	LDPK	_params+96
	SACL	_params+96
	SACH	_params+97
	.line	12
	LACK	2000
	LDPK	_params+178
	SACL	_params+178
	SACH	_params+179
	SACL	_params+142
	SACH	_params+143
	SACL	_params+176
	SACH	_params+177
	SACL	_params+140
	SACH	_params+141
	SACL	_params+174
	SACH	_params+175
	SACL	_params+138
	SACH	_params+139
	.line	16
	LACK	1500
	SACL	_params+184
	SACH	_params+185
	SACL	_params+148
	SACH	_params+149
	SACL	_params+182
	SACH	_params+183
	SACL	_params+146
	SACH	_params+147
	SACL	_params+180
	SACH	_params+181
	SACL	_params+144
	SACH	_params+145
	.line	20
	LACK	1000
	LDPK	_params+232
	SACL	_params+232
	SACH	_params+233
	LDPK	_params+214
	SACL	_params+214
	SACH	_params+215
	LDPK	_params+230
	SACL	_params+230
	SACH	_params+231
	LDPK	_params+212
	SACL	_params+212
	SACH	_params+213
	LDPK	_params+228
	SACL	_params+228
	SACH	_params+229
	LDPK	_params+210
	SACL	_params+210
	SACH	_params+211
	.line	24
	LACK	10000
	LDPK	_params+262
	SACL	_params+262
	SACH	_params+263
	SACL	_params+260
	SACH	_params+261
	SACL	_params+258
	SACH	_params+259
EPI0_4:
	.line	28
	SBRK	2
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	876,000000000H,1

	.sym	_set_fsample,_set_fsample,32,2,0
	.globl	_set_fsample

	.func	884
******************************************************
* FUNCTION DEF : _set_fsample
******************************************************
_set_fsample:
	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,3
	LAR	AR0,*0+

	.sym	_ltemp,1,5,1,32
	.line	6
	LDPK	_params+24
	ZALS	_params+24
	ADDH	_params+25
	BZ	L42
	.line	7
 	clrc	XF		; Set the CODEC sampling rate to 48 Ksps
	.line	8
	LALK	FL0
	LRLK	AR3,_fsample+0
	MAR	* ,AR3
	RPTK	1
	TBLR	*+
	B	L43
L42:
	.line	11
		setc	XF		; Set the CODEC sampling rate to 8 Ksps
	.line	12
	LALK	FL1
	LRLK	AR3,_fsample+0
	MAR	* ,AR3
	RPTK	1
	TBLR	*+
L43:
EPI0_5:
	.line	18
	MAR	* ,AR1
	SBRK	4
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	901,000000000H,3

	.sym	_reset_port,_reset_port,32,2,0
	.globl	_reset_port

	.func	910
******************************************************
* FUNCTION DEF : _reset_port
******************************************************
_reset_port:
	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,1
	LAR	AR0,*0+,AR0

	.line	3
	LACK	0
	SACL	* 
	OUT	* ,0fff5h
	.line	4
	LACK	26352
	SACL	* 
	OUT	* ,0fff6h
	.line	5
	LACK	8192
	SACL	* 
	OUT	* ,0fff5h
	.line	6
	LACK	0
	SACL	* 
	OUT	* ,0fff5h
	.line	8
	LACK	8872
	SACL	* 
	OUT	* ,0fff5h
	.line	12
	LACK	26352
	SACL	* 
	OUT	* ,0fff6h,AR1
EPI0_6:
	.line	13
	SBRK	2
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	922,000000000H,1

	.sym	_sign_on,_sign_on,32,2,0
	.globl	_sign_on

	.func	930
******************************************************
* FUNCTION DEF : _sign_on
******************************************************
_sign_on:
	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,1
	LAR	AR0,*0+

	.line	6
	LACK	-1
	SACL	*+
	LACK	1
	SACL	*+
	LRLK	AR3,SL69
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
	.line	7
	LALK	30,15
	ORK	16960
	SACL	*+
	SACH	*+
	CALL	_wait
	SBRK	2
	.line	27
	LACK	0
	SACL	*+
	LACK	1
	SACL	*+
	LRLK	AR3,SL70
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
	.line	28
	LALK	3,15
	ORK	1696
	SACL	*+
	SACH	*+
	CALL	_wait
	SBRK	2
	.line	29
	LACK	0
	SACL	*+
	LACK	9
	SACL	*+
	LRLK	AR3,SL71
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
	.line	30
	LALK	3,15
	ORK	1696
	SACL	*+
	SACH	*+
	CALL	_wait
	SBRK	2
	.line	31
	LACK	0
	SACL	*+
	LACK	11
	SACL	*+
	LRLK	AR3,SL71
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
	.line	32
	LALK	3,15
	ORK	1696
	SACL	*+
	SACH	*+
	CALL	_wait
	SBRK	2
	.line	33
	LACK	0
	SACL	*+
	LACK	13
	SACL	*+
	LRLK	AR3,SL71
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
	.line	34
	LALK	3,15
	ORK	1696
	SACL	*+
	SACH	*+
	CALL	_wait
	SBRK	2
	.line	35
	LACK	0
	SACL	*+
	LACK	15
	SACL	*+
	LRLK	AR3,SL72
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
	.line	36
	LALK	12,15
	ORK	6784
	SACL	*+
	SACH	*+
	CALL	_wait
	SBRK	2
	.line	39
	CALL	_update_disp_left
	.line	40
	LACK	1
	SACL	*+
	CALL	_update_disp_right
	MAR	*-
EPI0_7:
	.line	58
	SBRK	2
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	987,000000000H,1

	.sym	_parse_command,_parse_command,32,2,0
	.globl	_parse_command

	.func	998
******************************************************
* FUNCTION DEF : _parse_command
******************************************************
_parse_command:
	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,39
	LAR	AR0,*0+,AR2

	.sym	_i,1,4,1,16
	.sym	_j,2,4,1,16
	.sym	_itemp,3,4,1,16
	.sym	_iflag,4,4,1,16
	.sym	_cursortemp,5,4,1,16
	.sym	_execute_flag,6,4,1,16
	.sym	_program_flag,7,4,1,16
	.sym	_prog_err_flag,8,4,1,16
	.sym	_param_ptr_temp,9,4,1,16
	.sym	_index_ab_temp,10,4,1,16
	.sym	_flag_options_temp,11,4,1,16
	.sym	_change_value_flag,12,4,1,16
	.sym	_param_value,13,5,1,32
	.sym	_value_ptr,15,18,1,16
	.sym	_cptr,16,18,1,16
	.sym	_ctemp,17,2,1,16
	.sym	_craw,18,2,1,16
	.sym	_carray,19,50,1,272,,17
	.sym	_utemp,36,14,1,16
	.line	3
	LACK	0
	LARK	AR2,6
	MAR	*0+
	SACL	*+
	SACL	*+
	SACL	* 
	.line	12
	SBRK	3
	BLKD	#_cursor_pos,* 
	.line	14
	LDPK	_write_ptr
	LAC	_write_ptr
	SUB	_read_ptr
	BZ	L45
	MAR	* ,AR0
L44:
	.line	15
	LDPK	_read_ptr
	LAC	_read_ptr
	ADDK	1
	SACL	_read_ptr
	SUBK	1
	ADLK	_serial_in_buf+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR2
	LARK	AR2,18
	MAR	*0+
	SACL	* 
	.line	16
	LAC	* ,AR1
	SACL	*+
	CALL	_tolower
	MAR	*-,AR2
	LARK	AR2,17
	MAR	*0+
	SACL	* 
	.line	17
	LACK	127
	LDPK	_read_ptr
	AND	_read_ptr
	SACL	_read_ptr
	B	L46
L47:
	.line	23
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	LAC	* 
	SUBK	97
	BNZ	L48
	LACK	1
	LDPK	_p_state
	SACL	_p_state
L48:
	.line	24
	LACK	0
	LDPK	_sn_ok_flag
	SACL	_sn_ok_flag
	.line	25
	B	L49
L50:
	.line	27
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	LAC	* 
	SUBK	116
	BNZ	L51
	LACK	2
	LDPK	_p_state
	SACL	_p_state
	B	L49
L51:
	.line	28
	LAC	* 
	SUBK	97
	BNZ	L53
	LACK	1
	LDPK	_p_state
	SACL	_p_state
	B	L49
L53:
	.line	29
	LACK	0
	LDPK	_p_state
	SACL	_p_state
	.line	30
	B	L49
L54:
	.line	32
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	LAC	* 
	SUBK	97
	BNZ	L55
	LACK	3
	LDPK	_p_state
	SACL	_p_state
	B	L49
L55:
	.line	33
	LAC	* 
	SUBK	115
	BNZ	L57
	LACK	5
	LDPK	_p_state
	SACL	_p_state
	B	L49
L57:
	.line	34
	LAC	* 
	SUBK	32
	BNZ	L58
	LACK	2
	LDPK	_p_state
	SACL	_p_state
	B	L49
L58:
	.line	35
	LACK	0
	LDPK	_p_state
	SACL	_p_state
	.line	36
	B	L49
L59:
	.line	38
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	LAC	* 
	SUBK	116
	BNZ	L60
	LACK	2
	LDPK	_p_state
	SACL	_p_state
	B	L49
L60:
	.line	39
	LAC	* 
	SUBK	108
	BNZ	L62
	LACK	4
	LDPK	_p_state
	SACL	_p_state
	B	L49
L62:
	.line	40
	LACK	0
	LDPK	_p_state
	SACL	_p_state
	.line	41
	B	L49
L63:
	.line	43
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	LAC	* 
	SUBK	108
	BNZ	L64
	.line	44
	LACK	1
	LDPK	_sn_ok_flag
	SACL	_sn_ok_flag
	.line	45
	LACK	20
	SACL	_p_state
	B	L49
L64:
	.line	47
	LACK	0
	LDPK	_p_state
	SACL	_p_state
	.line	48
	B	L49
L66:
	.line	50
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	LAC	* 
	SUBK	110
	BNZ	L67
	LACK	6
	LDPK	_p_state
	SACL	_p_state
	B	L49
L67:
	.line	51
	LACK	0
	LDPK	_p_state
	SACL	_p_state
	.line	52
	B	L49
L69:
	.line	54
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	LAC	* 
	SUBK	58
	BNZ	L70
	LACK	7
	LDPK	_p_state
	SACL	_p_state
	B	L49
L70:
	.line	55
	LACK	0
	LDPK	_p_state
	SACL	_p_state
	.line	56
	B	L49
L72:
	.line	58
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	LAC	* ,AR1
	SACL	*+
	CALL	_isdigit
	MAR	*-
	BZ	L73
	.line	59
	LACK	8
	LDPK	_p_state
	SACL	_p_state
	.line	60
	LACK	15
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	AND	* ,AR1
	SACL	* 
	SSXM
	LAC	* 
	SACL	_p_long
	SACH	_p_long+1
	B	L49
L73:
	.line	62
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	LAC	* 
	SUBK	32
	BNZ	L75
	LACK	7
	LDPK	_p_state
	SACL	_p_state
	B	L49
L75:
	.line	63
	LACK	0
	LDPK	_p_state
	SACL	_p_state
	.line	64
	B	L49
L76:
	.line	66
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	LAC	* ,AR1
	SACL	*+
	CALL	_isdigit
	MAR	*-
	BZ	L77
	.line	67
	LACK	10
	SACL	*+
	SACH	*+
	SACL	*+
	LDPK	_p_long
	ZALS	_p_long
	ADDH	_p_long+1
	MAR	*-
	CALL	L$$MPY
	MAR	* ,AR2
	LARK	AR2,37
	MAR	*0+
	SACL	*+
	SACH	* 
	LACK	15
	SBRK	21
	AND	* ,AR1
	SACL	* 
	SSXM
	LAC	* ,AR2
	ADRK	20
	ADDS	*+
	ADDH	* 
	SACL	_p_long
	SACH	_p_long+1
	.line	68
	LACK	8
	SACL	_p_state
	B	L49
L77:
	.line	70
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	LAC	* 
	SUBK	32
	BNZ	L79
	LACK	8
	LDPK	_p_state
	SACL	_p_state
	B	L49
L79:
	.line	71
	LAC	* 
	SUBK	44
	BNZ	L80
	.line	72
	LDPK	_serial_number
	LT	_serial_number
	MPYK	1
	LPH	_serial_number+1
	ZALS	_p_long
	ADDH	_p_long+1
	SPAC
	BNZ	L81
	.line	73
	LACK	1
	SACL	_sn_ok_flag
	.line	74
	LACK	20
	SACL	_p_state
	B	L49
L81:
	.line	77
	LACK	0
	SACL	_p_long
	SACH	_p_long+1
	.line	78
	LACK	8
	SACL	_p_state
	B	L49
L80:
	.line	83
	SSXM
	LDPK	_read_ptr
	LAC	_read_ptr
	SUBK	1
	SACL	_read_ptr
	ADDK	1
	BNZ	L83
	.line	84
	LACK	127
	SACL	_read_ptr
L83:
	.line	86
	LT	_serial_number
	MPYK	1
	LPH	_serial_number+1
	ZALS	_p_long
	ADDH	_p_long+1
	SPAC
	BNZ	L84
	.line	87
	LACK	1
	SACL	_sn_ok_flag
	B	L85
L84:
	.line	90
	LACK	0
	SACL	_sn_ok_flag
L85:
	.line	92
	LACK	20
	SACL	_p_state
	.line	94
	B	L49
L86:
	.line	97
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	LAC	* ,AR1
	SACL	*+
	CALL	_isalpha
	MAR	*-
	BZ	L87
	.line	98
	LACK	0
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	SACL	* 
	SSXM
	LAC	* 
	SUBK	17
	BGEZ	L89
L88:
	.line	99
	LAC	* ,AR0
	ADLK	_value_str+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LACK	0
	SACL	* ,AR1
	SACL	* ,AR2
	LAC	* ,AR0
	ADLK	_parameter_str+0,0
	SACL	* 
	LAR	AR3,* ,AR1
	LAC	* ,AR3
	SACL	* ,AR2
	.line	98
	LAC	* 
	ADDK	1
	SACL	* 
	LAC	* 
	SUBK	17
	BLZ	L88
L89:
	.line	101
	ADRK	16
	BLDD	* ,#_parameter_str
	.line	102
	LACK	21
	LDPK	_p_state
	SACL	_p_state
	B	L49
L87:
	.line	104
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	LAC	* 
	SUBK	13
	BNZ	L91
	LACK	0
	LDPK	_p_state
	SACL	_p_state
	B	L49
L91:
	.line	105
	LACK	20
	LDPK	_p_state
	SACL	_p_state
	.line	106
	B	L49
L92:
	.line	108
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	LAC	* 
	SUBK	13
	BNZ	L93
	.line	109
	LACK	1
	SBRK	11
	SACL	* 
	.line	110
	LACK	0
	LDPK	_p_state
	SACL	_p_state
	B	L49
L93:
	.line	112
	LAC	* 
	SUBK	58
	BNZ	L95
	.line	113
	LACK	22
	LDPK	_p_state
	SACL	_p_state
	.line	114
	MAR	* ,AR1
	LACK	7
	SACL	*+
	LRLK	AR3,SL73
	SAR	AR3,*+
	RSXM
	LALK	_parameter_str+0
	SACL	*+
	CALL	_strncmp
	SBRK	3
	BNZ	L49
	.line	115
	LALK	_no_func_a+0
	LDPK	_func_addr_a
	SACL	_func_addr_a
	.line	116
	LALK	_no_func_b+0
	LDPK	_func_addr_b
	SACL	_func_addr_b
	.line	117
	LACK	-4
	LDPK	_assembly_flag
	AND	_assembly_flag
	SACL	_assembly_flag
	.line	119
	LACK	40
	LDPK	_p_state
	SACL	_p_state
	B	L49
L95:
	.line	123
	LALK	_parameter_str+0
	MAR	* ,AR1
	SACL	*+
	CALL	_strlen
	MAR	*-,AR2
	LARK	AR2,1
	MAR	*0+
	SACL	* 
	.line	124
	SSXM
	LAC	* 
	SUBK	16
	BGEZ	L97
	.line	125
	ADLK	_parameter_str+16,0
	MAR	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR2
	ADRK	16
	LAC	* ,AR3
	SACL	* 
	B	L49
L97:
	.line	128
	LACK	0
	LDPK	_p_state
	SACL	_p_state
	.line	131
	B	L49
L99:
	.line	134
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	LAC	* 
	SUBK	13
	BNZ	L100
	.line	135
	LACK	1
	SBRK	11
	SACL	* 
	.line	136
	LACK	0
	LDPK	_p_state
	SACL	_p_state
	B	L49
L100:
	.line	138
	LAC	* 
	SUBK	58
	BNZ	L102
	.line	139
	LDPK	_func_addr_a
	BLDD	_func_addr_a,#_func_addr_temp_a
	.line	140
	LDPK	_func_addr_b
	BLDD	_func_addr_b,#_func_addr_temp_b
	.line	141
	LALK	_no_func_a+0
	LDPK	_func_addr_a
	SACL	_func_addr_a
	.line	142
	LALK	_no_func_b+0
	LDPK	_func_addr_b
	SACL	_func_addr_b
	.line	143
	LACK	0
	LDPK	_data_count
	SACL	_data_count
	.line	144
	LACK	30
	SACL	_p_state
	B	L49
L102:
	.line	147
	LALK	_value_str+0
	MAR	* ,AR1
	SACL	*+
	CALL	_strlen
	MAR	*-,AR2
	LARK	AR2,1
	MAR	*0+
	SACL	* 
	.line	148
	SSXM
	LAC	* 
	SUBK	16
	BGEZ	L103
	.line	149
	ADLK	_value_str+16,0
	MAR	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR2
	ADRK	17
	LAC	* ,AR3
	SACL	* 
	B	L49
L103:
	.line	152
	LACK	0
	LDPK	_p_state
	SACL	_p_state
	.line	155
	B	L49
L105:
	.line	159
	LACK	31
	LDPK	_p_state
	SACL	_p_state
	.line	160
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	LAC	* 
	SUBK	32
	BNZ	L106
	LACK	30
	SACL	_p_state
	B	L112
L106:
	.line	161
	LAC	* 
	SUBK	45
	BNZ	L108
	.line	162
	SPLK	#-1,_sign_mult
	.line	163
	LACK	0
	SACL	_p_long
	SACH	_p_long+1
	B	L112
L108:
	.line	165
	LAC	* ,AR1
	SACL	*+
	CALL	_isdigit
	MAR	*-
	BZ	L109
	.line	166
	LACK	1
	LDPK	_sign_mult
	SACL	_sign_mult
	.line	167
	LACK	15
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	AND	* ,AR1
	SACL	* 
	SSXM
	LAC	* 
	SACL	_p_long
	SACH	_p_long+1
	B	L112
L109:
	.line	169
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	LAC	* 
	SUBK	13
	BZ	L111

	.sym	_p_cont1,L111,0,6,0
	.line	170
	LACK	0
	LDPK	_p_state
	SACL	_p_state
	B	L112
L113:
	.line	173
	LACK	0
	LDPK	_index_ab_p
	SACL	_index_ab_p
	.line	174
	B	L49
L115:
	.line	176
	LACK	1
	LDPK	_index_ab_p
	SACL	_index_ab_p
	.line	177
	B	L49
L116:
	.line	179
	LACK	2
	LDPK	_index_ab_p
	SACL	_index_ab_p
	.line	180
	B	L49
L117:
	.line	182
	LACK	0
	LDPK	_p_state
	SACL	_p_state
	.line	183
	B	L49
L112:
	.line	171
	SSXM
	LDPK	_parameter_str
	LAC	_parameter_str
	SUBK	97
	BZ	L113
	SUBK	1
	BZ	L115
	SUBK	4
	BZ	L116
	B	L117
	.line	185
L118:
	.line	187
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	LAC	* ,AR1
	SACL	*+
	CALL	_isdigit
	MAR	*-
	BZ	L119
	.line	188
	LACK	10
	SACL	*+
	SACH	*+
	SACL	*+
	LDPK	_p_long
	ZALS	_p_long
	ADDH	_p_long+1
	MAR	*-
	CALL	L$$MPY
	MAR	* ,AR2
	LARK	AR2,37
	MAR	*0+
	SACL	*+
	SACH	* 
	LACK	15
	SBRK	21
	AND	* ,AR1
	SACL	* 
	SSXM
	LAC	* ,AR2
	ADRK	20
	ADDS	*+
	ADDH	* 
	SACL	_p_long
	SACH	_p_long+1
	B	L120
L119:
	.line	190
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	LAC	* 
	SUBK	32
	BZ	LL14
	LAC	* 
	SUBK	13
	BNZ	L121
LL14:
	.line	191
	LACK	30
	LDPK	_p_state
	SACL	_p_state
	.line	192
	SSXM
	LAC	_sign_mult
	MAR	* ,AR1
	SACL	*+
	SACH	*+
	SACL	*+
	ZALS	_p_long
	ADDH	_p_long+1
	MAR	*-
	CALL	L$$MPY
	SACL	_p_long
	SACH	_p_long+1
	.line	193
	BNV	$+2
	LT	_p_long
	MPYK	1
	LPH	_p_long+1
	LACK	32767
	SPAC
	BNV	$+6
	BLZ	L122
	B	$+4
	BGEZ	L122
	.line	194
	LACK	32767
	SACL	_p_long
	SACH	_p_long+1
L122:
	.line	196
	BNV	$+2
	LT	_p_long
	MPYK	1
	LPH	_p_long+1
	SSXM
	LALK	65535,15
	SPAC
	BNV	$+6
	BGZ	L123
	B	$+4
	BLEZ	L123
	.line	197
	LALK	65535,15
	SACL	_p_long
	SACH	_p_long+1
L123:
	.line	199
	RSXM
	LAC	_data_count,14
	ADLK	46,15
	MAR	* ,AR2
	ADRK	19
	SACH	* ,1
	.line	200
	ZALS	_data_count
	ADDK	1
	SACL	_data_count
	SUBK	1
	ANDK	1
	ANDK	0FFFFh
	BZ	L124
	.line	201
	ZALH	_p_long
	MAR	*+
	SACL	*+
	SACH	* ,AR1
	LAC	_index_ab_p,1
	SACL	*+,AR2
	SBRK	2
	LT	*+,AR1
	MPYK	6
	PAC
	ADLK	_params+0,0
	MAR	*-
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	ZALS	* ,AR2
	OR	*+,AR3
	SACL	*+
	ZALS	* ,AR2
	OR	* ,AR3
	SACL	*-
	B	L125
L124:
	.line	204
	LAC	_index_ab_p,1
	MAR	* ,AR1
	SACL	* ,AR2
	LT	* ,AR1
	MPYK	6
	PAC
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	ZALS	_p_long
	ANDK	65535
	SACL	*+
	ZAC
	SACL	*-
L125:
	.line	206
	LDPK	_params+36
	LT	_params+36
	MPYK	1
	LPH	_params+37
	LACK	2
	SPAC
	BNZ	L126
	.line	207
	MAR	* ,AR2
	LARK	AR2,3
	MAR	*0+
	SPLK	#256,* 
	B	L127
L126:
	.line	210
	LACK	128
	MAR	* ,AR2
	LARK	AR2,3
	MAR	*0+
	SACL	* 
L127:
	.line	212
	LDPK	_data_count
	ZALS	_data_count
	SUBS	* 
	BLEZ	L120
	.line	213
	BLDD	* ,#_data_count
	.line	214
	B	L111
L121:
	.line	217
	LACK	0
	LDPK	_p_state
	SACL	_p_state
L120:
	.line	218
	LARK	AR2,17
	MAR	*0+
	LAC	* 
	SUBK	13
	BNZ	L49
L111:
	.line	220
	LDPK	_data_count
	ZALS	_data_count
	SUBK	3
	BGEZ	L130
	LACK	3
	SACL	_data_count
L130:
	.line	221
	LAC	_index_ab_p,1
	ADLK	_params+240,0
	MAR	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	ZALS	_data_count
	SACL	*+
	SACH	*-,AR0
	.line	222
	LAC	_index_ab_p,1
	ADLK	_params+246,0
	SACL	* 
	LAR	AR3,* ,AR3
	LACK	1
	SACL	*+
	SACH	*-,AR2
	.line	223
	BLDD	_func_addr_temp_a,#_func_addr_a
	.line	224
	BLDD	_func_addr_temp_b,#_func_addr_b
	.line	225
	LARK	AR2,6
	MAR	*0+
	SACL	* 
	.line	226
	LACK	0
	SACL	_p_state
	.line	228
	B	L49
L131:
	.line	232
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	LAC	* 
	SUBK	32
	BNZ	L132
	LACK	40
	LDPK	_p_state
	SACL	_p_state
	B	L49
L132:
	.line	233
	LAC	* 
	SUBK	115
	BNZ	L134
	LACK	41
	LDPK	_p_state
	SACL	_p_state
	B	L49
L134:
	.line	234
	LACK	0
	LDPK	_p_state
	SACL	_p_state
	.line	235
	B	L49
L135:
	.line	237
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	LAC	* 
	SUBK	32
	BNZ	L136
	LACK	41
	LDPK	_p_state
	SACL	_p_state
	B	L49
L136:
	.line	238
	LAC	* 
	SUBK	58
	BNZ	L138
	LACK	42
	LDPK	_p_state
	SACL	_p_state
	B	L49
L138:
	.line	239
	LACK	0
	LDPK	_p_state
	SACL	_p_state
	.line	240
	B	L49
L139:
	.line	242
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	LAC	* ,AR1
	SACL	*+
	CALL	_isdigit
	MAR	*-
	BZ	L140
	.line	243
	LACK	15
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	AND	* 
	LDPK	_p_uint1
	SACL	_p_uint1
	.line	244
	LACK	43
	SACL	_p_state
	B	L49
L140:
	.line	246
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	LAC	* 
	SUBK	32
	BNZ	L142
	LACK	42
	LDPK	_p_state
	SACL	_p_state
	B	L49
L142:
	.line	247
	LACK	0
	LDPK	_p_state
	SACL	_p_state
	.line	248
	B	L49
L143:
	.line	250
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	LAC	* ,AR1
	SACL	*+
	CALL	_isdigit
	MAR	*-
	BZ	L144
	.line	251
	LDPK	_p_uint1
	LT	_p_uint1
	MPYK	10
	LACK	15
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	AND	* 
	APAC
	SACL	_p_uint1
	.line	252
	LACK	43
	SACL	_p_state
	B	L49
L144:
	.line	254
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	LAC	* 
	SUBK	32
	BNZ	L146
	LACK	43
	LDPK	_p_state
	SACL	_p_state
	B	L49
L146:
	.line	255
	LAC	* 
	SUBK	108
	BNZ	L147
	LACK	44
	LDPK	_p_state
	SACL	_p_state
	B	L49
L147:
	.line	256
	LACK	0
	LDPK	_p_state
	SACL	_p_state
	.line	257
	B	L49
L148:
	.line	259
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	LAC	* 
	SUBK	32
	BNZ	L149
	LACK	44
	LDPK	_p_state
	SACL	_p_state
	B	L49
L149:
	.line	260
	LAC	* 
	SUBK	58
	BNZ	L151
	LACK	45
	LDPK	_p_state
	SACL	_p_state
	B	L49
L151:
	.line	261
	LACK	0
	LDPK	_p_state
	SACL	_p_state
	.line	262
	B	L49
L152:
	.line	264
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	LAC	* ,AR1
	SACL	*+
	CALL	_isdigit
	MAR	*-
	BZ	L153
	.line	265
	LACK	15
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	AND	* 
	LDPK	_p_uint2
	SACL	_p_uint2
	.line	266
	LACK	46
	SACL	_p_state
	B	L49
L153:
	.line	268
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	LAC	* 
	SUBK	32
	BNZ	L155
	LACK	45
	LDPK	_p_state
	SACL	_p_state
	B	L49
L155:
	.line	269
	LACK	0
	LDPK	_p_state
	SACL	_p_state
	.line	270
	B	L49
L156:
	.line	272
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	LAC	* ,AR1
	SACL	*+
	CALL	_isdigit
	MAR	*-
	BZ	L157
	.line	273
	LDPK	_p_uint2
	LT	_p_uint2
	MPYK	10
	LACK	15
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	AND	* 
	APAC
	SACL	_p_uint2
	.line	274
	LACK	46
	SACL	_p_state
	B	L49
L157:
	.line	276
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	LAC	* 
	SUBK	32
	BNZ	L159
	LACK	46
	LDPK	_p_state
	SACL	_p_state
	B	L49
L159:
	.line	277
	LAC	* 
	SUBK	100
	BNZ	L160
	LACK	47
	LDPK	_p_state
	SACL	_p_state
	B	L49
L160:
	.line	278
	LACK	0
	LDPK	_p_state
	SACL	_p_state
	.line	279
	B	L49
L161:
	.line	281
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	LAC	* 
	SUBK	32
	BNZ	L162
	LACK	47
	LDPK	_p_state
	SACL	_p_state
	B	L49
L162:
	.line	282
	LAC	* 
	SUBK	58
	BNZ	L164
	.line	285
	LDPK	_p_uint1
	LAC	_p_uint1
	BZ	LL15
	LAC	_p_uint1
	SUBK	8192
	BZ	LL15
	LAC	_p_uint1
	SUBK	16384
	BZ	LL15
	LAC	_p_uint1
	SUBK	24576
	BZ	LL15
	ZALS	_p_uint1
	RSXM
	SUBK	-32768
	BZ	LL15
	ZALS	_p_uint1
	SUBK	-24576
	BZ	LL15
	ZALS	_p_uint1
	SUBK	-16384
	BZ	LL15
	ZALS	_p_uint1
	SUBK	-8192
	BNZ	L165
LL15:
	ZALS	_p_uint2
	SUBK	8192
	BGZ	L165
	LAC	_p_uint2
	BZ	L165
	.line	288
	LAC	_sn_ok_flag
	BZ	L166
	.line	289
	MAR	* ,AR1
	LACK	-1
	SACL	*+
	LACK	1
	SACL	*+
	LRLK	AR3,SL74
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
L166:
	.line	291
	LACK	0
	LDPK	_data_count
	SACL	_data_count
	.line	292
	SACL	_check_sum
	SACH	_check_sum+1
	.line	293
	LACK	48
	SACL	_p_state
	B	L49
L165:
	.line	296
	LACK	1
	SBRK	9
	SACL	* 
	.line	297
	LACK	0
	SACL	_p_state
	B	L49
L164:
	.line	300
	LACK	0
	LDPK	_p_state
	SACL	_p_state
	.line	301
	B	L49
L168:
	.line	303
	LDPK	_p_uint2
	LAC	_p_uint2,1
	ANDK	0FFFFh
	SUBS	_data_count
	BLEZ	L169
	.line	304
	RSXM
	LAC	_data_count,14
	MAR	* ,AR2
	LARK	AR2,36
	MAR	*0+
	SACH	* ,1
	.line	305
	BIT	_data_count,15
	BBZ	L170
	.line	306
	LAC	* ,AR0
	ADLK	_flash_data+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR2
	SBRK	18
	OR	* ,AR1
	SACL	*+,AR2
	ADRK	18
	LAC	* ,AR0
	ADLK	_flash_data+0,0
	SACL	* 
	LAR	AR3,* ,AR1
	MAR	*-
	LAC	* ,AR3
	SACL	* 
	.line	307
	LAC	_sn_ok_flag
	BZ	L173
	.line	308
	MAR	* ,AR2
	ZALS	* 
	ADDK	1
	ANDK	47
	ANDK	0FFFFh
	BNZ	L173
	.line	309
	MAR	* ,AR1
	LACK	0
	SACL	*+
	LACK	5
	SACL	*+
	LACK	6
	SACL	*+,AR2
	LACK	1
	ADDS	* ,AR1
	SACL	*+
	SACH	*+
	CALL	_disp_num
	SBRK	5
	B	L173
L170:
	.line	314
	SBRK	18
	LAC	* ,8,AR1
	SACL	*+,AR2
	ADRK	18
	LAC	* ,AR0
	ADLK	_flash_data+0,0
	SACL	* 
	LAR	AR3,* ,AR1
	MAR	*-
	LAC	* ,AR3
	SACL	* 
L173:
	.line	316
	LDPK	_check_sum
	ZALS	_check_sum
	ADDH	_check_sum+1
	SSXM
	MAR	* ,AR2
	LARK	AR2,18
	MAR	*0+
	ADD	* 
	SACL	_check_sum
	SACH	_check_sum+1
	.line	317
	LAC	_data_count
	ADDK	1
	SACL	_data_count
L169:
	.line	319
	LAC	_p_uint2,1
	ANDK	0FFFFh
	SUBS	_data_count
	BNZ	L49
	.line	320
	LAC	_sn_ok_flag
	BZ	L175
	.line	321
	MAR	* ,AR1
	LACK	0
	SACL	*+
	LACK	5
	SACL	*+
	LACK	6
	SACL	*+,AR0
	ZALS	_data_count
	SUBK	1
	SACL	* 
	RSXM
	LAC	* ,14
	SACH	* ,1
	ZALS	* ,AR1
	ADDK	1
	SACL	*+
	SACH	*+
	CALL	_disp_num
	SBRK	5
	.line	322
	LALK	30,15
	ORK	16960
	SACL	*+
	SACH	*+
	CALL	_wait
	SBRK	2
L175:
	.line	324
	LACK	0
	LDPK	_data_count
	SACL	_data_count
	.line	325
	SACL	_p_sum
	SACH	_p_sum+1
	.line	326
	LACK	49
	SACL	_p_state
	.line	328
	B	L49
L176:
	.line	330
	LDPK	_data_count
	ZALS	_data_count
	SUBK	8
	BGEZ	L177
	.line	331
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	LAC	* ,AR1
	SACL	*+
	CALL	_isdigit
	MAR	*-
	BZ	L178
	.line	332
	LACK	10
	SACL	*+
	SACH	*+
	SACL	*+
	LDPK	_p_sum
	ZALS	_p_sum
	ADDH	_p_sum+1
	MAR	*-
	CALL	L$$MPY
	MAR	* ,AR2
	LARK	AR2,37
	MAR	*0+
	SACL	*+
	SACH	* 
	LACK	15
	SBRK	21
	AND	* ,AR1
	SACL	* 
	SSXM
	LAC	* ,AR2
	ADRK	20
	ADDS	*+
	ADDH	* 
	SACL	_p_sum
	SACH	_p_sum+1
	B	L177
L178:
	.line	334
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	LAC	* 
	SUBK	32
	BNZ	L180
	LACK	49
	LDPK	_p_state
	SACL	_p_state
	B	L177
L180:
	.line	336
	LACK	1
	SBRK	9
	SACL	* 
	.line	337
	LACK	0
	LDPK	_p_state
	SACL	_p_state
L177:
	.line	340
	ZALS	_data_count
	ADDK	1
	SACL	_data_count
	SUBK	1
	ANDK	0FFFFh
	SUBK	7
	BNZ	L49
	.line	341
	LT	_check_sum
	MPYK	1
	LPH	_check_sum+1
	ZALS	_p_sum
	ADDH	_p_sum+1
	SPAC
	BNZ	L182
	.line	342
	LACK	1
	MAR	* ,AR2
	LARK	AR2,7
	MAR	*0+
	SACL	* 
	B	L183
L182:
	.line	345
	LACK	1
	MAR	* ,AR2
	LARK	AR2,8
	MAR	*0+
	SACL	* 
L183:
	.line	347
	LACK	0
	SACL	_p_state
	.line	349
	B	L49
	.line	352
L46:
	.line	21
	SSXM
	LAC	_p_state
	BZ	L47
	SUBK	1
	BZ	L50
	SUBK	1
	BZ	L54
	SUBK	1
	BZ	L59
	SUBK	1
	BZ	L63
	SUBK	1
	BZ	L66
	SUBK	1
	BZ	L69
	SUBK	1
	BZ	L72
	SUBK	1
	BZ	L76
	SUBK	12
	BZ	L86
	SUBK	1
	BZ	L92
	SUBK	1
	BZ	L99
	SUBK	8
	BZ	L105
	SUBK	1
	BZ	L118
	SUBK	9
	BZ	L131
	SUBK	1
	BZ	L135
	SUBK	1
	BZ	L139
	SUBK	1
	BZ	L143
	SUBK	1
	BZ	L148
	SUBK	1
	BZ	L152
	SUBK	1
	BZ	L156
	SUBK	1
	BZ	L161
	SUBK	1
	BZ	L168
	SUBK	1
	BZ	L176
L49:
	.line	364
	LDPK	_sn_ok_flag
	LAC	_sn_ok_flag
	BZ	L185
	MAR	* ,AR2
	LARK	AR2,6
	MAR	*0+
	LAC	* 
	BZ	L185
	.line	365
	MAR	* ,AR1
	LACK	5
	SACL	*+
	LRLK	AR3,SL75
	SAR	AR3,*+
	RSXM
	LALK	_parameter_str+0
	SACL	*+
	CALL	_strncmp
	SBRK	3
	BNZ	L186
	.line	366
	CALL	_initialize
	B	L195
L186:
	.line	369
	LACK	7
	SACL	*+
	LRLK	AR3,SL76
	SAR	AR3,*+
	RSXM
	LALK	_parameter_str+0
	SACL	*+
	CALL	_strncmp
	SBRK	3
	BNZ	L188
	.line	370
	LDPK	_value_str
	LAC	_value_str
	BZ	L189
	.line	371
	LACK	-1
	SACL	*+
	LACK	1
	SACL	*+
	LRLK	AR3,SL77
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
	.line	372
	LACK	-1
	SACL	*+
	LACK	1
	SACL	*+
	LALK	_value_str+0
	SACL	*+
	CALL	_disp_text
	SBRK	3
	.line	373
	LACK	0
	LDPK	_flash_cursor_flag
	SACL	_flash_cursor_flag
	B	L195
L189:
	.line	378
	CALL	_update_disp_left
	.line	379
	LACK	1
	SACL	*+
	CALL	_update_disp_right
	MAR	*-
	.line	380
	LACK	1
	LDPK	_params_changed
	SACL	_params_changed
	.line	381
	SACL	_flash_cursor_flag
	B	L195
L188:
	.line	384
	LDPK	_quietsn_flag
	LAC	_quietsn_flag
	BNZ	L191
	LACK	6
	SACL	*+
	LRLK	AR3,SL78
	SAR	AR3,*+
	RSXM
	LALK	_parameter_str+0
	SACL	*+
	CALL	_strncmp
	SBRK	3
	BNZ	L191
	.line	385
	MAR	* ,AR0
	IN	* ,0fff5h
	LAC	* 
	ANDK	65407
	SACL	* 
	OUT	* ,0fff5h,AR1
	.line	388
	CALL	_rand
	SACL	*+
	SACH	*+
	SACL	*+
	LACK	22
	MAR	*-
	CALL	L$$MPY
	SACL	*+
	SACH	*+
	CALL	_wait
	SBRK	2
	.line	389
	LALK	_serial_str+0
	SACL	*+
	CALL	_xmit
	MAR	*-
	B	L195
L191:
	.line	391
	LACK	7
	SACL	*+
	LRLK	AR3,SL79
	SAR	AR3,*+
	RSXM
	LALK	_parameter_str+0
	SACL	*+
	CALL	_strncmp
	SBRK	3
	BNZ	L192
	.line	392
	LACK	1
	LDPK	_quietsn_flag
	SACL	_quietsn_flag
	B	L195
L192:
	.line	394
	LACK	4
	SACL	*+
	LRLK	AR3,SL80
	SAR	AR3,*+
	RSXM
	LALK	_parameter_str+0
	SACL	*+
	CALL	_strncmp
	SBRK	3
	BNZ	L193
	.line	395
	LACK	-1
	SACL	*+
	LACK	1
	SACL	*+
	LRLK	AR3,SL77
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
	.line	396
	LACK	-1
	SACL	*+
	LACK	1
	SACL	*+
	LALK	_value_str+0
	SACL	*+
	CALL	_disp_text
	SBRK	3
	.line	397
	LACK	0
	LDPK	_flash_cursor_flag
	SACL	_flash_cursor_flag
	.line	398
	LALK	_value_str+0
	SACL	*+
	CALL	_xmit
	MAR	*-
	B	L195
L193:
	.line	403
	MAR	* ,AR3
	LARK	AR3,3
	MAR	*0+,AR1
	SAR	AR3,*+
	CALL	_param_struct_search
	MAR	*-,AR2
	LARK	AR2,9
	MAR	*0+
	SACL	* 
	.line	404
	SBRK	6
	LAC	* 
	BZ	L195

	.sym	_parse_continue,L195,0,6,0
	.line	405
	.line	407
	LACK	0
	ADRK	8
	SACL	* 
	.line	408
	SBRK	8
	LAC	* 
	SUBK	3
	BNZ	L196
	.line	409
	LACK	2
	ADRK	7
	SACL	* 
	B	L197
L196:
	.line	411
	LAC	* 
	SUBK	1
	BNZ	L198
	.line	412
	LACK	0
	ADRK	7
	SACL	*+
	.line	413
	LACK	1
	SACL	* 
	B	L197
L198:
	.line	415
	LAC	* 
	SUBK	2
	BNZ	L197
	.line	416
	LDPK	_parameter_str
	LAC	_parameter_str
	SUBK	97
	BNZ	L200
	.line	417
	LACK	0
	ADRK	7
	SACL	* 
	B	L197
L200:
	.line	419
	LAC	_parameter_str
	SUBK	98
	BNZ	L195
	.line	420
	LACK	1
	ADRK	7
	SACL	* 
	.line	423
L197:
	.line	429
	LARK	AR2,11
	MAR	*0+
	LAC	* 
	BNZ	L203
	.line	430
	MAR	*-
	LAC	* ,1,AR0
	ADLK	_params+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR2
	SBRK	7
	SACL	* 
	.line	432
	LDPK	_params+36
	ZALS	_params+36
	ADDH	_params+37
	BNZ	LL17
	ADRK	7
	LAC	* 
	SUBK	2
	BNZ	L195
LL17:
	ZALS	_params+36
	ADDH	_params+37
	BZ	L204
	LARK	AR2,10
	MAR	*0+
	LAC	* 
	SUBK	2
	BZ	L195
	.line	434
L204:
	.line	437
	LARK	AR2,9
	MAR	*0+
	LAC	* 
	BZ	L203
	SBRK	6
	LAC	* ,AR0
	ADLK	_param_ptr_start+0,0
	SACL	* 
	LAR	AR3,* ,AR2
	SSXM
	ADRK	6
	LAC	* ,AR3
	SUB	* 
	BLZ	L195
	MAR	* ,AR2
	SBRK	6
	LAC	* ,AR0
	ADLK	_param_ptr_end+0,0
	SACL	* 
	LAR	AR3,* ,AR2
	ADRK	6
	LAC	* ,AR3
	SUB	* 
	BGZ	L195
	.line	438
L203:
	.line	442
	LALK	_value_str+0
	MAR	* ,AR2
	LARK	AR2,15
	MAR	*0+
	SACL	* 
	.line	443
	LAR	AR3,* ,AR3
	LAC	* 
	SUBK	32
	BNZ	L207
	MAR	* ,AR2
L206:
	LAR	AR3,* ,AR3
	MAR	*+,AR2
	SAR	AR3,* ,AR3
	LAC	* ,AR2
	SUBK	32
	BZ	L206
L207:
	.line	444
	LACK	0
	MAR	* ,AR2
	SBRK	2
	SACL	*+
	SACH	* ,AR3
	.line	445
	LAC	* 
	LARK	AR4,1
	BNZ	LL19
	LARK	AR4,0
LL19:
	MAR	* ,AR2
	SBRK	2
	SAR	AR4,* 
	.line	446
	SBRK	3
	LT	* ,AR0
	MPYK	3
	PAC
	ADLK	_param_struct+0,0
	SACL	* 
	LAR	AR5,* ,AR5
	MAR	*+
	LAC	* ,AR2
	ADRK	27
	SACL	* 
	.line	447
	RSXM
	LAC	* ,0
	ANDK	0FFFFh,15
	BZ	L208
	.line	448
	LACK	255
	AND	* 
	SBRK	33
	SACL	* 
	.line	452
	LACK	0
	SBRK	2
	SACL	* 
	SSXM
	LAC	* 
	ADRK	2
	SUB	* 
	BGEZ	L210
	ADRK	10
L209:
	.line	453
	LACK	-1
	SACL	*+
	SACH	* 
	.line	454
	SBRK	5
	LT	* ,AR0
	MPYK	3
	PAC
	ADLK	_param_struct+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	ADRK	2
	LAC	* ,AR2
	SBRK	8
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR2
	ADRK	15
	SACL	* 
	.line	455
	LAR	AR3,* ,AR3
	LAC	* 
	SUBK	32
	BNZ	L212
	MAR	* ,AR2
L211:
	LAR	AR3,* ,AR3
	MAR	*+,AR2
	SAR	AR3,* ,AR3
	LAC	* ,AR2
	SUBK	32
	BZ	L211
L212:
	.line	456
	MAR	* ,AR2
	MAR	*-
	LAC	* ,AR1
	SACL	*+
	SAR	AR3,*+
	CALL	_string_compare
	SBRK	2
	BZ	L213
	.line	457
	SSXM
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	LAC	* 
	ADRK	12
	SACL	*+
	SACH	* 
	.line	458
	B	L210
L213:
	.line	452
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	LAC	* 
	ADDK	1
	SACL	* 
	SSXM
	LAC	* 
	ADRK	2
	SUB	* 
	ADRK	10
	BLZ	L209
L210:
	.line	461
	BNV	$+2
	LARK	AR2,13
	MAR	*0+
	LT	*+
	MPYK	1
	LPH	* 
	LACK	0
	SPAC
	BNV	$+6
	BLZ	L195
	B	$+4
	BGZ	L195
	.line	462
	B	L215
L208:
	.line	467
	MAR	* ,AR3
	LAC	* 
	SUBK	45
	LARK	AR5,1
	BZ	LL20
	LARK	AR5,0
LL20:
	MAR	* ,AR2
	SBRK	32
	SAR	AR5,* ,AR5
	BANZ	$+4,*
	B	L216
	MAR	* ,AR3
	MAR	*+,AR2
	ADRK	11
	SAR	AR3,* 
L216:
	.line	469
	LACK	7
	MAR	* ,AR2
	LARK	AR2,36
	MAR	*0+
	AND	* 
	SBRK	33
	SACL	* 
	.line	470
	LACK	0
	SBRK	2
	SACL	* 
	ADRK	14
L217:
	.line	472
	LAR	AR3,* ,AR3
	LAC	*+,AR2
	SAR	AR3,* 
	ADRK	2
	SACL	* 
	.line	473
	LAC	* ,AR1
	SACL	*+
	CALL	_isdigit
	MAR	*-,AR2
	LARK	AR2,2
	MAR	*0+
	SACL	*-
	.line	474
	LAC	* 
	BNZ	L218
	MAR	*+
	LAC	* 
	BNZ	L218
	.line	475
	LACK	1
	MAR	*-
	SACL	* 
	B	L219
L218:
	.line	478
	LARK	AR2,2
	MAR	*0+
	LAC	* 
	BZ	L220
	.line	479
	LACK	10
	MAR	* ,AR1
	SACL	*+
	SACH	*+
	SACL	*+,AR2
	ADRK	11
	ZALS	*+
	ADDH	* ,AR1
	MAR	*-
	CALL	L$$MPY
	MAR	* ,AR2
	ADRK	23
	SACL	*+
	SACH	* 
	LACK	15
	SBRK	21
	AND	* ,AR1
	SACL	* 
	SSXM
	LAC	* ,AR2
	ADRK	20
	ADDS	*+
	ADDH	* 
	SBRK	25
	SACL	*+
	SACH	* 
	B	L221
L220:
	.line	482
	LACK	10
	MAR	* ,AR1
	SACL	*+
	SACH	*+
	SACL	*+,AR2
	ADRK	11
	ZALS	*+
	ADDH	*-,AR1
	MAR	*-
	CALL	L$$MPY
	MAR	* ,AR2
	SACL	*+
	SACH	* 
L221:
	.line	484
	SBRK	13
	LAC	* 
	BZ	L219
	ADDK	1
	SACL	* 
L219:
	.line	486
	SSXM
	LAC	* 
	ADRK	2
	SUB	* 
	ADRK	12
	BLEZ	L217
	.line	488
	SBRK	11
	LAC	* 
	BZ	L223
	.line	489
	ADRK	9
	ZALS	*+
	ADDH	*-
	NEG
	SACL	*+
	SACH	* 
L223:
	.line	493
	LACK	1
	LDPK	_params_changed_copy
	SACL	_params_changed_copy
	.line	494
	MAR	* ,AR1
	BLKD	_index_ab+0,*+,AR2
	LARK	AR2,9
	MAR	*0+
	LAC	* ,AR1
	SACL	*+
	CALL	_update_dsp
	SBRK	2
	.line	495
	BNV	$+2
	LDPK	_max_value
	LT	_max_value
	MPYK	1
	LPH	_max_value+1
	MAR	* ,AR2
	LARK	AR2,13
	MAR	*0+
	ZALS	*+
	ADDH	* 
	SPAC
	BNV	$+6
	BGZ	L224
	B	$+4
	BLEZ	L224
	.line	496
	MAR	*-
	RPTK	1
	BLKD	#_max_value,*+
L224:
	.line	498
	BNV	$+2
	LT	_min_value
	MPYK	1
	LPH	_min_value+1
	LARK	AR2,13
	MAR	*0+
	ZALS	*+
	ADDH	* 
	SPAC
	BNV	$+6
	BLZ	L215
	B	$+4
	BGEZ	L215
	.line	499
	MAR	*-
	RPTK	1
	BLKD	#_min_value,*+
L215:
	.line	503
	LARK	AR2,11
	MAR	*0+
	BLDD	* ,#_flag_options
	.line	504
	SBRK	2
	BLDD	*+,#_param_ptr
	.line	505
	BLDD	* ,#_index_ab
	.line	508
	ADRK	2
	LAC	* 
	BZ	L226
	.line	509
	LDPK	_index_ab
	LAC	_index_ab,1
	MAR	*+,AR1
	SACL	* 
	LT	_param_ptr
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR2
	ZALS	*+
	ADDH	* ,AR3
	SACL	*+
	SACH	*-
	.line	510
	LACK	3
	SACL	_params_changed
	B	L227
L226:
	.line	513
	LDPK	_index_ab
	LAC	_index_ab,1
	MAR	*+,AR1
	SACL	* 
	LT	_param_ptr
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	ZALS	*+
	ADDH	*-,AR2
	SACL	*+
	SACH	* 
	.line	514
	ADRK	22
	LAC	* ,0
	ANDK	0FFFFh,15
	BZ	L228
	.line	515
	SBRK	27
	LAC	* 
	SUBK	1
	BZ	L233
	.line	521
	LAC	* 
	SUBK	12
	BNZ	L231
	.line	522
	MAR	* ,AR3
	LARK	AR3,19
	MAR	*0+,AR1
	SAR	AR3,*+,AR4
	LARK	AR4,1
	MAR	*0+,AR1
	SAR	AR4,*+
	LACK	2
	SACL	*+
	LACK	220
	SACL	*+
	SACH	*+
	CALL	_num2string
	SBRK	5
	SACL	*+
	CALL	_xmit
	MAR	*-
	B	L233
L231:
	.line	524
	LAC	* 
	SUBK	13
	BNZ	L232
	.line	525
	MAR	* ,AR3
	LARK	AR3,19
	MAR	*0+,AR1
	SAR	AR3,*+,AR4
	LARK	AR4,1
	MAR	*0+,AR1
	SAR	AR4,*+
	LACK	0
	SACL	*+
	ZALS	_serial_number
	ADDH	_serial_number+1
	SACL	*+
	SACH	*+
	CALL	_num2string
	SBRK	5
	SACL	*+
	CALL	_xmit
	MAR	*-
	B	L233
L232:
	.line	529
	LT	* ,AR0
	MPYK	3
	PAC
	ADLK	_param_struct+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	ADRK	2
	LAC	* ,AR2
	ADRK	4
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR1
	SACL	*+
	CALL	_xmit
	MAR	*-
	B	L233
L228:
	.line	533
	LACK	7
	AND	* 
	SBRK	33
	SACL	* ,AR3
	.line	535
	LARK	AR3,19
	MAR	*0+,AR1
	SAR	AR3,*+,AR4
	LARK	AR4,1
	MAR	*0+,AR1
	SAR	AR4,*+
	SACL	*+,AR2
	ADRK	10
	ZALS	*+
	ADDH	* ,AR1
	SACL	*+
	SACH	*+
	CALL	_num2string
	SBRK	5
	SACL	*+
	CALL	_xmit
	MAR	*-
L233:
	.line	537
	LACK	1
	LDPK	_params_changed
	SACL	_params_changed
L227:
	.line	539
	CALL	_update_disp_left,AR1
	.line	540
	LACK	1
	SACL	*+
	CALL	_update_disp_right
	MAR	*-
	.line	541
	LACK	1
	LDPK	_flash_cursor_flag
	SACL	_flash_cursor_flag
L195:
	.line	548
	LACK	0
	MAR	* ,AR2
	LARK	AR2,6
	MAR	*0+
	SACL	* 
L185:
	.line	553
	LDPK	_sn_ok_flag
	LAC	_sn_ok_flag
	BZ	L234
	MAR	* ,AR2
	LARK	AR2,8
	MAR	*0+
	LAC	* 
	BZ	L234
	.line	554
	MAR	* ,AR1
	LACK	-1
	SACL	*+
	LACK	1
	SACL	*+
	LRLK	AR3,SL81
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
	.line	555
	LALK	30,15
	ORK	16960
	SACL	*+
	SACH	*+
	CALL	_wait
	SBRK	2
	.line	556
	LACK	-1
	SACL	*+
	LACK	1
	SACL	*+
	LRLK	AR3,SL82
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
	.line	557
	LALK	61,15
	ORK	1152
	SACL	*+
	SACH	*+
	CALL	_wait
	SBRK	2
	.line	558
	CALL	_update_disp_left
	.line	559
	LACK	1
	SACL	*+
	CALL	_update_disp_right
	MAR	*-,AR2
	.line	560
	LACK	2
	LDPK	_params_changed
	SACL	_params_changed
	.line	561
	LACK	0
	LARK	AR2,8
	MAR	*0+
	SACL	* 
L234:
	.line	566
	LAC	_sn_ok_flag
	BZ	L235
	MAR	* ,AR2
	LARK	AR2,7
	MAR	*0+
	LAC	* 
	BZ	L235
	.line	575
	LAC	_p_uint1
	BZ	LL21
	ZALS	_p_uint1
	ADRK	30
	SACL	*+
	SACH	*-
	LT	*+
	MPYK	1
	LPH	* 
	LALK	1,15
	SPAC
	BNZ	L236
LL21:
	.line	576
	LACK	0
	LARK	AR2,2
	MAR	*0+
	SACL	*+
	.line	577
	SACL	* 
	.line	578
	LACK	66
	SBRK	2
	SACL	* 
	SSXM
	LAC	* 
	SUBK	71
	BGEZ	L238
L237:
	.line	579
	LAC	* ,AR0
	ADLK	_flash_data+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR2
	ADRK	35
	SACL	* 
	.line	580
	RSXM
	LACK	65280
	AND	* ,AR0
	SACL	* 
	LAC	* ,7,AR1
	SACH	*+,1,AR2
	SBRK	34
	LAC	* 
	ADDK	1
	SACL	* ,AR0
	SUBK	1
	SACL	* 
	LAR	AR3,* ,AR3
	MAR	*0+,AR1
	MAR	*-
	LAC	* ,AR3
	ADRK	19
	SACL	* ,AR2
	.line	581
	LACK	255
	ADRK	34
	AND	* ,AR1
	SACL	*+,AR2
	SBRK	34
	LAC	* 
	ADDK	1
	SACL	* ,AR0
	SUBK	1
	SACL	* 
	LAR	AR3,* ,AR3
	MAR	*0+,AR1
	MAR	*-
	LAC	* ,AR3
	ADRK	19
	SACL	* ,AR2
	.line	582
	ADRK	34
	LAC	* 
	SUBK	12336
	LARK	AR3,1
	BNZ	LL22
	LARK	AR3,0
LL22:
	SBRK	33
	LAC	* ,AR0
	SAR	AR3,*
	ADD	* ,AR2
	SACL	* 
	.line	578
	SBRK	2
	LAC	* 
	ADDK	1
	SACL	* 
	SSXM
	LAC	* 
	SUBK	71
	BLZ	L237
L238:
	.line	585
	ADRK	2
	LAC	* 
	BZ	LL23
	MAR	* ,AR3
	LARK	AR3,19
	MAR	*0+,AR1
	SAR	AR3,*+
	CALL	_valid_serial
	MAR	*-
	BNZ	L236
LL23:
	.line	586
	LACK	0
	MAR	* ,AR2
	LARK	AR2,2
	MAR	*0+
	SACL	*-
	.line	587
	LACK	66
	SACL	* 
	SSXM
	LAC	* 
	SUBK	71
	BGEZ	L236
	MAR	*+
L240:
	.line	588
	LAC	* 
	ADDK	1
	SACL	*+,AR0
	SUBK	1
	ADLK	_serial_str+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,8,AR2
	SACL	*-
	.line	589
	LAC	* 
	ADDK	1
	SACL	*+,AR0
	SUBK	1
	ADLK	_serial_str+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR2
	OR	* ,AR1
	SACL	*+,AR2
	SBRK	2
	LAC	* ,AR0
	ADLK	_flash_data+0,0
	SACL	* 
	LAR	AR3,* ,AR1
	MAR	*-
	LAC	* ,AR3
	SACL	* ,AR2
	.line	587
	LAC	* 
	ADDK	1
	SACL	* 
	LAC	*+
	SUBK	71
	BLZ	L240
L236:
	.line	597
	MAR	* ,AR1
	LACK	-1
	SACL	*+
	LACK	1
	SACL	*+
	LRLK	AR3,SL83
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
	.line	598
	LACK	0
	LDPK	_flash_locked
	SACL	_flash_locked
	.line	599
	LRLK	AR3,SL84
	SAR	AR3,*+
	LACK	1
	SACL	*+
	LALK	_flash_data+0
	SACL	*+
	BLKD	_p_uint2+0,*+
	BLKD	_p_uint1+0,*+
	CALL	_prog_flash
	SBRK	5
	MAR	* ,AR2
	LARK	AR2,3
	MAR	*0+
	SACL	* 
	.line	600
	LAC	* 
	BNZ	L235
	.line	601
	MAR	* ,AR1
	LACK	-1
	SACL	*+
	LACK	1
	SACL	*+
	LRLK	AR3,SL85
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
	.line	602
	LACK	0
	SACL	*+
	LACK	2
	SACL	*+
	LACK	7
	SACL	*+
	RSXM
	LDPK	_p_uint1
	LAC	_p_uint1,2
	SACH	* ,1
	ZALS	* 
	SACL	*+
	SACH	*+
	CALL	_disp_num
	SBRK	5
	.line	609
	LACK	0
	MAR	* ,AR2
	LARK	AR2,7
	MAR	*0+
	SACL	* 
L235:
	.line	14
	LDPK	_write_ptr
	LAC	_write_ptr
	SUB	_read_ptr
	MAR	* ,AR0
	BNZ	L44
L45:
	.line	617
	LAC	_cursor_pos
	MAR	* ,AR2
	LARK	AR2,5
	MAR	*0+
	SUB	* 
	BZ	L243
	.line	618
	MAR	* ,AR1
	LACK	0
	SACL	*+
	LACK	1
	SACL	*+
	LRLK	AR3,SL9
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
L243:
EPI0_8:
	.line	623
	MAR	* ,AR1
	SBRK	40
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	1620,000000000H,39

	.sym	_txrxint_c,_txrxint_c,32,2,0
	.globl	_txrxint_c

	.func	1633
******************************************************
* FUNCTION DEF : _txrxint_c
******************************************************
_txrxint_c:
	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,26
	LAR	AR0,*0+

	.sym	_i,1,4,1,16
	.sym	_j,2,4,1,16
	.sym	_temp_pos,3,4,1,16
	.sym	_type,4,4,1,16
	.sym	_nstart,5,4,1,16
	.sym	_n_offset,6,4,1,16
	.sym	_nlength,7,4,1,16
	.sym	_nfrac,8,4,1,16
	.sym	_nlabel,9,4,1,16
	.sym	_io,10,4,1,16
	.sym	_sum2,11,4,1,16
	.sym	_io0_sum,12,4,1,16
	.sym	_io1_sum,13,4,1,16
	.sym	_io2_sum,14,4,1,16
	.sym	_count_start,15,14,1,16
	.sym	_jj,16,5,1,32
	.sym	_carray,18,50,1,128,,8
	.line	11
	LDPK	_iosr_copy
	BIT	_iosr_copy,1
	BBZ	L244
	.line	12
	MAR	* ,AR0
	IN	* ,0fff5h
	LAC	* 
	ANDK	65503
	SACL	* 
	OUT	* ,0fff5h
	.line	13
	LACK	16384
	SACL	* 
	OUT	* ,0fff6h
L244:
	.line	23
	BIT	_iosr_copy,7
	BBZ	L245
	.line	24
	LACK	-4
	LDPK	_assembly_flag
	AND	_assembly_flag
	SACL	_assembly_flag
	.line	29
	MAR	* ,AR0
	IN	* ,0fff4h
	LAC	* ,AR1
	SACL	* ,AR0
	LDPK	_write_ptr
	LAC	_write_ptr
	ADDK	1
	SACL	_write_ptr
	SUBK	1
	ADLK	_serial_in_buf+0,0
	SACL	* 
	LAR	AR3,* ,AR1
	LAC	* ,AR3
	SACL	* 
	.line	31
	LACK	127
	AND	_write_ptr
	SACL	_write_ptr
	.line	33
	BLKD	#_params+12,_auto_vu_count
	.line	36
	LAC	_write_ptr
	SUB	_read_ptr
	BZ	LL26
	MAR	* ,AR0
	IN	* ,0fff6h
	ZALS	* 
	ANDK	9728
	BZ	EPI0_9
LL26:
	.line	39
	LACK	1
	SACL	_serial_error_flag
	.line	42
	B	EPI0_9
L245:
	.line	49
	LACK	240
	AND	_iosr_copy
	BZ	EPI0_9
	.line	51
	LACK	0
	MAR	* ,AR2
	LARK	AR2,11
	MAR	*0+
	SACL	*+
	SACL	*+
	SACL	*+
	SACL	* 
	.line	53
	SBRK	4
	BLKD	#_iosr_copy,* 
	.line	54
	ADRK	5
	IN	* ,0fffah
	.line	55
	LAC	* ,AR1
	SACL	*+
	CALL	_delta_t
	MAR	*-
	SUBK	500
	BGEZ	L249
	MAR	* ,AR2
L248:
	.line	57
	LACK	1
	LARK	AR2,10
	MAR	*0+
	AND	* 
	ADRK	2
	ADD	* 
	SACL	* 
	.line	58
	SSXM
	SBRK	2
	LAC	* ,14
	ANDK	1,15
	ADRK	3
	ADD	* ,15
	SACH	* ,1
	.line	59
	SBRK	3
	LAC	* ,13
	ANDK	1,15
	ADRK	4
	ADD	* ,15
	SACH	* ,1,AR0
	.line	60
	IN	* ,0fff6h
	LAC	* ,AR2
	SBRK	4
	SACL	*+
	.line	61
	LAC	* 
	ADDK	1
	SACL	* 
	.line	55
	ADRK	4
	LAC	* ,AR1
	SACL	*+
	CALL	_delta_t
	MAR	*-,AR2
	SUBK	500
	BLZ	L248
L249:
	.line	63
	SSXM
	MAR	* ,AR2
	LARK	AR2,11
	MAR	*0+
	LAC	* ,14
	SACH	* ,1
	.line	66
	LDPK	_sw_down
	BLDD	_sw_down,#_sw_down_old
	.line	67
	ADRK	3
	LAC	* 
	SBRK	3
	SUB	* 
	LARK	AR3,1
	BLZ	LL27
	LARK	AR3,0
LL27:
	SAR	AR3,_sw_down
	.line	68
	LAC	_sw_down
	BZ	L250
	LAC	_sw_down_old
	BNZ	L250
	.line	69
	LACK	1
	SACL	_sw_pressed
L250:
	.line	71
	LAC	_sw_down
	BNZ	L251
	LAC	_sw_down_old
	BZ	L251
	.line	72
	LACK	1
	SACL	_sw_released
L251:
	.line	97
	BLKD	#_gray_code,_gray_code_old
	.line	98
	MAR	*+
	LAC	*-
	SUB	* 
	LARK	AR4,1
	BGZ	LL28
	LARK	AR4,0
LL28:
	ADRK	2
	LAC	* 
	SBRK	2
	SUB	* 
	LARK	AR5,1
	BGZ	LL29
	LARK	AR5,0
LL29:
	MAR	* ,AR0
	SAR	AR5,*
	LAC	* ,1
	SAR	AR4,*
	OR	* 
	SACL	_gray_code
	B	L252
L254:
	.line	102
	LACK	1
	LDPK	_cw
	SACL	_cw
	.line	103
	B	L255
L257:
	.line	106
	LACK	1
	LDPK	_ccw
	SACL	_ccw
	.line	107
	B	L255
L252:
	.line	99
	LAC	_gray_code_old,2
	OR	_gray_code
	SACL	* 
	LAC	* 
	SUBK	4
	BZ	L257
	SUBK	3
	BZ	L254
	SUBK	1
	BZ	L254
	SUBK	3
	BZ	L257
L255:
	.line	114
	LAC	_sw_pressed
	BNZ	LL31
	LAC	_cw
	BNZ	LL31
	LAC	_ccw
	BZ	L258
LL31:
	.line	115
	LACK	64
	OR	_port0_copy
	SACL	_port0_copy
	.line	116
	MAR	* ,AR0
	SACL	* 
	OUT	* ,00h
	.line	117
	BLKD	#_params+12,_auto_vu_count
L258:
	.line	122
	LACK	1
	SACL	_flash_cursor_flag
	.line	124
	LT	_param_ptr
	MPYK	3
	PAC
	ADLK	_param_struct+0,0
	MAR	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	RSXM
	MAR	*+
	LAC	* ,5,AR2
	ANDK	15,15
	ADLK	1,15
	LARK	AR2,5
	MAR	*0+
	SACH	*-,1,AR0
	.line	125
	LT	_param_ptr
	MPYK	3
	PAC
	ADLK	_param_struct+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	MAR	*+
	LAC	* ,0,AR2
	SACH	* ,1
	.line	126
	LAC	* 
	BZ	L259
	.line	127
	LACK	0
	ADRK	2
	SACL	*+
	.line	128
	LACK	1
	SACL	*+
	.line	129
	LACK	0
	SACL	* 
	B	L260
L259:
	.line	132
	LT	_param_ptr
	MPYK	3
	PAC
	ADLK	_param_struct+0,0
	MAR	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	MAR	*+
	LAC	* ,8,AR2
	ANDK	7,15
	ADRK	2
	SACH	*+,1,AR0
	.line	133
	LT	_param_ptr
	MPYK	3
	PAC
	ADLK	_param_struct+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	MAR	*+
	LAC	* ,12,AR2
	ANDK	15,15
	SACH	*+,1,AR0
	.line	134
	LT	_param_ptr
	MPYK	3
	PAC
	ADLK	_param_struct+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LACK	7
	MAR	*+
	AND	* ,AR2
	SACL	* 
L260:
	.line	137
	LAC	_sw_pressed
	BZ	L261
	LAC	_cursor_pos
	SUBK	1
	BZ	L261
	LT	_param_ptr
	MPYK	3
	PAC
	ADLK	_param_struct+0,0
	MAR	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	MAR	*+
	BIT	* ,1
	BBZ	L261
	.line	138
	LACK	1
	SACL	_press_flag
	.line	139
	LACK	-4
	LDPK	_assembly_flag
	AND	_assembly_flag
	SACL	_assembly_flag
	.line	140
	MAR	* ,AR1
	LACK	-1
	SACL	*+
	LACK	1
	SACL	*+
	LRLK	AR3,SL86
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
	.line	142
	LACK	0
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	SACL	* 
	SSXM
	LAC	* 
	SUBK	7
	BGEZ	L263
	MAR	* ,AR0
L262:
	.line	143
	LDPK	_param_ptr
	LT	_param_ptr
	MPYK	3
	PAC
	ADLK	_param_struct+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR2
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR2
	LAR	AR4,* ,AR4
	MAR	*0+,AR3
	LAC	* ,AR4
	ADRK	18
	SACL	* ,AR2
	.line	144
	LAR	AR3,* ,AR3
	MAR	*0+
	ADRK	18
	LAC	* 
	SUBK	58
	BZ	L263
	.line	145
	.line	142
	MAR	* ,AR2
	LAC	* 
	ADDK	1
	SACL	* 
	SSXM
	LAC	* ,AR0
	SUBK	7
	BLZ	L262
L263:
	.line	148
	MAR	* ,AR2
	LAR	AR3,* ,AR3
	MAR	*0+
	LACK	0
	ADRK	18
	SACL	* ,AR1
	.line	149
	SACL	*+
	LACK	10
	SACL	*+,AR3
	LARK	AR3,18
	MAR	*0+,AR1
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
	.line	150
	LACK	0
	SACL	*+
	LACK	1
	SACL	*+
	LRLK	AR3,SL9
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
	B	L265
L261:
	.line	152
	LAC	_press_flag
	BZ	L266
	.line	153
	LAC	_sw_pressed
	BZ	L267
	.line	154
	LACK	3
	SACL	_params_changed
	.line	155
	LACK	0
	SACL	_press_flag
	B	L265
L267:
	.line	157
	LAC	_cw
	BNZ	LL32
	LAC	_ccw
	BZ	L265
LL32:
	.line	158
	MAR	* ,AR1
	LACK	0
	SACL	*+
	LACK	1
	SACL	*+
	LRLK	AR3,SL87
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
	.line	159
	LALK	22,15
	ORK	29104
	SACL	*+
	SACH	*+
	CALL	_wait
	SBRK	2
	.line	160
	CALL	_update_disp_left
	.line	161
	LACK	1
	SACL	*+
	CALL	_update_disp_right
	MAR	*-,AR0
	.line	162
	LACK	0
	LDPK	_press_flag
	SACL	_press_flag
	.line	164
	IN	* ,0fff6h
	LAC	* 
	ANDK	3
	SACL	_gray_code
	.line	165
	LACK	240
	SACL	* 
	OUT	* ,0fff6h
	B	L265
L266:
	.line	168
	LAC	_sw_pressed
	BNZ	LL33
	LAC	_sw_down
	BZ	L270
	LAC	_cw
	BZ	L270
LL33:
	.line	170
	MAR	* ,AR2
	SBRK	5
	BLKD	#_cursor_pos,* 
	.line	171
	LAC	* 
	SUBK	1
	BNZ	L271
	.line	172
	ADRK	2
	LAC	*+
	ADD	* 
	SBRK	3
	SACL	* 
	B	L272
L271:
	.line	175
	ADDK	2
	SACL	* 
	.line	176
	SSXM
	ADRK	2
	LAC	* 
	ADRK	2
	ADD	* 
	SBRK	4
	SUB	* 
	BGZ	L273
	.line	177
	LACK	1
	SACL	* 
	B	L272
L273:
	.line	179
	ADRK	5
	LAC	* 
	BZ	L272
	SBRK	3
	LAC	* 
	SUBK	1
	ADRK	2
	ADD	*+
	SUB	* 
	SBRK	5
	SUB	* 
	BNZ	L272
	.line	180
	LAC	* 
	ADDK	1
	SACL	* 
L272:
	.line	183
	MAR	* ,AR1
	LACK	1
	SACL	*+,AR2
	LARK	AR2,3
	MAR	*0+
	LAC	* ,AR1
	SACL	*+
	LRLK	AR3,SL9
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
	B	L265
L270:
	.line	185
	LAC	_sw_down
	BZ	L276
	LAC	_ccw
	BZ	L276
	.line	187
	MAR	* ,AR2
	SBRK	5
	BLKD	#_cursor_pos,* 
	.line	188
	LAC	* 
	SUBK	1
	BNZ	L277
	.line	189
	ADRK	2
	LAC	* 
	SUBK	1
	ADRK	2
	ADD	* 
	SBRK	4
	SACL	* 
	B	L278
L277:
	.line	192
	SACL	* 
	.line	193
	SSXM
	LAC	* 
	ADRK	2
	SUB	* 
	BGEZ	L279
	.line	194
	LACK	1
	SBRK	2
	SACL	* 
	B	L278
L279:
	.line	196
	ADRK	3
	LAC	* 
	BZ	L278
	SBRK	3
	LAC	* 
	SUBK	1
	ADRK	2
	ADD	*+
	SUB	* 
	SBRK	5
	SUB	* 
	BNZ	L278
	.line	197
	LAC	* 
	SUBK	1
	SACL	* 
L278:
	.line	200
	MAR	* ,AR1
	LACK	1
	SACL	*+,AR2
	LARK	AR2,3
	MAR	*0+
	LAC	* ,AR1
	SACL	*+
	LRLK	AR3,SL9
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
	B	L265
L276:
	.line	202
	LAC	_cw
	BNZ	LL34
	LAC	_ccw
	BZ	L282
LL34:
	LACK	3
	LDPK	_assembly_flag
	AND	_assembly_flag
	MAR	* ,AR1
	SACL	* 
	SSXM
	LAC	* 
	SUBK	3
	BNZ	L282
	.line	203
	LACK	-4
	AND	_assembly_flag
	SACL	_assembly_flag
	.line	205
	CALL	_update_disp_left
	.line	206
	BLKD	_cursor_pos_vu+0,*+
	CALL	_update_disp_right
	MAR	*-
	.line	207
	LACK	1
	LDPK	_params_changed
	SACL	_params_changed
	B	L265
L282:
	.line	209
	LDPK	_cw
	LAC	_cw
	BNZ	LL35
	LAC	_ccw
	BZ	L265
LL35:
	LAC	_down_turn_flag
	BNZ	L265
	.line	210
	LACK	-4
	LDPK	_assembly_flag
	AND	_assembly_flag
	SACL	_assembly_flag
	.line	212
	LDPK	_cursor_pos
	LAC	_cursor_pos
	SUBK	1
	BNZ	L284
	.line	213
	CALL	_inc_dec_param_ptr,AR1
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	SACL	* ,AR1
	.line	214
	CALL	_update_disp_left
	.line	215
	LACK	1
	SACL	*+
	CALL	_update_disp_right
	MAR	*-,AR2
	.line	216
	LACK	1
	LDPK	_params_changed
	SACL	_params_changed
	.line	217
	LARK	AR2,1
	MAR	*0+
	LAC	* 
	BZ	L265
	.line	218
	MAR	* ,AR1
	LACK	550
	SACL	*+
	LACK	30
	SACL	*+
	CALL	_beep
	SBRK	2
	.line	219
	LALK	6,15
	ORK	3392
	SACL	*+
	SACH	*+
	CALL	_wait
	SBRK	2
	.line	221
	MAR	* ,AR0
	IN	* ,0fff6h
	LAC	* 
	ANDK	3
	LDPK	_gray_code
	SACL	_gray_code
	.line	222
	LACK	240
	SACL	* 
	OUT	* ,0fff6h
	B	L265
L284:
	.line	226
	MAR	* ,AR2
	SBRK	4
	LAC	* 
	BZ	L287
	.line	227
	LT	_param_ptr
	MPYK	3
	PAC
	ADLK	_param_struct+0,0
	MAR	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	LACK	255
	MAR	*+
	AND	* ,AR2
	ADRK	5
	SACL	* ,AR1
	.line	228
	LAC	_index_ab,1
	SACL	* 
	LT	_param_ptr
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR2
	SBRK	8
	SACL	* 
	.line	229
	ADRK	8
	LAC	* ,AR1
	SACL	*+,AR3
	LARK	AR3,1
	MAR	*0+,AR1
	SAR	AR3,*+
	CALL	_inc_dec_mod
	SBRK	2
	.line	230
	LDPK	_index_ab
	LAC	_index_ab,1
	SACL	* 
	LT	_param_ptr
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR2
	SSXM
	LARK	AR2,1
	MAR	*0+
	LAC	* ,AR3
	SACL	*+
	SACH	*-
	B	L288
L287:
	.line	233
	ADRK	4
	LAC	* 
	BNZ	L289
	.line	234
	SBRK	3
	LAC	* 
	SUBK	1
	ADRK	2
	ADD	* 
	SUB	_cursor_pos
	SBRK	6
	SACL	* 
	B	L290
L289:
	.line	237
	SSXM
	MAR	*-
	LAC	* 
	SBRK	2
	ADD	* 
	ADRK	3
	SUB	* 
	SUB	_cursor_pos
	BGZ	L291
	.line	238
	SBRK	3
	LAC	* 
	SUBK	1
	ADRK	2
	ADD	* 
	SUB	_cursor_pos
	SBRK	6
	SACL	* 
	B	L290
L291:
	.line	241
	SBRK	3
	LAC	* 
	SUBK	2
	ADRK	2
	ADD	* 
	SUB	_cursor_pos
	SBRK	6
	SACL	* 
L290:
	.line	244
	LAC	* ,AR1
	SACL	*+
	LAC	_index_ab,1
	SACL	* 
	LT	_param_ptr
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	ADD	* 
	SACL	*+
	CALL	_add_sub_pow
	SBRK	2
L288:
	.line	246
	MAR	* ,AR1
	BLKD	_cursor_pos+0,*+
	CALL	_update_disp_right
	MAR	*-
	.line	247
	LACK	2
	LDPK	_params_changed
	SACL	_params_changed
L265:
	.line	251
	LDPK	_sw_down
	LAC	_sw_down
	BZ	LL37
	LAC	_cw
	BNZ	LL36
	LAC	_ccw
	BNZ	LL36
LL37:
	ZAC
	B	LL38
LL36:
	LACK	1
LL38:
	SACL	_down_turn_flag
	.line	254
	LACK	0
	SACL	_sw_pressed
	.line	255
	SACL	_sw_released
	.line	256
	SACL	_cw
	.line	257
	SACL	_ccw
	.line	259
	LACK	65471
	AND	_port0_copy
	SACL	_port0_copy
	.line	260
	MAR	* ,AR0
	SACL	* 
	OUT	* ,00h
EPI0_9:
	.line	266
	MAR	* ,AR1
	SBRK	27
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	1898,000000000H,26

	.sym	_reset_lcd,_reset_lcd,32,2,0
	.globl	_reset_lcd

	.func	1907
******************************************************
* FUNCTION DEF : _reset_lcd
******************************************************
_reset_lcd:
	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,3
	LAR	AR0,*0+

	.sym	_i,1,4,1,16
	.sym	_j,2,4,1,16
	.line	6
	LACK	4100
	SACL	*+
	LACK	3
	SACL	*+
	CALL	_write_lcd_nibble
	SBRK	2
	.line	7
	LACK	100
	SACL	*+
	LACK	3
	SACL	*+
	CALL	_write_lcd_nibble
	SBRK	2
	.line	8
	LACK	100
	SACL	*+
	LACK	3
	SACL	*+
	CALL	_write_lcd_nibble
	SBRK	2
	.line	9
	LACK	100
	SACL	*+
	LACK	2
	SACL	*+
	CALL	_write_lcd_nibble
	SBRK	2
	.line	12
	LACK	40
	SACL	*+
	SACL	*+
	CALL	_write_lcd_inst
	SBRK	2
	.line	13
	LACK	40
	SACL	*+
	LACK	12
	SACL	*+
	CALL	_write_lcd_inst
	SBRK	2
	.line	14
	LACK	0
	LDPK	_cursor_flag
	SACL	_cursor_flag
	.line	15
	LACK	1
	SACL	_flash_cursor_flag
	.line	16
	LACK	1640
	SACL	*+
	LACK	1
	SACL	*+
	CALL	_write_lcd_inst
	SBRK	2
	.line	17
	LACK	40
	SACL	*+
	LACK	6
	SACL	*+
	CALL	_write_lcd_inst
	SBRK	2
	.line	18
	LACK	1640
	SACL	*+
	LACK	2
	SACL	*+
	CALL	_write_lcd_inst
	SBRK	2
	.line	21
	LACK	40
	SACL	*+
	LACK	64
	SACL	*+
	CALL	_write_lcd_inst
	SBRK	2
	.line	22
	LACK	7
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	SACL	* 
	SSXM
	LAC	* 
	BLZ	L294
	MAR	*+
L293:
	.line	23
	LACK	1
	SACL	* 
	LAC	* 
	SUBK	9
	BGEZ	L296
L295:
	.line	24
	LAC	*-
	SUB	* 
	BGZ	L297
	.line	25
	MAR	* ,AR1
	LACK	40
	SACL	*+
	LACK	0
	SACL	*+
	CALL	_write_lcd_data
	SBRK	2
	B	L298
L297:
	.line	28
	MAR	* ,AR1
	LACK	40
	SACL	*+
	LACK	14
	SACL	*+
	CALL	_write_lcd_data
	SBRK	2
L298:
	.line	23
	MAR	* ,AR2
	LARK	AR2,2
	MAR	*0+
	LAC	* 
	ADDK	1
	SACL	* 
	SSXM
	LAC	* 
	SUBK	9
	BLZ	L295
L296:
	.line	22
	MAR	*-
	LAC	* 
	SUBK	1
	SACL	* 
	LAC	*+
	BGEZ	L293
L294:
	.line	33
	MAR	* ,AR1
	LACK	40
	SACL	*+
	LACK	128
	SACL	*+
	CALL	_write_lcd_inst
	SBRK	2
	.line	34
	LACK	1
	LDPK	_cursor_pos
	SACL	_cursor_pos
EPI0_10:
	.line	36
	SBRK	4
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	1942,000000000H,3

	.sym	_disp_text,_disp_text,32,2,0
	.globl	_disp_text

	.func	1962
******************************************************
* FUNCTION DEF : _disp_text
******************************************************
_disp_text:

LF11	.set	0

	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,2
	LAR	AR0,*0+,AR2

	.sym	_text,-3+LF11,18,9,16
	.sym	_position,-4+LF11,4,9,16
	.sym	_cursor_on,-5+LF11,4,9,16
	.sym	_port_temp,1,14,1,16
	.line	2
	.line	5
	LARK	AR2,1
	MAR	*0+
	IN	* ,0fff5h,AR0
	.line	6
	IN	* ,0fff5h
	LAC	* 
	ANDK	65023
	SACL	* 
	OUT	* ,0fff5h,AR2
	.line	10
	SSXM
	SBRK	6-LF11
	LAC	* 
	BLEZ	L299
	LDPK	_cursor_flag
	LAC	_cursor_flag
	BNZ	L299
	.line	11
	MAR	* ,AR1
	LACK	40
	SACL	*+
	LACK	14
	SACL	*+
	CALL	_write_lcd_inst
	SBRK	2
	.line	12
	LACK	1
	LDPK	_cursor_flag
	SACL	_cursor_flag
L299:
	.line	14
	SSXM
	MAR	* ,AR2
	LARK	AR2,-5+LF11
	MAR	*0+
	LAC	* 
	BGEZ	L300
	LDPK	_cursor_flag
	LAC	_cursor_flag
	BZ	L300
	.line	15
	MAR	* ,AR1
	LACK	40
	SACL	*+
	LACK	12
	SACL	*+
	CALL	_write_lcd_inst
	SBRK	2
	.line	16
	LACK	0
	LDPK	_cursor_flag
	SACL	_cursor_flag
L300:
	.line	19
	MAR	* ,AR2
	LARK	AR2,-4+LF11
	MAR	*0+
	LAC	* 
	LDPK	_cursor_pos
	SUB	_cursor_pos
	BZ	L301
	.line	20
	SSXM
	LAC	* 
	SUBK	9
	BGEZ	L302
	.line	21
	MAR	* ,AR1
	LACK	40
	SACL	*+,AR2
	LAC	* ,AR1
	ADDK	127
	SACL	*+
	CALL	_write_lcd_inst
	SBRK	2
	B	L303
L302:
	.line	24
	MAR	* ,AR1
	LACK	40
	SACL	*+,AR2
	LAC	* ,AR1
	ADDK	183
	SACL	*+
	CALL	_write_lcd_inst
	SBRK	2
L303:
	.line	26
	MAR	* ,AR2
	LARK	AR2,-4+LF11
	MAR	*0+
	BLDD	* ,#_cursor_pos
L301:
	.line	29
	MAR	*+
	LAR	AR3,* ,AR3
	LAC	* 
	BZ	L305
	MAR	* ,AR1
L304:
	.line	30
	LACK	40
	SACL	*+,AR2
	LAR	AR3,* ,AR3
	LAC	*+,AR2
	SAR	AR3,* ,AR1
	SACL	*+
	CALL	_write_lcd_data
	SBRK	2
	.line	31
	LDPK	_cursor_pos
	LAC	_cursor_pos
	ADDK	1
	SACL	_cursor_pos
	.line	32
	LAC	_cursor_pos
	SUBK	9
	BNZ	L306
	.line	33
	LACK	40
	SACL	*+
	LACK	192
	SACL	*+
	CALL	_write_lcd_inst
	SBRK	2
L306:
	.line	29
	MAR	* ,AR2
	LARK	AR2,-3+LF11
	MAR	*0+
	LAR	AR3,* ,AR3
	LAC	* ,AR1
	BNZ	L304
L305:
	.line	36
	MAR	* ,AR2
	ADRK	4-LF11
	OUT	* ,0fff5h,AR1
EPI0_11:
	.line	37
	SBRK	3
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	1998,000000000H,2

	.sym	_disp_num,_disp_num,32,2,0
	.globl	_disp_num

	.func	2025
******************************************************
* FUNCTION DEF : _disp_num
******************************************************
_disp_num:

LF12	.set	0

	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,21
	LAR	AR0,*0+,AR3

	.sym	_num,-4+LF12,5,9,32
	.sym	_position,-5+LF12,4,9,16
	.sym	_width,-6+LF12,4,9,16
	.sym	_nfrac,-7+LF12,4,9,16
	.sym	_tempc,1,50,1,272,,17
	.sym	_text,18,18,1,16
	.sym	_i,19,4,1,16
	.sym	_strlength,20,4,1,16
	.line	2
	.line	8
	LARK	AR3,1
	MAR	*0+,AR1
	SAR	AR3,*+,AR4
	LARK	AR4,20
	MAR	*0+,AR1
	SAR	AR4,*+,AR2
	LARK	AR2,-7+LF12
	MAR	*0+
	LAC	* ,AR1
	SACL	*+,AR2
	ADRK	3
	ZALS	*+
	ADDH	* ,AR1
	SACL	*+
	SACH	*+
	CALL	_num2string
	SBRK	5
	MAR	* ,AR2
	LARK	AR2,18
	MAR	*0+
	SACL	* 
	.line	11
	SSXM
	ADRK	2
	LAC	* 
	SBRK	26-LF12
	SUB	* 
	BLEZ	L307
	.line	12
	MAR	* ,AR3
	LARK	AR3,17
	MAR	*0+,AR2
	ADRK	24-LF12
	SAR	AR3,*+
	.line	13
	LACK	0
	SACL	* 
	LAC	* 
	SBRK	25-LF12
	SUB	* 
	BGEZ	L307
	ADRK	24-LF12
L308:
	.line	14
	LAR	AR3,* ,AR3
	MAR	*-,AR2
	SAR	AR3,*+,AR3
	LACK	42
	SACL	* ,AR2
	.line	13
	LAC	* 
	ADDK	1
	SACL	* 
	LAC	* 
	SBRK	25-LF12
	SUB	* 
	ADRK	24-LF12
	BLZ	L308
L307:
	.line	18
	LARK	AR2,20
	MAR	*0+
	LAC	* 
	ADDK	1
	SACL	* 
	SUBK	1
	SBRK	26-LF12
	SUB	* 
	BGEZ	L311
	ADRK	24-LF12
L310:
	.line	19
	LAR	AR3,* ,AR3
	MAR	*-,AR2
	SAR	AR3,* ,AR3
	LACK	32
	SACL	* ,AR2
	.line	18
	ADRK	2
	LAC	* 
	ADDK	1
	SACL	* 
	SUBK	1
	SBRK	26-LF12
	SUB	* 
	ADRK	24-LF12
	BLZ	L310
L311:
	.line	23
	MAR	* ,AR1
	LACK	0
	SACL	*+,AR2
	LARK	AR2,-5+LF12
	MAR	*0+
	LAC	* ,AR1
	SACL	*+,AR2
	ADRK	23-LF12
	LAC	* ,AR1
	SACL	*+
	CALL	_disp_text
	SBRK	3
EPI0_12:
	.line	25
	SBRK	22
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	2049,000000000H,21

	.sym	_num2string,_num2string,98,2,0
	.globl	_num2string

	.func	2069
******************************************************
* FUNCTION DEF : _num2string
******************************************************
_num2string:

LF13	.set	0

	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,5
	LAR	AR0,*0+

	.sym	_num,-4+LF13,5,9,32
	.sym	_nfrac,-5+LF13,4,9,16
	.sym	_strlength,-6+LF13,20,9,16
	.sym	_str17,-7+LF13,18,9,16
	.sym	_bufptr,1,18,1,16
	.sym	_neg,2,4,1,16
	.sym	_unum,3,5,1,32
	.line	2
	.line	4
	BNV	$+2
	MAR	* ,AR2
	LARK	AR2,-4+LF13
	MAR	*0+
	LT	*+
	MPYK	1
	LPH	* 
	LACK	0
	SPAC
	LARK	AR3,1
	BNV	$+6
	BLZ	LL44
	B	$+4
	BGZ	LL44
	LARK	AR3,0
LL44:
	ADRK	5-LF13
	SAR	AR3,* 
	.line	5
	LAC	* 
	BZ	LL45
	SBRK	6-LF13
	ZALS	*+
	ADDH	* 
	NEG
	B	LL46
LL45:
	SBRK	6-LF13
	ZALS	*+
	ADDH	* 
LL46:
	ADRK	6-LF13
	SACL	*+
	SACH	* 
	.line	7
	SBRK	11-LF13
	LAC	* 
	ADDK	16
	ADRK	8-LF13
	SACL	* 
	.line	8
	LAR	AR4,* ,AR4
	LACK	0
	SACL	* ,AR2
	.line	9
	SBRK	7-LF13
	LAR	AR5,* ,AR5
	SACL	* ,AR1
L312:
	.line	12
	LACK	10
	SACL	*+
	SACH	*+,AR2
	LARK	AR2,3
	MAR	*0+
	ZALS	*+
	ADDH	* ,AR1
	CALL	L$$MODS
	ADDK	48
	SACL	*+,AR2
	SBRK	3
	LAR	AR3,* ,AR3
	MAR	*-,AR2
	SAR	AR3,* ,AR1
	MAR	*-
	LAC	* ,AR3
	SACL	* ,AR2
	.line	13
	SBRK	7-LF13
	LAR	AR3,*+,AR3
	LAC	* 
	ADDK	1
	SACL	* 
	.line	14
	LAC	* ,AR2
	SUB	* 
	BNZ	L313
	.line	15
	ADRK	6-LF13
	LAR	AR4,* ,AR4
	MAR	*-,AR2
	SAR	AR4,* ,AR4
	LACK	46
	SACL	* ,AR3
	.line	16
	LAC	* 
	ADDK	1
	SACL	* ,AR2
	.line	17
	LACK	0
	SBRK	6-LF13
	SACL	* 
L313:
	.line	19
	LACK	10
	MAR	* ,AR1
	SACL	*+
	SACH	*+,AR2
	ADRK	8-LF13
	ZALS	*+
	ADDH	*-,AR1
	CALL	L$$DIVS
	MAR	* ,AR2
	SACL	*+
	SACH	*-,AR1
	BNZ	L312
	MAR	* ,AR2
	SBRK	8-LF13
	LAC	* ,AR1
	BNZ	L312
	.line	22
	MAR	* ,AR2
	ADRK	7-LF13
	LAC	* 
	BZ	L314
	.line	23
	MAR	*-
	LAR	AR4,* ,AR4
	MAR	*-,AR2
	SAR	AR4,* ,AR4
	LACK	45
	SACL	* ,AR3
	.line	24
	LAC	* 
	ADDK	1
	SACL	* 
L314:
	.line	27
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	ZALS	* 
EPI0_13:
	.line	28
	MAR	* ,AR1
	SBRK	6
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	2096,000000000H,5

	.sym	_write_lcd_inst,_write_lcd_inst,32,2,0
	.globl	_write_lcd_inst

	.func	2109
******************************************************
* FUNCTION DEF : _write_lcd_inst
******************************************************
_write_lcd_inst:

LF14	.set	0

	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,5
	LAR	AR0,*0+,AR2

	.sym	_byte,-3+LF14,4,9,16
	.sym	_wait_usec,-4+LF14,4,9,16
	.sym	_i,1,4,1,16
	.sym	_iend,2,4,1,16
	.sym	_nibh,3,4,1,16
	.sym	_nibl,4,4,1,16
	.line	2
	.line	5
	LACK	240
	LARK	AR2,-3+LF14
	MAR	*0+
	AND	* ,AR0
	SACL	* 
	SSXM
	LAC	* ,11,AR2
	ADRK	6-LF14
	SACH	* ,1
	.line	6
	LACK	15
	SBRK	6-LF14
	AND	* 
	ADRK	7-LF14
	SACL	*-,AR0
	.line	8
	LACK	65519
	LDPK	_port0_copy
	AND	_port0_copy
	SACL	_port0_copy
	.line	9
	SACL	* 
	OUT	* ,00h,AR2
	.line	12
	ANDK	208
	ORK	32
	OR	* ,AR0
	SACL	_port0_copy
	.line	13
	SACL	* 
	OUT	* ,00h,AR2
	.line	15
	ANDK	208
	OR	*+,AR0
	SACL	_port0_copy
	.line	16
	SACL	* 
	OUT	* ,00h,AR2
	.line	19
	ANDK	208
	ORK	32
	OR	* ,AR0
	SACL	_port0_copy
	.line	20
	SACL	* 
	OUT	* ,00h,AR2
	.line	22
	ANDK	208
	OR	* ,AR0
	SACL	_port0_copy
	.line	23
	SACL	* 
	OUT	* ,00h,AR2
	.line	25
	SBRK	8-LF14
	LAC	* ,AR1
	SACL	*+
	SACH	*+
	CALL	_wait
	SBRK	2
EPI0_14:
	.line	26
	SBRK	6
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	2134,000000000H,5

	.sym	_write_lcd_data,_write_lcd_data,32,2,0
	.globl	_write_lcd_data

	.func	2147
******************************************************
* FUNCTION DEF : _write_lcd_data
******************************************************
_write_lcd_data:

LF15	.set	0

	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,5
	LAR	AR0,*0+,AR2

	.sym	_byte,-3+LF15,4,9,16
	.sym	_wait_usec,-4+LF15,4,9,16
	.sym	_i,1,4,1,16
	.sym	_iend,2,4,1,16
	.sym	_nibh,3,4,1,16
	.sym	_nibl,4,4,1,16
	.line	2
	.line	5
	LACK	240
	LARK	AR2,-3+LF15
	MAR	*0+
	AND	* ,AR0
	SACL	* 
	SSXM
	LAC	* ,11,AR2
	ADRK	6-LF15
	SACH	* ,1
	.line	6
	LACK	15
	SBRK	6-LF15
	AND	* 
	ADRK	7-LF15
	SACL	*-,AR0
	.line	8
	LACK	16
	LDPK	_port0_copy
	OR	_port0_copy
	SACL	_port0_copy
	.line	9
	SACL	* 
	OUT	* ,00h,AR2
	.line	12
	ANDK	208
	ORK	32
	OR	* ,AR0
	SACL	_port0_copy
	.line	13
	SACL	* 
	OUT	* ,00h,AR2
	.line	15
	ANDK	208
	OR	*+,AR0
	SACL	_port0_copy
	.line	16
	SACL	* 
	OUT	* ,00h,AR2
	.line	19
	ANDK	208
	ORK	32
	OR	* ,AR0
	SACL	_port0_copy
	.line	20
	SACL	* 
	OUT	* ,00h,AR2
	.line	22
	ANDK	208
	OR	* ,AR0
	SACL	_port0_copy
	.line	23
	SACL	* 
	OUT	* ,00h,AR2
	.line	25
	SBRK	8-LF15
	LAC	* ,AR1
	SACL	*+
	SACH	*+
	CALL	_wait
	SBRK	2
EPI0_15:
	.line	26
	SBRK	6
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	2172,000000000H,5

	.sym	_write_lcd_nibble,_write_lcd_nibble,32,2,0
	.globl	_write_lcd_nibble

	.func	2185
******************************************************
* FUNCTION DEF : _write_lcd_nibble
******************************************************
_write_lcd_nibble:

LF16	.set	0

	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,3
	LAR	AR0,*0+,AR0

	.sym	_nibble,-3+LF16,4,9,16
	.sym	_wait_usec,-4+LF16,4,9,16
	.sym	_i,1,4,1,16
	.sym	_iend,2,4,1,16
	.line	2
	.line	5
	LACK	65519
	LDPK	_port0_copy
	AND	_port0_copy
	SACL	_port0_copy
	.line	6
	SACL	* 
	OUT	* ,00h,AR2
	.line	8
	ANDK	208
	ORK	32
	LARK	AR2,-3+LF16
	MAR	*0+
	OR	* ,AR0
	SACL	_port0_copy
	.line	9
	SACL	* 
	OUT	* ,00h,AR2
	.line	10
	ANDK	208
	OR	*-,AR0
	SACL	_port0_copy
	.line	11
	SACL	* 
	OUT	* ,00h,AR2
	.line	13
	SSXM
	LAC	* ,AR1
	SACL	*+
	SACH	*+
	CALL	_wait
	SBRK	2
EPI0_16:
	.line	14
	SBRK	4
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	2198,000000000H,3

	.sym	_delta_t,_delta_t,36,2,0
	.globl	_delta_t

	.func	2209
******************************************************
* FUNCTION DEF : _delta_t
******************************************************
_delta_t:

LF17	.set	0

	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,2
	LAR	AR0,*0+,AR0

	.sym	_count_start,-3+LF17,14,9,16
	.sym	_itemp,1,4,1,16
	.line	2
	.line	5
	IN	* ,0fffah
	ZALS	* ,AR1
	RSXM
	SFR
	SACL	* ,AR2
	LARK	AR2,-3+LF17
	MAR	*0+
	LAC	* ,14,AR1
	SUB	* ,15,AR2
	ADRK	4-LF17
	SACH	* ,1
	.line	8
	SSXM
	LAC	* 
	BLZ	LL52
	LAC	* 
	B	EPI0_17
LL52:
	ZALS	* ,AR1
	RSXM
	ADDK	-32768
	SACL	* 
	SSXM
	LAC	* 
EPI0_17:
	.line	10
	MAR	* ,AR1
	SBRK	3
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	2218,000000000H,2

	.sym	_wait,_wait,32,2,0
	.globl	_wait

	.func	2231
******************************************************
* FUNCTION DEF : _wait
******************************************************
_wait:

LF18	.set	0

	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,5
	LAR	AR0,*0+,AR2

	.sym	_wait_usec,-4+LF18,5,9,32
	.sym	_i,1,4,1,16
	.sym	_wait_course,2,4,1,16
	.sym	_wait_fine,3,4,1,16
	.sym	_count_start,4,14,1,16
	.line	2
	.line	7
	RSXM
	LARK	AR2,-4+LF18
	MAR	*0+
	LAC	*+,2,AR0
	SACH	*,AR2
	SSXM
	LAC	* ,2,AR0
	OR	*,AR2
	ADRK	5-LF18
	SACL	* 
	.line	8
	LACK	16383
	SBRK	6-LF18
	AND	* 
	ADRK	7-LF18
	SACL	*+
	.line	10
	IN	* ,0fffah
L315:
	.line	11
	LAC	* ,AR1
	SACL	*+
	CALL	_delta_t
	MAR	*-,AR2
	SSXM
	LARK	AR2,3
	MAR	*0+
	SUB	*+
	BLZ	L315
	.line	13
	SBRK	2
	LAC	* 
	BZ	EPI0_18
	.line	16
	LACK	0
	MAR	*-
	SACL	* 
	LAC	*+
	SUB	* 
	BGEZ	EPI0_18
	ADRK	2
L317:
	.line	17
	IN	* ,0fffah
L319:
	.line	18
	LARK	AR2,4
	MAR	*0+
	LAC	* ,AR1
	SACL	*+
	CALL	_delta_t
	MAR	*-,AR2
	SUBK	16384
	BLZ	L319
	.line	16
	LARK	AR2,1
	MAR	*0+
	LAC	* 
	ADDK	1
	SACL	* 
	SSXM
	LAC	*+
	SUB	* 
	ADRK	2
	BLZ	L317
EPI0_18:
	.line	22
	MAR	* ,AR1
	SBRK	6
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	2252,000000000H,5

	.sym	_wait_n_samples,_wait_n_samples,32,2,0
	.globl	_wait_n_samples

	.func	2259
******************************************************
* FUNCTION DEF : _wait_n_samples
******************************************************
_wait_n_samples:

LF19	.set	0

	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,1
	LAR	AR0,*0+

	.sym	_n,-3+LF19,4,9,16
	.line	2
	.line	3
	RPTK	1
	BLKD	_fsample+0,*+
	CALL	F$$FTOL
	SACL	*+
	SACH	*+,AR2
	SSXM
	LARK	AR2,-3+LF19
	MAR	*0+
	LAC	* ,AR1
	SACL	*+
	SACH	*+
	SACL	*+
	LALK	30,15
	ORK	16960
	MAR	*-
	CALL	L$$MPY
	CALL	L$$DIVS
	SACL	*+
	SACH	*+
	CALL	_wait
	SBRK	2
EPI0_19:
	.line	5
	SBRK	2
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	2263,000000000H,1

	.sym	_prog_flash,_prog_flash,36,2,0
	.globl	_prog_flash

	.func	2316
******************************************************
* FUNCTION DEF : _prog_flash
******************************************************
_prog_flash:

LF20	.set	0

	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,22
	LAR	AR0,*0+,AR2

	.sym	_start,-3+LF20,14,9,16
	.sym	_length,-4+LF20,14,9,16
	.sym	_datawords,-5+LF20,30,9,16
	.sym	_erase_flag,-6+LF20,4,9,16
	.sym	_error_text,-7+LF20,18,9,16
	.sym	_i,1,14,1,16
	.sym	_j,2,14,1,16
	.sym	_status1,3,14,1,16
	.sym	_status2,4,14,1,16
	.sym	_word,5,14,1,16
	.sym	_byte,6,14,1,16
	.sym	_xbyte,7,14,1,16
	.sym	_sector_start,8,14,1,16
	.sym	_port_temp,9,14,1,16
	.sym	_carray,10,50,1,160,,10
	.sym	_error_code,20,5,1,32
	.line	2
	.line	5
	LACK	0
	LARK	AR2,20
	MAR	*0+
	SACL	*+
	SACH	* 
	.line	7
	LDPK	_flash_locked
	LAC	_flash_locked
	BZ	L320
	.line	8
	LACK	1
	MAR	*-
	SACL	*+
	SACH	* 

	.sym	_prog_error,L321,0,6,0
	.line	9
	B	L321
L320:
	.line	12
	SBRK	12
	IN	* ,0fff5h,AR0
	.line	13
	IN	* ,0fff5h
	LAC	* 
	ANDK	65023
	SACL	* 
	OUT	* ,0fff5h
	.line	14
	IN	* ,0fff5h
	LAC	* 
	ORK	4
	SACL	* 
	OUT	* ,0fff5h,AR2
	.line	16
	SBRK	12-LF20
	ZALS	* 
	SUBK	16384
	BGEZ	L322
	.line	17
	MAR	* ,AR0
	IN	* ,0fff6h
	LAC	* ,AR1
	ANDK	15
	ORK	4
	SACL	* 
	OUT	* ,0fff6h
	.line	18
	LACK	65523
	LDPK	_out_atten
	AND	_out_atten
	SACL	_out_atten
	B	L323
L322:
	.line	20
	ZALS	* 
	RSXM
	SUBK	-32768
	BGEZ	L324
	.line	21
	ADDK	16384
	SACL	* ,AR0
	.line	22
	IN	* ,0fff6h
	LAC	* ,AR1
	ANDK	15
	ORK	4
	SACL	* 
	OUT	* ,0fff6h
	.line	23
	LACK	12
	LDPK	_out_atten
	OR	_out_atten
	SACL	_out_atten
	B	L323
L324:
	.line	25
	ZALS	* 
	SUBK	-16384
	BGEZ	L325
	.line	26
	LAC	* 
	ADLK	65535,15
	SACL	* ,AR0
	.line	27
	IN	* ,0fff6h
	LAC	* 
	ANDK	11
	SACL	* 
	OUT	* ,0fff6h
	.line	28
	LACK	65523
	LDPK	_out_atten
	AND	_out_atten
	SACL	_out_atten
	B	L323
L325:
	.line	31
	LAC	* 
	ADDK	16384
	ADLK	65534,15
	SACL	* ,AR0
	.line	32
	IN	* ,0fff6h
	LAC	* 
	ANDK	11
	SACL	* 
	OUT	* ,0fff6h
	.line	33
	LACK	12
	LDPK	_out_atten
	OR	_out_atten
	SACL	_out_atten
L323:
	.line	36
	MAR	* ,AR1
	LACK	20
	SACL	*+
	CALL	_wait_n_samples
	MAR	*-,AR2
	.line	39
	LARK	AR2,-3+LF20
	MAR	*0+
	ZALS	* 
	SUBK	8192
	BGEZ	LL59
	ZALS	*-
	ADDS	* 
	ANDK	0FFFFh
	SUBK	8192
	BGZ	LL58
LL59:
	LARK	AR2,-3+LF20
	MAR	*0+
	ZALS	* 
	SUBK	8192
	BLZ	L326
	ZALS	*-
	ADDS	* 
	ANDK	0FFFFh
	SUBK	16384
	BLEZ	L326
LL58:
	.line	40
	LACK	1
	ADRK	24-LF20
	SACL	*+
	SACH	* 
	.line	41
	B	L321
L326:
	.line	44
	LDPK	_greg_ptr
	LAR	AR3,_greg_ptr
	LACK	128
	MAR	* ,AR3
	SACL	* ,AR0
	.line	45
	LACK	768
	SACL	* 
	OUT	* ,0fffch,AR4
	.line	48
	LARK	AR4,54613
	LACK	170
	SACL	* ,AR5
	.line	49
	LARK	AR5,43690
	LACK	85
	SACL	* ,AR4
	.line	50
	LACK	240
	SACL	* 
	.line	52
	LDPK	_out_atten
	BIT	_out_atten,12
	BBZ	L327
	.line	57
	LACK	0
	MAR	* ,AR2
	LARK	AR2,2
	MAR	*0+
	SACL	*-
	.line	58
	LACK	132
	SACL	* 
	ZALS	* 
	SUBK	142
	BGEZ	L329
L328:
	.line	59
	LAC	*+,AR0
	ADLK	1,15
	SACL	* 
	LAR	AR3,* ,AR3
	LACK	255
	AND	* ,AR1
	SACL	* ,AR2
	LAC	* 
	ADDK	1
	SACL	* ,AR0
	SUBK	1
	SACL	* 
	LAR	AR3,* ,AR3
	MAR	*0+,AR1
	LAC	* ,AR3
	ADRK	10
	SACL	* ,AR2
	.line	58
	MAR	*-
	LAC	* 
	ADDK	1
	SACL	* 
	ZALS	* 
	SUBK	142
	BLZ	L328
L329:
	.line	61
	MAR	* ,AR3
	LARK	AR3,10
	MAR	*0+,AR1
	SAR	AR3,*+
	CALL	_valid_serial
	MAR	*-
	BZ	L327
	.line	62
	LACK	6
	MAR	* ,AR2
	LARK	AR2,20
	MAR	*0+
	SACL	*+
	SACH	* 
	.line	63
	B	L321
L327:
	.line	67
	MAR	* ,AR2
	LARK	AR2,-6+LF20
	MAR	*0+
	LAC	* 
	BZ	L331
	.line	68
	ADRK	3
	ZALS	* 
	SUBK	8192
	BGEZ	L332
	.line	69
	LACK	0
	ADRK	11-LF20
	SACL	* 
	B	L333
L332:
	.line	72
	ADRK	11-LF20
	SPLK	#16384,* 
L333:
	.line	76
	LARK	AR3,54613
	LACK	170
	MAR	* ,AR3
	SACL	* ,AR4
	.line	77
	LARK	AR4,43690
	LACK	85
	SACL	* ,AR3
	.line	78
	LACK	128
	SACL	* 
	.line	79
	LACK	170
	SACL	* ,AR4
	.line	80
	LACK	85
	SACL	* ,AR2
	.line	81
	LAC	* ,AR0
	ADLK	1,15
	SACL	* 
	LAR	AR5,* ,AR5
	LACK	48
	SACL	* ,AR1
	.line	82
	LACK	100
	SACL	*+
	SACH	*+
	CALL	_wait
	SBRK	2
	MAR	* ,AR2
	LARK	AR2,8
	MAR	*0+
L334:
	.line	84
	LAC	* ,AR0
	ADLK	1,15
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR2
	SBRK	5
	SACL	* 
	.line	85
	ADRK	5
	LAC	* ,AR0
	ADLK	1,15
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR2
	SBRK	4
	SACL	* 
	.line	86
	LACK	64
	AND	*-,AR1
	SACL	* ,AR2
	LACK	64
	AND	* ,AR1
	SUBS	* 
	BZ	L336
	MAR	* ,AR2
	BIT	* ,8
	BBNZ	L336

	.sym	_pf_1,L336,0,6,0
	.line	87
	.line	90
	BIT	* ,10
	ADRK	5
	BBZ	L334
	.line	91
	LAC	* ,AR0
	ADLK	1,15
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR2
	SBRK	5
	SACL	* 
	.line	92
	BIT	* ,8
	BBNZ	L336
	.line	93
	.line	95
	LACK	2
	ADRK	17
	SACL	*+
	SACH	* 
	.line	96
	B	L321
L336:
	.line	98
	LARK	AR3,54613
	LACK	170
	MAR	* ,AR3
	SACL	* ,AR4
	.line	99
	LARK	AR4,43690
	LACK	85
	SACL	* ,AR3
	.line	100
	LACK	240
	SACL	* ,AR2
	.line	102
	LARK	AR2,8
	MAR	*0+
	LAC	* 
	SBRK	7
	SACL	* 
	ADRK	7
	ZALS	* 
	ADDK	16384
	ANDK	0FFFFh
	SBRK	7
	SUBS	* 
	BLEZ	L331
L338:
	.line	103
	LAC	* ,AR0
	ADLK	1,15
	SACL	* 
	LAR	AR3,* ,AR3
	LACK	255
	AND	* 
	SUBK	255
	BZ	L340
	.line	104
	LACK	3
	MAR	* ,AR2
	ADRK	19
	SACL	*+
	SACH	* 
	.line	105
	B	L321
L340:
	.line	102
	MAR	* ,AR2
	LAC	* 
	ADDK	1
	SACL	* 
	ADRK	7
	ZALS	* 
	ADDK	16384
	ANDK	0FFFFh
	SBRK	7
	SUBS	* 
	BGZ	L338
L331:
	.line	111
	LACK	0
	LARK	AR2,2
	MAR	*0+
	SACL	* 
	.line	112
	SBRK	5-LF20
	LAC	* ,1
	SACL	* 
	.line	113
	LAC	*-
	ADD	* ,1
	SACL	*+
	.line	114
	LAC	* 
	ADRK	4-LF20
	SACL	* 
	ZALS	* 
	SBRK	5-LF20
	SUBS	* 
	BGEZ	L342
	ADRK	5-LF20
L341:
	.line	115
	BIT	* ,15
	BBZ	L343
	.line	116
	LACK	255
	ADRK	4
	AND	*+
	SACL	* 
	B	L344
L343:
	.line	119
	MAR	*+
	LAC	* 
	ADDK	1
	SACL	* 
	SUBK	1
	SBRK	7-LF20
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR2
	ADRK	10-LF20
	SACL	* 
	.line	120
	RSXM
	LAC	*+,7
	SACH	* ,1
L344:
	.line	122
	LARK	AR3,54613
	LACK	170
	MAR	* ,AR3
	SACL	* ,AR4
	.line	123
	LARK	AR4,43690
	LACK	85
	SACL	* ,AR3
	.line	124
	LACK	240
	SACL	* ,AR2
	.line	125
	SBRK	5
	LAC	* ,AR0
	ADLK	1,15
	SACL	* 
	LAR	AR5,* ,AR5
	LAC	* ,AR2
	ADRK	5
	XOR	*+
	ANDK	255
	SACL	* 
	.line	126
	LAC	* 
	BZ	L351
	.line	127
	ZALS	*-
	AND	* 
	BZ	L346
	.line	128
	LACK	4
	ADRK	14
	SACL	*+
	SACH	* 
	.line	129
	B	L321
L346:
	.line	131
	LACK	170
	MAR	* ,AR3
	SACL	* ,AR4
	.line	132
	LACK	85
	SACL	* ,AR3
	.line	133
	LACK	160
	SACL	* ,AR2
	.line	134
	SBRK	5
	LAC	* ,AR0
	ADLK	1,15
	SACL	* 
	LAR	AR5,* ,AR2
	ADRK	5
	LAC	* ,AR5
	SACL	* ,AR2
	SBRK	5
L347:
	.line	136
	LAC	* ,AR0
	ADLK	1,15
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR2
	ADRK	2
	SACL	* 
	.line	137
	SBRK	2
	LAC	* ,AR0
	ADLK	1,15
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR2
	ADRK	3
	SACL	* 
	.line	138
	LACK	64
	AND	*-,AR1
	SACL	* ,AR2
	LACK	64
	AND	* ,AR1
	SUBS	* 
	BZ	L351
	LACK	128
	MAR	* ,AR2
	ADRK	3
	AND	* ,AR1
	SACL	*+,AR2
	LACK	128
	SBRK	3
	AND	* ,AR1
	MAR	*-
	SUBS	* 
	BZ	L351

	.sym	_pf_3,L349,0,6,0
	.line	139
	.line	142
	MAR	* ,AR2
	BIT	* ,10
	SBRK	2
	BBZ	L347
	.line	143
	LAC	* ,AR0
	ADLK	1,15
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR2
	ADRK	2
	SACL	* 
	.line	144
	LACK	128
	ADRK	3
	AND	* ,AR1
	SACL	*+,AR2
	LACK	128
	SBRK	3
	AND	* ,AR1
	MAR	*-
	SUBS	* 
	BZ	L351
	.line	145
	.line	147
	LACK	5
	MAR	* ,AR2
	ADRK	17
	SACL	*+
	SACH	* 
	.line	148
	B	L321
L349:
	.line	151
L351:
	.line	114
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	LAC	* 
	ADDK	1
	SACL	* 
	ZALS	* 
	SBRK	5-LF20
	SUBS	* 
	ADRK	5-LF20
	BLZ	L341
L342:

	.sym	_prog_out,L352,0,6,0
L352:
	.line	155
	LACK	512
	MAR	* ,AR0
	SACL	* 
	OUT	* ,0fffch
	.line	157
	IN	* ,0fff5h
	LAC	* 
	ANDK	65531
	SACL	* 
	OUT	* ,0fff5h,AR1
	.line	158
	LACK	400
	SACL	*+
	SACH	*+
	CALL	_wait
	SBRK	2
	.line	159
	LACK	240
	MAR	* ,AR0
	SACL	* 
	OUT	* ,0fff6h,AR2
	.line	160
	LARK	AR2,9
	MAR	*0+
	OUT	* ,0fff5h
	.line	161
	LACK	65523
	LDPK	_out_atten
	AND	_out_atten
	SACL	_out_atten
	.line	162
	LACK	1
	LDPK	_flash_locked
	SACL	_flash_locked
	.line	164
	ADRK	11
	ZALS	*+
	ADDH	* 
	BZ	L353
	.line	165
	LACK	1
	B	EPI0_20
L353:
	.line	168
	LACK	0
	B	EPI0_20
L321:
	.line	173
	LARK	AR3,54613
	LACK	170
	MAR	* ,AR3
	SACL	* ,AR4
	.line	174
	LARK	AR4,43690
	LACK	85
	SACL	* ,AR3
	.line	175
	LACK	240
	SACL	* ,AR1
	.line	178
	LACK	-1
	SACL	*+
	LACK	1
	SACL	*+
	LRLK	AR5,SL88
	SAR	AR5,*+
	CALL	_disp_text
	SBRK	3
	.line	179
	LACK	0
	SACL	*+
	LACK	1
	SACL	*+
	LACK	8
	SACL	*+,AR2
	LARK	AR2,20
	MAR	*0+
	ZALS	*+
	ADDH	* ,AR1
	SACL	*+
	SACH	*+
	CALL	_disp_num
	SBRK	5
	.line	180
	LACK	-1
	SACL	*+
	LACK	9
	SACL	*+,AR2
	LARK	AR2,-7+LF20
	MAR	*0+
	LAC	* ,AR1
	SACL	*+
	CALL	_disp_text
	SBRK	3
	.line	183
	LALK	61,15
	ORK	1152
	SACL	*+
	SACH	*+
	CALL	_wait
	SBRK	2
	.line	184
	B	L352
EPI0_20:
	.line	186
	MAR	* ,AR1
	SBRK	23
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	2501,000000000H,22

	.sym	_read_flash,_read_flash,32,2,0
	.globl	_read_flash

	.func	2519
******************************************************
* FUNCTION DEF : _read_flash
******************************************************
_read_flash:

LF21	.set	0

	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,4
	LAR	AR0,*0+,AR2

	.sym	_start,-3+LF21,14,9,16
	.sym	_length,-4+LF21,14,9,16
	.sym	_datawords,-5+LF21,30,9,16
	.sym	_i,1,14,1,16
	.sym	_j,2,14,1,16
	.sym	_port_temp,3,14,1,16
	.line	2
	.line	5
	LARK	AR2,3
	MAR	*0+
	IN	* ,0fff5h,AR0
	.line	6
	IN	* ,0fff5h
	LAC	* 
	ANDK	65023
	SACL	* 
	OUT	* ,0fff5h
	.line	7
	IN	* ,0fff5h
	LAC	* 
	ORK	4
	SACL	* 
	OUT	* ,0fff5h,AR2
	.line	9
	SBRK	6-LF21
	ZALS	* 
	SUBK	16384
	BGEZ	L354
	.line	10
	MAR	* ,AR0
	IN	* ,0fff6h
	LAC	* ,AR1
	ANDK	15
	ORK	4
	SACL	* 
	OUT	* ,0fff6h
	.line	11
	LACK	65523
	LDPK	_out_atten
	AND	_out_atten
	SACL	_out_atten
	B	L355
L354:
	.line	13
	ZALS	* 
	RSXM
	SUBK	-32768
	BGEZ	L356
	.line	14
	ADDK	16384
	SACL	* ,AR0
	.line	15
	IN	* ,0fff6h
	LAC	* ,AR1
	ANDK	15
	ORK	4
	SACL	* 
	OUT	* ,0fff6h
	.line	16
	LACK	12
	LDPK	_out_atten
	OR	_out_atten
	SACL	_out_atten
	B	L355
L356:
	.line	18
	ZALS	* 
	SUBK	-16384
	BGEZ	L357
	.line	19
	LAC	* 
	ADLK	65535,15
	SACL	* ,AR0
	.line	20
	IN	* ,0fff6h
	LAC	* 
	ANDK	11
	SACL	* 
	OUT	* ,0fff6h
	.line	21
	LACK	65523
	LDPK	_out_atten
	AND	_out_atten
	SACL	_out_atten
	B	L355
L357:
	.line	24
	LAC	* 
	ADDK	16384
	ADLK	65534,15
	SACL	* ,AR0
	.line	25
	IN	* ,0fff6h
	LAC	* 
	ANDK	11
	SACL	* 
	OUT	* ,0fff6h
	.line	26
	LACK	12
	LDPK	_out_atten
	OR	_out_atten
	SACL	_out_atten
L355:
	.line	29
	MAR	* ,AR1
	LACK	20
	SACL	*+
	CALL	_wait_n_samples
	MAR	*-,AR3
	.line	32
	LDPK	_greg_ptr
	LAR	AR3,_greg_ptr
	LACK	128
	SACL	* ,AR0
	.line	33
	LACK	768
	SACL	* 
	OUT	* ,0fffch,AR4
	.line	36
	LARK	AR4,54613
	LACK	170
	SACL	* ,AR5
	.line	37
	LARK	AR5,43690
	LACK	85
	SACL	* ,AR4
	.line	38
	LACK	240
	SACL	* ,AR2
	.line	41
	LACK	0
	LARK	AR2,2
	MAR	*0+
	SACL	* 
	.line	42
	SBRK	5-LF21
	LAC	* ,1
	SACL	* 
	.line	43
	LAC	*-
	ADD	* ,1
	SACL	*+
	.line	44
	LAC	* 
	ADRK	4-LF21
	SACL	* 
	ZALS	* 
	SBRK	5-LF21
	SUBS	* 
	BGEZ	L359
	ADRK	5-LF21
L358:
	.line	45
	LAC	* ,AR0
	ADDK	32769
	SACL	* 
	LAR	AR3,* ,AR3
	LACK	255
	AND	* ,AR1
	SACL	* ,AR2
	LAC	*+,AR0
	ADLK	1,15
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,8,AR1
	OR	* 
	SACL	*+,AR2
	LAC	* 
	ADDK	1
	SACL	* 
	SUBK	1
	SBRK	7-LF21
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR1
	MAR	*-
	LAC	* ,AR3
	SACL	* ,AR2
	.line	44
	ADRK	6-LF21
	LAC	* 
	ADDK	2
	SACL	* 
	ZALS	* 
	SBRK	5-LF21
	SUBS	* 
	ADRK	5-LF21
	BLZ	L358
L359:
	.line	48
	LACK	512
	MAR	* ,AR0
	SACL	* 
	OUT	* ,0fffch
	.line	50
	IN	* ,0fff5h
	LAC	* 
	ANDK	65531
	SACL	* 
	OUT	* ,0fff5h,AR1
	.line	51
	LACK	400
	SACL	*+
	SACH	*+
	CALL	_wait
	SBRK	2
	.line	52
	LACK	240
	MAR	* ,AR0
	SACL	* 
	OUT	* ,0fff6h,AR2
	.line	53
	LARK	AR2,3
	MAR	*0+
	OUT	* ,0fff5h,AR1
	.line	54
	LACK	65523
	LDPK	_out_atten
	AND	_out_atten
	SACL	_out_atten
EPI0_21:
	.line	56
	SBRK	5
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	2574,000000000H,4

	.sym	_valid_serial,_valid_serial,36,2,0
	.globl	_valid_serial

	.func	2593
******************************************************
* FUNCTION DEF : _valid_serial
******************************************************
_valid_serial:

LF22	.set	0

	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,16
	LAR	AR0,*0+,AR2

	.sym	_carray,-3+LF22,18,9,16
	.sym	_i,1,4,1,16
	.sym	_code,2,4,1,16
	.sym	_num,3,52,1,160,,10
	.sym	_sumo,13,4,1,16
	.sym	_sume,14,4,1,16
	.sym	_sumss,15,4,1,16
	.line	2
	.line	3
	LACK	0
	LARK	AR2,13
	MAR	*0+
	SACL	*+
	SACL	*+
	SACL	* 
	.line	5
	SBRK	14
	SACL	* 
	SSXM
	LAC	* 
	SUBK	10
	BGEZ	L361
L360:
	.line	6
	LAC	* 
	SBRK	4-LF22
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR2
	ADRK	5-LF22
	SACL	* 
	.line	7
	LAC	*-,AR1
	SUBK	48
	SACL	* ,AR2
	LAR	AR3,* ,AR3
	MAR	*0+,AR1
	LAC	* ,AR3
	ADRK	3
	SACL	* ,AR2
	.line	8
	MAR	*+
	LAC	* 
	SUBK	48
	BLZ	LL65
	LAC	* 
	SUBK	57
	BLEZ	L362
LL65:
	.line	9
	LACK	0
	B	EPI0_22
L362:
	.line	5
	MAR	*-
	LAC	* 
	ADDK	1
	SACL	* 
	LAC	* 
	SUBK	10
	BLZ	L360
L361:
	.line	13
	ADRK	2
	LAC	* 
	ADRK	2
	ADD	* ,AR1
	SACL	* 
	LAC	* ,AR2
	ADRK	2
	ADD	* 
	ADRK	6
	SACL	* 
	.line	15
	LAC	* 
	SUBK	10
	BLZ	L364
L363:
	.line	16
	LAC	* 
	SUBK	10
	SACL	* 
	.line	15
	LAC	* 
	SUBK	10
	BGEZ	L363
L364:
	.line	19
	SBRK	7
	LAC	* 
	SBRK	2
	ADD	* ,AR1
	SACL	* 
	LAC	* ,AR2
	ADRK	4
	ADD	* 
	ADRK	6
	SACL	* 
	.line	21
	LAC	* 
	SUBK	10
	BLZ	L366
L365:
	.line	22
	LAC	* 
	SUBK	10
	SACL	* 
	.line	21
	LAC	* 
	SUBK	10
	BGEZ	L365
L366:
	.line	25
	SBRK	10
	LAC	*-
	ADD	* ,AR1
	SACL	* 
	LAC	* ,AR2
	ADRK	2
	ADD	*+,AR1
	SACL	* 
	LAC	* ,AR2
	ADD	*+,AR1
	SACL	* 
	LAC	* ,AR2
	ADD	*+,AR1
	SACL	* 
	LAC	* ,AR2
	ADD	* 
	ADRK	5
	ADD	*+
	ADD	*+
	SACL	* 
	.line	27
	SBRK	2
	LAC	* 
	SBRK	4
	SUB	* 
	BNZ	LL66
	ADRK	5
	LAC	* 
	SBRK	4
	SUB	* 
	BNZ	LL66
	MAR	*+
	LT	*+
	MPYK	10
	PAC
	ADD	* 
	ADRK	3
	SUB	* 
	BZ	L367
LL66:
	.line	28
	LACK	0
	B	EPI0_22
L367:
	.line	31
	LACK	1
EPI0_22:
	.line	32
	MAR	* ,AR1
	SBRK	17
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	2624,000000000H,16

	.sym	_update_disp_left,_update_disp_left,32,2,0
	.globl	_update_disp_left

	.func	2632
******************************************************
* FUNCTION DEF : _update_disp_left
******************************************************
_update_disp_left:
	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,41
	LAR	AR0,*0+,AR3

	.sym	_i,1,4,1,16
	.sym	_pos,2,4,1,16
	.sym	_pos2,3,4,1,16
	.sym	_ctemp,4,2,1,16
	.sym	_ctemp1,5,50,1,272,,17
	.sym	_ctemp2,22,50,1,272,,17
	.sym	_c1ptr,39,18,1,16
	.sym	_c2ptr,40,18,1,16
	.line	11
	LARK	AR3,5
	MAR	*0+,AR2
	LARK	AR2,39
	MAR	*0+
	SAR	AR3,*+,AR4
	.line	12
	LARK	AR4,22
	MAR	*0+,AR2
	SAR	AR4,* 
	.line	14
	LACK	0
	SBRK	38
	SACL	* 
L368:
	.line	16
	LAC	* 
	ADDK	1
	SACL	* ,AR1
	SUBK	1
	SACL	* ,AR0
	LDPK	_param_ptr
	LT	_param_ptr
	MPYK	3
	PAC
	ADLK	_param_struct+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR1
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR2
	ADRK	37
	LAR	AR4,* ,AR3
	LAC	* ,AR4
	SACL	*+,AR2
	SAR	AR4,* 
	SBRK	35
	SACL	* 
	.line	17
	LAC	* 
	SUBK	35
	BZ	LL69
	LAC	* 
	SBRK	2
	BNZ	L368
LL69:
	.line	19
	LARK	AR2,4
	MAR	*0+
	LAC	* 
	BNZ	L369
	.line	20
	MAR	* ,AR1
	LACK	-1
	SACL	*+
	LACK	1
	SACL	*+,AR3
	LARK	AR3,5
	MAR	*0+,AR1
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
	B	L370
L369:
	.line	23
	ADRK	35
	LAR	AR3,* ,AR3
	LACK	0
	MAR	*-
	SACL	* ,AR2
	.line	24
	SBRK	36
	SACL	*-
L371:
	.line	26
	LAC	* 
	ADDK	1
	SACL	* ,AR1
	SUBK	1
	SACL	* ,AR0
	LDPK	_param_ptr
	LT	_param_ptr
	MPYK	3
	PAC
	ADLK	_param_struct+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR1
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR2
	ADRK	2
	SACL	* 
	.line	27
	LAC	* 
	SUBK	35
	BZ	L372
	LAC	* 
	SUBK	95
	BZ	L372
	LAC	* 
	SUBK	46
	BZ	L372
	LAC	* 
	BZ	L372
	MAR	*-
	LAC	* 
	BNZ	L372
	.line	28
	MAR	*-
	LAC	*+
	SACL	* 
L372:
	.line	30
	LARK	AR2,3
	MAR	*0+
	LAC	* 
	BZ	L373
	.line	31
	ADRK	37
	LAR	AR3,* 
	SBRK	36
	LAC	* ,AR3
	SACL	*+,AR2
	ADRK	36
	SAR	AR3,* 
L373:
	.line	33
	LARK	AR2,4
	MAR	*0+
	LAC	* 
	SBRK	2
	BNZ	L371
	.line	35
	MAR	*+
	LAC	* 
	BZ	L374
	.line	36
	MAR	* ,AR1
	LACK	-1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+,AR3
	LARK	AR3,22
	MAR	*0+,AR1
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
L374:
	.line	38
	MAR	* ,AR1
	LACK	-1
	SACL	*+
	LACK	1
	SACL	*+,AR3
	LARK	AR3,5
	MAR	*0+,AR1
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
L370:
	.line	41
	LDPK	_flag_options
	LAC	_flag_options
	BNZ	L375
	.line	42
	LAC	_index_ab
	BNZ	L376
	.line	43
	LACK	0
	SACL	*+
	LACK	1
	SACL	*+
	LRLK	AR3,SL89
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
	B	L375
L376:
	.line	45
	LAC	_index_ab
	SUBK	1
	BNZ	L378
	.line	46
	LACK	0
	SACL	*+
	LACK	1
	SACL	*+
	LRLK	AR3,SL90
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
	B	L375
L378:
	.line	48
	LAC	_index_ab
	SUBK	2
	BNZ	L375
	.line	49
	LACK	0
	SACL	*+
	LACK	1
	SACL	*+
	LRLK	AR3,SL91
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
L375:
EPI0_23:
	.line	54
	SBRK	42
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	2685,000000000H,41

	.sym	_update_disp_right,_update_disp_right,32,2,0
	.globl	_update_disp_right

	.func	2693
******************************************************
* FUNCTION DEF : _update_disp_right
******************************************************
_update_disp_right:

LF24	.set	0

	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,5
	LAR	AR0,*0+,AR0

	.sym	_pos,-3+LF24,4,9,16
	.sym	_type,1,4,1,16
	.sym	_nstart,2,4,1,16
	.sym	_nlength,3,4,1,16
	.sym	_nfrac,4,4,1,16
	.line	2
	.line	7
	LDPK	_param_ptr
	LT	_param_ptr
	MPYK	3
	PAC
	ADLK	_param_struct+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	RSXM
	MAR	*+
	LAC	* ,0,AR2
	LARK	AR2,1
	MAR	*0+
	SACH	*+,1,AR0
	.line	8
	LT	_param_ptr
	MPYK	3
	PAC
	ADLK	_param_struct+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	MAR	*+
	LAC	* ,5,AR2
	ANDK	15,15
	ADLK	1,15
	SACH	*-,1
	.line	10
	LAC	* 
	BZ	L380
	.line	11
	MAR	*+,AR1
	LACK	0
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	LAC	_index_ab,1
	SACL	* 
	LT	_param_ptr
	MPYK	6
	PAC
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* 
	LT	_param_ptr
	MPYK	3
	PAC
	ADLK	_param_struct+0,0
	SACL	* 
	LAR	AR4,* ,AR3
	LAC	* ,AR4
	ADRK	2
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR1
	SACL	*+
	CALL	_disp_text
	SBRK	3
	B	L381
L380:
	.line	14
	LT	_param_ptr
	MPYK	3
	PAC
	ADLK	_param_struct+0,0
	MAR	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	MAR	*+
	LAC	* ,12,AR2
	ANDK	15,15
	ADRK	2
	SACH	*+,1,AR0
	.line	15
	LT	_param_ptr
	MPYK	3
	PAC
	ADLK	_param_struct+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LACK	7
	MAR	*+
	AND	* ,AR2
	SACL	*-,AR1
	.line	16
	SACL	*+,AR2
	LAC	*-,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	LAC	_index_ab,1
	SACL	* 
	LT	_param_ptr
	MPYK	6
	PAC
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	ZALS	*+
	ADDH	*-,AR1
	SACL	*+
	SACH	*+
	CALL	_disp_num
	SBRK	5
L381:
	.line	19
	LACK	1
	SACL	*+,AR2
	LARK	AR2,-3+LF24
	MAR	*0+
	LAC	* ,AR1
	SACL	*+
	LRLK	AR3,SL9
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
EPI0_24:
	.line	22
	SBRK	6
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	2714,000000000H,5

	.sym	_beep,_beep,32,2,0
	.globl	_beep

	.func	2722
******************************************************
* FUNCTION DEF : _beep
******************************************************
_beep:

LF25	.set	0

	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,2
	LAR	AR0,*0+,AR2

	.sym	_duration,-3+LF25,14,9,16
	.sym	_period,-4+LF25,14,9,16
	.sym	_i,1,4,1,16
	.line	2
	.line	5
	LACK	0
	LARK	AR2,1
	MAR	*0+
	SACL	* 
	ZALS	* 
	SBRK	4-LF25
	SUBS	* 
	BGEZ	L383
	MAR	*-,AR0
L382:
	.line	6
	LACK	64
	LDPK	_port0_copy
	OR	_port0_copy
	SACL	_port0_copy
	.line	7
	SACL	* 
	OUT	* ,00h,AR2
	.line	8
	ZALS	* ,AR1
	SACL	*+
	SACH	*+
	CALL	_wait
	SBRK	2
	.line	9
	LACK	65471
	LDPK	_port0_copy
	AND	_port0_copy
	SACL	_port0_copy
	.line	10
	MAR	* ,AR0
	SACL	* 
	OUT	* ,00h,AR2
	.line	11
	LARK	AR2,-4+LF25
	MAR	*0+
	ZALS	* ,AR1
	SACL	*+
	SACH	*+
	CALL	_wait
	SBRK	2
	.line	5
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	LAC	* 
	ADDK	1
	SACL	* 
	ZALS	* 
	SBRK	4-LF25
	SUBS	*-,AR0
	BLZ	L382
L383:
EPI0_25:
	.line	14
	MAR	* ,AR1
	SBRK	3
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	2735,000000000H,2

	.sym	_store_all,_store_all,32,2,0
	.globl	_store_all

	.func	2765
******************************************************
* FUNCTION DEF : _store_all
******************************************************
_store_all:
	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,2
	LAR	AR0,*0+

	.sym	_itemp,1,4,1,16
	.line	6
	LDPK	_record
	SPLK	#65535,_record
	.line	7
	LACK	0
	LDPK	_flash_locked
	SACL	_flash_locked
	.line	8
	LRLK	AR3,SL92
	SAR	AR3,*+
	LACK	1
	SACL	*+
	LALK	_record+0
	SACL	*+
	LACK	1
	SACL	*+
	LACK	24576
	SACL	*+
	CALL	_prog_flash
	SBRK	5
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	SACL	* 
	.line	9
	LAC	* 
	BNZ	L384
	.line	10
	LACK	4
	SACL	* 
	SSXM
	LAC	* 
	BLZ	L384
L385:
	.line	11
	LAC	* ,AR1
	LDPK	_params+60
	SACL	_params+60
	SACH	_params+61
	.line	12
	LACK	3
	LDPK	_params_changed_copy
	SACL	_params_changed_copy
	.line	13
	CALL	_store
	.line	10
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	LAC	* 
	SUBK	1
	SACL	* 
	SSXM
	LAC	* 
	BGEZ	L385
L384:
EPI0_26:
	.line	16
	MAR	* ,AR1
	SBRK	3
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	2780,000000000H,2

	.sym	_xmit,_xmit,32,2,0
	.globl	_xmit

	.func	2788
******************************************************
* FUNCTION DEF : _xmit
******************************************************
_xmit:

LF27	.set	0

	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,1
	LAR	AR0,*0+,AR0

	.sym	_text,-3+LF27,18,9,16
	.line	2
	.line	4
	IN	* ,0fff5h
	LAC	* 
	ANDK	65407
	SACL	* 
	OUT	* ,0fff5h,AR2
	.line	8
	LARK	AR2,-3+LF27
	MAR	*0+
	LAR	AR3,* ,AR3
	LAC	* 
	BZ	L388
L389:
	.line	9
	MAR	* ,AR0
	IN	* ,0fff6h
	ZALS	* 
	ANDK	4096
	BZ	L389
	.line	10
	MAR	* ,AR2
	LARK	AR2,-3+LF27
	MAR	*0+
	LAR	AR3,* ,AR3
	OUT	*+,0fff4h,AR2
	SAR	AR3,* 
	.line	8
	LAR	AR3,* ,AR3
	LAC	* 
	BNZ	L389
L388:
	.line	12
	LACK	13
	MAR	* ,AR0
	SACL	* 
	OUT	* ,0fff4h
L390:
	.line	13
	IN	* ,0fff6h
	ZALS	* 
	ANDK	4096
	BZ	L390
	.line	15
	LACK	26368
	SACL	* 
	OUT	* ,0fff6h
	.line	16
	IN	* ,0fff5h
	LAC	* 
	ORK	128
	SACL	* 
	OUT	* ,0fff5h,AR1
EPI0_27:
	.line	18
	SBRK	2
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	2805,000000000H,1

	.sym	_param_struct_search,_param_struct_search,36,2,0
	.globl	_param_struct_search

	.func	2821
******************************************************
* FUNCTION DEF : _param_struct_search
******************************************************
_param_struct_search:

LF28	.set	0

	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,3
	LAR	AR0,*0+,AR2

	.sym	_match_type,-3+LF28,20,9,16
	.sym	_i,1,4,1,16
	.sym	_cptr,2,18,1,16
	.line	2
	.line	6
	LACK	0
	LARK	AR2,1
	MAR	*0+
	SACL	* 
	ZALS	* 
	SUBK	46
	BGEZ	L392
L391:
	.line	8
	LT	*+,AR0
	MPYK	3
	PAC
	ADLK	_param_struct+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR2
	SACL	* ,AR1
	.line	10
	RSXM
	LALK	_parameter_str+0
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	_string_compare
	SBRK	2
	BZ	L393
	.line	11
	MAR	* ,AR2
	LARK	AR2,-3+LF28
	MAR	*0+
	LAR	AR3,* ,AR3
	LACK	1
	SACL	* ,AR2
	.line	12
	SSXM
	ADRK	4-LF28
	LAC	* 
	B	EPI0_28
L393:
	.line	14
	MAR	* ,AR2
	LARK	AR2,2
	MAR	*0+
	LAR	AR3,* ,AR3
	LAC	*+,AR2
	SAR	AR3,* 
	SUBK	32
	BNZ	L394
	.line	15
	RSXM
	LALK	_parameter_str+0
	MAR	* ,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	_string_compare
	SBRK	2
	BZ	L395
	.line	16
	MAR	* ,AR2
	LARK	AR2,-3+LF28
	MAR	*0+
	LAR	AR3,* ,AR3
	LACK	3
	SACL	* ,AR2
	.line	17
	SSXM
	ADRK	4-LF28
	LAC	* 
	B	EPI0_28
L395:
	.line	19
	RSXM
	LALK	_parameter_str+1
	SACL	*+,AR2
	LARK	AR2,2
	MAR	*0+
	LAC	* ,AR1
	SACL	*+
	CALL	_string_compare
	SBRK	2
	BZ	L394
	.line	20
	MAR	* ,AR2
	LARK	AR2,-3+LF28
	MAR	*0+
	LAR	AR3,* ,AR3
	LACK	2
	SACL	* ,AR2
	.line	21
	SSXM
	ADRK	4-LF28
	LAC	* 
	B	EPI0_28
L394:
	.line	6
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	LAC	* 
	ADDK	1
	SACL	* 
	ZALS	* 
	SUBK	46
	BLZ	L391
L392:
	.line	26
	SBRK	4-LF28
	LAR	AR3,* ,AR3
	LACK	0
	SACL	* 
	.line	27
	LACK	0
EPI0_28:
	.line	28
	MAR	* ,AR1
	SBRK	4
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	2848,000000000H,3

	.sym	_string_compare,_string_compare,36,2,0
	.globl	_string_compare

	.func	2856
******************************************************
* FUNCTION DEF : _string_compare
******************************************************
_string_compare:

LF29	.set	0

	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,3
	LAR	AR0,*0+,AR2

	.sym	_s1,-3+LF29,18,9,16
	.sym	_s2,-4+LF29,18,9,16
	.sym	_ctemp1,1,2,1,16
	.sym	_ctemp2,2,2,1,16
	.line	2
	LARK	AR2,-3+LF29
	MAR	*0+
L397:
	.line	6
	LAR	AR3,* ,AR3
	LAC	*+,AR2
	SAR	AR3,* ,AR1
	SACL	*+
	CALL	_tolower
	MAR	*-,AR2
	LARK	AR2,1
	MAR	*0+
	SACL	* 
	.line	7
	SBRK	5-LF29
	LAR	AR3,* ,AR3
	LAC	*+,AR2
	SAR	AR3,* ,AR1
	SACL	*+
	CALL	_tolower
	MAR	*-,AR2
	LARK	AR2,2
	MAR	*0+
	SACL	*-
	.line	8
	LAC	* 
	BZ	LL77
	MAR	*+
	LAC	* 
	BNZ	L398
LL77:
	LACK	1
	B	EPI0_29
L398:
	.line	9
	MAR	*-
	LAC	*+
	SUB	* 
	BZ	L399
	LACK	0
	B	EPI0_29
L399:
	.line	10
	SBRK	5-LF29
	B	L397
EPI0_29:
	.line	11
	MAR	* ,AR1
	SBRK	4
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	2866,000000000H,3

	.sym	_inc_dec_param_ptr,_inc_dec_param_ptr,36,2,0
	.globl	_inc_dec_param_ptr

	.func	2913
******************************************************
* FUNCTION DEF : _inc_dec_param_ptr
******************************************************
_inc_dec_param_ptr:
	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,6
	LAR	AR0,*0+,AR2

	.sym	_func_a,1,4,1,16
	.sym	_func_b,2,4,1,16
	.sym	_func_c,3,4,1,16
	.line	5
	LARK	AR2,1
	MAR	*0+
	BLKD	#_params,*+
	.line	6
	BLKD	#_params+2,*+
	.line	7
	BLKD	#_params+4,* 
	B	L400
L401:
	.line	11
	LDPK	_param_ptr
	LAC	_param_ptr
	BNZ	L402
	.line	12
	LACK	13
	SACL	_param_ptr
	.line	13
	LACK	0
	SACL	_index_ab
	.line	14
	LACK	1
	SACL	_flag_options
	B	L405
L402:
	.line	16
	MAR	* ,AR2
	LARK	AR2,3
	MAR	*0+
	LAC	* ,AR0
	ADLK	_param_ptr_start+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	_param_ptr
	SUB	* 
	BNZ	L404
	.line	17
	LACK	0
	SACL	_param_ptr
	.line	18
	LACK	1
	B	EPI0_30
L404:
	.line	21
	LAC	_param_ptr
	SUBK	1
	SACL	_param_ptr
	.line	23
	B	L405
L406:
	.line	26
	LDPK	_param_ptr
	LAC	_param_ptr
	SUBK	1
	BNZ	L407
	.line	27
	MAR	* ,AR2
	LARK	AR2,3
	MAR	*0+
	LAC	* ,AR0
	ADLK	_param_ptr_end+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	BLDD	* ,#_param_ptr
	.line	28
	LACK	2
	SACL	_index_ab
	.line	29
	LACK	0
	SACL	_flag_options
	B	L405
L407:
	.line	32
	LAC	_param_ptr
	SUBK	1
	SACL	_param_ptr
	.line	33
	LAC	_param_ptr
	SUBK	1
	BNZ	L405
	.line	34
	LACK	1
	B	EPI0_30
	.line	37
L410:
	.line	40
	LDPK	_param_ptr
	LAC	_param_ptr
	BNZ	L411
	.line	41
	MAR	* ,AR2
	LARK	AR2,3
	MAR	*0+
	LAC	* ,AR0
	ADLK	_param_ptr_start+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	BLDD	* ,#_param_ptr
	B	L405
L411:
	.line	43
	MAR	* ,AR2
	LARK	AR2,3
	MAR	*0+
	LAC	* ,AR0
	ADLK	_param_ptr_end+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	_param_ptr
	SUB	* 
	BNZ	L413
	.line	44
	LACK	1
	SACL	_param_ptr
	.line	45
	LACK	0
	SACL	_index_ab
	.line	46
	LACK	1
	SACL	_flag_options
	.line	47
	LACK	1
	B	EPI0_30
L413:
	.line	50
	LAC	_param_ptr
	ADDK	1
	SACL	_param_ptr
	.line	52
	B	L405
L414:
	.line	55
	LDPK	_param_ptr
	LAC	_param_ptr
	SUBK	13
	BNZ	L415
	.line	56
	LACK	0
	SACL	_param_ptr
	.line	57
	LACK	2
	SACL	_index_ab
	.line	58
	LACK	0
	SACL	_flag_options
	.line	59
	LACK	1
	B	EPI0_30
L415:
	.line	62
	LAC	_param_ptr
	ADDK	1
	SACL	_param_ptr
	.line	64
	B	L405
L416:
	.line	67
	LDPK	_index_ab
	LAC	_index_ab
	BZ	L417
	.line	68
	LAC	_param_ptr
	BNZ	L418
	.line	69
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	LAC	* ,AR0
	ADLK	_param_ptr_end+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	BLDD	* ,#_param_ptr
	.line	70
	LACK	0
	SACL	_index_ab
	B	L405
L418:
	.line	72
	MAR	* ,AR2
	LARK	AR2,2
	MAR	*0+
	LAC	* ,AR0
	ADLK	_param_ptr_start+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	_param_ptr
	SUB	* 
	BNZ	L420
	.line	73
	LACK	0
	SACL	_param_ptr
	.line	74
	LACK	1
	B	EPI0_30
L420:
	.line	77
	LAC	_param_ptr
	SUBK	1
	SACL	_param_ptr
	B	L405
L417:
	.line	81
	LAC	_param_ptr
	BNZ	L422
	.line	82
	LACK	13
	SACL	_param_ptr
	.line	83
	LACK	0
	SACL	_index_ab
	.line	84
	LACK	1
	SACL	_flag_options
	B	L405
L422:
	.line	86
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	LAC	* ,AR0
	ADLK	_param_ptr_start+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	_param_ptr
	SUB	* 
	BNZ	L424
	.line	87
	LACK	0
	SACL	_param_ptr
	.line	88
	LACK	1
	B	EPI0_30
L424:
	.line	91
	LAC	_param_ptr
	SUBK	1
	SACL	_param_ptr
	.line	94
	B	L405
L425:
	.line	97
	LDPK	_param_ptr
	LAC	_param_ptr
	SUBK	1
	BNZ	L426
	.line	98
	MAR	* ,AR2
	LARK	AR2,2
	MAR	*0+
	LAC	* ,AR0
	ADLK	_param_ptr_end+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	BLDD	* ,#_param_ptr
	.line	99
	LACK	1
	SACL	_index_ab
	.line	100
	LACK	0
	SACL	_flag_options
	B	L405
L426:
	.line	103
	LAC	_param_ptr
	SUBK	1
	SACL	_param_ptr
	.line	104
	LAC	_param_ptr
	SUBK	1
	BNZ	L405
	.line	105
	LACK	1
	B	EPI0_30
	.line	108
L429:
	.line	111
	LDPK	_index_ab
	LAC	_index_ab
	BZ	L430
	.line	112
	LAC	_param_ptr
	BNZ	L431
	.line	113
	MAR	* ,AR2
	LARK	AR2,2
	MAR	*0+
	LAC	* ,AR0
	ADLK	_param_ptr_start+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	BLDD	* ,#_param_ptr
	B	L405
L431:
	.line	115
	MAR	* ,AR2
	LARK	AR2,2
	MAR	*0+
	LAC	* ,AR0
	ADLK	_param_ptr_end+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	_param_ptr
	SUB	* 
	BNZ	L433
	.line	116
	LACK	1
	SACL	_param_ptr
	.line	117
	LACK	0
	SACL	_index_ab
	.line	118
	LACK	1
	SACL	_flag_options
	.line	119
	LACK	1
	B	EPI0_30
L433:
	.line	122
	LAC	_param_ptr
	ADDK	1
	SACL	_param_ptr
	B	L405
L430:
	.line	126
	LAC	_param_ptr
	BNZ	L435
	.line	127
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	LAC	* ,AR0
	ADLK	_param_ptr_start+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	BLDD	* ,#_param_ptr
	B	L405
L435:
	.line	129
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	LAC	* ,AR0
	ADLK	_param_ptr_end+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	_param_ptr
	SUB	* 
	BNZ	L437
	.line	130
	LACK	0
	SACL	_param_ptr
	.line	131
	LACK	1
	SACL	_index_ab
	.line	132
	LACK	1
	B	EPI0_30
L437:
	.line	135
	LAC	_param_ptr
	ADDK	1
	SACL	_param_ptr
	.line	138
	B	L405
L438:
	.line	141
	LDPK	_param_ptr
	LAC	_param_ptr
	SUBK	13
	BNZ	L439
	.line	142
	LACK	0
	SACL	_param_ptr
	.line	143
	SACL	_flag_options
	.line	144
	LACK	1
	B	EPI0_30
L439:
	.line	147
	LAC	_param_ptr
	ADDK	1
	SACL	_param_ptr
	.line	149
	B	L405
L440:
	.line	152
	LDPK	_param_ptr
	LAC	_param_ptr
	BNZ	L441
	.line	153
	LACK	13
	SACL	_param_ptr
	.line	154
	LACK	1
	SACL	_flag_options
	B	L405
L441:
	.line	156
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	LAC	* ,AR0
	ADLK	_param_ptr_start+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	_param_ptr
	SUB	* 
	BNZ	L443
	.line	157
	LACK	0
	SACL	_param_ptr
	.line	158
	LACK	1
	B	EPI0_30
L443:
	.line	161
	LAC	_param_ptr
	SUBK	1
	SACL	_param_ptr
	.line	163
	B	L405
L444:
	.line	166
	LDPK	_param_ptr
	LAC	_param_ptr
	SUBK	1
	BNZ	L445
	.line	167
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	LAC	* ,AR0
	ADLK	_param_ptr_end+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	BLDD	* ,#_param_ptr
	.line	168
	LACK	0
	SACL	_flag_options
	B	L405
L445:
	.line	171
	LAC	_param_ptr
	SUBK	1
	SACL	_param_ptr
	.line	172
	LAC	_param_ptr
	SUBK	1
	BNZ	L405
	.line	173
	LACK	1
	B	EPI0_30
	.line	176
L448:
	.line	179
	LDPK	_param_ptr
	LAC	_param_ptr
	BNZ	L449
	.line	180
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	LAC	* ,AR0
	ADLK	_param_ptr_start+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	BLDD	* ,#_param_ptr
	B	L405
L449:
	.line	182
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	LAC	* ,AR0
	ADLK	_param_ptr_end+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	_param_ptr
	SUB	* 
	BNZ	L451
	.line	183
	LACK	1
	SACL	_param_ptr
	.line	184
	SACL	_flag_options
	.line	185
	LACK	1
	B	EPI0_30
L451:
	.line	188
	LAC	_param_ptr
	ADDK	1
	SACL	_param_ptr
	.line	190
	B	L405
L452:
	.line	193
	LDPK	_param_ptr
	LAC	_param_ptr
	SUBK	13
	BNZ	L453
	.line	194
	LACK	0
	SACL	_param_ptr
	.line	195
	SACL	_flag_options
	.line	196
	LACK	1
	B	EPI0_30
L453:
	.line	199
	LAC	_param_ptr
	ADDK	1
	SACL	_param_ptr
	.line	201
	B	L405
	.line	204
L400:
	.line	9
	LDPK	_cw
	LAC	_cw,1
	MAR	*+,AR1
	SACL	* 
	SSXM
	LAC	* ,AR2
	SACL	*+
	SACH	*-,AR0
	LDPK	_params+37
	LAC	_params+37,2
	SACL	*
	ZALH	*,AR2
	RSXM
	ADD	_params+36,2
	ADDS	*+
	ADDH	* ,AR0
	SSXM
	LDPK	_flag_options
	ADD	_flag_options
	SUBK	11
	BGZ	L405
	ADDK	11
	BLZ	L405
	ADLK	LL80
	TBLR	*
	LAC	*
	BACC
	.sect	".switch"
LL80:
	.word	L401
	.word	L406
	.word	L410
	.word	L414
	.word	L416
	.word	L425
	.word	L429
	.word	L438
	.word	L440
	.word	L444
	.word	L448
	.word	L452
	.text
L405:
	.line	207
	LACK	0
EPI0_30:
	.line	208
	MAR	* ,AR1
	SBRK	7
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	3120,000000000H,6

	.sym	_inc_dec_mod,_inc_dec_mod,32,2,0
	.globl	_inc_dec_mod

	.func	3127
******************************************************
* FUNCTION DEF : _inc_dec_mod
******************************************************
_inc_dec_mod:

LF31	.set	0

	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,1
	LAR	AR0,*0+,AR2

	.sym	_ptr,-3+LF31,20,9,16
	.sym	_mod,-4+LF31,4,9,16
	.line	2
	.line	3
	LARK	AR2,-4+LF31
	MAR	*0+
	LAC	* 
	BZ	L455
	.line	4
	LDPK	_cw
	LAC	_cw
	BZ	L456
	.line	5
	MAR	*+
	LAR	AR3,*-,AR3
	LAC	* 
	ADDK	1
	SACL	* 
	.line	6
	SSXM
	LAC	* ,AR2
	SUB	* 
	BLZ	L460
	LACK	0
	MAR	* ,AR3
	SACL	* 
	B	L460
L456:
	.line	9
	MAR	*+
	LAR	AR3,* ,AR3
	LAC	* 
	SUBK	1
	SACL	* 
	.line	10
	SSXM
	LAC	* 
	BGEZ	L460
	MAR	* ,AR2
	MAR	*-
	LAC	* ,AR3
	SUBK	1
	SACL	* 
	B	L460
L455:
	.line	14
	MAR	*+
	LAR	AR3,* ,AR3
	LACK	0
	SACL	* 
L460:
EPI0_31:
	.line	16
	MAR	* ,AR1
	SBRK	2
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	3142,000000000H,1

	.sym	_add_sub_pow,_add_sub_pow,32,2,0
	.globl	_add_sub_pow

	.func	3149
******************************************************
* FUNCTION DEF : _add_sub_pow
******************************************************
_add_sub_pow:

LF32	.set	0

	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,3
	LAR	AR0,*0+

	.sym	_ptr,-3+LF32,21,9,16
	.sym	_pow,-4+LF32,4,9,16
	.sym	_delta,1,5,1,32
	.line	2
	B	L461
L462:
	.line	7
	LACK	1
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	SACL	*+
	SACH	* 
	.line	8
	B	L463
L464:
	.line	10
	LACK	10
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	SACL	*+
	SACH	* 
	.line	11
	B	L463
L465:
	.line	13
	LACK	100
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	SACL	*+
	SACH	* 
	.line	14
	B	L463
L466:
	.line	16
	LACK	1000
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	SACL	*+
	SACH	* 
	.line	17
	B	L463
L467:
	.line	19
	LACK	10000
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	SACL	*+
	SACH	* 
	.line	20
	B	L463
L468:
	.line	22
	LALK	3,15
	ORK	1696
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	SACL	*+
	SACH	* 
	.line	23
	B	L463
L469:
	.line	25
	LALK	30,15
	ORK	16960
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	SACL	*+
	SACH	* 
	.line	26
	B	L463
L470:
	.line	28
	LALK	305,15
	ORK	5760
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	SACL	*+
	SACH	* 
	.line	29
	B	L463
L471:
	.line	31
	LALK	3051,15
	ORK	24832
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	SACL	*+
	SACH	* 
	.line	32
	B	L463
L472:
	.line	34
	LALK	30517,15
	ORK	18944
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	SACL	*+
	SACH	* 
	.line	35
	B	L463
L473:
	.line	37
	LACK	1
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	SACL	*+
	SACH	* 
	.line	38
	B	L463
L461:
	.line	5
	SSXM
	MAR	* ,AR2
	LARK	AR2,-4+LF32
	MAR	*0+
	LAC	* ,AR0
	SUBK	9
	BGZ	L473
	ADDK	9
	BLZ	L473
	ADLK	LL84
	TBLR	*
	LAC	*
	BACC
	.sect	".switch"
LL84:
	.word	L462
	.word	L464
	.word	L465
	.word	L466
	.word	L467
	.word	L468
	.word	L469
	.word	L470
	.word	L471
	.word	L472
	.text
L463:
	.line	41
	LDPK	_cw
	LAC	_cw
	BZ	L474
	.line	42
	MAR	* ,AR2
	LARK	AR2,-3+LF32
	MAR	*0+
	LAR	AR3,* ,AR3
	ZALS	*+
	ADDH	*-,AR2
	ADRK	4-LF32
	ADDS	*+
	ADDH	* ,AR3
	SACL	*+
	SACH	*-
	B	L475
L474:
	.line	45
	MAR	* ,AR2
	LARK	AR2,-3+LF32
	MAR	*0+
	LAR	AR3,* ,AR3
	ZALS	*+
	ADDH	*-,AR2
	ADRK	4-LF32
	SUBS	*+
	SUBH	* ,AR3
	SACL	*+
	SACH	*-
L475:
	.line	48
	BNV	$+2
	LT	_max_value
	MPYK	1
	LPH	_max_value+1
	ZALS	*+
	ADDH	*-
	SPAC
	BNV	$+6
	BGZ	L476
	B	$+4
	BLEZ	L476
	.line	49
	RPTK	1
	BLKD	#_max_value,*+
L476:
	.line	52
	MAR	* ,AR2
	SBRK	5-LF32
	LAR	AR3,* 
	BNV	$+2
	LT	_min_value
	MPYK	1
	LPH	_min_value+1
	MAR	* ,AR3
	ZALS	*+
	ADDH	*-
	SPAC
	BNV	$+6
	BLZ	L477
	B	$+4
	BGEZ	L477
	.line	53
	RPTK	1
	BLKD	#_min_value,*+
L477:
EPI0_32:
	.line	56
	MAR	* ,AR1
	SBRK	4
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	3204,000000000H,3

	.sym	_update_dsp,_update_dsp,32,2,0
	.globl	_update_dsp

	.func	3213
******************************************************
* FUNCTION DEF : _update_dsp
******************************************************
_update_dsp:

LF33	.set	0

	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,18
	LAR	AR0,*0+

	.sym	_param_ptr_tmp,-3+LF33,4,9,16
	.sym	_index_ab_tmp,-4+LF33,4,9,16
	.sym	_ltemp,1,5,1,32
	.sym	_itemp,3,4,1,16
	.sym	_iorder,4,4,1,16
	.sym	_cursor_temp,5,4,1,16
	.sym	_f1,6,6,1,32
	.sym	_f2,8,6,1,32
	.sym	_fcenter,10,6,1,32
	.sym	_fwidth,12,6,1,32
	.sym	_ftemp,14,6,1,32
	.line	2
	B	L478
L479:
	.line	10
	LDPK	_params_changed_copy
	LAC	_params_changed_copy
	SUBK	1
	BZ	EPI0_33
	.line	11
	MAR	* ,AR2
	LARK	AR2,-4+LF33
	MAR	*0+
	LAC	* ,AR1
	SACL	*+
	CALL	_gain
	MAR	*-,AR2
	.line	12
	LARK	AR2,-4+LF33
	MAR	*0+
	LAC	* ,AR1
	SACL	*+,AR0
	SACL	* 
	LAC	* ,1
	ADLK	_params+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR0
	ADLK	_param_ptr_start+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR1
	SACL	*+
	CALL	_update_dsp
	SBRK	2
	.line	13
	B	EPI0_33
L482:
	.line	16
	LACK	1
	LDPK	_assembly_flag
	OR	_assembly_flag
	SACL	_assembly_flag
	.line	17
	B	EPI0_33
L483:
	.line	20
	LDPK	_params+12
	BLDD	_params+12,#_auto_vu_count
	.line	21
	B	EPI0_33
L484:
	.line	24
	LACK	1
	LDPK	_min_value
	SACL	_min_value
	SACH	_min_value+1
	.line	25
	SSXM
	LAC	_max_in_level
	SACL	_max_value
	SACH	_max_value+1
	.line	26
	LAC	_params_changed_copy
	SUBK	1
	BZ	EPI0_33
	.line	27
	CALL	_set_all_gains,AR1
	.line	28
	B	EPI0_33
L486:
	.line	31
	LDPK	_params_changed_copy
	LAC	_params_changed_copy
	SUBK	1
	BZ	EPI0_33
	.line	32
	CALL	_set_fsample,AR1
	.line	33
	CALL	_init_freq_params
L488:
	.line	37
	LDPK	_params_changed_copy
	LAC	_params_changed_copy
	SUBK	1
	BZ	EPI0_33
	.line	39
	LDPK	_params+36
	ZALS	_params+36
	ADDH	_params+37
	BNZ	L490
	.line	40
	MAR	* ,AR1
	LACK	2
	SACL	*+
	CALL	_gain
	MAR	*-
	.line	41
	LACK	2
	SACL	*+,AR0
	LDPK	_params+4
	LAC	_params+4
	ADLK	_param_ptr_start+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR1
	SACL	*+
	CALL	_update_dsp
	SBRK	2
	B	EPI0_33
L490:
	.line	43
	LT	_params+36
	MPYK	1
	LPH	_params+37
	LACK	1
	SPAC
	BNZ	L492
	.line	44
	BNV	$+2
	LDPK	_params+102
	LT	_params+102
	MPYK	1
	LPH	_params+103
	LACK	128
	SPAC
	BNV	$+6
	BLZ	L493
	B	$+4
	BGEZ	L493
	LACK	127
	SACL	_params+102
	SACH	_params+103
L493:
	.line	45
	BNV	$+2
	LT	_params+120
	MPYK	1
	LPH	_params+121
	LACK	128
	SPAC
	BNV	$+6
	BLZ	L494
	B	$+4
	BGEZ	L494
	LACK	127
	SACL	_params+120
	SACH	_params+121
L494:
	.line	46
	BNV	$+2
	LT	_params+156
	MPYK	1
	LPH	_params+157
	LACK	128
	SPAC
	BNV	$+6
	BLZ	L495
	B	$+4
	BGEZ	L495
	LACK	127
	SACL	_params+156
	SACH	_params+157
L495:
	.line	47
	BNV	$+2
	LT	_params+192
	MPYK	1
	LPH	_params+193
	LACK	128
	SPAC
	BNV	$+6
	BLZ	L496
	B	$+4
	BGEZ	L496
	LACK	127
	SACL	_params+192
	SACH	_params+193
L496:
	.line	48
	BNV	$+2
	LDPK	_params+240
	LT	_params+240
	MPYK	1
	LPH	_params+241
	LACK	128
	SPAC
	BNV	$+6
	BLZ	L497
	B	$+4
	BGEZ	L497
	LACK	127
	SACL	_params+240
	SACH	_params+241
L497:
	.line	49
	MAR	* ,AR1
	LACK	0
	SACL	*+
	CALL	_gain
	MAR	*-
	.line	50
	LACK	0
	SACL	*+,AR0
	LDPK	_params
	LAC	_params
	ADLK	_param_ptr_start+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR1
	SACL	*+
	CALL	_update_dsp
	SBRK	2
	.line	51
	LACK	1
	SACL	*+
	CALL	_gain
	MAR	*-
	.line	52
	LACK	1
	SACL	*+,AR0
	LDPK	_params+2
	LAC	_params+2
	ADLK	_param_ptr_start+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR1
	SACL	*+
	CALL	_update_dsp
	SBRK	2
	B	EPI0_33
L492:
	.line	55
	LALK	_no_func_b+0
	LDPK	_func_addr_b
	SACL	_func_addr_b
	.line	56
	MAR	* ,AR1
	LACK	0
	SACL	*+
	CALL	_gain
	MAR	*-
	.line	57
	LACK	0
	SACL	*+,AR0
	LDPK	_params
	LAC	_params
	ADLK	_param_ptr_start+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR1
	SACL	*+
	CALL	_update_dsp
	SBRK	2
	.line	59
	B	EPI0_33
L498:
	.line	62
	LDPK	_params_changed_copy
	LAC	_params_changed_copy
	SUBK	1
	BZ	EPI0_33
	.line	63
	LDPK	_params+30
	LAC	_params+30
	BZ	LL87
	LACK	8
	LDPK	_assembly_flag
	OR	_assembly_flag
	MAR	* ,AR1
	SACL	* 
	LAC	* 
	B	LL88
LL87:
	LACK	-9
	LDPK	_assembly_flag
	AND	_assembly_flag
	MAR	* ,AR1
	SACL	* 
	LAC	* 
LL88:
	SACL	_assembly_flag
	.line	64
	B	EPI0_33
L500:
	.line	67
	LDPK	_params_changed_copy
	LAC	_params_changed_copy
	SUBK	1
	BZ	EPI0_33
	.line	68
	LDPK	_params+42
	LAC	_params+42
	BZ	LL89
	LACK	4
	LDPK	_assembly_flag
	OR	_assembly_flag
	MAR	* ,AR1
	SACL	* 
	LAC	* 
	B	LL90
LL89:
	LACK	-5
	LDPK	_assembly_flag
	AND	_assembly_flag
	MAR	* ,AR1
	SACL	* 
	LAC	* 
LL90:
	SACL	_assembly_flag
	.line	69
	B	EPI0_33
L502:
	.line	72
	LDPK	_params_changed_copy
	LAC	_params_changed_copy
	SUBK	1
	BZ	EPI0_33
	.line	74
	B	EPI0_33
L504:
	.line	77
	LDPK	_params_changed_copy
	LAC	_params_changed_copy
	SUBK	3
	BNZ	EPI0_33
	.line	78
	CALL	_initialize,AR1
	.line	79
	LALK	15,15
	ORK	8480
	SACL	*+
	SACH	*+
	CALL	_wait
	SBRK	2
	.line	80
	CALL	_store_all
	.line	81
	CALL	_update_disp_left
	.line	82
	LACK	1
	SACL	*+
	CALL	_update_disp_right
	MAR	*-
	.line	84
	B	EPI0_33
L506:
	.line	87
	CALL	_store,AR1
	.line	88
	B	EPI0_33
L507:
	.line	91
	CALL	_recall,AR1
	.line	92
	B	EPI0_33
L508:
	.line	97
	MAR	* ,AR1
	LACK	2
	SACL	*+
	LACK	4
	SACL	*+
	LACK	13
	SACL	*+
	LACK	220
	SACL	*+
	SACH	*+
	CALL	_disp_num
	SBRK	5
	.line	98
	LACK	0
	SACL	*+
	LACK	1
	SACL	*+
	LRLK	AR3,SL9
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
	.line	104
	B	EPI0_33
L509:
	.line	109
	MAR	* ,AR1
	LACK	0
	SACL	*+
	LACK	6
	SACL	*+
	LACK	11
	SACL	*+
	LDPK	_serial_number
	ZALS	_serial_number
	ADDH	_serial_number+1
	SACL	*+
	SACH	*+
	CALL	_disp_num
	SBRK	5
	.line	110
	LACK	0
	SACL	*+
	LACK	1
	SACL	*+
	LRLK	AR3,SL9
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
	.line	116
	B	EPI0_33
L510:
	.line	119
	SSXM
	LACK	-10000
	LDPK	_min_value
	SACL	_min_value
	SACH	_min_value+1
	.line	120
	LACK	10000
	SACL	_max_value
	SACH	_max_value+1
	.line	121
	LAC	_params_changed_copy
	SUBK	1
	BZ	EPI0_33
	.line	122
	MAR	* ,AR2
	LARK	AR2,-4+LF33
	MAR	*0+
	LAC	* ,AR1
	SACL	*+
	CALL	_gain
	MAR	*-,AR2
	.line	124
	LARK	AR2,-4+LF33
	MAR	*0+
	LAC	* 
	ADDK	1
	ADRK	7-LF33
	SACL	* 
	.line	125
	BIT	* ,15
	BBZ	L512
	.line	126
	LALK	_no_func_a+0
	LDPK	_func_addr_a
	SACL	_func_addr_a
L512:
	.line	128
	BIT	* ,14
	BBZ	EPI0_33
	.line	129
	LALK	_no_func_b+0
	LDPK	_func_addr_b
	SACL	_func_addr_b
	.line	131
	B	EPI0_33
L514:
	.line	133
	SSXM
	LACK	-10000
	LDPK	_min_value
	SACL	_min_value
	SACH	_min_value+1
	.line	134
	LACK	10000
	SACL	_max_value
	SACH	_max_value+1
	.line	135
	LAC	_params_changed_copy
	SUBK	1
	BZ	EPI0_33
	.line	136
	MAR	* ,AR2
	LARK	AR2,-4+LF33
	MAR	*0+
	LAC	* ,AR1
	SACL	*+
	CALL	_gain
	MAR	*-,AR2
	.line	138
	LARK	AR2,-4+LF33
	MAR	*0+
	LAC	* 
	ADDK	1
	ADRK	7-LF33
	SACL	* 
	.line	139
	BIT	* ,15
	BBZ	L516
	.line	140
	LALK	_allpass_func_a+0
	LDPK	_func_addr_a
	SACL	_func_addr_a
L516:
	.line	142
	BIT	* ,14
	BBZ	EPI0_33
	.line	143
	LALK	_allpass_func_b+0
	LDPK	_func_addr_b
	SACL	_func_addr_b
	.line	145
	B	EPI0_33
L525:
	.line	155
	SSXM
	LACK	-10000
	LDPK	_min_value
	SACL	_min_value
	SACH	_min_value+1
	.line	156
	LACK	10000
	SACL	_max_value
	SACH	_max_value+1
	.line	157
	LAC	_params_changed_copy
	SUBK	1
	BZ	EPI0_33
	.line	158
	MAR	* ,AR2
	LARK	AR2,-4+LF33
	MAR	*0+
	LAC	* ,AR1
	SACL	*+
	CALL	_gain
	MAR	*-
	.line	159
	B	EPI0_33
L528:
	.line	163
	LACK	3
	LDPK	_min_value
	SACL	_min_value
	SACH	_min_value+1
	.line	164
	LDPK	_params+36
	LT	_params+36
	MPYK	1
	LPH	_params+37
	LACK	2
	SPAC
	BNZ	L529
	.line	165
	LACK	256
	LDPK	_max_value
	SACL	_max_value
	SACH	_max_value+1
	B	L530
L529:
	.line	168
	LACK	128
	LDPK	_max_value
	SACL	_max_value
	SACH	_max_value+1
L530:
	.line	170
	LAC	_params_changed_copy
	SUBK	1
	BZ	EPI0_33
	.line	171
	MAR	* ,AR2
	LARK	AR2,-4+LF33
	MAR	*0+
	LAC	*+,1,AR1
	SACL	* ,AR2
	LAC	* ,AR0
	SUBK	1
	SACL	* 
	LT	* ,AR1
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	ZALS	*+
	ADDH	*-,AR1
	CALL	F$$LTOF
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	ADRK	9-LF33
	SACL	*+
	SACH	* 
	.line	172
	SBRK	11-LF33
	LAC	*+,1,AR1
	SACL	* ,AR2
	LT	* ,AR1
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR2
	ADRK	7-LF33
	SACL	* 

	.sym	_compute_filt,L532,0,6,0
	.line	173
	B	L532
L534:
	.line	177
	LACK	3
	LDPK	_min_value
	SACL	_min_value
	SACH	_min_value+1
	.line	178
	LDPK	_params+36
	LT	_params+36
	MPYK	1
	LPH	_params+37
	LACK	2
	SPAC
	BNZ	L535
	.line	179
	LACK	256
	LDPK	_max_value
	SACL	_max_value
	SACH	_max_value+1
	B	L536
L535:
	.line	182
	LACK	128
	LDPK	_max_value
	SACL	_max_value
	SACH	_max_value+1
L536:
	.line	184
	LAC	_params_changed_copy
	SUBK	1
	BZ	EPI0_33
	.line	185
	MAR	* ,AR2
	LARK	AR2,-4+LF33
	MAR	*0+
	LAC	*+,1,AR1
	SACL	* ,AR2
	LAC	* ,AR0
	SUBK	4
	SACL	* 
	LT	* ,AR1
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	ZALS	*+
	ADDH	*-,AR1
	CALL	F$$LTOF
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	ADRK	9-LF33
	SACL	*+
	SACH	* 
	.line	186
	SBRK	11-LF33
	LAC	*+,1,AR1
	SACL	* ,AR2
	LAC	* ,AR0
	SUBK	3
	SACL	* 
	LT	* ,AR1
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	ZALS	*+
	ADDH	*-,AR1
	CALL	F$$LTOF
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	ADRK	11-LF33
	SACL	*+
	SACH	* 
	.line	187
	SBRK	13-LF33
	LAC	*+,1,AR1
	SACL	* ,AR2
	LT	* ,AR1
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR2
	ADRK	7-LF33
	SACL	* 
	.line	188
	B	L532
L539:
	.line	192
	MAR	* ,AR1
	RPTK	1
	BLKD	_fsample+0,*+
	LALK	FL2
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$FTOL
	LDPK	_min_value
	SACL	_min_value
	SACH	_min_value+1
	.line	193
	RPTK	1
	BLKD	_fsample+0,*+
	LALK	FL3
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$FTOL
	SACL	_max_value
	SACH	_max_value+1
	.line	194
	LAC	_params_changed_copy
	SUBK	1
	BZ	EPI0_33
	.line	195
	MAR	* ,AR2
	LARK	AR2,-4+LF33
	MAR	*0+
	LAC	*+,1,AR1
	SACL	* ,AR2
	LT	* ,AR1
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	ZALS	*+
	ADDH	*-,AR1
	CALL	F$$LTOF
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	ADRK	9-LF33
	SACL	*+
	SACH	* 
	.line	196
	SBRK	11-LF33
	LAC	*+,1,AR1
	SACL	* ,AR2
	LAC	* ,AR0
	ADDK	1
	SACL	* 
	LT	* ,AR1
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR2
	ADRK	7-LF33
	SACL	* 
	.line	197
	B	L532
L542:
	.line	201
	MAR	* ,AR1
	RPTK	1
	BLKD	_fsample+0,*+
	LALK	FL2
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$FTOL
	LDPK	_min_value
	SACL	_min_value
	SACH	_min_value+1
	.line	202
	RPTK	1
	BLKD	_fsample+0,*+
	LALK	FL4
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$FTOL
	SACL	_max_value
	SACH	_max_value+1
	.line	203
	LAC	_params_changed_copy
	SUBK	1
	BZ	EPI0_33
	.line	204
	MAR	* ,AR2
	LARK	AR2,-4+LF33
	MAR	*0+
	LAC	*+,1,AR1
	SACL	* ,AR2
	LT	* ,AR1
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	ZALS	*+
	ADDH	*-,AR1
	CALL	F$$LTOF
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	ADRK	9-LF33
	SACL	*+
	SACH	* 
	.line	205
	SBRK	11-LF33
	LAC	*+,1,AR1
	SACL	* ,AR2
	LAC	* ,AR0
	ADDK	1
	SACL	* 
	LT	* ,AR1
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	ZALS	*+
	ADDH	*-,AR1
	CALL	F$$LTOF
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	ADRK	11-LF33
	SACL	*+
	SACH	* ,AR1
	.line	206
	RPTK	1
	BLKD	_fsample+0,*+
	LALK	FL5
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	MAR	* ,AR2
	SBRK	3
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$SUB
	CALL	F$$LT
	BZ	L544
	.line	207
	RPTK	1
	BLKD	_fsample+0,*+
	LALK	FL5
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	MAR	* ,AR2
	SBRK	3
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	*+,AR1
	SACL	*+
	CALL	F$$ADD
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	SACL	*+
	SACH	*-
	.line	208
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$FTOL
	MAR	* ,AR2
	ADRK	7
	SACL	*+
	SACH	* 
	SBRK	21-LF33
	LAC	*+,1,AR1
	SACL	* ,AR2
	LAC	* ,AR0
	ADDK	1
	SACL	* 
	LT	* ,AR1
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR2
	ADRK	19-LF33
	ZALS	*+
	ADDH	* ,AR3
	SACL	*+
	SACH	*-
L544:
	.line	210
	MAR	* ,AR1
	LALK	FL6
	RPTK	1
	TBLR	*+
	MAR	* ,AR2
	LARK	AR2,8
	MAR	*0+
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+,AR2
	SBRK	3
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$ADD
	CALL	F$$DIV
	CALL	F$$FTOL
	MAR	* ,AR2
	ADRK	9
	SACL	*+
	SACH	* 
	SBRK	21-LF33
	LAC	*+,1,AR1
	SACL	* ,AR2
	LAC	* ,AR0
	ADDK	2
	SACL	* 
	LT	* ,AR1
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR2
	ADRK	19-LF33
	ZALS	*+
	ADDH	* ,AR3
	SACL	*+
	SACH	*-,AR2
	.line	211
	SBRK	11
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$SUB
	CALL	F$$FTOL
	MAR	* ,AR2
	ADRK	7
	SACL	*+
	SACH	* 
	SBRK	21-LF33
	LAC	*+,1,AR1
	SACL	* ,AR2
	LAC	* ,AR0
	ADDK	3
	SACL	* 
	LT	* ,AR1
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR2
	ADRK	19-LF33
	ZALS	*+
	ADDH	* ,AR3
	SACL	*+
	SACH	*-,AR2
	.line	212
	SBRK	21-LF33
	LAC	*+,1,AR1
	SACL	* ,AR2
	LAC	* ,AR0
	ADDK	4
	SACL	* 
	LT	* ,AR1
	MPYK	6
	PAC
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR2
	ADRK	7-LF33
	SACL	* 
	.line	213
	B	L532
L546:
	.line	217
	MAR	* ,AR1
	RPTK	1
	BLKD	_fsample+0,*+
	LALK	FL7
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$FTOL
	LDPK	_min_value
	SACL	_min_value
	SACH	_min_value+1
	.line	218
	RPTK	1
	BLKD	_fsample+0,*+
	LALK	FL3
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$FTOL
	SACL	_max_value
	SACH	_max_value+1
	.line	219
	LAC	_params_changed_copy
	SUBK	1
	BZ	EPI0_33
	.line	220
	MAR	* ,AR2
	LARK	AR2,-4+LF33
	MAR	*0+
	LAC	*+,1,AR1
	SACL	* ,AR2
	LAC	* ,AR0
	SUBK	1
	SACL	* 
	LT	* ,AR1
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	ZALS	*+
	ADDH	*-,AR1
	CALL	F$$LTOF
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	ADRK	9-LF33
	SACL	*+
	SACH	* 
	.line	221
	SBRK	11-LF33
	LAC	*+,1,AR1
	SACL	* ,AR2
	LT	* ,AR1
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	ZALS	*+
	ADDH	*-,AR1
	CALL	F$$LTOF
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	ADRK	11-LF33
	SACL	*+
	SACH	* ,AR1
	.line	222
	RPTK	1
	BLKD	_fsample+0,*+
	LALK	FL5
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	MAR	* ,AR2
	SBRK	3
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$SUB
	CALL	F$$LT
	BZ	L548
	.line	223
	RPTK	1
	BLKD	_fsample+0,*+
	LALK	FL5
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	MAR	* ,AR2
	MAR	*-
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$SUB
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	SBRK	3
	SACL	*+
	SACH	*-
	.line	224
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$FTOL
	MAR	* ,AR2
	ADRK	9
	SACL	*+
	SACH	* 
	SBRK	21-LF33
	LAC	*+,1,AR1
	SACL	* ,AR2
	LAC	* ,AR0
	SUBK	1
	SACL	* 
	LT	* ,AR1
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR2
	ADRK	19-LF33
	ZALS	*+
	ADDH	* ,AR3
	SACL	*+
	SACH	*-
L548:
	.line	226
	MAR	* ,AR1
	LALK	FL6
	RPTK	1
	TBLR	*+
	MAR	* ,AR2
	LARK	AR2,8
	MAR	*0+
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+,AR2
	SBRK	3
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$ADD
	CALL	F$$DIV
	CALL	F$$FTOL
	MAR	* ,AR2
	ADRK	9
	SACL	*+
	SACH	* 
	SBRK	21-LF33
	LAC	*+,1,AR1
	SACL	* ,AR2
	LAC	* ,AR0
	ADDK	1
	SACL	* 
	LT	* ,AR1
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR2
	ADRK	19-LF33
	ZALS	*+
	ADDH	* ,AR3
	SACL	*+
	SACH	*-,AR2
	.line	227
	SBRK	11
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$SUB
	CALL	F$$FTOL
	MAR	* ,AR2
	ADRK	7
	SACL	*+
	SACH	* 
	SBRK	21-LF33
	LAC	*+,1,AR1
	SACL	* ,AR2
	LAC	* ,AR0
	ADDK	2
	SACL	* 
	LT	* ,AR1
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR2
	ADRK	19-LF33
	ZALS	*+
	ADDH	* ,AR3
	SACL	*+
	SACH	*-,AR2
	.line	228
	SBRK	21-LF33
	LAC	*+,1,AR1
	SACL	* ,AR2
	LAC	* ,AR0
	ADDK	3
	SACL	* 
	LT	* ,AR1
	MPYK	6
	PAC
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR2
	ADRK	7-LF33
	SACL	* 
	.line	229
	B	L532
L550:
	.line	233
	MAR	* ,AR2
	LARK	AR2,-4+LF33
	MAR	*0+
	LAC	*+,1,AR1
	SACL	* ,AR2
	LAC	* ,AR0
	ADDK	1
	SACL	* 
	LT	* ,AR1
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	ZALS	*+
	ADDH	*-,AR1
	CALL	F$$LTOF
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	ADRK	15-LF33
	SACL	*+
	SACH	*-,AR1
	.line	234
	LALK	FL8
	RPTK	1
	TBLR	*+
	LALK	FL6
	RPTK	1
	TBLR	*+
	MAR	* ,AR2
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	*-,AR1
	SACL	*+
	CALL	F$$DIV
	RPTK	1
	BLKD	_fsample+0,*+
	LALK	FL2
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$ADD
	CALL	F$$ADD
	CALL	F$$FTOL
	LDPK	_min_value
	SACL	_min_value
	SACH	_min_value+1
	.line	235
	LALK	FL8
	RPTK	1
	TBLR	*+
	LALK	FL6
	RPTK	1
	TBLR	*+
	MAR	* ,AR2
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$DIV
	RPTK	1
	BLKD	_fsample+0,*+
	LALK	FL3
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$SUB
	CALL	F$$ADD
	CALL	F$$FTOL
	SACL	_max_value
	SACH	_max_value+1
	.line	236
	LAC	_params_changed_copy
	SUBK	1
	BZ	EPI0_33
	.line	237
	MAR	* ,AR2
	SBRK	17-LF33
	LAC	*+,1,AR1
	SACL	* ,AR2
	LT	* ,AR1
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	ZALS	*+
	ADDH	*-,AR1
	CALL	F$$LTOF
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	ADRK	13-LF33
	SACL	*+
	SACH	*+,AR1
	.line	238
	LALK	FL6
	RPTK	1
	TBLR	*+
	MAR	* ,AR2
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	*+,AR1
	SACL	*+
	CALL	F$$DIV
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	SACL	*+
	SACH	*-
	.line	239
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+,AR2
	SBRK	5
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$SUB
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	SBRK	5
	SACL	*+
	SACH	* 
	.line	240
	ADRK	7
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+,AR2
	SBRK	5
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$ADD
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	SBRK	3
	SACL	*+
	SACH	* 
	.line	247
	SBRK	3
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$FTOL
	MAR	* ,AR2
	ADRK	9
	SACL	*+
	SACH	* 
	SBRK	21-LF33
	LAC	*+,1,AR1
	SACL	* ,AR2
	LAC	* ,AR0
	SUBK	2
	SACL	* 
	LT	* ,AR1
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR2
	ADRK	19-LF33
	ZALS	*+
	ADDH	* ,AR3
	SACL	*+
	SACH	*-,AR2
	.line	248
	SBRK	9
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$FTOL
	MAR	* ,AR2
	ADRK	7
	SACL	*+
	SACH	* 
	SBRK	21-LF33
	LAC	*+,1,AR1
	SACL	* ,AR2
	LAC	* ,AR0
	SUBK	1
	SACL	* 
	LT	* ,AR1
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR2
	ADRK	19-LF33
	ZALS	*+
	ADDH	* ,AR3
	SACL	*+
	SACH	*-,AR2
	.line	249
	SBRK	11
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$SUB
	CALL	F$$FTOL
	MAR	* ,AR2
	ADRK	7
	SACL	*+
	SACH	* 
	SBRK	21-LF33
	LAC	*+,1,AR1
	SACL	* ,AR2
	LAC	* ,AR0
	ADDK	1
	SACL	* 
	LT	* ,AR1
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR2
	ADRK	19-LF33
	ZALS	*+
	ADDH	* ,AR3
	SACL	*+
	SACH	*-,AR2
	.line	250
	SBRK	21-LF33
	LAC	*+,1,AR1
	SACL	* ,AR2
	LAC	* ,AR0
	ADDK	2
	SACL	* 
	LT	* ,AR1
	MPYK	6
	PAC
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR2
	ADRK	7-LF33
	SACL	* 
	.line	251
	B	L532
L553:
	.line	255
	MAR	* ,AR1
	RPTK	1
	BLKD	_fsample+0,*+
	LALK	FL5
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$FTOL
	LDPK	_min_value
	SACL	_min_value
	SACH	_min_value+1
	.line	256
	LALK	FL8
	RPTK	1
	TBLR	*+
	RPTK	1
	BLKD	_fsample+0,*+
	LALK	FL9
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$ADD
	CALL	F$$FTOL
	SACL	_max_value
	SACH	_max_value+1
	.line	257
	LAC	_params_changed_copy
	SUBK	1
	BZ	EPI0_33
	.line	258
	MAR	* ,AR2
	LARK	AR2,-4+LF33
	MAR	*0+
	LAC	*+,1,AR1
	SACL	* ,AR2
	LAC	* ,AR0
	SUBK	1
	SACL	* 
	LT	* ,AR1
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	ZALS	*+
	ADDH	*-,AR1
	CALL	F$$LTOF
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	ADRK	13-LF33
	SACL	*+
	SACH	* 
	.line	259
	SBRK	15-LF33
	LAC	*+,1,AR1
	SACL	* ,AR2
	LT	* ,AR1
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	ZALS	*+
	ADDH	*-,AR1
	CALL	F$$LTOF
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	ADRK	15-LF33
	SACL	*+
	SACH	*-,AR1
	.line	260
	LALK	FL6
	RPTK	1
	TBLR	*+
	MAR	* ,AR2
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	*+,AR1
	SACL	*+
	CALL	F$$DIV
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	SACL	*+
	SACH	*-
	.line	261
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+,AR2
	SBRK	5
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$SUB
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	SBRK	5
	SACL	*+
	SACH	* 
	.line	262
	ADRK	7
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+,AR2
	SBRK	5
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$ADD
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	SBRK	3
	SACL	*+
	SACH	* ,AR1
	.line	263
	RPTK	1
	BLKD	_fsample+0,*+
	LALK	FL2
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	MAR	* ,AR2
	SBRK	3
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$LT
	BZ	L555
	.line	264
	RPTK	1
	BLKD	_fsample+0,*+
	LALK	FL2
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	MAR	*-
	SACL	*+
	SACH	* 
	.line	265
	ADRK	5
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+,AR2
	SBRK	7
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	*+,AR1
	SACL	*+
	CALL	F$$ADD
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	SACL	*+
	SACH	* 
L555:
	.line	267
	MAR	* ,AR2
	LARK	AR2,8
	MAR	*0+
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	RPTK	1
	BLKD	_fsample+0,*+
	LALK	FL3
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$LT
	BZ	L556
	.line	268
	RPTK	1
	BLKD	_fsample+0,*+
	LALK	FL3
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	MAR	*-
	SACL	*+
	SACH	* 
	.line	269
	ADRK	3
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+,AR2
	SBRK	5
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$SUB
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	SBRK	3
	SACL	*+
	SACH	* 
L556:
	.line	271
	MAR	* ,AR2
	LARK	AR2,6
	MAR	*0+
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$FTOL
	MAR	* ,AR2
	ADRK	9
	SACL	*+
	SACH	* 
	SBRK	21-LF33
	LAC	*+,1,AR1
	SACL	* ,AR2
	LAC	* ,AR0
	SUBK	3
	SACL	* 
	LT	* ,AR1
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR2
	ADRK	19-LF33
	ZALS	*+
	ADDH	* ,AR3
	SACL	*+
	SACH	*-,AR2
	.line	272
	SBRK	9
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$FTOL
	MAR	* ,AR2
	ADRK	7
	SACL	*+
	SACH	* 
	SBRK	21-LF33
	LAC	*+,1,AR1
	SACL	* ,AR2
	LAC	* ,AR0
	SUBK	2
	SACL	* 
	LT	* ,AR1
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR2
	ADRK	19-LF33
	ZALS	*+
	ADDH	* ,AR3
	SACL	*+
	SACH	*-,AR1
	.line	273
	LALK	FL6
	RPTK	1
	TBLR	*+
	MAR	* ,AR2
	SBRK	9
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+,AR2
	SBRK	3
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$ADD
	CALL	F$$DIV
	CALL	F$$FTOL
	MAR	* ,AR2
	ADRK	9
	SACL	*+
	SACH	* 
	SBRK	21-LF33
	LAC	*+,1,AR1
	SACL	* ,AR2
	LAC	* ,AR0
	SUBK	1
	SACL	* 
	LT	* ,AR1
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR2
	ADRK	19-LF33
	ZALS	*+
	ADDH	* ,AR3
	SACL	*+
	SACH	*-,AR2
	.line	274
	SBRK	21-LF33
	LAC	*+,1,AR1
	SACL	* ,AR2
	LAC	* ,AR0
	ADDK	1
	SACL	* 
	LT	* ,AR1
	MPYK	6
	PAC
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR2
	ADRK	7-LF33
	SACL	* 
L532:
	.line	276
	MAR	* ,AR2
	LARK	AR2,-4+LF33
	MAR	*0+
	LAC	* ,AR1
	SACL	*+,AR2
	ADRK	8-LF33
	LAC	* ,AR1
	SACL	*+,AR2
	ADRK	4
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+,AR2
	SBRK	3
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	_compute_fir
	SBRK	6
	.line	277
	B	EPI0_33
L558:
	.line	281
	MAR	* ,AR1
	RPTK	1
	BLKD	_fsample+0,*+
	LALK	FL2
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$FTOL
	LDPK	_min_value
	SACL	_min_value
	SACH	_min_value+1
	.line	282
	RPTK	1
	BLKD	_fsample+0,*+
	LALK	FL3
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$FTOL
	SACL	_max_value
	SACH	_max_value+1
	.line	283
	LAC	_params_changed_copy
	SUBK	1
	BZ	EPI0_33
	.line	284
	MAR	* ,AR2
	LARK	AR2,-4+LF33
	MAR	*0+
	LAC	*+,1,AR1
	SACL	* ,AR2
	LT	* ,AR1
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	ZALS	*+
	ADDH	*-,AR1
	CALL	F$$LTOF
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	ADRK	9-LF33
	SACL	*+
	SACH	* 
	.line	285
	SBRK	11-LF33
	LAC	*+,1,AR1
	SACL	* ,AR2
	LAC	* ,AR0
	ADDK	1
	SACL	* 
	LT	* ,AR1
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	ZALS	*+
	ADDH	*-,AR1
	CALL	F$$LTOF
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	ADRK	11-LF33
	SACL	*+
	SACH	* 

	.sym	_compute_n,L560,0,6,0
	.line	286
	B	L560
L562:
	.line	290
	MAR	* ,AR1
	RPTK	1
	BLKD	_fsample+0,*+
	LALK	FL10
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$FTOL
	LDPK	_min_value
	SACL	_min_value
	SACH	_min_value+1
	.line	291
	RPTK	1
	BLKD	_fsample+0,*+
	LALK	FL11
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$FTOL
	SACL	_max_value
	SACH	_max_value+1
	.line	292
	LAC	_params_changed_copy
	SUBK	1
	BZ	EPI0_33
	.line	293
	MAR	* ,AR2
	LARK	AR2,-4+LF33
	MAR	*0+
	LAC	*+,1,AR1
	SACL	* ,AR2
	LAC	* ,AR0
	SUBK	1
	SACL	* 
	LT	* ,AR1
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	ZALS	*+
	ADDH	*-,AR1
	CALL	F$$LTOF
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	ADRK	9-LF33
	SACL	*+
	SACH	* 
	.line	294
	SBRK	11-LF33
	LAC	*+,1,AR1
	SACL	* ,AR2
	LT	* ,AR1
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	ZALS	*+
	ADDH	*-,AR1
	CALL	F$$LTOF
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	ADRK	11-LF33
	SACL	*+
	SACH	* 
L560:
	.line	296
	MAR	* ,AR2
	LARK	AR2,-4+LF33
	MAR	*0+
	LAC	* ,AR1
	SACL	*+,AR2
	ADRK	12-LF33
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+,AR2
	SBRK	3
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	_compute_notch
	SBRK	5
	.line	297
	B	EPI0_33
L564:
	.line	300
	LACK	3
	LDPK	_min_value
	SACL	_min_value
	SACH	_min_value+1
	.line	301
	LDPK	_params+36
	LT	_params+36
	MPYK	1
	LPH	_params+37
	LACK	2
	SPAC
	BNZ	L565
	.line	302
	LACK	256
	LDPK	_max_value
	SACL	_max_value
	SACH	_max_value+1
	B	L566
L565:
	.line	305
	LACK	128
	LDPK	_max_value
	SACL	_max_value
	SACH	_max_value+1
L566:
	.line	307
	LAC	_params_changed_copy
	SUBK	1
	BZ	EPI0_33
	.line	308
	MAR	* ,AR2
	LARK	AR2,-4+LF33
	MAR	*0+
	LAC	*+,1,AR1
	SACL	* ,AR2
	LAC	*-,AR0
	ADDK	1
	SACL	* 
	LT	* ,AR1
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	LACK	1
	SACL	*+
	SACH	*-,AR2
	.line	309
	LAC	*+,1,AR1
	SACL	* ,AR2
	LT	* ,AR1
	MPYK	6
	PAC
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR2
	ADRK	7-LF33
	SACL	* 

	.sym	_load_user,L568,0,6,0
	.line	310
	B	L568
L569:
	.line	313
	MAR	* ,AR2
	LARK	AR2,-4+LF33
	MAR	*0+
	LAC	*+,1,AR1
	SACL	* ,AR2
	LAC	* ,AR0
	SUBK	1
	SACL	* 
	LT	* ,AR1
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR2
	ADRK	7-LF33
	SACL	* 
	.line	314
	LACK	1
	LDPK	_min_value
	SACL	_min_value
	SACH	_min_value+1
	.line	315
	SSXM
	LAC	*+
	SACL	_max_value
	SACH	_max_value+1
	.line	317
	BLKD	#_cursor_pos,* 
	.line	319
	SBRK	9-LF33
	LAC	*+,1,AR1
	SACL	* ,AR2
	LT	* ,AR1
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR2
	SUBK	1
	ADRK	6-LF33
	SACL	* 
	.line	320
	SBRK	7-LF33
	LAC	* ,1,AR1
	SACL	*+,AR2
	SSXM
	ADRK	7-LF33
	LAC	* ,14,AR0
	ADLK	46,15
	SACH	* ,1
	LT	* ,AR1
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	MAR	*-
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	ZALS	*+
	ADDH	*-,AR2
	SBRK	2
	SACL	*+
	SACH	*+
	.line	321
	BIT	* ,15
	BBZ	L570
	.line	322
	MAR	*-,AR1
	LACK	0
	SACL	*+
	LACK	6
	SACL	*+
	LACK	11
	SACL	*+,AR2
	SSXM
	LAC	* ,AR1
	SACL	*+
	SACH	*+
	CALL	_disp_num
	SBRK	5
	B	L571
L570:
	.line	325
	MAR	* ,AR1
	LACK	0
	SACL	*+
	LACK	6
	SACL	*+
	LACK	11
	SACL	*+,AR2
	LACK	65535
	SBRK	2
	AND	* ,AR1
	SACL	* 
	SSXM
	LAC	* 
	SACL	*+
	SACH	*+
	CALL	_disp_num
	SBRK	5
L571:
	.line	328
	LACK	0
	SACL	*+,AR2
	LARK	AR2,5
	MAR	*0+
	LAC	* ,AR1
	SACL	*+
	LRLK	AR3,SL9
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
	.line	330
	LDPK	_params_changed_copy
	LAC	_params_changed_copy
	SUBK	1
	BZ	EPI0_33
L568:
	.line	333
	MAR	* ,AR2
	LARK	AR2,-4+LF33
	MAR	*0+
	LAC	* ,AR1
	SACL	*+,AR2
	ADRK	8-LF33
	LAC	* ,AR1
	SACL	*+
	CALL	_load_userfir
	SBRK	2
	.line	334
	B	EPI0_33
	.line	337
L478:
	.line	8
	SSXM
	MAR	* ,AR2
	LARK	AR2,-3+LF33
	MAR	*0+
	LAC	* ,AR0
	SUBK	45
	BGZ	EPI0_33
	ADDK	45
	BLZ	EPI0_33
	ADLK	LL91
	TBLR	*
	LAC	*
	BACC
	.sect	".switch"
LL91:
	.word	L479
	.word	L482
	.word	L483
	.word	L484
	.word	L486
	.word	L498
	.word	L488
	.word	L500
	.word	L502
	.word	L504
	.word	L506
	.word	L507
	.word	L508
	.word	L509
	.word	L510
	.word	L514
	.word	L539
	.word	L528
	.word	L525
	.word	L539
	.word	L528
	.word	L525
	.word	L542
	.word	L546
	.word	L550
	.word	L553
	.word	L534
	.word	L525
	.word	L542
	.word	L546
	.word	L550
	.word	L553
	.word	L534
	.word	L525
	.word	L558
	.word	L562
	.word	L525
	.word	L558
	.word	L562
	.word	L525
	.word	L564
	.word	L569
	.word	L525
	.word	EPI0_33
	.word	EPI0_33
	.word	L525
	.text
EPI0_33:
	.line	340
	MAR	* ,AR1
	SBRK	19
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	3552,000000000H,18

	.sym	_compute_fir,_compute_fir,32,2,0
	.globl	_compute_fir

	.func	3561
******************************************************
* FUNCTION DEF : _compute_fir
******************************************************
_compute_fir:

LF34	.set	0

	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,273
	LAR	AR0,*0+,AR2

	.sym	_f1,-4+LF34,6,9,32
	.sym	_f2,-6+LF34,6,9,32
	.sym	_iorder,-7+LF34,4,9,16
	.sym	_index_ab_tmp,-8+LF34,4,9,16
	.sym	_i,1,4,1,16
	.sym	_itemp,2,4,1,16
	.sym	_max_flag,3,4,1,16
	.sym	_iorderm1,4,4,1,16
	.sym	_iorderm1d2,5,4,1,16
	.sym	_iorderd2,6,4,1,16
	.sym	_ftemp1,7,6,1,32
	.sym	_ftemp2,9,6,1,32
	.sym	_d2fsf1,11,6,1,32
	.sym	_d2fsf2,13,6,1,32
	.sym	_coef_max,15,6,1,32
	.sym	_coefs,17,54,1,4096,,128
	.line	2
	.line	7
	LARK	AR2,-7+LF34
	MAR	*0+
	LAC	* 
	SUBK	1
	ADRK	11-LF34
	SACL	* 
	.line	8
	SSXM
	LAC	*+,14
	SACH	* ,1
	.line	9
	SBRK	12-LF34
	LAC	* ,14
	ADRK	13-LF34
	SACH	* ,1
	.line	12
	SBRK	13-LF34
	LAC	* 
	LDPK	_iorder_old
	SUB	_iorder_old
	BZ	L574
	.line	13
	LAC	* ,AR1
	SUBK	1
	CALL	F$$ITOF
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	ADRK	14-LF34
	SACL	*+
	SACH	* 
	.line	14
	LACK	0
	SBRK	7
	SACL	* 
	SSXM
	LAC	* 
	ADRK	4
	SUB	* 
	BGZ	L576
	ADRK	2
L575:
	.line	15
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+,AR2
	SBRK	7
	LAC	* ,AR1
	CALL	F$$ITOF
	LALK	FL12
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$DIV
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	ADRK	8
	SACL	*+
	SACH	*-
	.line	16
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	LALK	FL6
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	_cos
	SBRK	2
	SACL	*+
	SACH	*+
	LALK	FL13
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	MAR	* ,AR2
	LARK	AR2,9
	MAR	*0+
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	_cos
	SBRK	2
	SACL	*+
	SACH	*+
	LALK	FL14
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	LALK	FL15
	RPTK	1
	TBLR	*+
	CALL	F$$SUB
	CALL	F$$ADD
	MAR	*-,AR2
	LARK	AR2,1
	MAR	*0+
	LAC	* ,1,AR0
	ADLK	_window+0,0
	SACL	* 
	LAR	AR3,* ,AR1
	ZALH	*-
	ADDS	* ,AR3
	SACL	*+
	SACH	*-,AR2
	.line	14
	LAC	* 
	ADDK	1
	SACL	* 
	SSXM
	LAC	* 
	ADRK	4
	SUB	* 
	ADRK	2
	BLEZ	L575
L576:
	.line	18
	LARK	AR2,-7+LF34
	MAR	*0+
	BLDD	* ,#_iorder_old
L574:
	.line	22
	MAR	* ,AR1
	RPTK	1
	BLKD	_fsample+0,*+
	LALK	FL6
	RPTK	1
	TBLR	*+
	CALL	F$$DIV
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	ADRK	14-LF34
	SACL	*+
	SACH	* 
	.line	23
	SBRK	12-LF34
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+,AR2
	ADRK	10-LF34
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$MUL
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	ADRK	3
	SACL	*+
	SACH	* 
	.line	24
	SBRK	18-LF34
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+,AR2
	ADRK	12-LF34
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$MUL
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	ADRK	5
	SACL	*+
	SACH	* 
	B	L577
L578:
	.line	27
	LACK	0
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	SACL	* 
	SSXM
	LAC	* 
	ADRK	5
	SUB	* 
	BGEZ	L580
	SBRK	2
	MAR	* ,AR1
L579:
	.line	28
	LALK	FL6
	RPTK	1
	TBLR	*+
	MAR	* ,AR2
	LAC	* ,AR1
	CALL	F$$ITOF
	CALL	F$$DIV
	SSXM
	MAR	* ,AR2
	SBRK	3
	LAC	* ,AR1
	CALL	F$$ITOF
	CALL	F$$SUB
	LALK	FL16
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	ADRK	6
	SACL	*+
	SACH	*-
	.line	29
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	*-,AR1
	SACL	*+,AR2
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+,AR2
	ADRK	3
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$MUL
	CALL	_sin
	SBRK	2
	SACL	*+
	SACH	*+,AR2
	LARK	AR2,1
	MAR	*0+
	LAC	* ,1,AR0
	ADLK	_window+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	*+,AR1
	SACL	*+,AR3
	LAC	*-,AR1
	SACL	*+
	CALL	F$$MUL
	CALL	F$$DIV
	MAR	*-,AR2
	LAC	* ,1,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	MAR	*0+,AR1
	ZALH	*-
	ADDS	* ,AR3
	ADRK	17
	SACL	*+
	SACH	*-,AR2
	.line	27
	LAC	* 
	ADDK	1
	SACL	* 
	SSXM
	LAC	* 
	ADRK	5
	SUB	* 
	SBRK	2
	MAR	* ,AR1
	BLZ	L579
L580:
	.line	31
	MAR	* ,AR2
	LARK	AR2,-7+LF34
	MAR	*0+
	BIT	* ,15
	BBZ	L582
	.line	32
	ADRK	18-LF34
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+,AR2
	SBRK	11
	LAC	* ,1,AR0
	ADLK	_window+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	*+,AR1
	SACL	*+,AR3
	LAC	*-,AR1
	SACL	*+
	CALL	F$$MUL
	MAR	*-,AR2
	LAC	* ,1,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	MAR	*0+,AR1
	ZALH	*-
	ADDS	* ,AR3
	ADRK	17
	SACL	*+
	SACH	*-
	.line	34
	B	L582
L583:
	.line	37
	LACK	0
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	SACL	* 
	SSXM
	LAC	* 
	ADRK	5
	SUB	* 
	BGEZ	L585
	SBRK	2
	MAR	* ,AR1
L584:
	.line	38
	LALK	FL6
	RPTK	1
	TBLR	*+
	MAR	* ,AR2
	LAC	* ,AR1
	CALL	F$$ITOF
	CALL	F$$DIV
	SSXM
	MAR	* ,AR2
	SBRK	3
	LAC	* ,AR1
	CALL	F$$ITOF
	CALL	F$$SUB
	LALK	FL16
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	ADRK	6
	SACL	*+
	SACH	*-
	.line	39
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	*-,AR1
	SACL	*+,AR2
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+,AR2
	ADRK	3
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$MUL
	CALL	_sin
	SBRK	2
	SACL	*+
	SACH	*+,AR2
	LARK	AR2,7
	MAR	*0+
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	_sin
	SBRK	2
	SACL	*+
	SACH	*+
	CALL	F$$SUB
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	LAC	* ,1,AR0
	ADLK	_window+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	*+,AR1
	SACL	*+,AR3
	LAC	*-,AR1
	SACL	*+
	CALL	F$$MUL
	CALL	F$$DIV
	MAR	*-,AR2
	LAC	* ,1,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	MAR	*0+,AR1
	ZALH	*-
	ADDS	* ,AR3
	ADRK	17
	SACL	*+
	SACH	*-,AR2
	.line	37
	LAC	* 
	ADDK	1
	SACL	* 
	SSXM
	LAC	* 
	ADRK	5
	SUB	* 
	SBRK	2
	MAR	* ,AR1
	BLZ	L584
L585:
	.line	41
	MAR	* ,AR2
	LARK	AR2,-7+LF34
	MAR	*0+
	BIT	* ,15
	BBZ	L582
	.line	42
	ADRK	18-LF34
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	LALK	FL17
	RPTK	1
	TBLR	*+
	CALL	F$$SUB
	MAR	* ,AR2
	SBRK	11
	LAC	* ,1,AR0
	ADLK	_window+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	*+,AR1
	SACL	*+,AR3
	LAC	*-,AR1
	SACL	*+
	CALL	F$$MUL
	MAR	*-,AR2
	LAC	* ,1,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	MAR	*0+,AR1
	ZALH	*-
	ADDS	* ,AR3
	ADRK	17
	SACL	*+
	SACH	*-
	.line	44
	B	L582
L587:
	.line	47
	LACK	0
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	SACL	* 
	SSXM
	LAC	* 
	ADRK	5
	SUB	* 
	BGEZ	L589
	SBRK	2
	MAR	* ,AR1
L588:
	.line	48
	LALK	FL6
	RPTK	1
	TBLR	*+
	MAR	* ,AR2
	LAC	* ,AR1
	CALL	F$$ITOF
	CALL	F$$DIV
	SSXM
	MAR	* ,AR2
	SBRK	3
	LAC	* ,AR1
	CALL	F$$ITOF
	CALL	F$$SUB
	LALK	FL16
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	ADRK	6
	SACL	*+
	SACH	*-
	.line	49
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	*-,AR1
	SACL	*+,AR2
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+,AR2
	ADRK	3
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$MUL
	CALL	_sin
	SBRK	2
	SACL	*+
	SACH	*+,AR2
	LARK	AR2,7
	MAR	*0+
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+,AR2
	ADRK	5
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$MUL
	CALL	_sin
	SBRK	2
	SACL	*+
	SACH	*+
	CALL	F$$SUB
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	LAC	* ,1,AR0
	ADLK	_window+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	*+,AR1
	SACL	*+,AR3
	LAC	*-,AR1
	SACL	*+
	CALL	F$$MUL
	CALL	F$$DIV
	MAR	*-,AR2
	LAC	* ,1,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	MAR	*0+,AR1
	ZALH	*-
	ADDS	* ,AR3
	ADRK	17
	SACL	*+
	SACH	*-,AR2
	.line	47
	LAC	* 
	ADDK	1
	SACL	* 
	SSXM
	LAC	* 
	ADRK	5
	SUB	* 
	SBRK	2
	MAR	* ,AR1
	BLZ	L588
L589:
	.line	51
	MAR	* ,AR2
	LARK	AR2,-7+LF34
	MAR	*0+
	BIT	* ,15
	BBZ	L582
	.line	52
	ADRK	18-LF34
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$SUB
	MAR	* ,AR2
	SBRK	13
	LAC	* ,1,AR0
	ADLK	_window+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	*+,AR1
	SACL	*+,AR3
	LAC	*-,AR1
	SACL	*+
	CALL	F$$MUL
	MAR	*-,AR2
	LAC	* ,1,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	MAR	*0+,AR1
	ZALH	*-
	ADDS	* ,AR3
	ADRK	17
	SACL	*+
	SACH	*-
	.line	54
	B	L582
L591:
	.line	57
	LACK	0
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	SACL	* 
	SSXM
	LAC	* 
	ADRK	5
	SUB	* 
	BGEZ	L593
	SBRK	2
	MAR	* ,AR1
L592:
	.line	58
	LALK	FL6
	RPTK	1
	TBLR	*+
	MAR	* ,AR2
	LAC	* ,AR1
	CALL	F$$ITOF
	CALL	F$$DIV
	SSXM
	MAR	* ,AR2
	SBRK	3
	LAC	* ,AR1
	CALL	F$$ITOF
	CALL	F$$SUB
	LALK	FL16
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	ADRK	6
	SACL	*+
	SACH	*-
	.line	59
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	*-,AR1
	SACL	*+,AR2
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+,AR2
	ADRK	5
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$MUL
	CALL	_sin
	SBRK	2
	SACL	*+
	SACH	*+,AR2
	LARK	AR2,7
	MAR	*0+
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+,AR2
	ADRK	3
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$MUL
	CALL	_sin
	SBRK	2
	SACL	*+
	SACH	*+,AR2
	LARK	AR2,7
	MAR	*0+
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	_sin
	SBRK	2
	SACL	*+
	SACH	*+
	CALL	F$$ADD
	CALL	F$$SUB
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	LAC	* ,1,AR0
	ADLK	_window+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	*+,AR1
	SACL	*+,AR3
	LAC	*-,AR1
	SACL	*+
	CALL	F$$MUL
	CALL	F$$DIV
	MAR	*-,AR2
	LAC	* ,1,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	MAR	*0+,AR1
	ZALH	*-
	ADDS	* ,AR3
	ADRK	17
	SACL	*+
	SACH	*-,AR2
	.line	57
	LAC	* 
	ADDK	1
	SACL	* 
	SSXM
	LAC	* 
	ADRK	5
	SUB	* 
	SBRK	2
	MAR	* ,AR1
	BLZ	L592
L593:
	.line	61
	MAR	* ,AR2
	LARK	AR2,-7+LF34
	MAR	*0+
	BIT	* ,15
	BBZ	L582
	.line	62
	ADRK	20-LF34
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+,AR2
	SBRK	3
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	LALK	FL17
	RPTK	1
	TBLR	*+
	CALL	F$$ADD
	CALL	F$$SUB
	MAR	* ,AR2
	SBRK	11
	LAC	* ,1,AR0
	ADLK	_window+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	*+,AR1
	SACL	*+,AR3
	LAC	*-,AR1
	SACL	*+
	CALL	F$$MUL
	MAR	*-,AR2
	LAC	* ,1,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	MAR	*0+,AR1
	ZALH	*-
	ADDS	* ,AR3
	ADRK	17
	SACL	*+
	SACH	*-
	.line	64
	B	L582
	.line	67
L577:
	.line	25
	SBRK	22-LF34
	LAC	* ,1,AR0
	ADLK	_params+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	SSXM
	LAC	* 
	SUBK	2
	BZ	L578
	SUBK	1
	BZ	L583
	SUBK	1
	BZ	L587
	SUBK	1
	BZ	L591
L582:
	.line	71
	MAR	* ,AR2
	LARK	AR2,5
	MAR	*0+
	LAC	* ,1,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	MAR	*0+
	ADRK	17
	ZALS	*+
	ADDH	*-,AR2
	ADRK	10
	SACL	*+
	SACH	*-
	.line	72
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	LALK	FL18
	RPTK	1
	TBLR	*+
	CALL	F$$LT
	MAR	* ,AR2
	SBRK	13
	SACL	* 
	.line	73
	LAC	* 
	BZ	LL95
	MAR	* ,AR1
	LALK	FL19
	RPTK	1
	TBLR	*+
	B	LL96
LL95:
	MAR	* ,AR1
	LALK	FL20
	RPTK	1
	TBLR	*+
LL96:
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	ADRK	4
	SACL	*+
	SACH	* 
	.line	76
	LACK	1024
	LDPK	_out_gain
	OR	_out_gain
	SACL	_out_gain
	.line	126
	SBRK	16-LF34
	LAC	* 
	ADDK	1
	ADRK	10-LF34
	SACL	* 
	.line	127
	BIT	* ,15
	BBZ	L596
	.line	128
	LALK	_no_func_a+0
	LDPK	_func_addr_a
	SACL	_func_addr_a
	.line	129
	LACK	1023
	ADRK	2
	SUB	* 
	LDPK	_data_ptr_a
	SACL	_data_ptr_a
	.line	130
	LAC	* 
	SUBK	1
	LDPK	_orderm2_a
	SACL	_orderm2_a
	.line	131
	clrc	CNF		; map internal memory block B0 into Data space so we can write it
	.line	132
	LACK	0
	SBRK	3
	SACL	* 
	SSXM
	LAC	* 
	ADRK	4
	SUB	* 
	BGZ	L598
	SBRK	4
	MAR	* ,AR1
L597:
	.line	133
	LALK	FL8
	RPTK	1
	TBLR	*+
	MAR	* ,AR2
	LAC	* ,1,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	MAR	*0+
	ADRK	17
	LAC	*+,AR1
	SACL	*+,AR3
	LAC	*-,AR1
	SACL	*+,AR2
	ADRK	6
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$MUL
	CALL	F$$ADD
	CALL	F$$FTOI
	SACL	*+,AR2
	SSXM
	SBRK	4
	LAC	* 
	RSXM
	ADLK	_fir_coef+0,0
	SBRK	3
	SUB	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR1
	MAR	*-
	LAC	* ,AR3
	SACL	* ,AR1
	SACL	* ,AR2
	LAC	* ,AR0
	ADLK	_fir_coef+0,0
	SACL	* 
	LAR	AR3,* ,AR1
	LAC	* ,AR3
	SACL	* ,AR2
	.line	132
	LAC	* 
	ADDK	1
	SACL	* 
	SSXM
	LAC	* 
	ADRK	4
	SUB	* 
	SBRK	4
	MAR	* ,AR1
	BLEZ	L597
L598:
	.line	135
	setc	CNF		; map internal memory block B0 into Program space
	.line	136
	MAR	* ,AR2
	LARK	AR2,3
	MAR	*0+
	LAC	* 
	BZ	LL97
	LALK	_fir_15_a+0
	B	LL98
LL97:
	LALK	_fir_16_a+0
LL98:
	LDPK	_func_addr_a
	SACL	_func_addr_a
L596:
	.line	139
	LARK	AR2,2
	MAR	*0+
	BIT	* ,14
	BBZ	L599
	.line	140
	LALK	_no_func_b+0
	LDPK	_func_addr_b
	SACL	_func_addr_b
	.line	141
	LACK	895
	ADRK	2
	SUB	* 
	LDPK	_data_ptr_b
	SACL	_data_ptr_b
	.line	142
	LAC	* 
	SUBK	1
	LDPK	_orderm2_b
	SACL	_orderm2_b
	.line	143
	clrc	CNF		; map internal memory block B0 into Data space so we can write it
	.line	144
	LACK	0
	SBRK	3
	SACL	* 
	SSXM
	LAC	* 
	ADRK	4
	SUB	* 
	BGZ	L601
	SBRK	4
	MAR	* ,AR1
L600:
	.line	145
	LALK	FL8
	RPTK	1
	TBLR	*+
	MAR	* ,AR2
	LAC	* ,1,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	MAR	*0+
	ADRK	17
	LAC	*+,AR1
	SACL	*+,AR3
	LAC	*-,AR1
	SACL	*+,AR2
	ADRK	6
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$MUL
	CALL	F$$ADD
	CALL	F$$FTOI
	SACL	*+,AR2
	SBRK	4
	LAC	* 
	ADDK	128
	RSXM
	ADLK	_fir_coef+0,0
	SBRK	3
	SUB	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR1
	MAR	*-
	LAC	* ,AR3
	SACL	* ,AR1
	SACL	* ,AR2
	LAC	* ,AR0
	ADLK	_fir_coef+128,0
	SACL	* 
	LAR	AR3,* ,AR1
	LAC	* ,AR3
	SACL	* ,AR2
	.line	144
	LAC	* 
	ADDK	1
	SACL	* 
	SSXM
	LAC	* 
	ADRK	4
	SUB	* 
	SBRK	4
	MAR	* ,AR1
	BLEZ	L600
L601:
	.line	147
	setc	CNF		; map internal memory block B0 into Program space
	.line	148
	MAR	* ,AR2
	LARK	AR2,3
	MAR	*0+
	LAC	* 
	BZ	LL99
	LALK	_fir_15_b+0
	B	LL100
LL99:
	LALK	_fir_16_b+0
LL100:
	LDPK	_func_addr_b
	SACL	_func_addr_b
L599:
	.line	152
	LARK	AR2,-7+LF34
	MAR	*0+
	LAC	* ,AR1
	SACL	*+
	CALL	_wait_n_samples
	MAR	*-
	.line	153
	LACK	64511
	LDPK	_out_gain
	AND	_out_gain
	SACL	_out_gain
EPI0_34:
	.line	155
	SBRK	255
	SBRK	19
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	3715,000000000H,273

	.sym	_load_userfir,_load_userfir,32,2,0
	.globl	_load_userfir

	.func	3724
******************************************************
* FUNCTION DEF : _load_userfir
******************************************************
_load_userfir:

LF35	.set	0

	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,8
	LAR	AR0,*0+,AR2

	.sym	_iorder,-3+LF35,4,9,16
	.sym	_index_ab_tmp,-4+LF35,4,9,16
	.sym	_i,1,4,1,16
	.sym	_j,2,4,1,16
	.sym	_itemp,3,4,1,16
	.sym	_iorderm1,4,4,1,16
	.sym	_iorderm1d2,5,4,1,16
	.sym	_ltemp,6,5,1,32
	.line	2
	.line	6
	LARK	AR2,-3+LF35
	MAR	*0+
	LAC	* 
	SUBK	1
	ADRK	7-LF35
	SACL	* 
	.line	7
	SSXM
	LAC	*+,14
	SACH	* ,1
	.line	9
	LACK	1024
	LDPK	_out_gain
	OR	_out_gain
	SACL	_out_gain
	.line	11
	SBRK	9-LF35
	LAC	* 
	ADDK	1
	ADRK	7-LF35
	SACL	* 
	.line	12
	BIT	* ,15
	BBZ	L602
	.line	13
	LALK	_no_func_a+0
	LDPK	_func_addr_a
	SACL	_func_addr_a
	.line	14
	LACK	1023
	MAR	*+
	SUB	* 
	LDPK	_data_ptr_a
	SACL	_data_ptr_a
	.line	15
	LAC	* 
	SUBK	1
	LDPK	_orderm2_a
	SACL	_orderm2_a
	.line	16
	clrc	CNF		; map internal memory block B0 into Data space so we can write it
	.line	17
	LACK	0
	SBRK	2
	SACL	*-
	.line	18
	SACL	* 
	LAC	* 
	ADRK	4
	SUB	* 
	BGZ	L604
	SBRK	9-LF35
L603:
	.line	19
	LAC	* ,1,AR1
	SACL	*+,AR2
	ADRK	5-LF35
	LAC	* ,AR0
	ADDK	46
	SACL	* 
	LT	* ,AR1
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	MAR	*-
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	ZALS	*+
	ADDH	*-,AR2
	ADRK	5
	SACL	*+
	SACH	*-
	.line	20
	LACK	65535
	AND	* ,AR1
	SACL	*+,AR2
	SBRK	4
	LAC	* 
	ADDK	1
	SACL	* ,AR0
	SUBK	1
	ADLK	_fir_coef+0,0
	SACL	* 
	LAR	AR3,* ,AR1
	MAR	*-
	LAC	* ,AR3
	SACL	* ,AR2
	.line	21
	ADRK	5
	ZALS	* ,AR1
	SACL	*+,AR2
	SBRK	5
	LAC	* 
	ADDK	1
	SACL	*-,AR0
	SUBK	1
	ADLK	_fir_coef+0,0
	SACL	* 
	LAR	AR3,* ,AR1
	MAR	*-
	LAC	* ,AR3
	SACL	* ,AR2
	.line	18
	LAC	* 
	ADDK	1
	SACL	* 
	SSXM
	LAC	* 
	ADRK	4
	SUB	* 
	SBRK	9-LF35
	BLEZ	L603
L604:
	.line	23
	setc	CNF		; map internal memory block B0 into Program space
	.line	24
	LALK	_fir_15_a+0
	LDPK	_func_addr_a
	SACL	_func_addr_a
L602:
	.line	27
	LARK	AR2,3
	MAR	*0+
	BIT	* ,14
	BBZ	L605
	.line	28
	LALK	_no_func_b+0
	LDPK	_func_addr_b
	SACL	_func_addr_b
	.line	29
	LACK	895
	MAR	*+
	SUB	* 
	LDPK	_data_ptr_b
	SACL	_data_ptr_b
	.line	30
	LAC	* 
	SUBK	1
	LDPK	_orderm2_b
	SACL	_orderm2_b
	.line	31
	clrc	CNF		; map internal memory block B0 into Data space so we can write it
	.line	32
	LACK	128
	SBRK	2
	SACL	*-
	.line	33
	LACK	0
	SACL	* 
	LAC	* 
	ADRK	4
	SUB	* 
	BGZ	L607
	SBRK	9-LF35
L606:
	.line	34
	LAC	* ,1,AR1
	SACL	*+,AR2
	ADRK	5-LF35
	LAC	* ,AR0
	ADDK	46
	SACL	* 
	LT	* ,AR1
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	MAR	*-
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	ZALS	*+
	ADDH	*-,AR2
	ADRK	5
	SACL	*+
	SACH	*-
	.line	35
	LACK	65535
	AND	* ,AR1
	SACL	*+,AR2
	SBRK	4
	LAC	* 
	ADDK	1
	SACL	* ,AR0
	SUBK	1
	ADLK	_fir_coef+0,0
	SACL	* 
	LAR	AR3,* ,AR1
	MAR	*-
	LAC	* ,AR3
	SACL	* ,AR2
	.line	36
	ADRK	5
	ZALS	* ,AR1
	SACL	*+,AR2
	SBRK	5
	LAC	* 
	ADDK	1
	SACL	*-,AR0
	SUBK	1
	ADLK	_fir_coef+0,0
	SACL	* 
	LAR	AR3,* ,AR1
	MAR	*-
	LAC	* ,AR3
	SACL	* ,AR2
	.line	33
	LAC	* 
	ADDK	1
	SACL	* 
	SSXM
	LAC	* 
	ADRK	4
	SUB	* 
	SBRK	9-LF35
	BLEZ	L606
L607:
	.line	38
	setc	CNF		; map internal memory block B0 into Program space
	.line	39
	LALK	_fir_15_b+0
	LDPK	_func_addr_b
	SACL	_func_addr_b
L605:
	.line	42
	LARK	AR2,-3+LF35
	MAR	*0+
	LAC	* ,AR1
	SACL	*+
	CALL	_wait_n_samples
	MAR	*-
	.line	43
	LACK	64511
	LDPK	_out_gain
	AND	_out_gain
	SACL	_out_gain
EPI0_35:
	.line	45
	SBRK	9
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	3768,000000000H,8

	.sym	_compute_notch,_compute_notch,32,2,0
	.globl	_compute_notch

	.func	3777
******************************************************
* FUNCTION DEF : _compute_notch
******************************************************
_compute_notch:

LF36	.set	0

	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,25
	LAR	AR0,*0+

	.sym	_fn,-4+LF36,6,9,32
	.sym	_fw,-6+LF36,6,9,32
	.sym	_index_ab_tmp,-7+LF36,4,9,16
	.sym	_iptr,1,20,1,16
	.sym	_itemp,2,4,1,16
	.sym	_t1,3,6,1,32
	.sym	_t2,5,6,1,32
	.sym	_t3,7,6,1,32
	.sym	_k1,9,6,1,32
	.sym	_k2,11,6,1,32
	.sym	_c1,13,6,1,32
	.sym	_c2,15,6,1,32
	.sym	_d1,17,6,1,32
	.sym	_d2,19,6,1,32
	.sym	_g1,21,6,1,32
	.sym	_g2,23,6,1,32
	.line	2
	.line	8
	LALK	FL21
	RPTK	1
	TBLR	*+
	RPTK	1
	BLKD	_fsample+0,*+
	MAR	* ,AR2
	LARK	AR2,-4+LF36
	MAR	*0+
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	LALK	FL22
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$DIV
	CALL	F$$ADD
	CALL	_sin
	SBRK	2
	SACL	*+
	SACH	*+
	CALL	F$$NEG
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	LARK	AR2,9
	MAR	*0+
	SACL	*+
	SACH	* ,AR1
	.line	9
	RPTK	1
	BLKD	_fsample+0,*+
	MAR	* ,AR2
	SBRK	16-LF36
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	LALK	FL16
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$DIV
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	ADRK	8-LF36
	SACL	*+
	SACH	*-,AR1
	.line	11
	LALK	FL21
	RPTK	1
	TBLR	*+
	MAR	* ,AR2
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$ADD
	CALL	_sin
	SBRK	2
	MAR	* ,AR2
	LARK	AR2,5
	MAR	*0+
	SACL	*+
	SACH	* 
	.line	12
	SBRK	3
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	_sin
	SBRK	2
	MAR	* ,AR2
	LARK	AR2,7
	MAR	*0+
	SACL	*+
	SACH	*-
	.line	13
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+,AR2
	SBRK	3
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	*+,AR1
	SACL	*+
	CALL	F$$ADD
	MAR	* ,AR2
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+,AR2
	SBRK	3
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$SUB
	CALL	F$$DIV
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	ADRK	5
	SACL	*+
	SACH	* 
	.line	22
	LALK	FL23
	ADRK	5
	RPTK	1
	TBLR	*+
	.line	23
	LALK	FL23
	RPTK	1
	TBLR	*+
	.line	24
	SBRK	4
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+,AR2
	SBRK	9
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	*-,AR1
	SACL	*+,AR2
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$MUL
	LALK	FL17
	RPTK	1
	TBLR	*+
	CALL	F$$SUB
	CALL	F$$DIV
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	ADRK	3
	SACL	*+
	SACH	* 
	.line	25
	ADRK	5
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+,AR2
	SBRK	9
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	*-,AR1
	SACL	*+,AR2
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$MUL
	LALK	FL17
	RPTK	1
	TBLR	*+
	CALL	F$$SUB
	CALL	F$$DIV
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	ADRK	3
	SACL	*+
	SACH	* 
	B	L608
L609:
	.line	31
	LALK	FL8
	MAR	* ,AR2
	LARK	AR2,21
	MAR	*0+
	RPTK	1
	TBLR	*+
	.line	32
	LALK	FL24
	RPTK	1
	TBLR	*+
	.line	33
	B	L610
L611:
	.line	37
	LALK	FL24
	MAR	* ,AR2
	LARK	AR2,21
	MAR	*0+
	RPTK	1
	TBLR	*+
	.line	38
	LALK	FL8
	RPTK	1
	TBLR	*+
	.line	39
	B	L610
	.line	42
L608:
	.line	28
	SBRK	23-LF36
	LAC	* ,1,AR0
	ADLK	_params+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	SSXM
	LAC	* 
	SUBK	6
	BZ	L609
	SUBK	1
	BZ	L611
L610:
	.line	47
	LACK	1024
	LDPK	_out_gain
	OR	_out_gain
	SACL	_out_gain
	.line	49
	MAR	* ,AR2
	LARK	AR2,-7+LF36
	MAR	*0+
	LAC	* 
	ADDK	1
	ADRK	9-LF36
	SACL	* 
	.line	50
	BIT	* ,15
	BBZ	L613
	.line	51
	LALK	_no_func_a+0
	LDPK	_func_addr_a
	SACL	_func_addr_a
	.line	52
	LDPK	_data_ptr_a
	SPLK	#1023,_data_ptr_a
	.line	54
	LALK	_coefdata+128
	MAR	*-
	SACL	* ,AR1
	.line	55
	LALK	FL8
	RPTK	1
	TBLR	*+
	MAR	* ,AR2
	ADRK	14
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	LALK	FL19
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$ADD
	CALL	F$$FTOI
	MAR	* ,AR2
	SBRK	15
	LAR	AR3,* ,AR3
	SACL	*+,AR2
	SAR	AR3,* ,AR1
	.line	56
	LALK	FL8
	RPTK	1
	TBLR	*+
	MAR	* ,AR2
	ADRK	10
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	LALK	FL19
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$ADD
	CALL	F$$FTOI
	MAR	* ,AR2
	SBRK	11
	LAR	AR3,* ,AR3
	SACL	*+,AR2
	SAR	AR3,* ,AR1
	.line	57
	LALK	FL8
	RPTK	1
	TBLR	*+
	MAR	* ,AR2
	ADRK	18
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	LALK	FL19
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$ADD
	CALL	F$$FTOI
	MAR	* ,AR2
	SBRK	19
	LAR	AR3,* ,AR3
	SACL	*+,AR2
	SAR	AR3,* ,AR1
	.line	58
	LALK	FL8
	RPTK	1
	TBLR	*+
	MAR	* ,AR2
	ADRK	10
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	LALK	FL19
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$ADD
	CALL	F$$FTOI
	MAR	* ,AR2
	SBRK	11
	LAR	AR3,* ,AR3
	SACL	*+,AR2
	SAR	AR3,* ,AR1
	.line	59
	LALK	FL8
	RPTK	1
	TBLR	*+
	MAR	* ,AR2
	ADRK	12
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	LALK	FL19
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$ADD
	CALL	F$$FTOI
	MAR	* ,AR2
	SBRK	13
	LAR	AR3,* ,AR3
	SACL	*+,AR2
	SAR	AR3,* ,AR1
	.line	60
	LALK	FL8
	RPTK	1
	TBLR	*+
	MAR	* ,AR2
	ADRK	8
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	LALK	FL19
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$ADD
	CALL	F$$FTOI
	MAR	* ,AR2
	SBRK	9
	LAR	AR3,* ,AR3
	SACL	*+,AR2
	SAR	AR3,* ,AR1
	.line	61
	LALK	FL8
	RPTK	1
	TBLR	*+
	MAR	* ,AR2
	ADRK	16
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	LALK	FL19
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$ADD
	CALL	F$$FTOI
	MAR	* ,AR2
	SBRK	17
	LAR	AR3,* ,AR3
	SACL	*+,AR2
	SAR	AR3,* ,AR1
	.line	62
	LALK	FL8
	RPTK	1
	TBLR	*+
	MAR	* ,AR2
	ADRK	8
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	LALK	FL19
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$ADD
	CALL	F$$FTOI
	MAR	* ,AR2
	SBRK	9
	LAR	AR3,* ,AR3
	SACL	*+,AR2
	SAR	AR3,* ,AR1
	.line	63
	LALK	FL8
	RPTK	1
	TBLR	*+
	MAR	* ,AR2
	ADRK	20
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	LALK	FL25
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$ADD
	CALL	F$$FTOI
	MAR	* ,AR2
	SBRK	21
	LAR	AR3,* ,AR3
	SACL	*+,AR2
	SAR	AR3,* ,AR1
	.line	64
	LALK	FL8
	RPTK	1
	TBLR	*+
	MAR	* ,AR2
	ADRK	22
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	LALK	FL25
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$ADD
	CALL	F$$FTOI
	MAR	* ,AR2
	SBRK	23
	LAR	AR3,* ,AR3
	SACL	*+,AR2
	SAR	AR3,* 
	.line	65
	LALK	_notch_a+0
	LDPK	_func_addr_a
	SACL	_func_addr_a
L613:
	.line	68
	LARK	AR2,2
	MAR	*0+
	BIT	* ,14
	BBZ	L614
	.line	69
	LALK	_no_func_b+0
	LDPK	_func_addr_b
	SACL	_func_addr_b
	.line	70
	LDPK	_data_ptr_b
	SPLK	#895,_data_ptr_b
	.line	72
	LALK	_coefdata+0
	MAR	*-
	SACL	* ,AR1
	.line	73
	LALK	FL8
	RPTK	1
	TBLR	*+
	MAR	* ,AR2
	ADRK	14
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	LALK	FL19
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$ADD
	CALL	F$$FTOI
	MAR	* ,AR2
	SBRK	15
	LAR	AR3,* ,AR3
	SACL	*+,AR2
	SAR	AR3,* ,AR1
	.line	74
	LALK	FL8
	RPTK	1
	TBLR	*+
	MAR	* ,AR2
	ADRK	10
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	LALK	FL19
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$ADD
	CALL	F$$FTOI
	MAR	* ,AR2
	SBRK	11
	LAR	AR3,* ,AR3
	SACL	*+,AR2
	SAR	AR3,* ,AR1
	.line	75
	LALK	FL8
	RPTK	1
	TBLR	*+
	MAR	* ,AR2
	ADRK	18
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	LALK	FL19
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$ADD
	CALL	F$$FTOI
	MAR	* ,AR2
	SBRK	19
	LAR	AR3,* ,AR3
	SACL	*+,AR2
	SAR	AR3,* ,AR1
	.line	76
	LALK	FL8
	RPTK	1
	TBLR	*+
	MAR	* ,AR2
	ADRK	10
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	LALK	FL19
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$ADD
	CALL	F$$FTOI
	MAR	* ,AR2
	SBRK	11
	LAR	AR3,* ,AR3
	SACL	*+,AR2
	SAR	AR3,* ,AR1
	.line	77
	LALK	FL8
	RPTK	1
	TBLR	*+
	MAR	* ,AR2
	ADRK	12
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	LALK	FL19
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$ADD
	CALL	F$$FTOI
	MAR	* ,AR2
	SBRK	13
	LAR	AR3,* ,AR3
	SACL	*+,AR2
	SAR	AR3,* ,AR1
	.line	78
	LALK	FL8
	RPTK	1
	TBLR	*+
	MAR	* ,AR2
	ADRK	8
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	LALK	FL19
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$ADD
	CALL	F$$FTOI
	MAR	* ,AR2
	SBRK	9
	LAR	AR3,* ,AR3
	SACL	*+,AR2
	SAR	AR3,* ,AR1
	.line	79
	LALK	FL8
	RPTK	1
	TBLR	*+
	MAR	* ,AR2
	ADRK	16
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	LALK	FL19
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$ADD
	CALL	F$$FTOI
	MAR	* ,AR2
	SBRK	17
	LAR	AR3,* ,AR3
	SACL	*+,AR2
	SAR	AR3,* ,AR1
	.line	80
	LALK	FL8
	RPTK	1
	TBLR	*+
	MAR	* ,AR2
	ADRK	8
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	LALK	FL19
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$ADD
	CALL	F$$FTOI
	MAR	* ,AR2
	SBRK	9
	LAR	AR3,* ,AR3
	SACL	*+,AR2
	SAR	AR3,* ,AR1
	.line	81
	LALK	FL8
	RPTK	1
	TBLR	*+
	MAR	* ,AR2
	ADRK	20
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	LALK	FL25
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$ADD
	CALL	F$$FTOI
	MAR	* ,AR2
	SBRK	21
	LAR	AR3,* ,AR3
	SACL	*+,AR2
	SAR	AR3,* ,AR1
	.line	82
	LALK	FL8
	RPTK	1
	TBLR	*+
	MAR	* ,AR2
	ADRK	22
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	LALK	FL25
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$ADD
	CALL	F$$FTOI
	MAR	* ,AR2
	SBRK	23
	LAR	AR3,* ,AR3
	SACL	*+,AR2
	SAR	AR3,* 
	.line	83
	LALK	_notch_b+0
	LDPK	_func_addr_b
	SACL	_func_addr_b
L614:
	.line	87
	LACK	64511
	LDPK	_out_gain
	AND	_out_gain
	SACL	_out_gain
EPI0_36:
	.line	89
	MAR	* ,AR1
	SBRK	26
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	3865,000000000H,25

	.sym	_set_all_gains,_set_all_gains,32,2,0
	.globl	_set_all_gains

	.func	3873
******************************************************
* FUNCTION DEF : _set_all_gains
******************************************************
_set_all_gains:
	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,5
	LAR	AR0,*0+

	.sym	_cl,1,6,1,32
	.sym	_ftemp,3,6,1,32
	.line	7
	LDPK	_params+18
	ZALS	_params+18
	ADDH	_params+19
	CALL	F$$LTOF
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	LARK	AR2,1
	MAR	*0+
	SACL	*+
	SACH	* 
	.line	8
	LACK	1
	LDPK	_gopt_a
	SACL	_gopt_a
	ZALS	_gopt_a
	SUBK	16
	BGEZ	L616
	MAR	*-,AR1
L615:
	.line	9
	SSXM
	LDPK	_max_in_level
	LAC	_max_in_level
	CALL	F$$ITOF,AR1
	MAR	* ,AR2
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	*+,AR1
	SACL	*+,AR0
	LAC	_gopt_a,1
	ADLK	_in_cal_levels+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	*+,AR1
	SACL	*+,AR3
	LAC	*-,AR1
	SACL	*+
	LALK	FL26
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$MUL
	CALL	F$$DIV
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	SACL	*+
	SACH	*-
	.line	10
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	LALK	FL6
	RPTK	1
	TBLR	*+
	CALL	F$$LE
	BNZ	L616
	.line	11
	.line	8
	LAC	_gopt_a
	ADDK	1
	SACL	_gopt_a
	ZALS	_gopt_a
	SUBK	16
	MAR	* ,AR2
	SBRK	3
	MAR	* ,AR1
	BLZ	L615
L616:
	.line	14
	LAC	_gopt_a
	SUBK	1
	SACL	_gopt_a
	.line	15
	LACK	65520
	LDPK	_out_gain
	AND	_out_gain
	LDPK	_gopt_a
	OR	_gopt_a
	LDPK	_out_gain
	SACL	_out_gain
	.line	16
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+,AR0
	LDPK	_gopt_a
	LAC	_gopt_a,1
	ADLK	_in_cal_levels+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	*+,AR1
	SACL	*+,AR3
	LAC	*-,AR1
	SACL	*+
	LALK	FL27
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$MUL
	SSXM
	LAC	_max_in_level
	CALL	F$$ITOF
	CALL	F$$DIV
	MAR	*-
	ZALH	*-
	ADDS	* 
	SACL	_scale_k_a
	SACH	_scale_k_a+1
	.line	20
	LACK	1
	SACL	_gopt_b
	ZALS	_gopt_b
	SUBK	16
	BGEZ	L619
	MAR	* ,AR2
	MAR	*-,AR1
L618:
	.line	21
	SSXM
	LDPK	_max_in_level
	LAC	_max_in_level
	CALL	F$$ITOF,AR1
	MAR	* ,AR2
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	*+,AR1
	SACL	*+,AR0
	LAC	_gopt_b,1
	ADLK	_in_cal_levels+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	*+,AR1
	SACL	*+,AR3
	LAC	*-,AR1
	SACL	*+
	LALK	FL26
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$MUL
	CALL	F$$DIV
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	SACL	*+
	SACH	*-
	.line	22
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	LALK	FL6
	RPTK	1
	TBLR	*+
	CALL	F$$LE
	BNZ	L619
	.line	23
	.line	20
	LAC	_gopt_b
	ADDK	1
	SACL	_gopt_b
	ZALS	_gopt_b
	SUBK	16
	MAR	* ,AR2
	SBRK	3
	MAR	* ,AR1
	BLZ	L618
L619:
	.line	26
	LAC	_gopt_b
	SUBK	1
	SACL	_gopt_b
	.line	27
	MAR	* ,AR0
	SACL	* 
	LAC	* ,4,AR1
	SACL	* 
	LACK	65295
	LDPK	_out_gain
	AND	_out_gain
	OR	* ,AR2
	SACL	_out_gain
	.line	28
	LARK	AR2,1
	MAR	*0+
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+,AR0
	LDPK	_gopt_b
	LAC	_gopt_b,1
	ADLK	_in_cal_levels+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	*+,AR1
	SACL	*+,AR3
	LAC	*-,AR1
	SACL	*+
	LALK	FL27
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$MUL
	SSXM
	LAC	_max_in_level
	CALL	F$$ITOF
	CALL	F$$DIV
	MAR	*-
	ZALH	*-
	ADDS	* 
	SACL	_scale_k_b
	SACH	_scale_k_b+1
	.line	31
	LDPK	_params+36
	ZALS	_params+36
	ADDH	_params+37
	BZ	L621
	.line	32
	LACK	0
	SACL	*+
	CALL	_gain
	MAR	*-
	.line	33
	LACK	1
	SACL	*+
	CALL	_gain
	MAR	*-
	B	L622
L621:
	.line	36
	LACK	2
	SACL	*+
	CALL	_gain
	MAR	*-
L622:
EPI0_37:
	.line	39
	SBRK	6
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	3911,000000000H,5

	.sym	_gain,_gain,32,2,0
	.globl	_gain

	.func	3920
******************************************************
* FUNCTION DEF : _gain
******************************************************
_gain:

LF38	.set	0

	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,8
	LAR	AR0,*0+,AR2

	.sym	_index_ab_tmp,-3+LF38,4,9,16
	.sym	_itemp,1,4,1,16
	.sym	_tgain,2,6,1,32
	.sym	_tlin,4,6,1,32
	.sym	_ftemp,6,6,1,32
	.line	2
	.line	7
	LARK	AR2,-3+LF38
	MAR	*0+
	LAC	* ,1,AR1
	SACL	* ,AR2
	LAC	* ,1,AR0
	ADLK	_params+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR0
	ADLK	_param_ptr_end+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LT	* ,AR1
	MPYK	6
	PAC
	RSXM
	ADLK	_params+0,0
	ADD	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	ZALS	*+
	ADDH	*-,AR1
	CALL	F$$LTOF
	LALK	FL28
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	ADRK	5-LF38
	SACL	*+
	SACH	* 
	.line	9
	SBRK	6-LF38
	LAC	* 
	ADDK	1
	ADRK	4-LF38
	SACL	* 
	.line	11
	BIT	* ,15
	BBZ	L623
	.line	12
	SSXM
	LDPK	_max_in_level
	LAC	_max_in_level
	CALL	F$$ITOF,AR1
	LAC	_gopt_a,1
	ADLK	_in_cal_levels+0,0
	MAR	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	*+,AR1
	SACL	*+,AR3
	LAC	*-,AR1
	SACL	*+
	LALK	FL26
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$DIV
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	ADRK	3
	SACL	*+
	SACH	*-,AR1
	.line	13
	RPTK	1
	BLKD	_scale_k_a+0,*+
	MAR	* ,AR2
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$MUL
	MAR	* ,AR2
	SBRK	3
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$DIV
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	ADRK	3
	SACL	*+
	SACH	* 
	.line	14
	LACK	1
	SACL	_aopt_a
	ZALS	_aopt_a
	SUBK	32
	BGEZ	L625
L624:
	.line	15
	LARK	AR2,6
	MAR	*0+
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	_fabs
	SBRK	2
	SACL	*+
	SACH	*+,AR0
	LDPK	_aopt_a
	LAC	_aopt_a,1
	ADLK	_out_cal+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	*+,AR1
	SACL	*+,AR3
	LAC	*-,AR1
	SACL	*+
	CALL	F$$LE
	BNZ	L625
	.line	16
	.line	14
	LAC	_aopt_a
	ADDK	1
	SACL	_aopt_a
	ZALS	_aopt_a
	SUBK	32
	MAR	* ,AR2
	BLZ	L624
L625:
	.line	19
	LAC	_aopt_a
	SUBK	1
	SACL	_aopt_a
	.line	20
	MAR	* ,AR0
	SACL	* 
	LAC	* ,4,AR1
	SACL	* 
	LACK	65039
	LDPK	_out_atten
	AND	_out_atten
	OR	* ,AR2
	SACL	_out_atten
	.line	21
	LARK	AR2,4
	MAR	*0+
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+,AR0
	LDPK	_aopt_a
	LAC	_aopt_a,1
	ADLK	_out_cal+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	*+,AR1
	SACL	*+,AR3
	LAC	*-,AR1
	SACL	*+
	CALL	F$$MUL
	MAR	* ,AR2
	SBRK	3
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$DIV
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	ADRK	3
	SACL	*+
	SACH	*-,AR1
	.line	22
	LALK	FL8
	RPTK	1
	TBLR	*+
	MAR	* ,AR2
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	LALK	FL29
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$ADD
	CALL	F$$FTOI
	LDPK	_t_reg_scale_a
	SACL	_t_reg_scale_a
L623:
	.line	24
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	BIT	* ,14
	BBZ	L627
	.line	25
	SSXM
	LDPK	_max_in_level
	LAC	_max_in_level
	CALL	F$$ITOF,AR1
	LAC	_gopt_b,1
	ADLK	_in_cal_levels+0,0
	MAR	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	*+,AR1
	SACL	*+,AR3
	LAC	*-,AR1
	SACL	*+
	LALK	FL26
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$DIV
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	ADRK	3
	SACL	*+
	SACH	*-,AR1
	.line	26
	RPTK	1
	BLKD	_scale_k_b+0,*+
	MAR	* ,AR2
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$MUL
	MAR	* ,AR2
	SBRK	3
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$DIV
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	ADRK	3
	SACL	*+
	SACH	* 
	.line	27
	LACK	1
	SACL	_aopt_b
	ZALS	_aopt_b
	SUBK	32
	BGEZ	L629
L628:
	.line	28
	LARK	AR2,6
	MAR	*0+
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	_fabs
	SBRK	2
	SACL	*+
	SACH	*+,AR0
	LDPK	_aopt_b
	LAC	_aopt_b,1
	ADLK	_out_cal+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	*+,AR1
	SACL	*+,AR3
	LAC	*-,AR1
	SACL	*+
	CALL	F$$LE
	BNZ	L629
	.line	29
	.line	27
	LAC	_aopt_b
	ADDK	1
	SACL	_aopt_b
	ZALS	_aopt_b
	SUBK	32
	MAR	* ,AR2
	BLZ	L628
L629:
	.line	32
	LAC	_aopt_b
	SUBK	1
	SACL	_aopt_b
	.line	33
	MAR	* ,AR0
	SACL	* 
	LAC	* ,9,AR1
	SACL	* 
	LACK	49663
	LDPK	_out_atten
	AND	_out_atten
	OR	* ,AR2
	SACL	_out_atten
	.line	34
	LARK	AR2,4
	MAR	*0+
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+,AR0
	LDPK	_aopt_b
	LAC	_aopt_b,1
	ADLK	_out_cal+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	*+,AR1
	SACL	*+,AR3
	LAC	*-,AR1
	SACL	*+
	CALL	F$$MUL
	MAR	* ,AR2
	SBRK	3
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$DIV
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	ADRK	3
	SACL	*+
	SACH	*-,AR1
	.line	35
	LALK	FL8
	RPTK	1
	TBLR	*+
	MAR	* ,AR2
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	LALK	FL29
	RPTK	1
	TBLR	*+
	CALL	F$$MUL
	CALL	F$$ADD
	CALL	F$$FTOI
	LDPK	_t_reg_scale_b
	SACL	_t_reg_scale_b
L627:
EPI0_38:
	.line	41
	MAR	* ,AR1
	SBRK	9
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	3960,000000000H,8

	.sym	_vu_update,_vu_update,32,2,0
	.globl	_vu_update

	.func	3970
******************************************************
* FUNCTION DEF : _vu_update
******************************************************
_vu_update:
	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,6
	LAR	AR0,*0+,AR2

	.sym	_temp_pos,1,4,1,16
	.sym	_vu_temp,2,14,1,16
	.sym	_carray,3,50,1,48,,3
	.line	7
	LACK	0
	LARK	AR2,5
	MAR	*0+
	SACL	* ,AR1
	.line	9
	BLKD	_in_a_hold+0,*+
	RPTK	1
	BLKD	_scale_k_a+0,*+
	CALL	_convert_to_vu
	SBRK	3
	MAR	* ,AR2
	LARK	AR2,2
	MAR	*0+
	SACL	* 
	.line	10
	ZALS	* 
	LDPK	_in_a_vu_level
	SUBS	_in_a_vu_level
	BLZ	L631
	.line	11
	BLDD	* ,#_in_a_vu_level
L631:
	.line	14
	MAR	* ,AR1
	BLKD	_in_b_hold+0,*+
	RPTK	1
	BLKD	_scale_k_b+0,*+
	CALL	_convert_to_vu
	SBRK	3
	MAR	* ,AR2
	LARK	AR2,2
	MAR	*0+
	SACL	* 
	.line	15
	ZALS	* 
	LDPK	_in_b_vu_level
	SUBS	_in_b_vu_level
	BLZ	L632
	.line	16
	BLDD	* ,#_in_b_vu_level
L632:
	.line	19
	MAR	* ,AR1
	BLKD	_out_a_hold+0,*+
	RPTK	1
	BLKD	_scale_k_a+0,*+
	CALL	_convert_to_vu
	SBRK	3
	MAR	* ,AR2
	LARK	AR2,2
	MAR	*0+
	SACL	* 
	.line	20
	ZALS	* 
	LDPK	_out_a_vu_level
	SUBS	_out_a_vu_level
	BLZ	L633
	.line	21
	BLDD	* ,#_out_a_vu_level
L633:
	.line	24
	MAR	* ,AR1
	BLKD	_out_b_hold+0,*+
	RPTK	1
	BLKD	_scale_k_b+0,*+
	CALL	_convert_to_vu
	SBRK	3
	MAR	* ,AR2
	LARK	AR2,2
	MAR	*0+
	SACL	* 
	.line	25
	ZALS	* 
	LDPK	_out_b_vu_level
	SUBS	_out_b_vu_level
	BLZ	L634
	.line	26
	BLDD	* ,#_out_b_vu_level
L634:
	.line	30
	MAR	*-
	BLKD	#_cursor_pos,* ,AR0
	.line	31
	LAC	_in_a_vu_level
	ADLK	_vu_chars+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR2
	ADRK	2
	SACL	*+,AR0
	.line	32
	LAC	_in_b_vu_level
	ADLK	_vu_chars+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR2
	SACL	* ,AR1
	.line	33
	LACK	-1
	SACL	*+
	LACK	10
	SACL	*+,AR3
	LARK	AR3,3
	MAR	*0+,AR1
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
	.line	35
	LDPK	_out_a_vu_level
	LAC	_out_a_vu_level
	ADLK	_vu_chars+0,0
	MAR	* ,AR0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR2
	LARK	AR2,3
	MAR	*0+
	SACL	*+,AR0
	.line	36
	LAC	_out_b_vu_level
	ADLK	_vu_chars+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR2
	SACL	* ,AR1
	.line	37
	LACK	-1
	SACL	*+
	LACK	15
	SACL	*+,AR3
	LARK	AR3,3
	MAR	*0+,AR1
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
	.line	39
	LACK	0
	SACL	*+,AR2
	LARK	AR2,1
	MAR	*0+
	LAC	* ,AR1
	SACL	*+
	LRLK	AR3,SL9
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
	.line	43
	SSXM
	LDPK	_vu_counter
	LAC	_vu_counter
	ADDK	1
	SACL	_vu_counter
	SUBK	1
	SUBK	2
	BLZ	L635
	.line	44
	LACK	0
	SACL	_vu_counter
	.line	45
	LDPK	_in_a_hold
	SACL	_in_a_hold
	.line	46
	LDPK	_in_b_hold
	SACL	_in_b_hold
	.line	47
	LDPK	_out_a_hold
	SACL	_out_a_hold
	.line	48
	LDPK	_out_b_hold
	SACL	_out_b_hold
	.line	49
	LDPK	_in_a_vu_level
	LAC	_in_a_vu_level
	BZ	L636
	.line	50
	SUBK	1
	SACL	_in_a_vu_level
L636:
	.line	52
	LAC	_in_b_vu_level
	BZ	L637
	.line	53
	SUBK	1
	SACL	_in_b_vu_level
L637:
	.line	55
	LAC	_out_a_vu_level
	BZ	L638
	.line	56
	SUBK	1
	SACL	_out_a_vu_level
L638:
	.line	58
	LAC	_out_b_vu_level
	BZ	L635
	.line	59
	SUBK	1
	SACL	_out_b_vu_level
L635:
EPI0_39:
	.line	63
	SBRK	7
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	4032,000000000H,6

	.sym	_convert_to_vu,_convert_to_vu,46,2,0
	.globl	_convert_to_vu

	.func	4040
******************************************************
* FUNCTION DEF : _convert_to_vu
******************************************************
_convert_to_vu:

LF40	.set	0

	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,4
	LAR	AR0,*0+,AR2

	.sym	_scale,-4+LF40,6,9,32
	.sym	_signal,-5+LF40,4,9,16
	.sym	_ftemp,1,6,1,32
	.sym	_itemp,3,4,1,16
	.line	2
	.line	7
	SSXM
	LARK	AR2,-5+LF40
	MAR	*0+
	LAC	*+,AR1
	CALL	F$$ITOF
	MAR	* ,AR2
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	F$$MUL
	MAR	*-
	ZALH	*-
	ADDS	* ,AR2
	ADRK	4-LF40
	SACL	*+
	SACH	*-
	.line	8
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	LALK	FL30
	RPTK	1
	TBLR	*+
	CALL	F$$LT
	BZ	L640
	.line	9
	LALK	FL30
	MAR	* ,AR2
	MAR	*-
	RPTK	1
	TBLR	*+
L640:
	.line	26
	MAR	* ,AR2
	LARK	AR2,1
	MAR	*0+
	LAC	*+,AR1
	SACL	*+,AR2
	LAC	*+,AR1
	SACL	*+
	CALL	F$$FTOI
	MAR	* ,AR2
	SACL	* 
	.line	27
	LAC	* 
	SUBK	32767
	BNZ	L641
	LACK	12
	B	EPI0_40
L641:
	.line	28
	SSXM
	LAC	* 
	SUBK	16384
	BLZ	L642
	LACK	8
	B	EPI0_40
L642:
	.line	29
	LAC	* 
	SUBK	8192
	BLZ	L643
	LACK	7
	B	EPI0_40
L643:
	.line	30
	LAC	* 
	SUBK	4096
	BLZ	L644
	LACK	6
	B	EPI0_40
L644:
	.line	31
	LAC	* 
	SUBK	2048
	BLZ	L645
	LACK	5
	B	EPI0_40
L645:
	.line	32
	LAC	* 
	SUBK	1024
	BLZ	L646
	LACK	4
	B	EPI0_40
L646:
	.line	33
	LAC	* 
	SUBK	512
	BLZ	L647
	LACK	3
	B	EPI0_40
L647:
	.line	34
	LAC	* 
	SUBK	256
	BLZ	L648
	LACK	2
	B	EPI0_40
L648:
	.line	35
	LAC	* 
	SUBK	128
	BLZ	L649
	LACK	1
	B	EPI0_40
L649:
	.line	36
	LACK	0
EPI0_40:
	.line	39
	MAR	* ,AR1
	SBRK	5
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	4078,000000000H,4

	.sym	_led_update,_led_update,32,2,0
	.globl	_led_update

	.func	4087
******************************************************
* FUNCTION DEF : _led_update
******************************************************
_led_update:
	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,1
	LAR	AR0,*0+

	.line	6
	LACK	65527
	LDPK	_out_atten
	AND	_out_atten
	SACL	* ,AR0
	LACK	256
	LDPK	_in_error_stick
	AND	_in_error_stick
	SACL	* 
	RSXM
	LAC	* ,10,AR0
	SACH	* ,1
	LAC	* ,AR1
	OR	* 
	LDPK	_out_atten
	SACL	_out_atten
	.line	7
	ANDK	65531
	SACL	* ,AR0
	LACK	512
	LDPK	_in_error_stick
	AND	_in_error_stick
	SACL	* 
	LAC	* ,8,AR0
	SACH	* ,1
	LAC	* ,AR1
	OR	* 
	LDPK	_out_atten
	SACL	_out_atten
	.line	10
	SSXM
	LDPK	_led_counter
	LAC	_led_counter
	ADDK	1
	SACL	_led_counter
	SUBK	1
	SUBK	20
	BLZ	L650
	.line	11
	LACK	0
	SACL	_led_counter
	.line	12
	LACK	65279
	LDPK	_in_error_stick
	AND	_in_error_stick
	SACL	_in_error_stick
	.line	13
	ANDK	65023
	SACL	_in_error_stick
L650:
EPI0_41:
	.line	17
	SBRK	2
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	4103,000000000H,1

	.sym	_store,_store,32,2,0
	.globl	_store

	.func	4122
******************************************************
* FUNCTION DEF : _store
******************************************************
_store:
	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,24
	LAR	AR0,*0+,AR3


	.sect	".const"
	.sblock	".const"
.init0$1$$1:
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.word	0
	.text
	.sym	_i,1,4,1,16
	.sym	_j,2,4,1,16
	.sym	_itemp,3,4,1,16
	.sym	_retrieved_flag,4,52,1,160,,10
	.sym	_next_loc,14,14,1,16
	.sym	_prev_loc,15,14,1,16
	.sym	_loc_code,16,14,1,16
	.sym	_ptr,17,14,1,16
	.sym	_ptr_m1,18,14,1,16
	.sym	_current_loc,19,14,1,16
	.sym	_fd_ptr,20,14,1,16
	.sym	_fd_ptr_m1,21,14,1,16
	.sym	_nrecord,22,14,1,16
	.sym	_uptr,23,30,1,16
	.line	4
	LARK	AR3,4
	MAR	*0+,AR4
	LRLK	AR4,.init0$1$$1+0
	LARK	AR5,9
LL110:
	LAC	*+,AR3
	SACL	*+,AR5
	BANZ	LL110,*-,AR4
	.line	5
	LACK	0
	MAR	* ,AR2
	LARK	AR2,15
	MAR	*0+
	SACL	* 
	ADRK	3
	SACL	* 
	.line	7
	ADRK	3
	SACL	* 
	.line	10
	LDPK	_min_value
	SACL	_min_value
	SACH	_min_value+1
	.line	11
	LACK	4
	SACL	_max_value
	SACH	_max_value+1
	.line	13
	LAC	_params_changed_copy
	SUBK	3
	BNZ	EPI0_42
	.line	14
	.line	17
	MAR	*+
	SPLK	#1050,* 
	.line	19
	SBRK	3
	BLKD	#_params+60,* ,AR0
	.line	21
	BLKD	#_func_addr_a,_func_addr_temp_a
	.line	22
	BLKD	#_func_addr_b,_func_addr_temp_b
	.line	23
	LALK	_no_func_a+0
	LDPK	_func_addr_a
	SACL	_func_addr_a
	.line	24
	LALK	_no_func_b+0
	LDPK	_func_addr_b
	SACL	_func_addr_b
	.line	26
	IN	* ,0fff5h
	LAC	* 
	ANDK	65023
	SACL	* 
	OUT	* ,0fff5h,AR1
	.line	27
	LACK	0
	SACL	*+
	LACK	8
	SACL	*+
	LACK	1
	SACL	*+,AR2
	ZALS	* ,AR1
	SACL	*+
	SACH	*+
	CALL	_disp_num
	SBRK	5
	.line	28
	LACK	0
	SACL	*+
	LACK	9
	SACL	*+
	LRLK	AR3,SL93
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
	.line	29
	LACK	400
	SACL	*+
	LACK	75
	SACL	*+
	CALL	_beep
	SBRK	2
	.line	32
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	SPLK	#24576,* ,AR1
	.line	33
	LALK	_record+0
	SACL	*+
	LACK	5
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	_read_flash
	SBRK	3
	.line	34
	MAR	* ,AR2
	LARK	AR2,14
	MAR	*0+
	BLKD	#_record,* 
	.line	35
	ZALS	* 
	RSXM
	SUBK	-1
	BZ	L653
	MAR	* ,AR1
L652:
	.line	37
	CALL	_record_bad,AR1
	BNZ	L655

	.sym	_store_error,L655,0,6,0
	.line	38
	.line	40
	MAR	* ,AR2
	LARK	AR2,17
	MAR	*0+
	LAC	*+
	SACL	* 
	.line	41
	SBRK	4
	LAC	* 
	ADRK	3
	SACL	* ,AR1
	.line	42
	LALK	_record+0
	SACL	*+
	LACK	5
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	_read_flash
	SBRK	3
	.line	43
	MAR	* ,AR2
	LARK	AR2,14
	MAR	*0+
	BLKD	#_record,* 
	.line	35
	ZALS	* ,AR1
	RSXM
	SUBK	-1
	BNZ	L652
L653:
	.line	47
	LACK	32767
	MAR	* ,AR2
	ADRK	3
	SUBS	* 
	ANDK	0FFFFh
	ADRK	5
	SUBS	* 
	BGEZ	L656
	.line	52
	LACK	0
	SBRK	2
	SACL	* 
	.line	53
	SBRK	2
	LAC	* 
	SBRK	3
	SACL	* 
	.line	54
	ADRK	4
	LAR	AR3,* ,AR3
	MAR	*0+
	LACK	1
	ADRK	4
	SACL	* ,AR2
	SBRK	4
L658:
	.line	57
	LAC	* 
	ADRK	2
	SACL	* ,AR1
	.line	58
	LALK	_record+0
	SACL	*+
	LACK	6
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	_read_flash
	SBRK	3
	.line	59
	MAR	* ,AR2
	LARK	AR2,14
	MAR	*0+
	BLKD	#_record,*+
	.line	60
	BLKD	#_record+1,*+
	.line	61
	BLKD	#_record+5,* ,AR1
	.line	62
	CALL	_record_bad
	BZ	L659
L655:
	.line	64
	MAR	* ,AR1
	LACK	-1
	SACL	*+
	LACK	1
	SACL	*+
	LRLK	AR3,SL94
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
	.line	65
	LALK	45,15
	ORK	25440
	SACL	*+
	SACH	*+
	CALL	_wait
	SBRK	2

	.sym	_store_out,L660,0,6,0
	.line	66
	B	L660
L659:
	.line	68
	MAR	* ,AR2
	LARK	AR2,15
	MAR	*0+
	LAC	* 
	BZ	LL111
	MAR	*+
	LAR	AR3,* ,AR3
	MAR	*0+
	ADRK	4
	LAC	* ,AR2
	SUBK	1
	MAR	*-
	BZ	L658
LL111:
	.line	70
	MAR	*+
	LAR	AR3,* ,AR3
	MAR	*0+
	ADRK	4
	LAC	* 
	BNZ	L661
	.line	73
	MAR	* ,AR2
	ADRK	4
	LAC	* ,AR1
	ADLK	_flash_data+0,0
	SACL	*+,AR2
	ADRK	2
	LAC	* ,AR1
	SACL	*+,AR2
	SBRK	5
	LAC	* ,AR1
	SACL	*+
	CALL	_read_flash
	SBRK	3
	.line	74
	MAR	* ,AR2
	LARK	AR2,20
	MAR	*0+
	LAC	* 
	ADDK	24576
	ADRK	2
	ADD	* ,AR1
	SACL	*+,AR2
	SBRK	2
	LAC	*+,AR0
	ADLK	_flash_data+0,0
	SACL	* 
	LAR	AR3,* ,AR1
	MAR	*-
	LAC	* ,AR3
	SACL	* ,AR2
	.line	75
	LAC	*-,AR1
	ADDK	24576
	SACL	* ,AR2
	LAC	* ,AR0
	ADLK	_flash_data+1,0
	SACL	* 
	LAR	AR3,* ,AR1
	LAC	* ,AR3
	SACL	* ,AR2
	.line	76
	LAC	*+
	SACL	*+
	.line	77
	ADD	* 
	SBRK	2
	SACL	* 
	.line	78
	SBRK	4
	LAR	AR3,* ,AR3
	MAR	*0+
	LACK	1
	ADRK	4
	SACL	* 
L661:
	.line	80
	MAR	* ,AR2
	MAR	*-
	LAC	* 
	BNZ	L658
	.line	83
	LACK	0
	LDPK	_flash_data+1
	SACL	_flash_data+1
	.line	84
	LDPK	_flash_locked
	SACL	_flash_locked
	.line	85
	LRLK	AR3,SL95
	MAR	* ,AR1
	SAR	AR3,*+
	LACK	1
	SACL	*+
	LALK	_flash_data+0
	SACL	*+,AR2
	ADRK	5
	LAC	* ,AR1
	SACL	*+
	LACK	24576
	SACL	*+
	CALL	_prog_flash
	SBRK	5
	MAR	* ,AR2
	LARK	AR2,3
	MAR	*0+
	SACL	* 
	.line	86
	LAC	* 
	BNZ	L660
	.line	87
	.line	89
	ADRK	17
	LAC	* 
	ADDK	24576
	SBRK	3
	SACL	* 
	.line	91
	LAC	* 
	ADRK	5
	SUB	* 
	SBRK	4
	SACL	* 
L656:
	.line	95
	LARK	AR2,22
	MAR	*0+
	LAC	* 
	SBRK	5
	ADD	*+
	LDPK	_record
	SACL	_record
	.line	96
	BLDD	*+,#_record+1
	.line	97
	LACK	220
	SACL	_record+2
	.line	98
	BLKD	#_serial_number,_record+3
	.line	99
	BLKD	#_serial_number+1,_record+4
	.line	100
	BLDD	* ,#_record+5
	.line	106
	LACK	6
	SBRK	17
	SACL	* 
	.line	107
	LALK	_params+0
	ADRK	21
	SACL	* 
	.line	108
	LACK	0
	SBRK	22
	SACL	* 
	ZALS	* 
	SUBK	174
	BGEZ	L664
	ADRK	22
L663:
	.line	109
	LAR	AR3,* 
	SBRK	21
	LAC	* 
	ADDK	1
	SACL	* ,AR0
	SUBK	1
	ADLK	_record+0,0
	SACL	* 
	LAR	AR4,* ,AR3
	LAC	*+,AR2
	ADRK	21
	SAR	AR3,* ,AR4
	SACL	* ,AR2
	.line	110
	LAR	AR3,* 
	SBRK	21
	LAC	* 
	ADDK	1
	SACL	* ,AR0
	SUBK	1
	ADLK	_record+0,0
	SACL	* 
	LAR	AR4,* ,AR3
	LAC	*+,AR2
	ADRK	21
	SAR	AR3,* ,AR4
	SACL	* ,AR2
	.line	111
	LAR	AR3,* 
	SBRK	21
	LAC	* 
	ADDK	1
	SACL	* ,AR0
	SUBK	1
	ADLK	_record+0,0
	SACL	* 
	LAR	AR4,* ,AR3
	LAC	*+,AR2
	ADRK	21
	SAR	AR3,* ,AR4
	SACL	* ,AR2
	.line	112
	LAR	AR3,* 
	SBRK	21
	LAC	* 
	ADDK	1
	SACL	* ,AR0
	SUBK	1
	ADLK	_record+0,0
	SACL	* 
	LAR	AR4,* ,AR3
	LAC	*+,AR2
	ADRK	21
	SAR	AR3,* ,AR4
	SACL	* ,AR2
	.line	113
	LAR	AR3,* 
	SBRK	21
	LAC	* 
	ADDK	1
	SACL	* ,AR0
	SUBK	1
	ADLK	_record+0,0
	SACL	* 
	LAR	AR4,* ,AR3
	LAC	*+,AR2
	ADRK	21
	SAR	AR3,* ,AR4
	SACL	* ,AR2
	.line	114
	LAR	AR3,* 
	SBRK	21
	LAC	* 
	ADDK	1
	SACL	* ,AR0
	SUBK	1
	ADLK	_record+0,0
	SACL	* 
	LAR	AR4,* ,AR3
	LAC	*+,AR2
	ADRK	21
	SAR	AR3,* ,AR4
	SACL	* ,AR2
	.line	108
	SBRK	22
	LAC	* 
	ADDK	1
	SACL	* 
	ZALS	* 
	SUBK	174
	ADRK	22
	BLZ	L663
L664:
	.line	118
	LACK	0
	LDPK	_flash_locked
	SACL	_flash_locked
	.line	119
	LRLK	AR3,SL96
	MAR	* ,AR1
	SAR	AR3,*+
	SACL	*+
	LALK	_record+0
	SACL	*+,AR2
	LARK	AR2,22
	MAR	*0+
	LAC	* ,AR1
	SACL	*+,AR2
	SBRK	5
	LAC	* ,AR1
	SACL	*+
	CALL	_prog_flash
	SBRK	5
L660:
	.line	125
	LDPK	_func_addr_temp_a
	BLDD	_func_addr_temp_a,#_func_addr_a
	.line	126
	BLDD	_func_addr_temp_b,#_func_addr_b
	.line	127
	LALK	15,15
	ORK	8480
	MAR	* ,AR1
	SACL	*+
	SACH	*+
	CALL	_wait
	SBRK	2
	.line	128
	CALL	_update_disp_left
	.line	129
	LACK	1
	SACL	*+
	CALL	_update_disp_right
	MAR	*-,AR0
	.line	130
	IN	* ,0fff5h
	LAC	* 
	ORK	512
	SACL	* 
	OUT	* ,0fff5h
	.line	131
	LACK	0
	LDPK	_sw_down
	SACL	_sw_down
EPI0_42:
	.line	132
	MAR	* ,AR1
	SBRK	25
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	4253,000000000H,24

	.sym	_recall,_recall,32,2,0
	.globl	_recall

	.func	4261
******************************************************
* FUNCTION DEF : _recall
******************************************************
_recall:
	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,12
	LAR	AR0,*0+,AR2

	.sym	_i,1,4,1,16
	.sym	_j,2,4,1,16
	.sym	_next_loc,3,14,1,16
	.sym	_prev_loc,4,14,1,16
	.sym	_loc_code,5,14,1,16
	.sym	_ptr,6,14,1,16
	.sym	_ptr_m1,7,14,1,16
	.sym	_current_loc,8,14,1,16
	.sym	_desired_loc_ptr,9,14,1,16
	.sym	_nrecord,10,14,1,16
	.sym	_uptr,11,30,1,16
	.line	5
	LACK	0
	LARK	AR2,4
	MAR	*0+
	SACL	* 
	ADRK	3
	SACL	* 
	.line	6
	ADRK	2
	SACL	* 
	.line	9
	LDPK	_min_value
	SACL	_min_value
	SACH	_min_value+1
	.line	10
	LACK	4
	SACL	_max_value
	SACH	_max_value+1
	.line	12
	LAC	_params_changed_copy
	SUBK	3
	BNZ	EPI0_43
	.line	13
	.line	16
	MAR	*+
	SPLK	#1050,* 
	.line	18
	SBRK	2
	BLKD	#_params+66,* ,AR0
	.line	20
	LALK	_no_func_a+0
	LDPK	_func_addr_a
	SACL	_func_addr_a
	.line	21
	LALK	_no_func_b+0
	LDPK	_func_addr_b
	SACL	_func_addr_b
	.line	23
	IN	* ,0fff5h
	LAC	* 
	ANDK	65023
	SACL	* 
	OUT	* ,0fff5h,AR1
	.line	24
	LACK	0
	SACL	*+
	LACK	7
	SACL	*+
	LACK	1
	SACL	*+,AR2
	ZALS	* ,AR1
	SACL	*+
	SACH	*+
	CALL	_disp_num
	SBRK	5
	.line	25
	LACK	0
	SACL	*+
	LACK	8
	SACL	*+
	LRLK	AR3,SL97
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
	.line	26
	LACK	400
	SACL	*+
	LACK	75
	SACL	*+
	CALL	_beep
	SBRK	2
	.line	29
	MAR	* ,AR2
	LARK	AR2,6
	MAR	*0+
	SPLK	#24576,* ,AR1
	.line	30
	LALK	_record+0
	SACL	*+
	LACK	6
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	_read_flash
	SBRK	3
	.line	31
	MAR	* ,AR2
	LARK	AR2,3
	MAR	*0+
	BLKD	#_record,* 
	.line	32
	ADRK	2
	BLKD	#_record+5,* 
	.line	33
	SBRK	2
	ZALS	* 
	RSXM
	SUBK	-1
	BZ	L667
	MAR	* ,AR1
L666:
	.line	34
	CALL	_record_bad,AR1
	BNZ	L669

	.sym	_recall_error,L669,0,6,0
	.line	35
	.line	37
	MAR	* ,AR2
	LARK	AR2,5
	MAR	*0+
	LAC	* 
	ADRK	3
	SUB	* 
	BNZ	L670
	.line	38
	SBRK	2
	LAC	* 
	ADRK	3
	SACL	* 
L670:
	.line	40
	LARK	AR2,6
	MAR	*0+
	LAC	*+
	SACL	* 
	.line	41
	SBRK	4
	LAC	* 
	ADRK	3
	SACL	* ,AR1
	.line	42
	LALK	_record+0
	SACL	*+
	LACK	6
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	_read_flash
	SBRK	3
	.line	43
	MAR	* ,AR2
	LARK	AR2,3
	MAR	*0+
	BLKD	#_record,* 
	.line	44
	ADRK	2
	BLKD	#_record+5,* 
	.line	33
	SBRK	2
	ZALS	* ,AR1
	RSXM
	SUBK	-1
	BNZ	L666
L667:
	.line	48
	MAR	* ,AR2
	ADRK	6
	LAC	* 
	BNZ	L671
L669:
	.line	51
	MAR	* ,AR1
	LACK	-1
	SACL	*+
	LACK	1
	SACL	*+
	LRLK	AR3,SL98
	SAR	AR3,*+
	CALL	_disp_text
	SBRK	3
	.line	52
	LALK	30,15
	ORK	16960
	SACL	*+
	SACH	*+
	CALL	_wait
	SBRK	2

	.sym	_recall_out,L672,0,6,0
	.line	53
	B	L672
L671:
	.line	57
	LALK	_record+0
	MAR	*+,AR1
	SACL	*+,AR2
	LAC	*-,AR1
	SACL	*+,AR2
	LAC	* ,AR1
	SACL	*+
	CALL	_read_flash
	SBRK	3
	.line	60
	LACK	6
	MAR	* ,AR2
	LARK	AR2,2
	MAR	*0+
	SACL	* 
	.line	61
	LALK	_params+0
	ADRK	9
	SACL	* 
	.line	62
	LACK	0
	SBRK	10
	SACL	* 
	ZALS	* 
	SUBK	174
	BGEZ	L672
	MAR	*+
L673:
	.line	63
	LAC	* 
	ADDK	1
	SACL	* ,AR0
	SUBK	1
	ADLK	_record+0,0
	SACL	* 
	LAR	AR3,* ,AR2
	ADRK	9
	LAR	AR4,* ,AR3
	LAC	* ,AR4
	SACL	*+,AR2
	SAR	AR4,* 
	.line	64
	SBRK	9
	LAC	* 
	ADDK	1
	SACL	* ,AR0
	SUBK	1
	ADLK	_record+0,0
	SACL	* 
	LAR	AR3,* ,AR2
	ADRK	9
	LAR	AR4,* ,AR3
	LAC	* ,AR4
	SACL	*+,AR2
	SAR	AR4,* 
	.line	65
	SBRK	9
	LAC	* 
	ADDK	1
	SACL	* ,AR0
	SUBK	1
	ADLK	_record+0,0
	SACL	* 
	LAR	AR3,* ,AR2
	ADRK	9
	LAR	AR4,* ,AR3
	LAC	* ,AR4
	SACL	*+,AR2
	SAR	AR4,* 
	.line	66
	SBRK	9
	LAC	* 
	ADDK	1
	SACL	* ,AR0
	SUBK	1
	ADLK	_record+0,0
	SACL	* 
	LAR	AR3,* ,AR2
	ADRK	9
	LAR	AR4,* ,AR3
	LAC	* ,AR4
	SACL	*+,AR2
	SAR	AR4,* 
	.line	67
	SBRK	9
	LAC	* 
	ADDK	1
	SACL	* ,AR0
	SUBK	1
	ADLK	_record+0,0
	SACL	* 
	LAR	AR3,* ,AR2
	ADRK	9
	LAR	AR4,* ,AR3
	LAC	* ,AR4
	SACL	*+,AR2
	SAR	AR4,* 
	.line	68
	SBRK	9
	LAC	* 
	ADDK	1
	SACL	* ,AR0
	SUBK	1
	ADLK	_record+0,0
	SACL	* 
	LAR	AR3,* ,AR2
	ADRK	9
	LAR	AR4,* ,AR3
	LAC	* ,AR4
	SACL	*+,AR2
	SAR	AR4,* 
	.line	62
	SBRK	10
	LAC	* 
	ADDK	1
	SACL	* 
	ZALS	*+
	SUBK	174
	BLZ	L673
L672:
	.line	75
	CALL	_set_fsample,AR1
	.line	76
	LDPK	_params+12
	BLDD	_params+12,#_auto_vu_count
	.line	77
	LAC	_params+30
	BZ	LL114
	LACK	8
	LDPK	_assembly_flag
	OR	_assembly_flag
	SACL	* 
	LAC	* 
	B	LL115
LL114:
	LACK	-9
	LDPK	_assembly_flag
	AND	_assembly_flag
	SACL	* 
	LAC	* 
LL115:
	SACL	_assembly_flag
	.line	78
	LDPK	_params+42
	LAC	_params+42
	BZ	LL116
	LACK	4
	LDPK	_assembly_flag
	OR	_assembly_flag
	SACL	* 
	LAC	* 
	B	LL117
LL116:
	LACK	-5
	LDPK	_assembly_flag
	AND	_assembly_flag
	SACL	* 
	LAC	* 
LL117:
	SACL	_assembly_flag
	.line	81
	LACK	0
	LDPK	_in_a_hold
	SACL	_in_a_hold
	.line	82
	LDPK	_in_b_hold
	SACL	_in_b_hold
	.line	83
	LDPK	_out_a_hold
	SACL	_out_a_hold
	.line	84
	LDPK	_out_b_hold
	SACL	_out_b_hold
	.line	85
	LDPK	_in_a_vu_level
	SACL	_in_a_vu_level
	.line	86
	SACL	_in_b_vu_level
	.line	87
	SACL	_out_a_vu_level
	.line	88
	SACL	_out_b_vu_level
	.line	91
	SACL	_param_ptr
	.line	92
	SACL	_flag_options
	.line	93
	LDPK	_params+36
	ZALS	_params+36
	ADDH	_params+37
	BZ	LL118
	LACK	0
	B	LL119
LL118:
	LACK	2
LL119:
	LDPK	_index_ab
	SACL	_index_ab
	.line	95
	CALL	_set_all_gains
	.line	99
	LDPK	_params+36
	ZALS	_params+36
	ADDH	_params+37
	BNZ	L675
	.line	100
	LACK	2
	SACL	*+,AR0
	LAC	_params+4
	ADLK	_param_ptr_start+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR1
	SACL	*+
	CALL	_update_dsp
	SBRK	2
	B	L676
L675:
	.line	102
	LT	_params+36
	MPYK	1
	LPH	_params+37
	LACK	1
	SPAC
	BNZ	L677
	.line	103
	LACK	0
	SACL	*+,AR0
	LAC	_params
	ADLK	_param_ptr_start+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR1
	SACL	*+
	CALL	_update_dsp
	SBRK	2
	.line	104
	LACK	1
	SACL	*+,AR0
	LDPK	_params+2
	LAC	_params+2
	ADLK	_param_ptr_start+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR1
	SACL	*+
	CALL	_update_dsp
	SBRK	2
	B	L676
L677:
	.line	107
	LALK	_no_func_b+0
	LDPK	_func_addr_b
	SACL	_func_addr_b
	.line	108
	LACK	0
	SACL	*+,AR0
	LDPK	_params
	LAC	_params
	ADLK	_param_ptr_start+0,0
	SACL	* 
	LAR	AR3,* ,AR3
	LAC	* ,AR1
	SACL	*+
	CALL	_update_dsp
	SBRK	2
L676:
	.line	110
	LACK	0
	LDPK	_min_value
	SACL	_min_value
	SACH	_min_value+1
	.line	111
	LACK	4
	SACL	_max_value
	SACH	_max_value+1
	.line	113
	LALK	18,15
	ORK	10176
	SACL	*+
	SACH	*+
	CALL	_wait
	SBRK	2
	.line	116
	CALL	_update_disp_left
	.line	117
	LACK	1
	SACL	*+
	CALL	_update_disp_right
	MAR	*-,AR0
	.line	118
	IN	* ,0fff5h
	LAC	* 
	ORK	512
	SACL	* 
	OUT	* ,0fff5h
	.line	119
	LACK	0
	LDPK	_sw_down
	SACL	_sw_down
EPI0_43:
	.line	120
	MAR	* ,AR1
	SBRK	13
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	4380,000000000H,12

	.sym	_record_bad,_record_bad,36,2,0
	.globl	_record_bad

	.func	4390
******************************************************
* FUNCTION DEF : _record_bad
******************************************************
_record_bad:
	POPD	*+
	SAR	AR0,*+
	SAR	AR1,*
	LARK	AR0,1
	LAR	AR0,*0+

	.line	5
	LDPK	_record+2
	LAC	_record+2
	SUBK	220
	BNZ	LL122
	LAC	_record+3
	LDPK	_serial_number
	SUB	_serial_number
	BNZ	LL122
	LDPK	_record+4
	LAC	_record+4
	LDPK	_serial_number+1
	SUB	_serial_number+1
	BNZ	LL122
	ZAC
	B	EPI0_44
LL122:
	LACK	1
EPI0_44:
	.line	6
	SBRK	2
	LAR	AR0,*-
	PSHD	*
	RET

	.endfunc	4395,000000000H,1

	.sym	_serial_number,_serial_number,5,2,32
	.globl	_serial_number

	.sym	_func_addr_temp_a,_func_addr_temp_a,14,2,16
	.globl	_func_addr_temp_a

	.sym	_func_addr_temp_b,_func_addr_temp_b,14,2,16
	.globl	_func_addr_temp_b

	.sym	_record,_record,62,2,16800,,1050
	.globl	_record

	.sym	_max_in_level,_max_in_level,4,2,16
	.globl	_max_in_level

	.sym	_window,_window,54,2,4096,,128
	.globl	_window

	.sym	_port0_copy,_port0_copy,14,2,16
	.globl	_port0_copy

	.sym	_gray_code_old,_gray_code_old,4,2,16
	.globl	_gray_code_old

	.sym	_in_b_vu_level,_in_b_vu_level,14,2,16
	.globl	_in_b_vu_level

	.sym	_p_uint1,_p_uint1,14,2,16
	.globl	_p_uint1

	.sym	_p_uint2,_p_uint2,14,2,16
	.globl	_p_uint2

	.sym	_cursor_flag,_cursor_flag,4,2,16
	.globl	_cursor_flag

	.sym	_params_changed,_params_changed,4,2,16
	.globl	_params_changed

	.sym	_data_count,_data_count,14,2,16
	.globl	_data_count

	.sym	_out_b_vu_level,_out_b_vu_level,14,2,16
	.globl	_out_b_vu_level

	.sym	_gray_code,_gray_code,4,2,16
	.globl	_gray_code

	.sym	_aopt_a,_aopt_a,14,2,16
	.globl	_aopt_a

	.sym	_cursor_pos_vu,_cursor_pos_vu,4,2,16
	.globl	_cursor_pos_vu

	.sym	_aopt_b,_aopt_b,14,2,16
	.globl	_aopt_b

	.sym	_serial_in_buf,_serial_in_buf,50,2,2048,,128
	.globl	_serial_in_buf

	.sym	_press_flag,_press_flag,4,2,16
	.globl	_press_flag

	.sym	_p_state,_p_state,4,2,16
	.globl	_p_state

	.sym	_flash_locked,_flash_locked,4,2,16
	.globl	_flash_locked

	.sym	_ccw,_ccw,4,2,16
	.globl	_ccw

	.sym	_check_sum,_check_sum,5,2,32
	.globl	_check_sum

	.sym	_param_ptr,_param_ptr,4,2,16
	.globl	_param_ptr

	.sym	_cw,_cw,4,2,16
	.globl	_cw

	.sym	_auto_vu_count,_auto_vu_count,4,2,16
	.globl	_auto_vu_count

	.sym	_out_a_vu_level,_out_a_vu_level,14,2,16
	.globl	_out_a_vu_level

	.sym	_params,_params,245,2,16704,,174,3
	.globl	_params

	.sym	_index_ab_p,_index_ab_p,4,2,16
	.globl	_index_ab_p

	.sym	_flag_options,_flag_options,4,2,16
	.globl	_flag_options

	.sym	_params_changed_copy,_params_changed_copy,4,2,16
	.globl	_params_changed_copy

	.sym	_cursor_pos,_cursor_pos,4,2,16
	.globl	_cursor_pos

	.sym	_p_sum,_p_sum,5,2,32
	.globl	_p_sum

	.sym	_sn_ok_flag,_sn_ok_flag,4,2,16
	.globl	_sn_ok_flag

	.sym	_serial_error_flag,_serial_error_flag,4,2,16
	.globl	_serial_error_flag

	.sym	_sw_down,_sw_down,4,2,16
	.globl	_sw_down

	.sym	_quietsn_flag,_quietsn_flag,4,2,16
	.globl	_quietsn_flag

	.sym	_flash_data,_flash_data,62,2,131072,,8192
	.globl	_flash_data

	.sym	_testcount,_testcount,4,2,16
	.globl	_testcount

	.sym	_value_str,_value_str,50,2,272,,17
	.globl	_value_str

	.sym	_max_value,_max_value,5,2,32
	.globl	_max_value

	.sym	_flash_cursor_flag,_flash_cursor_flag,4,2,16
	.globl	_flash_cursor_flag

	.sym	_fsample,_fsample,6,2,32
	.globl	_fsample

	.sym	_ioData,_ioData,14,2,16
	.globl	_ioData

	.sym	_scale_k_a,_scale_k_a,6,2,32
	.globl	_scale_k_a

	.sym	_sw_down_old,_sw_down_old,4,2,16
	.globl	_sw_down_old

	.sym	_scale_k_b,_scale_k_b,6,2,32
	.globl	_scale_k_b

	.sym	_imr_ptr,_imr_ptr,30,2,16
	.globl	_imr_ptr

	.sym	_vu_counter,_vu_counter,4,2,16
	.globl	_vu_counter

	.sym	_in_a_vu_level,_in_a_vu_level,14,2,16
	.globl	_in_a_vu_level

	.sym	_p_long,_p_long,5,2,32
	.globl	_p_long

	.sym	_read_ptr,_read_ptr,4,2,16
	.globl	_read_ptr

	.sym	_down_turn_flag,_down_turn_flag,4,2,16
	.globl	_down_turn_flag

	.sym	_gopt_a,_gopt_a,14,2,16
	.globl	_gopt_a

	.sym	_min_value,_min_value,5,2,32
	.globl	_min_value

	.sym	_gopt_b,_gopt_b,14,2,16
	.globl	_gopt_b

	.sym	_parameter_str,_parameter_str,50,2,272,,17
	.globl	_parameter_str

	.sym	_serial_str,_serial_str,50,2,176,,11
	.globl	_serial_str

	.sym	_ifr_ptr,_ifr_ptr,30,2,16
	.globl	_ifr_ptr

	.sym	_index_ab,_index_ab,4,2,16
	.globl	_index_ab

	.sym	_write_ptr,_write_ptr,4,2,16
	.globl	_write_ptr

	.sym	_greg_ptr,_greg_ptr,30,2,16
	.globl	_greg_ptr

	.sym	_iorder_old,_iorder_old,4,2,16
	.globl	_iorder_old

	.sym	_sw_pressed,_sw_pressed,4,2,16
	.globl	_sw_pressed

	.sym	_sign_mult,_sign_mult,4,2,16
	.globl	_sign_mult

	.sym	_led_counter,_led_counter,4,2,16
	.globl	_led_counter

	.sym	_sw_released,_sw_released,4,2,16
	.globl	_sw_released
******************************************************
* DEFINE STRING CONSTANTS                            *
******************************************************
	.sect	".const"
SL0:	.byte	"NoFunc     ",0
SL1:	.byte	"AllPass    ",0
SL2:	.byte	"LowPass    ",0
SL3:	.byte	"HighPass   ",0
SL4:	.byte	"BandPass   ",0
SL5:	.byte	"BandStop   ",0
SL6:	.byte	"Notch      ",0
SL7:	.byte	"InvNotch   ",0
SL8:	.byte	"UserFIR    ",0
SL9:	.byte	0
SL10:	.byte	"N",0
SL11:	.byte	"Y",0
SL12:	.byte	" 8KHz",0
SL13:	.byte	"48KHz",0
SL14:	.byte	"Analog ",0
SL15:	.byte	"WtNoise",0
SL16:	.byte	"A&B Common ",0
SL17:	.byte	"A&BSeparate",0
SL18:	.byte	"Ch A Only  ",0
SL19:	.byte	"N   ",0
SL20:	.byte	"Y   ",0
SL21:	.byte	"press",0
SL22:	.byte	" FUNC:",0
SL23:	.byte	"Levels-In  Out  ",0
SL24:	.byte	"RevertToLevels:",0
SL25:	.byte	"FullScalIn:#_Vpp",0
SL26:	.byte	"SampleRate:",0
SL27:	.byte	"InputSrc:",0
SL28:	.byte	"Mode:",0
SL29:	.byte	"Cascade Ch A&B:",0
SL30:	.byte	"Master Mode:",0
SL31:	.byte	"Initialize:",0
SL32:	.byte	"Store:  # press ",0
SL33:	.byte	"Recall: # press ",0
SL34:	.byte	"Firmware:  V",0
SL35:	.byte	"Serial No:",0
SL36:	.byte	" NFgain:###_.##x",0
SL37:	.byte	" APgain:###_.##x",0
SL38:	.byte	" LPfcut: #####Hz",0
SL39:	.byte	" LPorder:    ###",0
SL40:	.byte	" LPgain:###_.##x",0
SL41:	.byte	" HPfcut: #####Hz",0
SL42:	.byte	" HPorder:    ###",0
SL43:	.byte	" HPgain:###_.##x",0
SL44:	.byte	" BPf1:   #####Hz",0
SL45:	.byte	" BPf2:   #####Hz",0
SL46:	.byte	" BPfcntr:#####Hz",0
SL47:	.byte	" BPfwdth:#####Hz",0
SL48:	.byte	" BPorder:    ###",0
SL49:	.byte	" BPgain:###_.##x",0
SL50:	.byte	" BSf1:   #####Hz",0
SL51:	.byte	" BSf2:   #####Hz",0
SL52:	.byte	" BSfcntr:#####Hz",0
SL53:	.byte	" BSfwdth:#####Hz",0
SL54:	.byte	" BSorder:    ###",0
SL55:	.byte	" BSgain:###_.##x",0
SL56:	.byte	" Nfnotch:#####Hz",0
SL57:	.byte	" Nfwidth:#####Hz",0
SL58:	.byte	" Ngain: ###_.##x",0
SL59:	.byte	" INfcntr:#####Hz",0
SL60:	.byte	" INfwdth:#####Hz",0
SL61:	.byte	" INgain:###_.##x",0
SL62:	.byte	" UForder:    ###",0
SL63:	.byte	" UFtap:##_      ",0
SL64:	.byte	" UFgain:###_.##x",0
SL65:	.byte	" Sfreq:#####.#Hz",0
SL66:	.byte	" Sphase:###.#deg",0
SL67:	.byte	" Samp:   ###_.##",0
SL68:	.byte	"RS-232 Error!   ",0
SL69:	.byte	"Versa-Filter2-20",0
SL70:	.byte	"Testing         ",0
SL71:	.byte	".",0
SL72:	.byte	"OK",0
SL73:	.byte	"program",0
SL74:	.byte	"Recvd    0 Words",0
SL75:	.byte	"reset",0
SL76:	.byte	"display",0
SL77:	.byte	"                ",0
SL78:	.byte	"sendsn",0
SL79:	.byte	"quietsn",0
SL80:	.byte	"echo",0
SL81:	.byte	"PARAMETER ERROR ",0
SL82:	.byte	"WRITE ABORTED!  ",0
SL83:	.byte	"Programming...  ",0
SL84:	.byte	"inUpdate",0
SL85:	.byte	"Sector   Prog OK",0
SL86:	.byte	"Press to        ",0
SL87:	.byte	"Cancelled       ",0
SL88:	.byte	"ProgERR",0
SL89:	.byte	"a",0
SL90:	.byte	"b",0
SL91:	.byte	" ",0
SL92:	.byte	"in Erase",0
SL93:	.byte	" Stored ",0
SL94:	.byte	"Error-MemCorrupt",0
SL95:	.byte	"inStore1",0
SL96:	.byte	"inStore2",0
SL97:	.byte	" Recalled",0
SL98:	.byte	"Location Blank! ",0
*************************************************
* DEFINE FLOATING POINT CONSTANTS               *
*************************************************
	.text
FL0:	.float	4.8e4
FL1:	.float	8.e3
FL2:	.float	4.166666666666666e-3
FL3:	.float	4.166666666666667e-1
FL4:	.float	4.083333333333334e-1
FL5:	.float	8.333333333333333e-3
FL6:	.float	2.
FL7:	.float	1.25e-2
FL8:	.float	5.e-1
FL9:	.float	4.125e-1
FL10:	.float	2.083333333333334e-4
FL11:	.float	2.083333333333334e-1
FL12:	.float	6.28318530717959
FL13:	.float	3.233315142896e-2
FL14:	.float	4.8550251793519e-1
FL15:	.float	4.8216433063585e-1
FL16:	.float	3.14159265359
FL17:	.float	1.
FL18:	.float	4.999e-1
FL19:	.float	3.2768e4
FL20:	.float	6.5536e4
FL21:	.float	1.5707963268
FL22:	.float	6.28318530718
FL23:	.float	-1.
FL24:	.float	0.
FL25:	.float	8.192e3
FL26:	.float	1.6e1
FL27:	.float	8.
FL28:	.float	1.e-2
FL29:	.float	2.56e2
FL30:	.float	3.2767e4
*****************************************************
* UNDEFINED REFERENCES                              *
*****************************************************
	.global	L$$MPY
	.global	L$$DIVS
	.global	L$$MODS
	.global	F$$FTOL
	.global	F$$LTOF
	.global	F$$MUL
	.global	F$$SUB
	.global	F$$LT
	.global	F$$ADD
	.global	F$$DIV
	.global	F$$ITOF
	.global	F$$FTOI
	.global	F$$NEG
	.global	F$$LE
	.end
