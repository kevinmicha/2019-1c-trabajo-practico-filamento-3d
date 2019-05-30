;--------------------------------------------------------------------
;	Módulo display.asm
;--------------------------------------------------------------------
 
;--------------------------------------------------------------------
;	Constantes del display
;--------------------------------------------------------------------
.equ	CLR_LCD		= 0x01 ; Clear Display
.equ	HOME_LCD	= 0x02 ; Return Home
.equ	MODE_LCD	= 0x04 ; Entry Mode Set (Increment, No Shift)
.equ	ID		= 1		  ; 	Increment/Decrement
.equ	SH		= 0		  ;		Shift

.equ	CTRL_LCD	= 0x08 ; Display control
.equ	LCD_ON	= 2		  ; 	bit de encendido/apagado del display
.equ	CUR_ON  = 1		  ;    	control del cursor
.equ	BLK_ON	= 0		  ; 	control de parpadeo

.equ	SHIFT_LCD	= 0x10 ; Cursor or Display Shift
.equ	SC		= 3
.equ	RL		= 2

.equ	CONF_LCD	= 0x20	; Function Set (DispLines=1, N=0, Font=1)
.equ	LINE		= 3		; 	Display Lines: 0/1 para 1/2 líneas.
.equ	BITS		= 4		;  	Bits de la interfaz 1/0= 8 bits/4 bits
.equ	FONT		= 2		;	    Font, 1: 5x8 dots.

.equ	BUSY_FLAG	= 7


;--------------------------------------------------------------------
;	Inicializo el LDC para interfaz de 4 bits.
;--------------------------------------------------------------------
LCD_INI:
			ldi		t1,200			; espero 40 mS
			rcall	DELAY_mS
			ldi		t1,200			; espero 40 mS
			rcall	DELAY_mS			

			;	CONFIG_DISPLAY				; 2 líneas, 4 bits y 5x8 dots		
			ldi		lcd_data,	CONF_LCD|(1<<LINE)|(0<<BITS)|(0<<FONT)
			rcall	WR_NIB_H_COMM
			ldi		t1,200
			rcall	delay_us
			ldi		t1,200
			rcall	delay_us
			ldi		lcd_data,	CONF_LCD|(1<<LINE)|(0<<BITS)|(0<<FONT)
			rcall	WR_NIB_H_COMM
			ldi		lcd_data,	CONF_LCD|(1<<LINE)|(0<<BITS)|(0<<FONT)
			rcall	WR_NIB_L_COMM
			ldi		t1,200
			rcall	delay_us
			ldi		t1,200
			rcall	delay_us

			;	DISPLAY ON/OFF
			ldi		lcd_data,	CTRL_LCD|(0<<LCD_ON)|(0<<CUR_ON)|(0<<BLK_ON)
			rcall	WR_NIB_H_COMM
			ldi		lcd_data,	CTRL_LCD|(0<<LCD_ON)|(0<<CUR_ON)|(0<<BLK_ON)
			rcall	WR_NIB_L_COMM
			ldi		t1,200
			rcall	delay_us
			ldi		t1,200
			rcall	delay_us

			;	CLEAR DISPLAY
			ldi		lcd_data,	CLR_LCD
			rcall	WR_NIB_H_COMM
			ldi		lcd_data,	CLR_LCD
			rcall	WR_NIB_L_COMM
			ldi		t1,40
			rcall	DELAY_ms

			;	ENTRY MODE SET
			ldi		lcd_data,	MODE_LCD|(1<<ID)|(0<<SH)
			rcall	WR_NIB_H_COMM
			ldi		lcd_data,	MODE_LCD|(1<<ID)|(0<<SH)
			rcall	WR_NIB_L_COMM
			ldi		t1,200
			rcall	delay_us
			ldi		t1,200
			rcall	delay_us

			; 	CONFIG_DISPLAY
			ldi		lcd_data,	CONF_LCD|(1<<LINE)|(0<<BITS)|(0<<FONT)
			rcall	WLCD_COMM

			ldi		lcd_data,	CLR_LCD
			rcall	WLCD_COMM

			;	DISPLAY=ON, CURSOR=OFF, BLINK=OFF
			ldi		lcd_data,	CTRL_LCD|(1<<LCD_ON)|(1<<CUR_ON)|(0<<BLK_ON)
			rcall	WLCD_COMM

			ret

;--------------------------------------------------------------------
;	WR_NIB_H_COMM: WRite NIBble High COMMand
;--------------------------------------------------------------------
WR_NIB_H_COMM:
			push	t0

			in		t0,	PORTD
			andi	t0,	$0F
			andi	lcd_data,	$F0

			or		lcd_data,	t0

			cbi		PORTC,	LCD_RS	; selecciono reg. de instrucciones
			nop
			cbi		PORTC,	LCD_RW	; selecciono escritura
			nop
			nop
			out		PORTD,	lcd_data	; pongo dato del nibble alto

			nop
			sbi		PORTC,	LCD_E		; pulso de habilitación
			nop
			cbi		PORTC,	LCD_E
			nop
			pop		t0
			ret

;--------------------------------------------------------------------
;	WR_NIB_L_COMM: WRite NIBble Low COMMand
;--------------------------------------------------------------------
WR_NIB_L_COMM:
			push	t0

			in		t0,	PORTD
			andi	t0,	$0F
			andi	lcd_data,	$0F
			swap	lcd_data

			or		lcd_data,	t0
			out		PORTD,	lcd_data	; pongo dato del nibble alto
	
			nop	
			sbi		PORTC,	LCD_E		; pulso de habilitación
			nop
			cbi		PORTC,	LCD_E
			nop

			nop
			sbi		PORTC,	LCD_RW	; selecciono escritura
			nop
			sbi		PORTC,	LCD_RS	; selecciono reg. de instrucciones
			nop

			pop		t0
			ret

;--------------------------------------------------------------------
;	WLCD_DATA:	Writes data to LCD
;--------------------------------------------------------------------
WLCD_DATA:
			sbi		PORTC,	LCD_RS	; selecciono reg. de datos
			rjmp	WR_LCD

;--------------------------------------------------------------------
;	WLCD_COMM:	Writes a command to LCD
;--------------------------------------------------------------------
WLCD_COMM:
			cbi		PORTC,	LCD_RS	; selecciono reg. de instrucciones

WR_LCD:
			push	t0
			push	lcd_data		; salvo el dato actual

			in		t0,	PORTD
			andi	t0,	$0F
			andi	lcd_data,	$F0
			or		lcd_data,	t0
			out		PORTD,	lcd_data	; pongo dato del nibble alto

			cbi		PORTC,	LCD_RW	; selecciono escritura
			sbi		PORTC,	LCD_E		; pulso de habilitación
			cbi		PORTC,	LCD_E

			pop		lcd_data		; recupero dato original
			swap	lcd_data
			in		t0,	PORTD
			andi	t0,	$0F
			andi	lcd_data,	$F0
			or		lcd_data,	t0
			out		PORTD,	lcd_data	; pongo dato del nibble bajo

			sbi		PORTC,	LCD_E		; pulso de habilitación
			cbi		PORTC,	LCD_E

			sbi		PORTC,	LCD_RW	; selecciono lectura.

ver_bf:
			cbi		PORTC,	LCD_RS	; selecciono reg. de instrucciones
			sbi		PORTC,	LCD_RW	; selecciono lectura

check_busy:
			nop
			nop

			sbi		PORTC,	LCD_E		; 1er pulso de habilitación
			in		t0,	DDRD
			andi	t0,	$0F
			out		DDRD,	t0			; configuro bus del LCD como entrada
			nop
			in		t0,	PIND			; leo busy flag
			bst		t0,	7			; T_flag<- busy flag
			cbi		PORTC,	LCD_E

			in		t0,	DDRD			; reestablezco config. bus del LCD
			ori		t0,$F0			; como salida.
			out		DDRD,t0
			nop
				
			sbi		PORTC,	LCD_E		; 2do pulso de habilitación
			cbi		PORTC,	LCD_E
		
			brts	check_busy

			pop		t0
			ret

;--------------------------------------------------------------------
;	SET_LCD_CUR
;--------------------------------------------------------------------
SET_LCD_CUR:
			mov		lcd_data,	cur_pos
			ori		lcd_data,	$80
			cpi		cur_pos,	$10
			brlo	first_line
			ori		lcd_data,	$40
			subi	lcd_data,	$10
first_line:
			mov		t4,	lcd_data
			rcall	WLCD_COMM
			mov		t0,	cur_pos
			andi	t0,	0x0F
			;	Erase from cur_pos to the end of the line
set_cur_loop:
			ldi		lcd_data,	0x20		
			rcall	WLCD_DATA
			inc		t0
			cpi		t0,	0x10
			brne	set_cur_loop
			mov		lcd_data,	t4
			rcall	WLCD_COMM
			ret

;--------------------------------------------------------------------
;	SHOW_HEX
;--------------------------------------------------------------------
SHOW_HEX:
			ldi		t4,	2
			mov		lcd_data,	datL
shift_hex:			
			lsr		lcd_data
			lsr		lcd_data
			lsr		lcd_data
			lsr		lcd_data
no_shift_hex:
			andi	lcd_data,	$0F
			cpi		lcd_data,	$A
			brlo	lower_hex
			ldi		t1,	7
			add		lcd_data,	t1
lower_hex:
			ldi		t1,	$30
			add		lcd_data,	t1
			rcall	WLCD_DATA
			dec		t4
			breq	end_show_hex
			mov		lcd_data,	datL
			rjmp	no_shift_hex
end_show_hex:
			ret					

;-------------------------------------------------------------------------
; recibe t0 y devuelve el ascii en r1|r0
;-------------------------------------------------------------------------
Hex2Ascii:
			mov		r0,	t0
			andi	t0, 0xF0
			swap	t0
			cpi		t0, 0x0A
			brlo	sumo_30h
			subi	t0,	-0x07
sumo_30h:
			subi	t0,	-0x30
			mov		r1, t0
			mov		t0, r0
			andi	t0, 0x0F
			cpi		t0, 0x0A
			brlo	sumo_30h_bajo
			subi	t0,	-0x07		; sino 0x30+0x07
sumo_30h_bajo:
			subi	t0, -0x30		; if r1<=9, sumo 30
			mov		r0, t0
			ret	

;--------------------------------------------------------------------
;	DISP_MSG: Lee de flash lo apuntado por Z y lo muestra en el display.
;	Termina cuando el dato leído es 0x00 (fin de string) 
;--------------------------------------------------------------------
Muestro_Char:
			rcall	WLCD_DATA


DISP_MSG:
			lpm		lcd_data,	Z+
			tst		lcd_data
			brne	Muestro_Char
			ret

;--------------------------------------------------------------------
;	DELAY_uS:	Time delay = t1*3+1 uS (min=1.875 uS)
;--------------------------------------------------------------------
DELAY_uS:
		dec		t1
		brne	DELAY_uS
		ret

;--------------------------------------------------------------------
;	DELAY_mS:	Time delay = t1 mS (min=1 mS)
;--------------------------------------------------------------------
DELAY_mS:
		mov		t0,t1
LOOP_mS:		
		ldi		t1,100
		rcall	DELAY_uS
		ldi		t1,100
		rcall	DELAY_uS
		ldi		t1,123
		rcall	DELAY_uS
		dec		t0
		brne	LOOP_mS
		ret

;--------------------------------------------------------------------
;	fin del módulo display.asm
;--------------------------------------------------------------------
