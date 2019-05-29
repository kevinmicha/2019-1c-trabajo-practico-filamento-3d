;--------------------------------------------------------------------
; Manejo de display LCD de 16x2 caracteres tipo HD44780
; MCU: ATmega8L @2MHz oscilador interno
;--------------------------------------------------------------------
 
;--------------------------------------------------------------------
;	Constantes globales
;--------------------------------------------------------------------
.equ	ENT	=	0
.equ	SPU	=	0	; sin pull-up
.equ	CPU	=	1	; con pull-up

.equ	SAL	=	1
.equ	HIG	=	1	; estado lógico alto
.equ	BAJO=	0	; estado lógico bajo

.equ	UNO	=	1
.equ	CERO=	0

;--------------------------------------------------------------------
;	Macros
;--------------------------------------------------------------------
.include "avr_macros.inc"

;--------------------------------------------------------------------
; Conexiones del MCU con el LCD:
;--------------------------------------------------------------------
;	Port C
.equ	LCD_RS	=0	; selector de registros (1=data/0=command)
.equ	LCD_RW	=1	; read/~write
.equ	LCD_E	=2	; enable rd/wr en el flanco de bajada.
.equ	LCD_BKL	=3	; backlight (0=on/1=off)

.equ	PORTC_DIR	=(SAL<<LCD_BKL)|(SAL<<LCD_E)|(SAL<<LCD_RW)|(SAL<<LCD_RS)
.equ	PORTC_INI	=(HIG<<LCD_BKL)|(BAJO<<LCD_E)|(BAJO<<LCD_RW)|(BAJO<<LCD_RS)


;	Port D
.equ	LCD_DB4		=4	; bus de datos (de 4 bits) del LCD
.equ	LCD_DB5		=5
.equ	LCD_DB6		=6
.equ	LCD_DB7		=7

;--------------------------------------------------------------------
;	Variables en registros
;--------------------------------------------------------------------
.def	t0			=r16
.def	t1			=r17
.def	t2			=r18
.def	t3			=r19
.def	t4			=r20
.def	cur_pos		=r21	; posición del cursor en la pantalla
.def	lcd_data	=r22
.def	datL		=r23

;--------------------------------------------------------------------
;	Programa principal
;--------------------------------------------------------------------
			.cseg
			.org	0x0000
			rjmp	RESET


			.org	INT_VECTORS_SIZE	; salteo todos los vectores de interrupción
RESET:
			ldi		t0, low(RAMEND)
			out		SPL, t0
			ldi		t0, high(RAMEND)
			out		SPH, t0				; inicializo pila

			rcall	PORTS_INI			; inicializo puertos
			rcall	LCD_INI				; configuro display
			ldi		ZL, low(MENSAJE*2)	; puntero a mensaje en flash
			ldi		ZH, high(MENSAJE*2)
			rcall	DISP_MSG			; muestra mensaje en pantalla

A_DORMIR:	
			ori		t0, (1<<SE)
			out		MCUCR, t0			; habilita la entrada en modo sueño
			sleep						; duerme una siesta hasta la próxima interrupción
			rjmp	A_DORMIR


;--------------------------------------------------------------------
;	Configuración e inicialización del los puertos de E/S
;--------------------------------------------------------------------
PORTS_INI:
			outi	DDRC,(SAL<<LCD_BKL)|(SAL<<LCD_E)|(SAL<<LCD_RW)|(SAL<<LCD_RS)
			outi	PORTC,(BAJO<<LCD_BKL)|(BAJO<<LCD_E)|(BAJO<<LCD_RW)|(BAJO<<LCD_RS)

			outi	DDRD,(SAL<<LCD_DB4)|(SAL<<LCD_DB5)|(SAL<<LCD_DB6)|(SAL<<LCD_DB7)
			outi	PORTD,(HIG<<LCD_DB4)|(HIG<<LCD_DB5)|(HIG<<LCD_DB6)|(HIG<<LCD_DB7)
			ret

;--------------------------------------------------------------------
;	Módulo con las rutinas para manejo del display
;--------------------------------------------------------------------		
.include "display.asm"

;--------------------------------------------------------------------
; Mensaje de prueba a escribir en el display: es una cadena de 
; caracteres imprimibles terminada en '\0'
;--------------------------------------------------------------------
MENSAJE:	.db		"Hola ",0
