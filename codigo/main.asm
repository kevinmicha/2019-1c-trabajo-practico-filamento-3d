
.include "m328Pdef.inc"

;--------------------------------------------------------------
;Timer
;--------------------------------------------------------------
; Se implementa un timer de 1 seg teniendo en cuenta que la frecuencia del oscilador es de 16 MHz

.cseg

.def    eventos =	R18
.def	contador =	R17
.equ	ciclos	=	61

.org	0x00															;salteo vector de interrupciones
RJMP	main

.org	INT_VECTORS_SIZE

main:
	CLR		R16															;inicializo en 0
	OUT		TCNT0,		R16
;configuro timer con prescaler de 1024 y modo normal
	IN		R16,		TCCR0A
	ANDI	R16,		~((1<<WGM00) | (1<<WGM01))
	OUT		TCCR0A,		R16

	IN		R16,		TCCR0B
	ORI		R16,		(1<<CS02) | (1<<CS00)
	OUT		TCCR0b,		R16

LOOP:																	;para monitorear flag TOV0
	IN		R16,		TIFR0
	SBRC	R16,		TOV0
	RJMP	NUEVO_CICLO
	RJMP	LOOP

NUEVO_CICLO:
	SBI		TIFR0,		TOV0											;cuando llega a 255 empiezo de nuevo 61 veces
	DEC		contador
	BRNE	LOOP
	
	LDI		R16,		185												;inicializo para contar los ciclos que faltan
	OUT		TCNT0,		R16

LOOP2:																	;para monitorear flag TOV0
	IN		R16,		TIFR0
	SBRS	R16,		TOV0
	RJMP	LOOP2

	LDI		EVENTOS,	1												;cambio valor de registro