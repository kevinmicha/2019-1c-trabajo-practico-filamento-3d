;-------------------------------------------------------------------------
; AVR - Configuraci�n y transmisi�n por puerto serie
; Tiene las rutinas 
;	RESET: programa principal para testear las rutinas
;	USART_init: configura el puerto serie para tx/rx por interrupciones
;	ISR_RX_USART_COMPLETA: rutina de servicio de interrupci�n de recepci�n
;	ISR_REG_USART_VACIO: rutina de serv. de int. de transmisi�n
;	TEST_TX: rutina de transmisi�n de prueba
;	TX_MSJ: rutina de transmisi�n gen�rica
;
; Ultima actualizaci�n: 2019-MAY-05 09:47
;-------------------------------------------------------------------------

;-------------------------------------------------------------------------
; MCU: ATmega8/ATmega328P 
; Seg�n el entorno de desarrollo, el MCU (MicroController Unit) se elige
; a) desde Project->Properties se elige el microcontrolador
; b) con una directiva de include.
; En ambos casos se termina incluyendo un archivo m8def.inc/m328Pdef.inc
; que definen un s�mbolo _M8DEF_INC_/_M328PDEF_INC_ que se usa en toda
; este c�digo para distinguir entre registros de uno y otro micro.
;-------------------------------------------------------------------------
; Si el modo de seleccionar el MCU es el b) descomentar el include que 
; corresponda:
;.include m8def.inc		; define los registros y constantes del Mega8
.include "m328Pdef.inc"	; define los registros y constantes del Mega328P
;-------------------------------------------------------------------------

#define	F_CPU	16000000	; frecuencia del reloj del micro (F_SYS)

;-------------------------------------------------------------------------
; Puerto donde se conecta un LED
;-------------------------------------------------------------------------
; En una placa Arduino uno el led est� en PINB.5 y se enciende con '1'
; para otros circuitos verificar la ubicaci�n de alg�n led y definir lo 
; siguiente:
.equ	PORT_LED	= PORTB	; registro del puerto
.equ	DIR_LED		= DDRB	; registro de direcciones del puerto
.equ	LED			= 5		; nro. de bit (contando de 0 a 7=MSbit)
							; nota: En 1/0 prende/apaga LED
.equ	DIR_ADC		= DDRD
.equ	ADC_DT		= 3
.equ	ADC_SCK		= 4
.equ	PORT_ADC    = PORTD
.equ	PIN_ADC		= PIND
;-------------------------------------------------------------------------
; MACROS
;-------------------------------------------------------------------------
; Inclusi�n de macros.  El *.inc debe estar en la misma carpeta que los 
; dem�s archivos fuente o bien incluir un path al mismo mediante:
;	Project->Properties->Toolchain->AVR Assembler->General->Include Paths
.include "avr_macros.inc"	
.listmac					; permite ver las macros en el listado *.lss

;-------------------------------------------------------------------------
; CONSTANTES
;-------------------------------------------------------------------------
.equ	BUF_SIZE	= 64	; tama�o en bytes del buffer de transmisi�n
.equ	LF	= 13			; '\n' caracter ascii de incremento de l�nea
.equ	CR  = 10			; '\r' caracter ascii de retorno de carro
.equ	ciclos = 61
.equ	OV_PARA_1S = 63
;-------------------------------------------------------------------------
; variables en SRAM
;-------------------------------------------------------------------------
		.dseg				; Segmento de datos (RAM)

ADC_B: .byte 3
TX_BUF:	.byte	BUF_SIZE	; buffer de transmisi�n serie
RX_BUF: .byte	BUF_SIZE	; buffer de recepci�n de datos por puerto serie
VERSION:.byte	1			; nro. de versi�n del programa


;-------------------------------------------------------------------------
; variables en registros
;-------------------------------------------------------------------------
.def	ptr_tx_L = r8		; puntero al buffer de datos a transmitir
.def	ptr_tx_H = r9
.def	bytes_a_tx = r14 	; nro. de bytes a transmitir desde el buffer
.def	bytes_recibidos = r13

.def	t0	= r16			; variable global auxiliar
.def	t1	= r17			; variable global auxiliar
.def	veces_ov_0 = r19
.def	contador2 = r21

.def	eventos = r20
.equ	EVENTO_RX_SERIE = 0
.equ	EVENTO_ADC_FIN = 1
.equ	EVENTO_1SEG = 2

;-------------------------------------------------------------------------
; CODIGO
;-------------------------------------------------------------------------
			.cseg
		rjmp	RESET			; interrupci�n del reset
		
		.org	OVF0addr
		rjmp	ISR_TOV0

		.org	INT1addr		; Dt bajo, aviso a main que tiene que leer dato
		rjmp	ISR_MUESTRA_ADC	
			
		.org	URXCaddr		; USART, Rx Complete
		rjmp	ISR_RX_USART_COMPLETA
	
		.org	UDREaddr		; USART Data Register Empty
		rjmp	ISR_REG_USART_VACIO

		.org 	INT_VECTORS_SIZE ; salteo todos los vectores de interrupci�n
RESET:	

		ldi		r16,LOW(RAMEND)
		out 	spl,r16
		ldi 	r16,HIGH(RAMEND)
		out 	sph,r16			; inicializaci�n del puntero a la pila

		sbi		DIR_LED, LED	; configuro como salida el puerto para manejar el LED
		sbi		PORT_LED, LED	; PRENDO el LED

		CBI		DIR_ADC,	ADC_DT	; inicializo pin 3 de puerto D como entrada  D.3=ADC_DT
		SBI		DIR_ADC,	ADC_SCK	; inicializo pin 4 del puerto D como salida  D.4=ADC_SCK

		CBI		PORT_ADC,	ADC_SCK	;adc en bajo consumo
		SBI		PORT_ADC,	ADC_DT	;dt es una entrada con pull up interno

		ldi		t0, 0x13
		sts		VERSION, t0		; versi�n actual de este m�dulo

		rcall	USART_init		; Configuro el puerto serie para tx/rx a 19.2 kbps
								; y habilito la interrupci�n de recepci�n de datos.

		;lds		R16,	EICRA												;configuro int 0 por flanco ascendente
		;ori		R16,	(1<<ISC01)|(1<<ISC00)                               ;NO SE USA
		;sts		EICRA,	R16

		lds		R16,	EICRA												;configuro int 1 por flanco descendente
		ori		R16,	(1<<ISC11)
		andi	R16,	~(1<<ISC10)
		sts		EICRA,	R16

;habilito interrupciones

		in		R16,	EIMSK
		ori		R16,	(1<<INT1)
		out		EIMSK,	R16

		rcall	INICIALIZAR_TIMER0			;TEMPORIZADOR GENERAL (marca los instantes de transmision por puerto serie)

		rcall	INICIALIZAR_TIMER2			;generador de cuadrada para monoestable
	
		ldi		ZH, high(ADC_B)
		ldi		zl,	low(ADC_B)
		ldi		r16,	8
		st		z+,	r16
		st		z+,	r16
		st		z+,	r16
		sei						; habilitaci�n global de todas las interrupciones

		rcall	TEST_TX			; transmite un mensaje de prueba

		

MAIN:							; Programa principal (bucle infinito)
		;rcall	MONITOREAR_TOV0
		tst		eventos			; Pas� algo?
		breq	MAIN			;	nada

		sbrc	eventos, EVENTO_ADC_FIN
		rjmp	EVENTO1

		sbrc	eventos, EVENTO_1SEG
		rjmp	EVENTO2

		clr		eventos
		rjmp	MAIN

EVENTO1:
		CBR		eventos,	(1<<EVENTO_ADC_FIN)
		rcall	LEER_BITS
		rjmp	main

EVENTO2:
		CBR		eventos,	(1<<EVENTO_1SEG)
		rcall	LEER_ADC
	
		rjmp	main


PROCESO_TRAMA_RX:
		// Los datos recibidos x puerto serie est�n a partir del direcci�n RAM RX_BUF
		// y terminan siempre con el caracter LF.   Adem�s "bytes_recibidos" me dice
		// cu�ntos bytes tiene la trama.
		cbr		eventos,(1<<EVENTO_RX_SERIE)	; Limpio flag del evento
		clr		bytes_recibidos					; limpio nro. de bytes recibidos porque no lo uso

		lds		t0, RX_BUF		; miro el 1er caracter de la trama recibida
		cpi		t0, '1'
		brne	VER_SI_ES_CERO

		sbi		PORT_LED, LED	; Si recibi� un '1', prende el LED
		rjmp	MAIN

LEER_ADC:
		; Leen ADC y guardan el valor le�do en ADC_B, ADC_B+1 y ADC_B+2
		ldiw  Z, (MSJ_DATO<<1)
		rcall TX_MSJ
		ret

VER_SI_ES_CERO:
		cpi		t0, '0'
		brne	MAIN

		cbi		PORT_LED, LED	; Si recibe un '0', apaga el LED
		rjmp	MAIN


;-------------------------------------------------------------------------
;					COMUNICACION SERIE
;-------------------------------------------------------------------------
#if	F_CPU == 8000000
.equ	BAUD_RATE	= 51	; 19.2 kbps e=0.2% 	@8MHz y U2X=1
#elif F_CPU == 16000000
.equ	BAUD_RATE	= 103	; 19.2 kbps  e=0.2% @16MHz y U2X=1
#else
.error "Falta calcular el baud rate para el F_CPU definido!"
#endif

;-------------------------------------------------------------------------
USART_init:
		push	t0
		push	t1
		pushw	X

#ifdef  _M328PDEF_INC_
		outi	UBRR0H, high(BAUD_RATE)
		outi	UBRR0L,low(BAUD_RATE)
		outi	UCSR0A, (1<<U2X0)
		; Trama: 8 bits de datos, sin paridad y 1 bit de stop, 
		outi 	UCSR0C,(0<<UPM01)|(0<<UPM00)|(0<<USBS0)|(0<<UCSZ02)|(1<<UCSZ01)|(1<<UCSZ00)
		; Configura los terminales de TX y RX; y habilita
		; 	�nicamente la int. de recepci�n
		outi	UCSR0B,(1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0)|(0<<UDRIE0)
#else
		outi	UBRRH,high(BAUD_RATE)	; Velocidad de transmisi�n
		outi	UBRRL,low(BAUD_RATE)
		outi	UCSRA,(1<<U2X)			; Modo asinc., doble velocidad
		outi 	UCSRC,(1<<URSEL)|(0<<UPM1)|(0<<UPM0)|(0<<USBS)|(1<<UCSZ1)|(1<<UCSZ0)
		outi	UCSRB,(1<<RXCIE)|(1<<RXEN)|(1<<TXEN)|(0<<UDRIE)
#endif
		movi	ptr_tx_L,LOW(TX_BUF)	; inicializa puntero al 
		movi	ptr_tx_H,HIGH(TX_BUF)	; buffer de transmisi�n.
	
		ldiw	X,TX_BUF				; limpia BUF_SIZE posiciones 
		ldi		t1, BUF_SIZE			; del buffer de transmisi�n
		clr		t0
loop_limpia:
		st		X+,t0
		dec		t1
		brne	loop_limpia
					
		clr		bytes_a_tx		; nada pendiente de transmisi�n
		clr		bytes_recibidos	; nada recibido a�n.

		popw	X
		pop		t1
		pop		t0
		ret


;-------------------------------------------------------------------------
; RECEPCION: Interrumpe cada vez que se recibe un byte x RS232.
;
; Recibe:	UDR (byte de dato)
; Devuelve: nada
;-------------------------------------------------------------------------
ISR_RX_USART_COMPLETA:
; EL registro UDR tiene un dato y deber�a ser procesado
		push	t0
		pushi	SREG
		pushw	Y

		ldiw	Y, RX_BUF
		add		YL, bytes_recibidos
		clr		t0
		adc		YH, t0

#ifdef  _M328PDEF_INC_
		input	t0,	UDR0
		;output	UDR0, t0
#else
		input	t0, UDR
#endif
		st		Y, t0
		inc		bytes_recibidos
		ldi		t0, BUF_SIZE
		cp		bytes_recibidos, t0
		brlo	BUF_RX_CON_ESPACIO

		clr		bytes_recibidos		; error, se sobrepas� el espacio disponible 
									; para mensajes recibidos x puerto serie.
									; Deber�a informar al main (pero no lo hago).
BUF_RX_CON_ESPACIO:		
		ld		t0, Y
ACA:
		cpi		t0, LF
		brne	FIN_ISR_RX_USART

		ldi		t0,(1<<EVENTO_RX_SERIE)
		or		eventos, t0

FIN_ISR_RX_USART:
		popw	Y
		popi	SREG
		pop		t0
    	reti 

;------------------------------------------------------------------------
; TRANSMISION: interrumpe cada vez que puede transmitir un byte.
; Se transmiten "bytes_a_tx" comenzando desde la posici�n TX_BUF del
; buffer. Si "bytes_a_tx" llega a cero, se deshabilita la interrupci�n.
;
; Recibe: 	bytes_a_tx.
; Devuelve: ptr_tx_H:ptr_tx_L, y bytes_a_tx.
;------------------------------------------------------------------------
ISR_REG_USART_VACIO:		; UDR est� vac�o
		push	t0
		push	t1
		pushi	SREG
		pushw	X


		tst		bytes_a_tx	; hay datos pendientes de transmisi�n?
		breq	FIN_TRANSMISION

		movw	XL,ptr_tx_L	; Recupera puntero al pr�ximo byte a tx.
		ld		t0,X+		; lee byte del buffer y apunta al

#ifdef  _M328PDEF_INC_
		output	UDR0, t0
#else
		output	UDR, t0		; sgte. dato a transmitir (en la pr�xima int.)
#endif

		cpi		XL,LOW(TX_BUF+BUF_SIZE)
		brlo	SALVA_PTR_TX
		cpi		XH,HIGH(TX_BUF+BUF_SIZE)
		brlo	SALVA_PTR_TX
		ldiw	X,TX_BUF	; ptr_tx=ptr_tx+1, (m�dulo BUF_SIZE)

SALVA_PTR_TX:
		movw	ptr_tx_L,XL	; preserva puntero a sgte. dato

		dec		bytes_a_tx	; Descuenta el nro. de bytes a tx. en 1
		brne	SIGUE_TX	; si quedan datos que transmitir
							;	vuelve en la pr�xima int.

FIN_TRANSMISION:			; si no hay nada que enviar,
#ifdef  _M328PDEF_INC_
		lds		t0, UCSR0B
		andi	t0, ~(1<<UDRIE0)
		sts		UCSR0B, t0
		;cbix		UCSR0B, UDRIE0
#else
		cbix	UCSRB,	UDRIE	; 	se deshabilita la interrupci�n.
#endif

sigue_tx:
		popw	X
		popi	SREG
		pop		t1
		pop		t0
		reti

;-------------------------------------------------------------------------
; TEST_TX: transmite el mensaje almacenado en memoria flash a partir
; de la direcci�n MSJ_TEST_TX que termina con 0x00 (el 0 no se transmite).
; Recibe: nada
; Devuelve: nada
;-------------------------------------------------------------------------
TEST_TX:
		pushw	Z
		push	t0

		ldiw	Z,(MSJ_TEST_TX*2)
		rcall	TX_MSJ

		pop		t0
		popw	Z
		ret
MSJ_DATO:
.db "ADC= 0x%", low(ADC_B), high(ADC_B), " %", low(ADC_B+1), high(ADC_B+1)
.db " %", low(ADC_B+2), high(ADC_B+2), '\n','\r',0,0

MSJ_TEST_TX:
;.db	"Puerto Serie Version 0.3 ",'\r','\n',0
.db		"Filamento proyecto %"
.dw		VERSION		; direcci�n RAM de la variable (byte hexa) a tx
.db		'\r','\n',0,0	; el 2do cero completa un nro. par de bytes

;-------------------------------------------------------------------------
; TX_MSJ: transmite el mensaje almacenado en memoria flash a partir
; de la direcci�n que se pase en el puntero Z.   El mensaje debe termina 
; con 0x00 (el 0 no se transmite).
; 
; Recibe: Z (=r31|r30)
; Devuelve: bytes_a_tx > 0
; Habilita la int. de transmisi�n serie con ISR en ISR_REG_USART_VACIO().
;-------------------------------------------------------------------------
TX_MSJ:
			push	t0
			pushi	SREG
			pushw	X

			movw	XL, ptr_tx_L	; toma el �ltimo valor del puntero

COPIA_A_TX_BUF:
			lpm		t0,Z+			; y copia de flash a ram
			tst		t0				; si encuentra un 0x00 en el mensaje, termina
			breq	ACTIVA_TX_MSJ	;	de cargar el buffer en RAM e incia la transmisi�n.

; Si el carcter es '%' seguido de un nro. hexadecimal de 16 bits, toma el byte
; de esa direcci�n de RAM, lo convierte a ASCII y lo pone en el buffer de transmisi�n.
			cpi		t0, '%'
			brne	NO_HAY_VARIABLES

			pushw	Y
			lpm		YL, Z+
			lpm		YH, Z+
			ld		t0, Y
			rcall	BYTE_2_ASCII	; devuelve en r1|r0 el ascii del byte
			popw	Y

			st		X+, r1
			inc		bytes_a_tx

			mov		t0, r0
			cpi		XL, low(TX_BUF+BUF_SIZE)
			brlo	NO_HAY_VARIABLES
			cpi		XH, high(TX_BUF+BUF_SIZE)
			brlo	NO_HAY_VARIABLES
			ldiw	X, TX_BUF		; ptr_tx++ m�dulo BUF_SIZE

NO_HAY_VARIABLES:
			st		X+,t0
			inc		bytes_a_tx

			cpi		XL, low(TX_BUF+BUF_SIZE)
			brlo	COPIA_A_TX_BUF
			cpi		XH, high(TX_BUF+BUF_SIZE)
			brlo	COPIA_A_TX_BUF
			ldiw	X, TX_BUF		; ptr_tx++ m�dulo BUF_SIZE

			rjmp	COPIA_A_TX_BUF
	
ACTIVA_TX_MSJ:						; habilita la int. de tx
#ifdef  _M328PDEF_INC_
			sbix	UCSR0B, UDRIE0
#else
			sbix	UCSRB,	UDRIE
#endif

			popw	X
			popi	SREG
			pop		t0
			ret

;-------------------------------------------------------------------------
;  Recibe t0 y devuelve el ascii en r1|r0
;  Por ejemplo, si t0=0xA3 devuelve r1='A'=0x41 r0='3'=0x33
;-------------------------------------------------------------------------
BYTE_2_ASCII:
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

ISR_MUESTRA_ADC:
		PUSH	R16
		PUSH	R17
		IN		R16,	SREG
		PUSH	R16


		IN		R16,	PIN_ADC
		LDI		R17,	5
DT_BAJO:
		SBRC	R16,	ADC_DT
		RJMP	FIN_ISR

		DEC		R17
		BRNE	DT_BAJO
		SBR		eventos,	(1<<EVENTO_ADC_FIN)

		in		R16,	EIMSK
		andi		R16,	~(1<<INT1)
		out		EIMSK,	R16

FIN_ISR:
		POP		R16
		OUT		SREG,	R16	
		POP	R17
		POP	R16
		RETI

LEER_BITS:		

		push	r16
		
		IN		R16,		SREG
		PUSH	R16

		PUSH	R12
		PUSH	R11
		PUSH	R10

		push contador2

		PUSH	YH
		PUSH	YL	
																;leo bits del puerto D bit 3 porque es donde esta conectado DT del AD

		;debag

		SBI		DDRC,  0

		LDI		contador2,	24											;hay que mandar 24 pulsos para sacar 24 bits. DT NO VUELVE A 1




LOOP:
		SBI		PORT_ADC,	ADC_SCK											;mando 1 al sck

		LDI		R16,		16											;espero 16 ciclos
ESPERO_1MICRO:
		DEC		R16
		BRNE	ESPERO_1MICRO

		CBI		PORT_ADC,	ADC_SCK											;mando 0 al sck

		CLC
		SBIC	PIN_ADC,		ADC_DT											;leo info del DT
		SEC

		BRCC	PORTC_0
		CBI		PORTC, 0
		RJMP	ROLEO

PORTC_0:
		SBI		PORTC,	0

ROLEO:
		ROL		R10
		ROL		R11
		ROL		R12

		LDI		R16,		8											;espero 8 ciclos
ESPERO_EN_BAJO_SCK:
		DEC		R16
		BRNE	ESPERO_EN_BAJO_SCK

		DEC		contador2
		BRNE	LOOP

		;LDI		YH,			HIGH(ADC_B)									;inicializo puntero
		;LDI		YL,			LOW(ADC_B)

		;MOV		R16,		R12
		;CPI		R16,		0XFF
		;BREQ	FIN

		STS		ADC_B,			R12
		STS		ADC_B+1,			R11
		STS		ADC_B+2,			R10
FIN:

		;mando 2 pulsos de clk pidiendo la siguiente conversion del canal b

		LDI		contador2,	2

SIGUIENTE_CONVERSION:
		SBI		PORT_ADC,	ADC_SCK											;mando 1 al sck

		LDI		R16,		16											;espero 16 ciclos
ESPERO_CLK_ALTO_SC:
		DEC		R16
		BRNE	ESPERO_CLK_ALTO_SC

		CBI		PORT_ADC,	ADC_SCK											;mando 0 al sck

		LDI		R16,		16											;espero 16 ciclos
ESPERO_EN_BAJO_SC:
		DEC		R16
		BRNE	ESPERO_EN_BAJO_SC

		DEC		contador2
		BRNE	SIGUIENTE_CONVERSION

		in		R16,	EIFR
		ori		R16,	(1<<INTF1)
		out		EIFR,	R16

		in		R16,	EIMSK
		ori		R16,	(1<<INT1)
		out		EIMSK,	R16

		POP		YL
		POP		YH
		POP		contador2
		POP		R10
		POP		R11
		POP		R12
		POP     R16

		OUT		SREG,		R16
		POP		R16

		
		RET



INICIALIZAR_TIMER0:
;corre a 16 M/1024 = 15625 Hz (periodo = 64 micros) 


	push	r24

	ldi		veces_ov_0, OV_PARA_1S	;overflow cada 16 ms (para 1 s necesito 63 overflows)

	CLR		R24															;inicializo en 0
	OUT		TCNT0,		R24
;configuro timer con prescaler de 1024 y modo normal
	;IN		R24,		TCCR0A
	;ANDI	R24,		~((1<<WGM00) | (1<<WGM01))
	;OUT		TCCR0A,		R24

	IN		R24,		TCCR0B
	ORI		R24,		(1<<CS02) | (1<<CS00)
	ANDI	R24,		~((1<<CS01))
	OUT		TCCR0B,		R24

	;LDS		R24,		TIMSK0
	input	r24,	TIMSK0
	ORI		R24,		(1<<TOIE0)
	output		TIMSK0,		R24

	pop		r24
	
	RET



ISR_TOV0:															;para monitorear flag TOV0
	PUSH	R16
	IN		R16,		SREG
	PUSH	R16
	PUSH	R17

	;LDI		R17,	(1<<LED)
	;IN		R16,	PORT_LED
	;EOR		R16,	r17
	;OUT		PORT_LED,	R16


	DEC		veces_ov_0
	BRNE	FIN_ISR_TOV0

	;paso un seg
	LDI		veces_ov_0,	OV_PARA_1S
	SBR		eventos,	(1<<EVENTO_1SEG)

FIN_ISR_TOV0:	

	POP		R17
	POP		R16
	OUT		SREG,	R16
	POP		R16

	RETI

;timer para generar cuadrada

INICIALIZAR_TIMER2:
	SBI		DDRB,		3
	LDI		R16,		79
	STS		OCR2A,		R16

	CLR		R25															;inicializo en 0
	STS		TCNT2,		R25
;configuro timer sin prescaler y modo normal
	LDS		R25,		TCCR2A
	ANDI	R25,		~(1<<WGM20)
	ORI		R25,		(1<<WGM21)
	STS		TCCR2A,		R25

	LDS		R25,		TCCR2B
	ORI		R25,		(1<<CS20)
	ANDI	R25,		~(1<<CS21)
	ANDI	R25,		~(1<<CS22)
	STS		TCCR2B,		R25

	LDS		R25,		TCCR2A
	ANDI	R25,		~(1<<COM2A1)
	ORI		R25,		(1<<COM2A0)
	STS		TCCR2A,		R25

	RET
	
;-------------------------------------------------------------------------
; fin del c�digo
;-------------------------------------------------------------------------;