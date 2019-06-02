.include "m328Pdef.inc"

;--------------------------------------------------------------
;Lectura de datos del ADC
;--------------------------------------------------------------
; Cuando los datos estan listos para ser leidos se habilita la interrupcion 1 para guardarlos en RAM

.dseg
ADC_B: .byte 3		;reservo 3 bytes de memoria para poder guardar los 24 bits

.cseg

.def	contador = R17

.org	0x00															;salteo vector de interrupciones
RJMP	start

.org	INT1addr														;cuando sucede la int el SP apunta a INT0addr (primera posicion del vect de int)
RJMP	LEER_BITS														;entonces ahi pongo que vaya a la ISR

.org	INT_VECTORS_SIZE				

start:
;configuro interrupcion para que sea por flanco descendente

		LDI		XH,		HIGH(EICRA)										;inicializo puntero
		LDI		XL,		LOW(EICRA)

		LD		R16,	X												;configuro int 1 por flanco ascentente
		ORI		R16,	(1<<ISC11)
		ANDI	R16,	~(1<<ISC10)
		ST		X,		R16

;habilito interrupciones

		IN		R16,	EIMSK
		ORI		R16,	(1<<INT1)
		OUT		EIMSK,	R16

		SEI

MAIN:
	RJMP MAIN															;no hace nada mientras no haya interrupcion

LEER_BITS:																;leo bits del puerto D bit 3 porque es donde esta conectado DT del AD
		CBI		DDRD,		3											;inicializo pin 3 de puerto D como entrada
		SBI		DDRD,		2											;inicializo pin 4 del puerto D como salida CONECTAR SCK A ESTE PIN
		LDI		contador,	26											;hay que mandar 26 pulsos
		CLR		R18

		LDI		ZH,			HIGH(ADC_B)									;inicializo puntero
		LDI		ZL,			LOW(ADC_B)

LOOP:
		SBI		PORTD,		2											;mando 1 al sck

		SER		R16														;espero 2 ciclos
		CLR		R16

		SBIS	PIND,		3											;leo info del DT
		RCALL	GUARDAR_CERO

		SBIC	PIND,		3
		RCALL	GUARDAR_UNO

		CBI		PORTD,		2											;mando 0 al sck

		LDI		R16,		11											;espero 11 ciclos
DE_NUEVO:
		DEC		R16
		BRNE	DE_NUEVO

		INC		R18														;guarde un bit entonces queda un lugar menos
;hasta aca teengo 1 ciclo en sck

		DEC		contador
		BRNE	LOOP

		RETI
;hasta aca tengo 26 ciclos

GUARDAR_CERO:
		CPI		R18,		7											;veo si tengo lugar en la misma palabra
		BRNE	NO_INC

		ADIW	ZH:ZL,		1											;paso al siguiente byte
		CLR		R18

NO_INC:
		LD		R16,		Z											;leo lo que ya estaba guardado en la direcc a la que apunta Z

		CPI		R18,		0											;veo si tengo lugar en la misma palabra
		BREQ	PRIMER_BIT

		
		CPI		R18,		1											;veo si tengo lugar en la misma palabra
		BREQ	SEGUNDO_BIT

		
		CPI		R18,		2											;veo si tengo lugar en la misma palabra
		BREQ	TERCER_BIT

		
		CPI		R18,		3											;veo si tengo lugar en la misma palabra
		BREQ	CUARTO_BIT

		
		CPI		R18,		4											;veo si tengo lugar en la misma palabra
		BREQ	QUINTO_BIT

		CPI		R18,		5											;veo si tengo lugar en la misma palabra
		BREQ	SEXTO_BIT

		CPI		R18,		6											;veo si tengo lugar en la misma palabra
		BREQ	SEPTIMO_BIT

		CPI		R18,		7											;veo si tengo lugar en la misma palabra
		BREQ	OCTAVO_BIT

PRIMER_BIT:
		ANDI	R16,		~(1<<0)
		ST		Z,			R16											;guardo nuevo valor
		RJMP	SALIDA

SEGUNDO_BIT:
		ANDI	R16,		~(1<<1)
		ST		Z,			R16											;guardo nuevo valor
		RJMP	SALIDA

TERCER_BIT:
		ANDI	R16,		~(1<<2)
		ST		Z,			R16											;guardo nuevo valor
		RJMP	SALIDA

CUARTO_BIT:
		ANDI	R16,		~(1<<3)
		ST		Z,			R16											;guardo nuevo valor
		RJMP	SALIDA

QUINTO_BIT:
		ANDI	R16,		~(1<<4)
		ST		Z,			R16											;guardo nuevo valor
		RJMP	SALIDA

SEXTO_BIT:
		ANDI	R16,		~(1<<5)
		ST		Z,			R16											;guardo nuevo valor
		RJMP	SALIDA

SEPTIMO_BIT:
		ANDI	R16,		~(1<<6)
		ST		Z,			R16											;guardo nuevo valor
		RJMP	SALIDA

OCTAVO_BIT:
		ANDI	R16,		~(1<<7)
		ST		Z,			R16											;guardo nuevo valor
		RJMP	SALIDA

GUARDAR_UNO:
		CPI		R18,		7											;veo si tengo lugar en la misma palabra
		BRNE	NO_INC1

		ADIW	ZH:ZL,		1											;paso al siguiente byte
		CLR		R18

NO_INC1:
		LD		R16,		Z											;leo lo que ya estaba guardado en la direcc a la que apunta Z

		CPI		R18,		0											;veo si tengo lugar en la misma palabra
		BREQ	PRIMER_BIT1

		
		CPI		R18,		1											;veo si tengo lugar en la misma palabra
		BREQ	SEGUNDO_BIT1

		
		CPI		R18,		2											;veo si tengo lugar en la misma palabra
		BREQ	TERCER_BIT1

		
		CPI		R18,		3											;veo si tengo lugar en la misma palabra
		BREQ	CUARTO_BIT1

		
		CPI		R18,		4											;veo si tengo lugar en la misma palabra
		BREQ	QUINTO_BIT1

		CPI		R18,		5											;veo si tengo lugar en la misma palabra
		BREQ	SEXTO_BIT1

		CPI		R18,		6											;veo si tengo lugar en la misma palabra
		BREQ	SEPTIMO_BIT1

		CPI		R18,		7											;veo si tengo lugar en la misma palabra
		BREQ	OCTAVO_BIT1

PRIMER_BIT1:
		ORI		R16,		(1<<0)
		ST		Z,			R16											;guardo nuevo valor
		RJMP	SALIDA

SEGUNDO_BIT1:
		ORI		R16,		(1<<1)
		ST		Z,			R16											;guardo nuevo valor
		RJMP	SALIDA

TERCER_BIT1:
		ORI		R16,		(1<<2)
		ST		Z,			R16											;guardo nuevo valor
		RJMP	SALIDA

CUARTO_BIT1:
		ORI		R16,		(1<<3)
		ST		Z,			R16											;guardo nuevo valor
		RJMP	SALIDA

QUINTO_BIT1:
		ORI		R16,		(1<<4)
		ST		Z,			R16											;guardo nuevo valor
		RJMP	SALIDA

SEXTO_BIT1:
		ORI		R16,		(1<<5)
		ST		Z,			R16											;guardo nuevo valor
		RJMP	SALIDA

SEPTIMO_BIT1:
		ORI		R16,		(1<<6)
		ST		Z,			R16											;guardo nuevo valor
		RJMP	SALIDA

OCTAVO_BIT1:
		ORI		R16,		(1<<7)
		ST		Z,			R16											;guardo nuevo valor
		RJMP	SALIDA

SALIDA:
	RET



