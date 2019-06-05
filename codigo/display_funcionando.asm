.include "m328Pdef.inc"
;----------------------------------------------------------------------
;Declaro constantes para identificar los puertos utilizados por el LCD
;----------------------------------------------------------------------
.equ		lcd_dprt = PORTB		;puerto de datos del lcd
.equ		lcd_dddr = DDRB			;puerto ddr del lcd
.equ		lcd_dpin = PINB			;puerto pin del lcd
.equ		lcd_cprt = PORTC		;puerto de comandos del lcd
.equ		lcd_cddr = DDRC			;puero de comandos ddr del lcd
.equ		lcd_cpin = PINC			;puerto de comandos pin del lcd
.equ		rs = 0					;registro rs del lcd
.equ	    rw = 1					;registro r/w del lcd
.equ	    en = 2					;registro en del lcd

main:
				ldi r16,low(ramend)		; Inicializo el Stack Pointer
				out spl,r16
				ldi r16,high(ramend)
				out sph,r16  

				rcall	ini_lcd			;inicio el LCD
		
				ldi		ZL,		low(MJE<<1)	;"Iniciando----" en pantalla
				ldi		ZH,		high(MJE<<1)
				rcall	enviar_frase
				rcall	delay_5seg
		

aca:			rjmp	aca

;#####################################################################################
;Rutinas para iniciar el LCD
;#####################################################################################

ini_lcd	:		ldi r21, 0xFF
				out lcd_dddr, r21		;configuro los puertos de datos como salida
				sbi ddrd, 6
				sbi ddrd, 7
				out lcd_cddr, r21		;configuro los puertos de comando como salida
				cbi lcd_cprt, en		;en = 0 para luego grabar
				rcall delay_10ms		;Espero a que se prenda el LCD
				rcall delay_10ms
				;Inicio al LCD en 2 lineas, matriz de 5x7
				ldi r16,0X38 
				rcall enviar_com 
				;Prendo el Display, Prendo el cursor
				ldi r16,0x0E 
				rcall enviar_com
				;Borro la pantalla
				ldi r16,0x01
				rcall enviar_com
				;Corro el cursor hacia la derecha
 				ldi r16,0x06
				rcall enviar_com
 				ret

;-----------------------------------------------------------------------------------------;
;rutinas para enviar datos o comandos al LCD
;-----------------------------------------------------------------------------------------;
enviar_frase :	;ldi r16,0X01			;Borro la pantalla
				;rcall enviar_com
				ldi r16,0X06			;Corro el cursor hacia la derecha
				rcall enviar_com
				ldi r17, 0
ciclo_subr1 :	inc r17
				cpi r17, 17
				brsh linea_2
				lpm r16, z+
				cpi r16,0
				breq salir_subr1
				rcall enviar_dato
				rjmp ciclo_subr1
linea_2 :		ldi r16,0XC0			;Comienzo en la segunda linea
				rcall enviar_com
				rcall ciclo_subr1
salir_subr1 :	ret


enviar_com:		cbi lcd_cprt, rs		;rs=0 para mandar comandos
				cbi lcd_cprt, rw		;rw=0 para escribir en el LCD
				sbi lcd_cprt, en		;E=1
				out lcd_dprt, r16
				rcall sdelay 
				cbi lcd_cprt, en		;E=0 es el flanco descendente (High-to-Low)
				sbic lcd_dprt, 6
				sbi  portd, 6
				sbis lcd_dprt, 6
				cbi  portd, 6
				sbic lcd_dprt, 7
				sbi  portd, 7
				sbis lcd_dprt, 7
				cbi  portd, 7
				rcall delay_10ms
				ret


enviar_dato: 	sbi lcd_cprt, rs ;rs=1 para mandar datos
				cbi lcd_cprt, rw ;rw=0 para escribir en el LCD
				sbi lcd_cprt, en ;E=1
				out lcd_dprt, r16	
				sbic lcd_dprt, 6
				sbi  portd, 6
				sbis lcd_dprt, 6
				cbi  portd, 6
				sbic lcd_dprt, 7
				sbi  portd, 7
				sbis lcd_dprt, 7
				cbi  portd, 7
				rcall sdelay	
				cbi lcd_cprt, en ;E=0  ;Es el flanco descendente (High-to-Low)
				rcall delay_10ms
				ret

				  
;#################################################################
;Delays
;################################################################

delay_40us :	push r16		;[2] + rcall [3] = 5
				ldi r16, 18		;[1] --> ciclos = 6
ciclo_40us :	dec r16			;[1]*18 --> ciclos = 24
				brne ciclo_40us	;[1/2]*18 --> ciclos = 33
				pop r16			;[2] --> ciclos = 35
				nop				;[1] --> ciclos = 36
				ret				;[4] --> ciclos = 40, para un clck de 1 Mhz delay = 40us


sdelay:			nop
				nop
				ret


delay_100us:	push r16		;[2] + rcall [3] = 5
				ldi r16,58		;[1] --> ciclos = 6
ciclo_100us :	dec r16			;[1]*58 --> ciclos = 64
				brne ciclo_100us;[1/2]*58 --> ciclos = 93
				pop r16			;[2] --> ciclos = 95
				nop             ;[1] --> ciclos = 96
				ret				;[4] --> ciclos = 100, con un clck = 1Mhz delay = 100us


delay_2ms	:  	push r16		;[2] + [3] del rcall = [5]
				push r17		;[2] --> ciclos = 7
				ldi r17,24		;[1] --> ciclos = 8
ciclo1_2ms	:	ldi r16,22		;[1]*24*33 -->ciclos = 800
ciclo2_2ms	:	dec r16			;[1]*22
				brne ciclo2_2ms	;[1/2]*22 = subtotal = 33
				dec r17			;[1]*24*33 -->ciclos = 1592
				brne ciclo1_2ms	;[1/2]*24*33 -->ciclos = 1988
				nop
				nop
				nop
				nop				;[4] -->ciclos = 1992
				pop r17			;[2] -->ciclos = 1994
				pop r16			;[2] -->ciclos = 1996
				ret				;[4] -->cilclos = 2000, para un clk = 1Mhz delay = 2,00 ms

				
delay_10ms	:  	push r16		
				push r17		
				ldi r17,22		
ciclo1_10ms	:	ldi r16,121		
ciclo2_10ms	:	dec r16			
				brne ciclo2_10ms	
				dec r17			
				brne ciclo1_10ms	
				nop
				nop				
				pop r17			
				pop r16			
				ret	
								
				
delay_100ms :   ldi r16, 20			;[1]
ciclo_100ms	:	rcall delay_2ms		;2ms*20
				dec r16				;2ms*20
				brne ciclo_100ms	;2ms*10 ---> aprox delay (20+20+10)*2ms = 100 ms 
				ret

delay_5seg :	ldi r17, 25
ciclo_dy5s:		rcall delay_100ms
				dec r17
				brne ciclo_dy5s
				ret


;-----------------------------------------------------------------------------
;Constantes en memoria Flash para utilizar en pantalla LCD
;-----------------------------------------------------------------------------
MJE : .db "son muchos gatitos",0