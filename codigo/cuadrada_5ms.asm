.include "m328Pdef.inc"

;--------------------------------------------------------------
;Rutinas de Retardo de 5ms
;--------------------------------------------------------------
; hacer un llamado tarda 5 ciclos

.def Temp1 = R16
.def temp2 = R17


delay5ms:
	ldi Temp1, 0xFF ;para 8mhz ; 1 ciclo
	out portb, Temp1
LOOP0:
	ldi temp2, 200 ; 1 ciclo
LOOP1:
	dec temp2 ; 1 ciclo
	brne LOOP1 ; 1 si es falso 2 si es verdadero
	dec Temp1 ; 1
	brne LOOP0 ; 2
ret

