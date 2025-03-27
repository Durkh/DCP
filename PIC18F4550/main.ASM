#INCLUDE <p18f4550.inc>	

	errorlevel -207
	errorlevel -205
	
	CONFIG	FOSC = HSPLL_HS
	CONFIG	PWRT = OFF
	CONFIG	BOR = OFF
	CONFIG	WDT = OFF
	CONFIG	XINST = OFF
	CONFIG  LVP = ON

; VARIABLES
	
	device_slave	equ	0
	device_master	equ	1
	
	SPEED_SLOW	equ	0
	SPEED_FAST1	equ	1
	SPEED_FAST2	equ	2
	SPEED_ULTRA	equ	3
	

	;;which timers to use
	DELAY_H		equ 	TMR3H
	DELAY_L		equ 	TMR3L
	
	BUS_ADDR 	equ	0x2
	BUS_PIN  	equ 	RA1
	BUS_DEVICE_TYPE equ	device_slave

	DEBUG_PIN  	equ 	RA2

	;;CHANGE BUS SPEED HERE vvvvvvvvvv
	BUS_SPEED 	equ	SPEED_SLOW

	CSMA_DELAY_H 	equ 0xff - 0x7
	CSMA_DELAY_L	equ 0xff - 0x80

	SYNC_DELAY_H	equ 0xff - 0x5d
	SYNC_DELAY_L	equ 0xff - 0xc0

	START_DELAY_H	equ 0xff - 0x1c
	START_DELAY_L	equ 0xff - 0x20

	DELTA_DELAY_H	equ 0xff - 0x3
	DELTA_DELAY_L	equ 0xff - 0xc0
	#DEFINE TIMER_CHECK 	BTFSS	TMR3H, 2, ACCESS

	CBLOCK	0x00	
					
		W_TEMP		
		STATUS_TEMP	

		COLLISION
		INT_EN
		I
		J

	ENDC			

	; This memory block, in Bank 2, is now reserved to DCP data to send
	CBLOCK	0x200	
		TX_DCP_TYPE
		TX_DCP_ADDR
		TX_DCP_DATA:252
	ENDC
	
	CBLOCK	0x201	
		TX_L3_SOH
		TX_L3_IDS
		TX_L3_IDD
		TX_L3_COD
		TX_L3_DATA:6
		TX_L3_PAD
		TX_L3_CRC
	ENDC			

;------------------------------------------------------;

	; This memory block, in Bank 3, is now reserved to read DCP data
	CBLOCK	0x300	
		RX_DCP_TYPE
		RX_DCP_ADDR
		RX_DCP_DATA:252
	ENDC			

	CBLOCK	0x301	
		RX_L3_SOH
		RX_L3_IDS
		RX_L3_IDD
		RX_L3_COD
		RX_L3_DATA:6
		RX_L3_PAD
		RX_L3_CRC
	ENDC			

;------------------------------------------------------;
; BOOTLOADER

	ORG	0x00			
	GOTO	INICIO
	
;------------------------------------------------------;

; HIGH-PRIORITY INTERRUPT
	ORG	0x08			
	MOVWF	W_TEMP		
	SWAPF	STATUS,W
	MOVWF	STATUS_TEMP	

SAI_INT
	SWAPF	STATUS_TEMP,W
	MOVWF	STATUS		
	SWAPF	W_TEMP,F
	SWAPF	W_TEMP,W	
	RETFIE


; LOW-PRIORITY INTERRUPT
	ORG	0x18			

	BTFSS	PIR2, TMR3IF
	RETFIE

	SETF	J, ACCESS

	RETFIE

;------------------------------------------------------;
; SUBROUTINES

read_bit: 

	RML0:	;wait for high signal
		BTFSS	PORTA, BUS_PIN, ACCESS
	BRA	RML0

	;set timer
	BSF	T3CON, TMR3ON

	CLRF	TMR3H, ACCESS		
	CLRF	TMR3L, ACCESS		

	RML1:	;high time wait
		BTFSC	PORTA, BUS_PIN, ACCESS
	BRA	RML1

	BCF	T3CON, TMR3ON
	MOVF	TMR3L, W

	TIMER_CHECK

	BRA 	RB_BIT0	;if the time was less than delta, then it's zero

	BSF	STATUS, C, ACCESS	;bit is 1
	RETURN

	RB_BIT0:
	BCF	STATUS, C, ACCESS 	;bit is 0

	RETURN

;------------------------------------------------------;

read_byte macro dest

	CALL	read_bit
	RLCF	J, F, ACCESS
	
	CALL	read_bit
	RLCF	J, F, ACCESS
	
	CALL	read_bit
	RLCF	J, F, ACCESS
	
	CALL	read_bit
	RLCF	J, F, ACCESS
	
	CALL	read_bit
	RLCF	J, F, ACCESS
	
	CALL	read_bit
	RLCF	J, F, ACCESS
	
	CALL	read_bit
	RLCF	J, F, ACCESS
	
	CALL	read_bit
	RLCF	J, F, ACCESS
	
	MOVFF	J, dest

endm

;------------------------------------------------------;

;;the size of the DATA to be sent must be stored on I
send_data:
	SL1: ;;this loops through the data
		MOVLW 	8
		MOVWF	J, ACCESS
		L2: ;;this loops through the bits

			;;we set the bus on high by releasing it
			BSF	TRISA, BUS_PIN, ACCESS

			MOVLW	DELTA_DELAY_H
			MOVWF 	TMR3H	
			MOVLW	DELTA_DELAY_L
			MOVWF 	TMR3L	

			RRCF	INDF2, 0	; getting each bit from the data

			;;check if the data is 1 or 0
			BTFSS	STATUS, C, ACCESS
			CALL delay 	; if data == 1, we have 2delta

			MOVLW	DELTA_DELAY_H
			MOVWF 	TMR3H	
			MOVLW	DELTA_DELAY_L
			MOVWF 	TMR3L
			CALL delay	; if data == 0, we have 1delta

			;;collision check
			CALL 	check_bus, FAST
			BTFSC 	COLLISION, ACCESS
			GOTO 	WAITING

			BCF	TRISA, BUS_PIN, ACCESS	;setting the bus low
			MOVLW	DELTA_DELAY_H
			MOVWF 	TMR3H	
			MOVLW	DELTA_DELAY_L
			MOVWF 	TMR3L
			CALL delay	

			BSF	TRISA, BUS_PIN, ACCESS	;releasing the bus again
			;;collision check
			CALL 	check_bus, FAST
			BTFSC 	COLLISION, ACCESS
			GOTO 	WAITING

		DECFSZ	J, ACCESS
		GOTO 	L2

		INCF	FSR2L, ACCESS 	;using FSR2L guarantees that we will always be in bank1

	DECFSZ 	I, ACCESS
	GOTO 	SL1


	RETURN


;------------------------------------------------------;

check_bus:
	BTFSS PORTA, BUS_PIN 	;check if pin is low
	SETF COLLISION, ACCESS

	RETURN FAST

;------------------------------------------------------;

;;busy wait delay
delay:
	BSF	T3CON, TMR3ON, ACCESS
	
	delay_loop:
	BTFSS 	PIR2, TMR3IF, ACCESS
	GOTO delay_loop

	BCF	T3CON, TMR3ON, ACCESS
	RETURN 

;------------------------------------------------------;

toggle_int:

	;;if we had int. enabled before, reenable
	BTFSS 	INT_EN, 0, ACCESS
	GOTO	int_setting

	BSF	INTCON, GIE, ACCESS
	CLRF	INT_EN, ACCESS
	RETURN

	int_setting:
	;;if GIE is enabled, disable
	BTFSS 	INTCON, GIE, ACCESS
	RETURN
	SETF	INT_EN, ACCESS
	BCF	INTCON, GIE, ACCESS
	
	RETURN

;------------------------------------------------------;

INICIO

	MOVLW	B'00000111'
	MOVWF	CMCON
	MOVLW	0x0F
	MOVWF	ADCON1, ACCESS
	MOVLW	B'01111011'
	MOVWF	TRISA, ACCESS		
	BSF	T3CON, RD16, ACCESS		

	MOVLW	B'11000000'
	MOVWF	INTCON, ACCESS	

	BCF	IPR2, TMR3IP, ACCESS
	
	MOVLW	BUS_ADDR
	MOVWF	TX_DCP_ADDR
	
	GOTO	p_init

MAIN 

LISTENING:	;CSMA

		BCF	LATA, DEBUG_PIN, ACCESS ;;debug
    
		CLRF	J, ACCESS

		;;set timer
		MOVLW	CSMA_DELAY_H
		MOVWF	TMR3H, ACCESS
		MOVLW	CSMA_DELAY_L
		MOVWF	TMR3L, ACCESS

		LL1:
			BTFSC	PORTA, BUS_PIN, ACCESS	; if the bus is 0, disable tmr interrupt and goto waiting
			BRA	LL_test

			BCF	PIE2, TMR3IE, ACCESS	;disabling timer interrupt
			GOTO 	WAITING
		LL_test:				; else wait until the tmr overwflows, setting J and continuing
		BTFSS 	J, ACCESS	;using J as interrupt flag
		BRA 	LL1
		
		BSF	LATA, DEBUG_PIN, ACCESS ;;debug
	STARTING:

		BCF	TRISA, BUS_PIN, ACCESS

		MOVLW	SYNC_DELAY_H
		MOVWF 	TMR3H	
		MOVLW	SYNC_DELAY_L
		MOVWF 	TMR3L
		
		BCF	LATA, DEBUG_PIN, ACCESS ;;debug
		CALL 	delay
		
		;sync bit signal
		BSF	TRISA, BUS_PIN, ACCESS

		MOVLW	START_DELAY_H
		MOVWF 	TMR3H	
		MOVLW	START_DELAY_L
		MOVWF 	TMR3L	
		CALL 	delay

		BSF	LATA, DEBUG_PIN, ACCESS	;;debug 
		BCF	TRISA, BUS_PIN, ACCESS

		MOVLW	START_DELAY_H
		MOVWF 	TMR3H	
		MOVLW	START_DELAY_L
		MOVWF 	TMR3L	
		CALL 	delay

	SENDING:
		MOVLB	2	;selecting bank 2, which contains the data to be sent

		CALL 	toggle_int	;disabling the interrupts, so it doesn't mess with the timings

		;;we want to check if type is one of 3 values
		MOVLW	1
		SUBWF	TX_DCP_TYPE, W

		sync:
			BTFSS	STATUS, N, ACCESS ;;if N is set, type was 0
			GOTO	WL3

			GOTO 	WAITING

		WL3:
			BTFSS	STATUS, Z, ACCESS ;;if Z is set, type was 1
			GOTO	generic

			;;we are setting the indf address to the L3 structure
			LFSR 	FSR2, 0x201

			MOVLW	12	;size of L3 packet
			MOVWF	I, ACCESS
			CALL 	send_data

			GOTO 	WAITING

		generic:
			;;we are setting the indf address to the DCP structure
			LFSR 	FSR2, 0x200

			MOVF	TX_DCP_TYPE, W, BANKED
			MOVWF	I, ACCESS

			CALL send_data

	WAITING:
		CALL 	toggle_int	;we restart the interruptions
p_init:		BCF	COLLISION, ACCESS

		WL1:
			BTFSS	PORTA, BUS_PIN, ACCESS
			GOTO 	READING
		BRA 	WL1

		MOVLW	0x3
		MOVWF	TX_DCP_TYPE
		
		MOVLW	0xC3
		MOVWF	TX_DCP_DATA
		
		;;TODO if there's something to send, GOTO starting
		BRA	STARTING

		GOTO 	WAITING
	READING:

		MOVLB	3	;selecting bank 3, which contains the data to be read

		CALL 	toggle_int	;disabling interruptions as to not mess with the timings
		BSF	TRISA, BUS_PIN, ACCESS	;set as input
		
		BCF	LATA, DEBUG_PIN, ACCESS ;;debug

		;;BSF	T3CON, TMR3ON, ACCESS
		
		MOVLW	8	;initializing loop
		MOVWF	I, ACCESS

		sync_read_loop:	; wait for sync 
		BTFSS	PORTA, BUS_PIN, ACCESS
		BRA 	sync_read_loop
		
		BSF	LATA, DEBUG_PIN, ACCESS ;;debug

		sync_read_high:	; wait for sync 
		BTFSC	PORTA, BUS_PIN, ACCESS
		BRA 	sync_read_high

		;;TODO check loop ^^^^
		BCF	LATA, DEBUG_PIN, ACCESS ;;debug
		
		;;reading flag
		RL1:
			CALL	read_bit
			RLCF	RX_DCP_TYPE, 1, BANKED

		DECFSZ	I, F, ACCESS
		BRA 	RL1

		DECF	RX_DCP_TYPE, W, BANKED ; if the flag is not 0, goto GENERIC
		BNN 	GENERIC


		RLL3:	;L3 reading loop

			MOVLW 	12
			MOVWF	I, ACCESS

			LFSR 	FSR2, 0x303	;address of DCP_DATA

			RL_L3_L1:
				BSF	LATA, DEBUG_PIN, ACCESS ;;debug
				read_byte 	POSTINC2	;we are reading into the L3 structure sequentially
				BCF	LATA, DEBUG_PIN, ACCESS ;;debug

			DECFSZ	I, ACCESS
			BRA 	RL_L3_L1

		GOTO WAITING

		GENERIC:	;Generic message reading loop

			;;reading ADDR
			read_byte 	RX_DCP_ADDR
			
			MOVLW	2
			SUBWF	RX_DCP_TYPE, W, BANKED
			MOVWF	I, ACCESS

			LFSR 	FSR2, 0x302	;address of DCP_DATA
			
			RL2:
				BSF	LATA, DEBUG_PIN, ACCESS ;;debug
				read_byte 	POSTINC2	;here, we read a byte into the data array and increment the pointer
				BCF	LATA, DEBUG_PIN, ACCESS ;;debug

			DECFSZ	I, ACCESS
			BRA 	RL1

	GOTO WAITING
END
