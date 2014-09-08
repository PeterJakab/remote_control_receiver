;**********************************************************************
;                                                                     *
;    Filename:	    decoder.asm                                       *
;    Date:                                                            *
;    File Version:                                                    *
;                                                                     *
;    Author:        el@jap.hu                                         *
;                   http://jap.hu/electronic/                         *
;**********************************************************************
;NOTES
;
; PA0-PA3-> outputs
; PA4    -> valid reception
; PA5    <- RX input
; PB0-PB3<- address bits
; PB4-PB7-> outputs
;
;
;**********************************************************************
;HISTORY
;
; 043-20050214 based on mrx-042
;              4/8-channel remote control decoder
;              outputs are latched or momentary (selectable)
;
; 043b-20120316 correct a bug with channel 4 interfering with the VALID led
;**********************************************************************

	list      p=16f628

	__CONFIG   _CP_OFF & _WDT_OFF & _PWRTE_ON & _HS_OSC & _LVP_OFF & _MCLRE_OFF & _BODEN_ON

#include <p16f628.inc>
#include "mrxv4.inc"

EXPIRE_TIMER	EQU 0x12
LATCH_MASK	EQU 0xff
VALID_BIT	EQU 4

.mainda UDATA 0x20

savew1		res 1
savestatus	res 1
savepclath	res 1
savefsr		res 1
bt		res 1
expire_cnt	res 1
cur_seq		res 1
cur_ch		res 1
cur_state	res 1

vectors		CODE 0
  		goto    main		; go to beginning of program
		nop
		nop
		nop
					; interrupt vector location
		goto itr

prog		CODE 5

channel_lookup  andlw 0x07
		addwf PCL, F
		dt 0x1, 0x2, 0x4, 0x8
		dt 0x10, 0x20, 0x40, 0x80

itr
	movwf	savew1
	movf	STATUS,w
	clrf	STATUS
	movwf	savestatus
	movf	PCLATH,w
	movwf	savepclath
	clrf	PCLATH

	movf	FSR,w
	movwf	savefsr

	btfsc	INTCON, T0IF
	call	t0_int_handler

	movf	savefsr,w
	movwf	FSR

	movf	savepclath,w
	movwf	PCLATH

	movf	savestatus,w
	movwf	STATUS

	swapf	savew1,f
	swapf	savew1,w

	retfie

main
		;HARDWARE INIT
		movlw 7
		movwf CMCON

		movlw (1<<VALID_BIT)
		MOVWF PORTA
		clrf PORTB

		BANKSEL TRISA

		movlw 0x20 ; A5=input
		movwf TRISA

		MOVLW 0x0f ; B0-B3=input
		movwf TRISB

		; setup TMR0 interrupt
		clrwdt ; changing default presc. assignment
		movlw 0x03 ; prescaler 1:16 assigned to TMR0
		movwf OPTION_REG ; T0CS selects internal CLK
		bsf INTCON, T0IE ; enable TMR0 int

		BANKSEL TMR0

		clrf TMR0

		clrf expire_cnt
		call mrx_init
		bsf INTCON, GIE

warm		clrf cur_state
		clrf cur_seq
		incf cur_seq, F

loop		call mrx_receive
		andlw 0xff
		bnz loop
		call mrx_chk_buf
		andlw 0xff
		bnz loop

		movf PORTB, W
		andlw 0x0f
		subwf (mrx_buffer), W
		bnz loop

rx_ok		movlw EXPIRE_TIMER ; indicate packet reception
		movwf expire_cnt

		movf (mrx_buffer+1), W ; if (seq==cur_seq) skip (only expire timer is updated)
		andlw 0xc0
		subwf cur_seq, W
		bz loop

		movf (mrx_buffer+1), W
		andlw 0xc0
		movwf cur_seq

		movf (mrx_buffer+1), W
		andlw 0x3f
		bz loop ; illegal channel data
		addlw 0xff
		andlw 0x0f
		movwf bt
		btfsc bt, 3
		goto loop ; ch>=8: illegal with this model
		call channel_lookup
		movwf cur_ch

		bcf INTCON, GIE
		btfsc (mrx_buffer+1), 5
		goto state_on_off

		; 00-0f: toggle or momentary ON
		movf cur_ch, W
		andlw LATCH_MASK
		bz state_on_off ; momentary ON

state_toggle	movf cur_ch, W
		xorwf cur_state, F
		goto state_done

state_on_off	movf cur_ch, W
		xorlw 0xff
		andwf cur_state, F ; 20-2f: latch ON
		movf cur_ch, W
		btfss (mrx_buffer+1), 4 ; 30-3f: latch OFF
		iorwf cur_state, F
		goto state_done

state_done	bsf STATUS, C ; set valid led ON
		call state_out
		bsf INTCON, GIE
		goto loop

t0_int_handler	bcf INTCON, T0IF
		movf expire_cnt, F
		bnz valid_on

		movlw LATCH_MASK ; clear momentary outputs
		andwf cur_state, F
		bcf STATUS, C ; set valid led OFF

state_out	; C: turn valid bit on or off?
		movlw ~(1<<VALID_BIT)
		andwf cur_state, W ; set valid led ON
		btfss STATUS, C
		iorlw (1<<VALID_BIT) ; set valid led OFF
		movwf PORTA
		movf cur_state, W
		movwf PORTB
		return

valid_on	decf expire_cnt, F
		;;bsf RX_LED
		return

		end

