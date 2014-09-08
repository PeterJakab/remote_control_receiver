;**********************************************************************
;                                                                     *
;    Filename:	    mrxv4.asm                                         *
;    Date:                                                            *
;    File Version:                                                    *
;                                                                     *
;    Author:        el@jap.hu                                         *
;    Www page:      http://jap.hu/electronic/                         *
;                                                                     * 
;                                                                     *
;**********************************************************************
;
; NOTES
;
; RF receiver prot.v4.2 (Manchester)
;
; HISTORY
;
; 040-20040220 - library version
;  aging function is removed
;  aging should be handled independently in interrupt
;
; 041-20041003 - define min_t, min_2t, max_2t as functions of T
;
; 042-20040102 codec-v4.2:
;              checksum with CRC-8
;              T=350 usec
;              3-byte buffer length
;

        list      p=16F628
        #include <p16F628.inc>

	GLOBAL mrx_buffer, mrx_bsum
	GLOBAL mrx_init, mrx_receive, mrx_chk_buf

;***** CONSTANT VALUES
hdrcntmin	EQU	0x0c ; minimum number of header bits to receive
hdrcntmax	EQU	0x10 ; maximum number of header bits to receive

; decoder time tolerances are set at : 0.5T, 1.5T, 2.5T
; measured in 9xinstr.time (9usec) counts

T		EQU	.39 ; half frame 350 usec (= T * 9 usec)

min_t		EQU	T/2  ; half frame (T) minimum time
min_2t		EQU	3*T/2 ; half frame (T) maximum time and full frame (2T) minimum time
max_2t		EQU	5*T/2 ; full frame (2T) maximum time

packet_len	EQU	2 ; packet length, check var. alloc!

; input port bit
#define RXBIT PORTA, 5

; normal decoder logic input
#define SKL btfsc
#define SKH btfss

; inverse decoder logic input
;define SKL btfss
;define SKH btfsc

;***** FLAGS
#define IF_SHORT flags, 0
#define FIRST_HALF flags, 1
#define HEADER flags, 2
#define VALID flags, 7

if_short_val	EQU	1 ; bit value of IF_SHORT flag
first_half_val	EQU	2 ; bit value of FIRST_HALF flag


mrxdata		UDATA

;***** VARIABLE DEFINITIONS
bitcnt		res	1
tmrval		res	1 ; timer value
bt		res	1 ; receive byte buffer
flags		res	1 ; decode logic status
btcnt		res	1 ; byte counter

mrx_buffer	res	2 ; receive packet buffer
mrx_bsum	res	1 ; receive buffer, checksum

mrxcode		CODE

mrx_receive	; receive a full manchester-encoded packet
s3		; set flags: first_half=1, if_short=0
		bsf FIRST_HALF
s4		bcf IF_SHORT

s5		; init before the received packet

		; set FSR to buffer start
		movlw mrx_buffer
		movwf FSR
		; set byte counter
		movlw (packet_len+1) ; bytes / packet
		movwf btcnt
		; set header receive mode
		bsf HEADER
		clrf bitcnt ; counting bit1-s in this mode

s2		; wait for a pulse
		SKH RXBIT
		goto s2

s6		; wait for end of (short) pulse up to min_2t
		clrf tmrval
s6_w		SKH RXBIT
		goto s7 ; goto s7 at end of pulse

		incf tmrval, F
		nop
		movlw min_2t
		subwf tmrval, W
		btfss STATUS, C
		goto s6_w

		; timeout, exit
		retlw 1 ; illegal startbit

s7		; start timer
		clrf tmrval

s8		; if (if_short & rxbit) goto s9
		; if (!if_short & !rxbit) goto s9
		; goto s10

		btfsc IF_SHORT
		; if_short = 1
		goto s8_ss1

s8_ss0		; if_short = 0
		SKL RXBIT
		goto s10 ; rxbit = 1, goto s10

s9_ss0		; if (timer > max_2t) exit - else goto s8
		movlw max_2t
		subwf tmrval, W
		btfsc STATUS, C
		retlw 2 ; signal too long

		incf tmrval, F
		goto s8_ss0

s8_ss1		; if_short = 1
		SKH RXBIT
		goto s10 ; rxbit = 0, goto s10

s9_ss1		; if (timer > max_2t) exit - else goto s8
		movlw max_2t
		subwf tmrval, W
		btfsc STATUS, C
		retlw 2 ; signal too long

		incf tmrval, F
		goto s8_ss1

s10		; invert if_short
		movlw if_short_val
		xorwf flags, F

s11		; if (timer < min_t) exit
		movlw min_t
		subwf tmrval, W
		btfss STATUS, C
		retlw 3 ; signal too short

s12		; if (timer < min_2t) goto s14
		movlw min_2t
		subwf tmrval, W
		btfss STATUS, C
		goto s14

s13		; if (first_half = 0) goto s16 - else exit
		btfss FIRST_HALF
		goto s16
		retlw 4 ; no mid-frame transition/out of sync

s14		; invert first_half
		movlw first_half_val
		xorwf flags, F

s15		; if (first_half = 1) goto 7
		btfsc FIRST_HALF
		goto s7

s16		; if_short is a decoded bit. Handle here
		btfss HEADER
		goto s16_not_header

		; header receiving mode
		btfss IF_SHORT
		goto s16_header_end

		; header bit is 1
		btfss bitcnt, 4 ; inc up to 16
		incf bitcnt, F  ; 16 is enough...

#ifdef NOMAXHDR
		; test for max header length
		movlw hdrcntmax
		subwf bitcnt, W
		btfss STATUS, C
#endif
		goto s7 ; loop back
		retlw 9 ; header too long

s16_header_end	; header ends indicated by a 0
		bcf HEADER

		; test for min header length
		movlw hdrcntmin
		subwf bitcnt, W
		btfss STATUS, C
		retlw 0x0a ; header too short

next_byte	movlw 0x0a
		movwf bitcnt
		goto s7 ; loop back

s16_not_header	; receiving bytes
		decf bitcnt, F
		bz s16_s4 ; if (bitcnt = 0) check for a byte-sep 1

		; if (bitcnt = 1) check for a byte-separator 0
		movlw 1
		xorwf bitcnt, W
		bnz s16_s2

		; test for a byte separator 1
		btfsc IF_SHORT
		goto s7
		retlw 7 ; byte-ending 1 not present

s16_s2		; bit is data

		rrf flags, W
		rlf bt, F
		goto s7

s16_s4		; check for a byte-separator 0
		btfsc IF_SHORT
		retlw 8 ; byte-ending 0 not present

		; OK, received byte is sane, store in buffer
		movf bt, W
		movwf INDF
		incf FSR, F

		decfsz btcnt, F
		goto next_byte

		retlw 0 ; OK, buffer received

		; buffer checking is not done automatically!
		; if returned value is 0, call mrx_chk_buf to check
		;

mrx_chk_buf	; check buffer sanity by chksum
		movlw mrx_buffer
		movwf FSR
		movlw (packet_len+1) ; number of bytes with the chksum byte
		movwf btcnt
		movlw 0xff
		movwf bt ; used as sum register

chk0		movf INDF, W
		; fast CRC-8 algorithm with poly x^8+x^5+x^4+1
		; executes in 23 cycles per update
		xorwf	bt,f
		clrw
		btfsc	bt,7
		xorlw	0x7a
		btfsc	bt,6
		xorlw	0x3d
		btfsc	bt,5
		xorlw	0x86
		btfsc	bt,4
		xorlw	0x43
		btfsc	bt,3
		xorlw	0xb9
		btfsc	bt,2
		xorlw	0xc4
		btfsc	bt,1
		xorlw	0x62
		btfsc	bt,0
		xorlw	0x31
		movwf	bt

		incf FSR, F
		decfsz btcnt, F
		goto chk0
		; correct checksum must be zero
		movf bt, W
		bnz chk_err
		retlw 0 ; result is in Z
chk_err		retlw 0x0c ; checksum error

mrx_init	return

	end                       ; directive 'end of program'

