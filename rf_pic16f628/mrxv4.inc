;**********************************************************************
;                                                                     *
;    Filename:	    mrxv4.inc                                         *
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
; Manchester decoding routine, library version
; half bit T ~= 620 usec with a 4MHz clock
;

	EXTERN mrx_buffer, mrx_bsum
	EXTERN mrx_init, mrx_receive, mrx_chk_buf

mrx_packet_len	EQU	2 ; packet length, check var. alloc!
