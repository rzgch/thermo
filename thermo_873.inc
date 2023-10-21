
	LIST     p=16F873		; PIC16F873/873A is the target processor
	#include "p16f873.inc" 	; Include header file
#ifdef CFG_CP
	__config	_RC_OSC & _WDT_OFF & _BODEN_OFF & _LVP_OFF & _CPD_ON & _CP_ALL
#else
	__config	_RC_OSC & _WDT_OFF & _BODEN_OFF & _LVP_OFF 
#endif
