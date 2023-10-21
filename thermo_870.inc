
	LIST     p=16F870		; PIC16F870 is the target processor
	#include "p16f870.inc" 	; Include header file

#ifdef CFG_CP
	__config	_RC_OSC & _WDT_OFF & _PWRTE_ON & _BODEN_OFF & _LVP_OFF & _WRT_ENABLE_ON & _CPD_ON & _CP_ALL 
#else
	__config	_RC_OSC & _WDT_OFF & _PWRTE_ON & _BODEN_OFF & _LVP_OFF & _WRT_ENABLE_ON
#endif
