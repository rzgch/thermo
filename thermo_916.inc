
	LIST     p=16F916	
	#include "p16f916.inc"
#ifdef CFG_CP
	__config	_EXTRC & _WDT_OFF & _BOREN_OFF & _MCLRE_ON & _PWRTE_ON & _CPD_ON & _CP_ALL
#else
	__config	_EXTRC & _WDT_OFF & _BOREN_OFF & _MCLRE_ON & _PWRTE_ON
#endif
