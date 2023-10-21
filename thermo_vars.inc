; vim: set syntax=tasm:
;==============================================================
; Temperature Control Program v9 (PIC16F870) 
;==============================================================

; select device
;#include "thermo_873.inc"  ; 0x960
#include "thermo_873A.inc"  ; 0xe40
;#include "thermo_870.inc"  ; 0xd00
;#include "thermo_916.inc"

; STANDARD 1 SENSOR : [NONE]
; STANDARD 3 SENSORS: OPTION_DT, OPTION_EXTRA, OPTION_MAIN2
; POOL     3 SENSORS: OPTION_DT, OPTION_EXTRA, OPTION_MAIN2, OPTION_POOL

; Device options	
#define OPTION_DT        	; include DT options
#define OPTION_EXTRA		; Extra check for backup control
#define OPTION_MAIN2		; 2nd sensor on tank
;#define OPTION_POOL		; Pool should not warm more than HiLimit even with solar
;#define OPTION_REVRES		; Reverse effect for backup

; Variables
; preprocessing directives
#define BANK1	bsf 	STATUS,RP0
#define BANK0	bcf 	STATUS,RP0

;#define debug
; global variable declarations
; pins
#define ResOn		PORTC,7
#define BackOn		PORTC,6
#define DispBack	PORTC,5
#define DispMain2	PORTC,4

; these addresses must be fixed and consecutive!!!!
DTLimit		EQU 0x21			; DT limit for pump configuration
HiLimit		EQU 0x22            ; High Limit
LoLimit		EQU 0x23            ; Low limit
buffMSB		EQU 0x24            ; MSB buffer to display
buffLSB		EQU 0x25			; LSB buffer to display
MainTH		EQU 0x26			; main temp high
MainTL		EQU 0x27			; main temp low
BackTH		EQU 0x28			; back temp high
BackTL		EQU 0x29			; back temp low
#ifdef OPTION_MAIN2
Main2TH		EQU 0x2A			; 2nd main temp high
Main2TL		EQU 0x2B			; 2nd main temp low
#else
Main2TH		EQU 0x26			; 2nd main temp high
Main2TL		EQU 0x27			; 2nd main temp low
#endif
TempTH		EQU 0x2C			; temporary used for display update
TempTL		EQU 0x2D			; ~
SSLimit		EQU 0x2E            ; Limit to prevent running backup when Solar is enough

; these addresses can be changed
cnt			EQU	0x30			; minor cnt in 5ms
Scnt		EQU 0x31			; major cnt in seconds
tmp			EQU 0x32			; few instructions scoped temporary
CurLim		EQU 0x33			; current selected limit type
LastLim		EQU 0x34			; Last limit saved before possible changes
BtnState	EQU 0x35			; last state of buttons
DigSinks    EQU 0x36			; PORTC digit sinks for display
Debnce		EQU	0x37
DTTimer		EQU	0x38
MinLimit	EQU 0x39
MaxLimit	EQU 0x3A
UpdateInt	EQU 0x3B

; used for b2bcd
count		EQU	0x40
R0			EQU	0x41
R1			EQU	0x42
R2			EQU	0x43
L_Byte		EQU	0x44
H_Byte		EQU	0x45

; push and pop macros to save registers on interrupts
WBuffer		EQU 0x46
StatBuffer	EQU 0x47
FSRTemp		EQU 0x48
H_tmp		EQU 0x49
L_tmp		EQU 0x4A

Flag		EQU 0x4B
#define DecOff		Flag,0
#define	IncOff		Flag,1
#define	EqualOn		Flag,2
#define	Decimal		Flag,3
#define BtnOn		Flag,4
#define BtnMode		Flag,5		; 1 = incr decr mode, 0 = temp mode
#define InitBtnOk	Flag,6		; 1 = a button was pressed
#define DebnceOn	Flag,7		; 1 = setting stage, 0 = text stage
Flag2		EQU 0x4C
#define	LSBout		Flag2,0
#define	LSBoutOK	Flag2,1
#define	DTon		Flag2,2
#define FirstOne	Flag2,3
#define LastRes     Flag2,4		; last resistance value
#define LastBack    Flag2,5		; last backup value

