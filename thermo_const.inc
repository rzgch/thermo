; vim: set syntax=tasm:
;==============================================================
; Temperature Control Program v9 (PIC16F870) 
;==============================================================

; these are program constants
; timing constants
TextDelay	EQU 0x03			; time to display 'Hi' 'Lo' 3 sec
SettDelay	EQU 0x04			; time for limit setting idle 4 sec
DebnceTime	EQU 0x30			; debounce x20ms
TimerSet	EQU .100			; (96 for 4.096Mhz) timer rollover = 5ms
DTModeTime	EQU .7				; time until dt mode is set
; setting limits
MinLimHL	EQU 0x14			; 20 = minimum possible Lo limit
MaxLimHL	EQU 0x50			; 80 = maximum possible Hi Limit
MinLimDT	EQU 0x01			; 1 = minimum possible dt setting
;MaxLimDT	EQU 0x0F			; 15 = maximum possible dt setting
MaxLimDT	EQU 0x32			; 50 = maximum possible dt setting						
; pump related
MinTankTmp	EQU 0x32			; 5 C = minimum temperature allowed
UpdateIntTime	EQU 0x54			; time to remain screen constant 0.5s

PumpDiff	EQU 0x3C			; (1 = 0x0A ; 3 = 0x1E, 5 = 0x32, 6 = 0x3C)
