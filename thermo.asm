; vim: set syntax=tasm:
;==============================================================
; Temperature Control Program (PIC16F870, PIC16F873, PIC16F873A) 
;==============================================================
;	1. Reads temperature from 3 sources
;	2. Displays on 3x7-SEG controled by a Toggle Button
;	3. Hi/Lo Limit Settings using '+' and '-' buttons
;	4. Resistance and Backup on/off control
;==============================================================
; OSC:		4.000MHz or preferably 4.096Mhz
; PORTA: 0,5 Temperature input 
;	(2.73<temp<4.753)/(0C<range<102.3), Vref+ = 4.753V, Vref- = 2.73V
; PORTB: 0-7 7-Seg , (0 buttons select, 4(Hi), 5(Lo), 6(DM))
; PORTC: 0-2 multi to 7-seg, 6 open/close pump, 7 open/close res 
;==============================================================
; START:            2004-01-11	
; MAJOR UPDATES:	2006-08-03
;                   2009-07-20
;                   2011-04-29
;==============================================================
; compiler information
;#define CFG_CP
    
	#include "thermo_vars.inc"
	#include "thermo_const.inc"

	ERRORLEVEL	-302

push	macro
	movwf	WBuffer
	swapf	WBuffer,F
	swapf	STATUS,W
	movwf	StatBuffer
		endm
	
pop		macro
	swapf	StatBuffer,W
	movwf	STATUS
	swapf	WBuffer,W
		endm

; EEPROM Data
	org 	2100
	de		H'06'		; DT
	de		H'32'		; Hi
	de		H'27'		; Lo

; program start
	org	0
	goto	initPIC
	
;interrupts here	
	org 4
	push
	call	handleInterrupts
	pop
	retfie
	
initPIC
; configure IO
	clrf	PORTA		; clean up
	clrf	PORTB		; ~
	clrf	PORTC		; ~
	movlw	0xff		; Configure porta IO
	BANK1
	movwf	TRISA		; potra all input
	clrf	TRISB		; portb all out
	clrf	TRISC		; portc all out
	movlw	B'10000100'	; configure option reg bit3=select tmr0, 2-0=PSA (1:32)
	movwf	OPTION_REG	; ~ prescaler and no pullups
	movlw	B'10001011'	; Analog pins select all analog
	movwf	ADCON1		; ~ and vref+, vref- select and RightJustified
	bsf		PIE2,EEIE	; enable eeprom write interrupt
	BANK0
;	clrf	PORTA		; clean up
;	clrf	PORTB		; ~
	clrf	PORTC		; ~
	movlw	B'11000001'	; select RC clock, enable AD
	movwf	ADCON0		; ~ configure AD conversion
; initialising variables
	clrw				; select address 0, HiLimit stored
	call	Read_EEDATA	; read mempry
	movwf	DTLimit		; save value in LoMSB
	movlw	0x01		; select address 1, LoLimit stored
	call	Read_EEDATA	; read memory
	movwf	HiLimit		; save value in HiMSB
	movlw	0x02		; select address 1, LoLimit stored
	call	Read_EEDATA	; read mempry
	movwf	LoLimit		; save value in LoMSB

; DO NOT REMOVE ??
	movf	DTLimit,W
	movwf	SSLimit
	rlf		SSLimit,F
	movf	LoLimit,W	
	addwf	SSLimit,F	

#ifdef OPTION_EXTRA	
	call	SaveSSLimit
#endif	
	bcf		ResOn		; res off for start
	clrf	Flag		; clear all flags
	clrf	Flag2
	clrf	CurLim
	clrf	cnt
	clrf	BtnState
	clrf	DigSinks
	movlw	MinLimHL
	movwf	MinLimit
	movlw	MaxLimHL
	movwf	MaxLimit
; configure interrupts	
	movlw	B'11100000'	; configure interrupts
	movwf	INTCON		; enable peripheral ints and global int
; configure timer	
	movlw	TimerSet	; initialize timer
	movwf	TMR0		; ~

main
	btfsc	BtnOn
	call	BtnSelect	; handle pressed button
	goto	main		; continue with normal execution
	
; 'Hi' or 'Lo' array
HiLo
	addwf	PCL,F
	retlw	B'11101111' ; 0 = dt
	retlw	B'10101011'	; 1 = Hi
	retlw	B'11001101'	; 2 = Lo
	
BCDtoSEG	; convert BCD number to segments
	andlw   0x0f        ; limit to 16 options (0-15)
	addwf	PCL,F
	; SEG     gfedcba.
	retlw	B'01111110'	;0
	retlw	B'00001100'	;1
	retlw	B'10110110'	;2
	retlw	B'10011110'	;3
	retlw	B'11001100'	;4
	retlw	B'11011010'	;5
	retlw	B'11111010'	;6
	retlw	B'00001110'	;7
	retlw	B'11111110'	;8
	retlw	B'11001110'	;9
	retlw	B'11101100'	;A 10 = H
	retlw	B'00100000'	;B 11 = i
	retlw	B'01110000'	;C 12 = L
	retlw	B'10111000'	;D 13 = o
    retlw	B'10111100' ;E 14 = d
	retlw	B'11110000'	;F 15 = t
    retlw	B'00000000'	;dummy 16

BtnSelect
#ifdef OPTION_DT
	btfsc	BtnState,2
	goto	DispModeBtn ; DT Specific
#endif
	call	InitBtnMode
	goto	DoneBtn
DispModeBtn
	call	ToggleDisp
DoneBtn
	bcf		BtnOn
	return

;**************************************************************
; INTERRUPT ROUTINES	
handleInterrupts
	clrf	STATUS			; bank0 (00)
	btfsc	INTCON,T0IF	; if TMR0 interrupt
	goto	handleTMR0	; ~ handle it
	btfsc	PIR2,EEIF		; check if its eeprom write interrupt
	bcf		PIR2,EEIF		; if yes clear interrupt bit
	return

handleTMR0
	movlw	TimerSet	; initialize timer
	movwf	TMR0		; ~
	bcf		INTCON,T0IF	; clear flag
	btfsc	DigSinks,0		; scan buttons
	call	ScanBtns	; ~ every 20ms
	btfss	InitBtnOk	; check if its btn mode
	goto	getTempMode	; if it's not then get new temp
	call	handleCnt	; if it is handle delays every 5ms
	goto	JustDisplay	; ~ and goto display
getTempMode
	btfsc	DigSinks,1		; get temps
	call	ScanAD		; ~ every 40ms at no buttons pressed
	btfsc	DigSinks,2		; set outputs
	call	ConfigOut	; ~ every 20ms
JustDisplay
	call	Display		; update display every 5ms
	return
	
;**************************************************************
ScanBtns				; Debnce is used for button auto increment
	btfss	DebnceOn	; check if its stalling time
	goto	scanSafe	; if not then continue scan
	decfsz	Debnce,F	; else continue stalling
	return				; ~
	bcf		DebnceOn	; if time over than clear Debnce flag
	return
scanSafe
	clrf	BtnState	; PORTB initialisation saved in BtnState 
	BANK1
	bcf		OPTION_REG,NOT_RBPU	; enable pullups
#ifdef OPTION_DT
	movlw	B'01110000'	; change portb i/o to accept buttons
#else
	movlw	B'00110000'	; change portb i/o to accept buttons
#endif
	movwf	TRISB		; ~
	BANK0
	clrf	PORTB
	nop
	swapf	PORTB,W
#ifdef OPTION_DT
	xorlw	B'00000111'
#else
	xorlw	B'00000011'
#endif
	btfsc	STATUS,Z	; if result not zero jump (= button pressed)
	goto	nopress
	movwf	BtnState	; Save value
	bsf		BtnOn
	movlw	DebnceTime
	movwf	Debnce
	bsf		DebnceOn
outnow
	BANK1
	bsf		OPTION_REG,NOT_RBPU	; disable pullups
	clrf	TRISB		; restore portb for display	
	BANK0
	clrf	PORTB		; clean up
	return
nopress
	bcf		DTon
	goto	outnow

InitBtnMode
	btfsc	InitBtnOk	; continue only if not already initialized
	goto	ChngLimMode; ~ then go next step
	bsf		InitBtnOk	; the init is ok
	movf	BtnState,W
	movwf	FSR			; save this result in FSR	
	movwf	CurLim
; displaying Hi or Lo
	call	HiLo		; call the array
	movwf	buffMSB		; save to location
	bsf		LSBout
	movlw	TextDelay	; delay to show the text
	movwf	Scnt			; ~
	movlw	DTLimit		; DTLimit(0) others consecutive HiLimit(+1) LoLimit(+2)
	addwf	FSR,F		; FSR holds HiMSB or LoMSB
	movf	INDF,W		; Save the last limit value
	movwf	LastLim		; ~
	return
	
ChngLimMode
	movlw	SettDelay	; if there is press then
	movwf	Scnt			; ~ reset timer
	call	checkLims
	btfss	BtnState,1	; if the result is 01 increment else 10 decrement
	goto	inc			; ~ increment
	call	decLimit	; ~ decrement
	goto	outChangeDisplay
inc
	call	incLimit	; increment limit
outChangeDisplay
	clrf	H_Byte		; set conversion parameters
	movf	INDF,W		; get changed value 
	movwf	L_Byte		; convert to BCF
	call	B2_BCD		; ~ do conversion
	movf	R2,W		; get result
	movwf	buffMSB		; move answer to buffMSB
	return

checkLims				; allow a minimum diff between the limits (Hi>Lo)
	movlw	B'11111100'	; clear previous flags
	andwf	Flag,F		; ~
	movf	LoLimit,W	; HiMSB=LoMSB
	subwf	HiLimit,W	; ~ HiLimit-LoLimit
	btfss	STATUS,Z	; if its not minmum than clear restrictions
	return
	movf	CurLim,W
	iorwf	Flag,F
	return
	
incLimit
	btfsc	IncOff
	return
	movf	MaxLimit,W	; if hilimit max then disallow increment
	xorwf	INDF,W
	btfss	STATUS,Z
	incf	INDF,F		; increment the limit
	return

decLimit
	btfsc	DecOff
	return
	movf	MinLimit,W	; if lolimit is min then disallow decrement
	xorwf	INDF,W
	btfss	STATUS,Z
	decf	INDF,F		; just decrement
	return
	
handleCnt
	decfsz	cnt,F		; decrement the count
	return				; if not zero then go again
	decfsz	Scnt,F		; another major counter
	return
	btfss	BtnMode		; if its zero check if its limchange mode
	goto	LimitChangeMode	; if its not then set it to limchange mode
; reset all from buttons mode to temp mode
	bcf		BtnMode		; if it was then clear button flags
	bcf		InitBtnOk	; ~
	bcf		BtnOn
	bcf		LSBout
	; write to eeprom here
	movf	LastLim,W		; load last limit
	xorwf	INDF,W			; check if the new limit has changed
	btfss	STATUS,Z		; if its changed
	call	Write_EEPROM	; save value to eeprom	
	; end write to eeprom
#ifdef OPTION_EXTRA	
	call	SaveSSLimit
#endif	
	; restore limits
	movlw	MinLimHL
	movwf	MinLimit
	movlw	MaxLimHL
	movwf	MaxLimit
	return
LimitChangeMode			; set to limchange mode
	movlw	SettDelay	; initialize timer
	movwf	Scnt			; ~ for setting mode
	bsf		BtnMode		; set limit change
	call	outChangeDisplay	; to display the previous value
	return
	
ToggleDisp
	btfsc	InitBtnOk	; if its Hi Lo buttons mode
	return				; ~ don't enter here
	btfsc	DTon		; else check if its already dt mode
	goto	contTmr		; if yes continue wait

	btfsc	DispBack
	goto	NextMain2
	btfsc	DispMain2
	goto	NextMain
	movlw	B'00100000'	; else toggle display
	goto	FinDispSelect
NextMain2
	movlw	B'00110000'	; else toggle display
	goto	FinDispSelect
NextMain
	movlw	B'00010000'	; else toggle display
FinDispSelect

	xorwf	PORTC,F		; ~
	movlw	DTModeTime	; prepare wait constant
	movwf	DTTimer		; ~
	bsf		FirstOne
	bsf		DTon		; set dt mode flag
	call	VarToDisplay; instantly update changed temp to screen
contTmr
	decfsz	DTTimer,F	; wait once
	return
	movlw	MinLimDT
	movwf	MinLimit
	movlw	MaxLimDT
	movwf	MaxLimit
	clrf	BtnState	; make btnstate=0
	goto	InitBtnMode
	
Display ; displaying one digit on every call
; MSD(0) -> Middle(1) -> LSD(2)
	clrwdt
	bcf		Decimal
	bcf		LSBoutOK	
	btfsc	DigSinks,1		; check previous digit middle?
	goto	LSD			; if yes display LSD
	btfsc	DigSinks,0		; MSD?
	goto	middle		; if yes display middle
MSD	; if it was LSD then display MSD
	clrf	DigSinks	; reset digsinks
	bsf		DigSinks,0
    swapf	buffMSB,W	; leftmost digit
	andlw	0x0f
	goto    finDisplay
middle
	btfss	LSBout
	bsf		Decimal
	rlf		DigSinks,F
;	bsf		DigSinks,1
	movlw	0x0f		; middle digit
	andwf	buffMSB,W
	goto    finDisplay	 
LSD
	btfsc	LSBout
	bsf		LSBoutOK
	rlf		DigSinks,F
;	bsf		DigSinks,2
	swapf	buffLSB,W	; rightmost digit
finDisplay
	movwf	tmp
	movlw	B'11111000'	; clear all sinks
	andwf	PORTC,F		; ~
	movf	tmp,W
    call    BCDtoSEG    ; set the digit
    btfsc	Decimal		; if a decimal is needed
    iorlw	B'00000001'	; add to PORTB
    clrf	PORTB
    btfss	LSBoutOK
    movwf   PORTB		; light up segments
;    movf	PORTC,W
;    xorwf	DigSinks,W
;    movwf	PORTC
    movf	DigSinks,W	; load Sinks
	xorwf	PORTC,F		; set appropriate sink
    return
	
; input: address in W	output: value in W
Read_EEDATA
	bsf		STATUS,RP1		; select bank2 (10)
	movwf	EEADR			; w = address => data memory address to read
	bsf		STATUS,RP0		; bank3 (11)
	bcf		EECON1,EEPGD	; 0 = data memory
	bsf		EECON1,RD		; start read memory
	bcf		STATUS,RP0		; bank2 (10)
	movf	EEDATA,W		; W = value
	bcf		STATUS,RP1		; bank0 (00)
	return
	
; input: address is calculated from FSR, Value in INDF	
; called always after btn press when variable address is still in FSR
Write_EEPROM
	movlw	0x21			; move address of Dt to W
	subwf	FSR,W			; FSR - 0x22 = 0(dt) 1(Hi) 2(Lo)
	bsf		STATUS,RP1 		; Bank2 (10)
	movwf	EEADR 			; Data Memory Address to write
	movf 	INDF,W			; the value for the data
	movwf 	EEDATA 			; Data Memory Value to write
	bsf 	STATUS,RP0 		; Bank3 (11)
	bcf 	EECON1,EEPGD	; select DATA memory
	bsf 	EECON1,WREN 	; Enable writes
disableint
	bcf 	INTCON,GIE 		; Disable all Interrupts
	btfsc	INTCON,GIE
	goto	disableint
; required sequence
	movlw 	0x55
	movwf 	EECON2 			; Write 0x55
	movlw 	0xAA
	movwf 	EECON2 			; Write 0xAA
	bsf 	EECON1,WR 		; Set WR bit to begin write
; end required sequence
	bsf 	INTCON,GIE 		; Enable Interrupts
	sleep 					; Wait for interrupt to signal write complete
	bcf 	EECON1,WREN 	; Disable writes
	return
	
; reads both channels and decides which one to display
; runs on interrupt so is never interrupted
ScanAD	; does not work in buttons mode so its OK to use FSR
	;AN0=xx000xxx	Tank
	;AN1=xx001xxx   Tank2
	;AN4=xx100xxx	Solar
	btfsc   ADCON0,3
    goto    Main2nd		
	swapf	ADCON0,W	; get current channel
	andlw	B'00000010'	; ~
    goto    GetRegister
Main2nd
    movlw   B'00000100' ; 4th register

GetRegister
	addlw	MainTH  	; add MainTH location to W
	movwf	FSR

    bsf     ADCON0,ADON
	movlw	.125		; wait required aquisition time
	call	Wait		; ~
	nop
	bsf		ADCON0,GO	; set GO to start conversion
conversion				; continue until GO is cleared
	btfsc	ADCON0,GO	; ~
	goto	conversion	; ~
	movf	ADRESH,W
	movwf	INDF		; get result into first location
	incf	FSR,F
	BANK1
	movf	ADRESL,W
	BANK0
	movwf	INDF		; get L result into second location
	bcf     STATUS,C    ; multiply by two
    rlf     INDF,F
    decf    FSR,F
    rlf     INDF,F
    bcf     ADCON0,ADON
    
#ifdef OPTION_DT
    ; prepare for next conversion
    btfsc   ADCON0,3
    goto    Main2Code
    btfsc   ADCON0,5
    goto    DTCode
    movlw   B'00100000'
    goto    NextADCode   
DTCode
    movlw   B'00101000'
    goto    NextADCode   
Main2Code
    movlw   B'00001000'
NextADCode 	
 	xorwf	ADCON0,F	; ~        

#endif

	decfsz	UpdateInt,F	; update display every 0.5s
	return   
	 
VarToDisplay
#ifdef OPTION_DT
    btfsc	DispBack	; CASE backup values
	goto	BackValues	
    btfss	DispMain2	; CASE Main2 values
	goto	MainValues	; OTHERWISE load main values	
MainValues2
	movf	Main2TH,W   ; Main2 Display
	movwf	H_tmp		
	movf	Main2TL,W	; Main2 Display	
	goto	FinalizeDisp
BackValues
	movf	BackTH,W	; if yes load back values
	movwf	H_tmp
	movf	BackTL,W
	goto	FinalizeDisp	
MainValues
#endif
	movf	MainTH,W    ; Main Display
	movwf	H_tmp		
	movf	MainTL,W	; Main Display
FinalizeDisp
	movwf	L_tmp		; finalize load
	; decide display
; ****************************************
	btfss	FirstOne
	goto	Decide
	bcf		FirstOne
	movwf	TempTL
	movf	H_tmp,W	; load value
	movwf	H_Byte
Decide
	movlw	TempTH
	movwf	FSR
	call	compare10bit
	movwf	tmp	
	btfsc	EqualOn		; if values are equal then do nothing
	goto	Current	; ~
	btfss	tmp,1		; if current>TempTh
	goto	Smaller		; if not then handle smaller
	btfsc	TempTL,0	; if previous is not odd
	goto	Current		; if it is odd then do current only
	bsf		TempTL,0	; else make odd
	movf	TempTH,W	; load value
	movwf	H_Byte
	movf	TempTL,W
	movwf	L_Byte
	goto	GoUpdate
Smaller
	btfss	TempTL,0
	bsf		L_tmp,0	
Current	; save current value to temp and display
	movf	H_tmp,W	; load value
	movwf	TempTH
	movwf	H_Byte
	movf	L_tmp,W
	movwf	TempTL
	movwf	L_Byte
GoUpdate
; ****************************************
	call	B2_BCD		; do conversion
	; result R1 1xMSD (RJ) + R2 2XLSD
	; we need R1 2xMSD + R2 1XLSD (LJ)
	swapf	R1,W		;
	movwf	buffMSB		; ~
	swapf	R2,W		; ~
	movwf	buffLSB		; ~
	movlw	B'00001111'
	andwf	buffLSB,W
	addwf	buffMSB,F
	movlw	B'11110000'
	andwf	buffLSB,F

	movlw	UpdateIntTime; reset update time
	movwf	UpdateInt	; ~
	return

Wait	; small delay
	movwf	tmp
Next
	decfsz	tmp,F
	goto	Next
	return

ConfigOut
	call	ConfigRes
#ifdef OPTION_DT
	call	ConfigPump
#endif
	return
    
ConfigPump
; ----------------------------------
	btfsc	LastBack	; check last value of Back
	goto    KeepOnPump
	bcf		BackOn		; else clear
    goto   	ContinueConfigPump
KeepOnPump
	bsf		BackOn		; if it was on then keep turning on
ContinueConfigPump
; ----------------------------------

#ifdef OPTION_POOL
; if > Hi + 2 then PUMP always OFF
	movlw	MainTH		; load MainTH (Hot)
	movwf	FSR			; ~ into FSR	
	clrf	H_tmp
	movf	HiLimit,W	; load HiLimit
	movwf	L_tmp
	movlw	0x02
	addwf	L_tmp,F
	call	Mult10		; multiply by 10
	call	compare10bit
	movwf	tmp
	btfsc	tmp,0
	goto	TurnOff
; if < Hi continue regular
	clrf	H_tmp
	movf	HiLimit,W	; load HiLimit
	movwf	L_tmp
	call	Mult10		; multiply by 10
	call	compare10bit
	movwf	tmp
	btfss	tmp,1
	return
#endif

	movf	Main2TH,W	; save MainH to temp
	movwf	H_tmp		; ~
	movf	Main2TL,W	; load main L-byte
;	movwf	L_tmp		; ~ save to temp
; make main2T + PumpDiff degrees
	addlw	PumpDiff
	movwf	L_tmp 
	btfsc	STATUS,C
	incf	H_tmp,F
; end make
	movlw	BackTH		; load BackTH
	movwf	FSR			; ~ into FSR
; Comparison 1: BackT<Main2T => OFF	
	call	compare10bit
	movwf	tmp
	btfsc	tmp,1
	goto	TurnOff		; turn off and out
; Comparison 2: BackT>Main2T+DTLimit => ON
	movf	DTLimit,W
	movwf	L_tmp
    clrf    H_tmp       ; *new clear H_tmp for multiplication
	call	Mult10
    movf    Main2TH,W   ; *new load Main2H 
    addwf   H_tmp,F     ; *new add to H_tmp
	movf	Main2TL,W
	addwf	L_tmp,F
	btfss	STATUS,C	; if carry
	goto	compareNow	; else compare
	incf	H_tmp,F		; if there is carry then increment Hi
compareNow
	call	compare10bit ; compare
	movwf	tmp
	btfsc	tmp,0		 ; is Back greater?
	goto	TurnOn
	goto	ThirdCheck
TurnOff
	bcf		LastBack
; Comparison 3: solar<MinTankTmp => ON
ThirdCheck
	clrf	H_tmp
	movlw	MinTankTmp
	movwf	L_tmp
	call	compare10bit
	movwf	tmp
	btfsc	tmp,1
TurnOn
	bsf		LastBack
	return

ConfigRes
	btfsc	LastRes		; check last value of Res
	goto    KeepOnRes
	bcf		ResOn		; else clear
    goto   	ContinueConfigRes
KeepOnRes 
	bsf		ResOn		; if it was on then keep turning on
ContinueConfigRes
	movlw	MainTH		; load MainTH
	movwf	FSR			; ~ into FSR	
; HiLimit check
	clrf	H_tmp
	movf	HiLimit,W	; load HiLimit
	movwf	L_tmp
	call	Mult10		; multiply by 10
	call	compare10bit ; compare temp with HiLimit
	movwf	tmp
	btfss	tmp,0		; turn res off if HiLimit<MainT
	goto	LoCheck	
#ifdef OPTION_REVRES
	bsf		LastRes
#else	
    bcf		LastRes		; turn res off
#endif	
	return
LoCheck
	clrf	H_tmp
	movf	LoLimit,W	; load LoLimit
	movwf	L_tmp
	call	Mult10		; multiply by 10
	call	compare10bit; compare temp with LoLimit
	movwf	tmp
	btfsc	tmp,1		; turn res on if LoLimit>MainT
#ifdef OPTION_REVRES
	bcf		LastRes
#else	
	bsf		LastRes		; turn res off
#endif	
#ifdef OPTION_EXTRA
	call 	CompareSolar
	btfsc	tmp,0
    bcf		LastRes		; turn res off	(NO REVERSE)
#endif		
	return

; multiply by ten
Mult10
	movlw	.10
	movwf	count
	clrw
Loop10
	addwf	L_tmp,W
	btfsc	STATUS,C
	incf	H_tmp,F
	decfsz	count,F
	goto	Loop10
	movwf	L_tmp	
	return

; compare tmp with INDF
;(in H_tmp,L_tmp,inout EqualOn)
compare10bit
	bcf		EqualOn
	movf	INDF,W		; load INDF (H)
	subwf	H_tmp,W		; H_tmp - INDF H
	btfsc	STATUS,Z	; if they are equal
	goto	continueComp; continue with lower byte
	btfss	STATUS,C	; set = positive
	retlw	0x01		; 1: tmp < INDF
	retlw	0x02		; 2: tmp > INDF
continueComp
	incf	FSR,F		; load next
	movf	INDF,W		; INDF (L)
	decf	FSR,F		; restore INDF (H)
	subwf	L_tmp,W		; L_tmp - INDF L
	btfsc	STATUS,Z	; if they are equal
	bsf		EqualOn		; set equality flag
	btfss	STATUS,C	; set = positive or equal
	retlw	0x01		; 1: tmp <  INDF
	retlw	0x02		; 2: tmp >= INDF

; Save an extra limit in order not to turn on RES in case Solar is enough	
#ifdef OPTION_EXTRA
SaveSSLimit
	movf	DTLimit,W
	movwf	SSLimit
	rlf		SSLimit,F
	movf	LoLimit,W	
	addwf	SSLimit,F	
	return

CompareSolar
	movlw	BackTH		; load BackTH
	movwf	FSR			; ~ into FSR	
	clrf	H_tmp
	movf	SSLimit,W	; load LoLimit
	movwf	L_tmp
	call	Mult10		; multiply by 10
	call	compare10bit; compare temp with LoLimit
	movwf	tmp
	return
#endif	

;*******************************************************************
;  Binary to BCD Converter (Microchip AppNotes)
;
;  Input: Two bytes of RAM; L_Byte, H_Byte
;  Output: Three bytes of RAM; R0, R1, R2
;
;  Usage: Put value into L_Byte and H_Byte, call B2_BCD, result in
;         R2 is LSD and R1 is MSD: Four digits (nibbles) in 2 bytes
;         Lower 4 bits of LSD are least sig. digit, upper 4 are 2nd
;         digit.
;*******************************************************************
B2_BCD
	swapf	FSR,W			; save FSR
	movwf	FSRTemp			; ~
	
	bcf     STATUS,0        ;Clear carry bit
	movlw   .16
	movwf   count           ;Set counter to 16
	clrf    R0
	clrf    R1
	clrf    R2
loop16
	rlf     L_Byte,F
	rlf     H_Byte,F
	rlf     R2,F
	rlf     R1,F
	rlf     R0,F
	decfsz  count,F
	goto    adjDEC
	swapf	FSRTemp,W	; restore FSR
	movwf	FSR			; ~
	retlw   0

adjDEC
	movlw   R2
	movwf   FSR
	call    adjBCD
	movlw   R1
	movwf   FSR
	call    adjBCD
	movlw   R0
	movwf   FSR
	call    adjBCD
	goto    loop16

adjBCD
	movlw   3
	addwf   0,W
	movwf   tmp
	btfsc   tmp,3        ;Test if result > 7
	movwf   0
	movlw   30
	addwf   0,W
	movwf   tmp
	btfsc   tmp,7        ;Test if result > 7
	movwf   0             ;Save as MSD
	retlw   0

;*******************************************************************
	
	end
