///////////////////////////////////////////////////////////////
////////// THOMAS JOHNSTON - z5017910             /////////////
//////////	LIFT PROJECT  -   COMP2121            /////////////
///////////////////////////////////////////////////////////////
; PB0 - Close button
; PB1 - Open button
; * - emergency button

.include "m2560def.inc"
;###########
;# .DEF    #
;###########
.def row = r16              ; current row number
.def col = r17              ; current column number
.def rmask = r18            ; mask for current row during scan
.def cmask = r19            ; mask for current column during scan
.def temp = r20 
.def digit = r21                    
.def currentFloor = r22     
.def debounceFlag = r23		; debounce for PB0, PB1 and keypad (100ms)
.def seconds = r24			; open/close - 1 sec
							; keep open - 3 sec
							; travel - 2 sec
;#############
;# .EQU      #
;#############
;##### Keypad #####
.equ PORTLDIR = 0xF0        ; PH7-4: output, PH3-0, input
.equ INITCOLMASK = 0xEF     ; scan from the rightmost column,
.equ INITROWMASK = 0x01     ; scan from the top row
.equ ROWMASK = 0x0F         ; for obtaining input from Port L
;##### LCD ########
.equ LCD_RS = 7
.equ LCD_E = 6
.equ LCD_RW = 5
.equ LCD_BE = 4
.equ F_CPU = 16000000
.equ DELAY_1MS = F_CPU / 4 / 1000 - 4
;#############
;# .MACRO    #
;#############
; clear a word (2 bytes) in memory
.macro clear2
    ldi YL, low(@0)    ; load the memory address to Y
    ldi YH, high(@0)
    clr temp
    st Y+, temp        ; clear the two bytes at @0 in SRAM
    st Y, temp
.endmacro
; clear a byte (1 byte) in a memory
.macro clear
    ldi YL, low(@0)    ; load the memory address to Y
    clr temp
    st Y, temp         ; clear the byte at @0 in SRAM
.endmacro
;#### LCD macros ###
.macro lcd_set
	sbi PORTA, @0
.endmacro
.macro lcd_clr
	cbi PORTA, @0
.endmacro
.macro do_lcd_command
	ldi temp, @0
	rcall lcd_command
	rcall lcd_wait
.endmacro
.macro do_lcd_data
	ldi temp, @0
	rcall lcd_data
	rcall lcd_wait
.endmacro
.macro do_lcd_rdata
	mov temp, @0
	subi temp, -'0'
	rcall lcd_data
	rcall lcd_wait
.endmacro

;#############
;# DSEG   #
;#############
.dseg
SecondCounter: .byte 2      ; Second counter for determining if one second has passed
DebounceCounter: .byte 2    ; Debounce counter for determining if 100ms have passed
CurrentPattern: .byte 2
 
;#############
;# CSEG #
;#############
.cseg
.org 0x0000
    jmp RESET
.org INT0addr
    jmp EXT_INT1		 ; interrupt for PB0
.org INT1addr
    jmp EXT_INT0		 ; interrupt for PB1
.org OVF0addr
    jmp Timer0OVF        ; interrupt for Timer0 overflow
	jmp DEFAULT          ; default service for all other interrupts
DEFAULT:  reti           ; no service, return from interrupt

RESET:
    ldi temp, low(RAMEND)  ; initialize the stack
    out SPL, temp
    ldi temp, high(RAMEND)
    out SPH, temp
	
	; KEYPAD setup
    ldi temp, PORTLDIR     ; PB7:4/PB3:0, out/in
    sts DDRL, temp         ; PORTL is input  (Keypad)
    ser temp               ; set the temp register; PORTC is output (LEDs)
    out DDRC, temp			
    out PORTC, temp

	; PB0 & PB1 setup
	ldi temp, (2 << ISC10) | (2 << ISC00) ; setting the interrupts for falling edge
	sts EICRA, temp                       ; storing them into EICRA 
	in temp, EIMSK                        ; taking the values inside the EIMSK  
	ori temp, (1<<INT0) | (1<<INT1)       ; oring the values with INT0 and INT1  
	out EIMSK, temp                       ; enabling interrput0 and interrupt1 (PB0/PB1)
	sei									  ; enabling the global interrupt..(MUST)

	; LED Setup
    ser temp1                ; PORTC is output
    out DDRC, temp1
    out PORTC, temp1         ; out PORTC all LEDs lit up
    out DDRG, temp1
    out PORTG, temp1

    ser temp1
    out DDRF, temp1
    out DDRA, temp1
    clr temp1
    out PORTF, temp1
    out PORTA, temp1

	; LCD setup
	ser temp
	out DDRF, temp
	out DDRA, temp
	clr temp
	out PORTF, temp
	out PORTA, temp
	do_lcd_command 0b00111000 ; 2x5x7
	rcall sleep_5ms
	do_lcd_command 0b00111000 ; 2x5x7
	rcall sleep_1ms
	do_lcd_command 0b00111000 ; 2x5x7
	do_lcd_command 0b00111000 ; 2x5x7
	do_lcd_command 0b00001000 ; display off?
	do_lcd_command 0b00000001 ; clear display
	do_lcd_command 0b00000110 ; increment, no display shift
	do_lcd_command 0b00001110 ; Cursor on, bar, no blink

	rjmp main

//////////////////
/// Interrupts ///
//////////////////
Timer0OVF:             ; interrupt subroutine to Timer0
    in temp, SREG
    push temp          ; Prologue starts.
    push YH            ; Save all conflict registers in the prologue.
    push YL
	push r27
	push r26
    push r25
    push r24

	; first counter - 100ms debounce counter
	checkFlagSet:				; if flag is set - run the debounce timer
		cpi debounceFlag, 1    
		breq newHundred
								; otherwise - don't need the debounce timer
		rjmp newSecond          ; go to second counter

	newHundred:			        ; if flag is set continue counting until 100 milliseconds
		lds r26, DebounceCounter
    	lds r27, DebounceCounter+1
    	adiw r27:r26, 1         ; Increase the temporary counter by one.

    	cpi r26, low(400)       ; pre-scale = 128; f = 10^6 Hz; 1 sec = 1000ms; 100ms = 10^6/128/10 = 781
    	ldi temp, high(400)
    	cpc temp, r27
    	brne notHundred			; 100 milliseconds have not passed

		clr debounceFlag 		; once 100 milliseconds have passed, set the debounceFlag to 0
	   	clear2 DebounceCounter	; Reset the debounce counter.
		clr r26
		clr r27					; Reset the debounce counter.

	newSecond:				    ; second counter - for displaying the pattern 3 times
	    lds r24, SecondCounter	; Load the value of the temporary counter.
    	lds r25, SecondCounter+1
    	adiw r25:r24, 1 		; Increase the temporary counter by one.

    	cpi r24, low(7812)      ; Check if (r25:r24) = 7812 ; 7812 = 10^6/128
    	ldi temp, high(7812)    ; 7812 = 10^6/128
    	cpc r25, temp
    	brne notSecond

		;cpi pattern_shown_once, 1	; if pattern has been shown - hide
		;breq hidePattern		
		
		;################ ADD func
		/*showPattern:
		
			out PORTC, curr_pattern			; show the pattern
			ldi pattern_shown_once, 1
		
		checkPattern:

			cpi bit_counter, 8			; if the whole new pattern has been set	
			breq reloadPattern			; reload it: copy to curr_pattern*/

		clear2 SecondCounter       ; Reset the temporary counter.                    
		inc seconds
		cpi seconds, 6
		brne newSecond				; if not - count a new second

		/*; if 6 seconds have passed clear pattern
		clr curr_pattern
		;ldi curr_pattern, 0b11001100
		out PORTC, curr_pattern
		*/
		clr seconds

    rjmp EndIF

; supplementary functions
notSecond: 						; Store the new value of the temporary counter.
    sts SecondCounter, r24
    sts SecondCounter+1, r25
	rjmp EndIF 

notHundred: 					; Store the new value of the debounce counter.
	sts DebounceCounter, r26
	sts DebounceCounter+1, r27
	rjmp EndIF

EndIF:
    pop r24					; Epilogue starts;
    pop r25					; Restore all conflict registers from the stack.
	pop r26
	pop r27
    pop YL
    pop YH
    pop temp
    out SREG, temp
    reti					; Return from the interrupt.

EXT_INT0:			; subroutine for push button 0 (CLOSE DOOR)
    in temp, SREG	; Prologue starts.
    push temp       ; Save all conflict registers in the prologue.
	cpi debounceFlag, 1		; if the button is still debouncing, ignore the interrupt
	breq END_INT0	

	ldi debounceFlag, 1		; set the debounce flag

	;cpi bit_counter, 8			; if bit counter is not 8 bit yet
	;brlt writeZero				; write 0 in the pattern and increase bit counter

	rjmp END_INT0

/*writeZero:
	; push button 0 wants to put 0 in LSB
	lsl new_pattern				; shift the pattern to the left by 1 bit
	ldi temp, 0b11111110		; load the complement of 00000001
	and new_pattern, temp 		; and the new pattern and r19

	inc bit_counter
	;ldi temp, 0b11101111
	;out PORTC, new_pattern			; write the pattern
	rjmp END_INT0*/

END_INT0:
    pop temp		; Epilogue of push button 0
    out SREG, temp
    reti            ; Return from the interrupt.

EXT_INT1:			; subroutine for push button 1 (OPEN DOOR) (Bonus Mark)
    in temp, SREG	; Prologue starts.
    push temp       ; Save all conflict registers in the prologue.
	; debounce check
	cpi debounceFlag, 1		; if the button is still debouncing, ignore the interrupt
	breq END_INT1

	ldi debounceFlag, 1		; set the debounce flag

	;cpi bit_counter, 8			; if bit counter is not 8 bit yet
	;brlt writeOne				; write 1 in the pattern and increase bit counter

	rjmp END_INT1

/*writeOne:
	; push button 1 wants to put 1 in LSB
	lsl new_pattern				; shift the pattern to the left by 1 bit
	ldi temp, 0b00000001
	or new_pattern, temp 		; or the new pattern and temp

	inc bit_counter
	rjmp END_INT1*/	

END_INT1:			
	;ser currentFloor
	;out DDRC, currentFloor 		; set Port C for output

	pop temp					; Epilogue of push button 1
    out SREG, temp
    reti						; Return from the interrupt.

main:
	clear2 DebounceCounter		; Initialize the debounce counter to 0
    clear2 SecondCounter		; Initialize the second counter to 0

	ldi temp, 0b00000000
    out TCCR0A, temp
    ldi temp, 0b00000010
    out TCCR0B, temp        ; Prescaling value=8
    ldi temp, 1<<TOIE0      ; = 128 microseconds
    sts TIMSK0, temp        ; T/C0 interrupt enable
    sei                     ; Enable global interrupt

initKeypad:
    ldi cmask, INITCOLMASK		; initial column mask
    clr col						; initial column
	clr temp
	clr digit

	ldi debounceFlag, 1			; otherwise set the flag now to init the debounce

colloop:
    cpi col, 4
    breq main               ; If all keys are scanned, repeat.
    sts PORTL, cmask        ; Otherwise, scan a column.
    ldi digit, 0xFF         ; Slow down the scan operation.
delay:
    dec digit
    brne delay              ; until digit is zero? - delay
    lds digit, PINL         ; Read PORTL
    andi digit, ROWMASK     ; Get the keypad output value
    cpi digit, 0xF          ; Check if any row is low
    breq nextcol            ; if not - switch to next column
	; If yes, find which row is low
    ldi rmask, INITROWMASK  ; initialize for row check
    clr row
; and going into the row loop
rowloop:
    cpi row, 4              ; is row already 4?
    breq nextcol            ; the row scan is over - next column
    mov temp, digit
    and temp, rmask        ; check un-masked bit
    breq convert            ; if bit is clear, the key is pressed
    inc row                 ; else move to the next row
    lsl rmask
    jmp rowloop
nextcol:                    ; if row scan is over
     lsl cmask				; logic left shift cmask
     inc col                ; increase col value
     jmp colloop            ; go to the next column
convert:
	cpi debounceFlag, 1		; if the button is still debouncing, ignore the keypad
	breq initKeypad	

    cpi col, 3              ; If the pressed key is in col 3
    breq letters            ; we have letter
    ; If the key is not in col 3 and
    cpi row, 3              ; if the key is in row 3,
    breq symbols            ; we have a symbol or 0
    mov digit, row          ; otherwise we have a number 1-9
    lsl digit
    add digit, row
    add digit, col          ; digit = row*3 + col
	subi digit, -1
    rjmp convert_end
letters:
    ldi digit, 'A'
    add digit, row          ; Get the ASCII value for the key (digit is now the ASCII val of the key)
	rjmp convert_end
symbols:
    cpi col, 0              ; Check if we have a star
    breq star				
    cpi col, 1              ; or if we have zero
    breq zero
    ldi digit, '#'          ; if not we have hash
	clr digit				; TEMP: not handling the hash now
    jmp convert_end
star:
    ldi digit, '*'          ; set to star
    rjmp emergency
zero:
    ldi digit, 0            ; set to zero in binary
	rjmp convert_end
convert_end:
	do_lcd_command 0b00000001   ; clear display
    out PORTC, digit            ; write value to PORTC LEDs

	do_lcd_rdata digit	        ; display the accumulator data every time
	do_lcd_command 0b11000000	; break to the next line
	do_lcd_rdata digit	        ; output current number

    rjmp initKeypad             ; restart the main loop

emergency:					    ; show "emergency 000" etc.
	rjmp convert_end

;################################################
;####Send a command to the LCD (lcd register)####
;################################################

lcd_command:
	out PORTF, temp
	rcall sleep_1ms
	lcd_set LCD_E
	rcall sleep_1ms
	lcd_clr LCD_E
	rcall sleep_1ms
	ret

lcd_data:
	out PORTF, temp
	lcd_set LCD_RS
	rcall sleep_1ms
	lcd_set LCD_E
	rcall sleep_1ms
	lcd_clr LCD_E
	rcall sleep_1ms
	lcd_clr LCD_RS
	ret

lcd_wait:
	push temp
	clr temp
	out DDRF, temp
	out PORTF, temp
	lcd_set LCD_RW
lcd_wait_loop:
	rcall sleep_1ms
	lcd_set LCD_E
	rcall sleep_1ms
	in temp, PINF
	lcd_clr LCD_E
	sbrc temp, 7
	rjmp lcd_wait_loop
	lcd_clr LCD_RW
	ser temp
	out DDRF, temp
	pop temp
	ret

sleep_1ms:
	push r24
	push r25
	ldi r25, high(DELAY_1MS)
	ldi r24, low(DELAY_1MS)
delayloop_1ms:
	sbiw r25:r24, 1
	brne delayloop_1ms
	pop r25
	pop r24
	ret

sleep_5ms:
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	ret	