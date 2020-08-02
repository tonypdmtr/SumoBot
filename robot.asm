;*******************************************************************************
;* Program   : SumoBot
;* Programmer: adhsoft0@gmail.com       Original
;*           : Tony Papadimitriou       Conversion to ASM8 and English
;* Purpose   : Reads two line sensors and one object proximity sensor
;*           : (from the robot): Don't leave the ring. Spin until you find an
;*           : opponent. Ram it. If not, move and turn.
;* Language  : Motorola/Freescale/NXP HC08/9S08 Assembly Language (aspisys.com/ASM8)
;* Status    : Licensed under GPLv3
;* History   : 20.08.02 Converted to ASM8 (http://www.aspisys.com/asm8.htm)
;*           :          Translated to English
;*******************************************************************************

                    #ListOff
          #ifexists qt4.inc
                    #Uses     qt4.inc
          #else
                    #Uses     /pemicro/ics08qtqyz/qtqy_registers.inc
          #endif
                    #ListOn

;*******************************************************************************
; Definition of origins of the memories to be used
;*******************************************************************************

RAM                 def       $0080
ROM                 def       $EE00
Vreset              def       $FFFE

TMOD_VALUE          equ       65531

;*******************************************************************************
; Definition of Variables / Constants - can be bit or registers -
;*******************************************************************************

          ;-------------------------------------- ; PORTA bits
SFL                 equ       0
SFR                 equ       1

STL                 equ       2
STR                 equ       3
          ;-------------------------------------- ; PORTB bits
M1A                 equ       0
M1B                 equ       1
M2A                 equ       2
M2B                 equ       3

SPA                 equ       4

;*******************************************************************************
                    #RAM                          ; RAM startup specification
;*******************************************************************************
                    org       RAM

;*******************************************************************************
; Reserve RAM spaces and assign labels or numbers
;*******************************************************************************

count               rmb       2                   ; counts the amount of timer overflows
time                rmb       2                   ; used to indicate when to wait (how many overflows)
random_number       rmb       2                   ; this will be used to generate a random number

;*******************************************************************************
                    #ROM                          ; Specification of the start of the program, which is stored in the ROM, from the indicated address
;*******************************************************************************
                    org       ROM

;*******************************************************************************
; Configuration of registers, such as that of the OSC, THE CONFIG1 - COP -
;*******************************************************************************

Start               proc
                    mov       #$01,CONFIG1        ; internal oscillator, turns off COP
          ;--------------------------------------
          ; Configuration of port registers and other modules
          ;
          ; PTA0: SFL
          ; PTA1: SFR
          ;
          ; PTA2: STL
          ; PTA3: STR
          ;
          ;
          ; PTB0: M1A
          ; PTB1: M1B
          ; PTB2: M2A
          ;
          ; PTB4: SPA
          ;--------------------------------------
                    mov       #%00001111,DDRB     ; 4 inputs, 4 outputs
                    clr       DDRA                ; configure all the PTA as input
                    clr       PORTB               ; clean the port
                    mov       #%00010000,PTBPUE   ; pull up SPA
          ;-------------------------------------- ; preparation of the TIM
                    clr       count               ; clean count

                    bset      5,TSC               ; turn off timer
                    mov       #$36,TSC            ; configure timer and prescaler / 64
                    lda       #]TMOD_VALUE
                    sta       TMODH               ; upload TMODH comparison record
                    lda       #[TMOD_VALUE        ; to get 5ms flag delay (251)
                    sta       TMODL               ; upload TMODL comparison record
                    bclr      5,TSC               ; turn on timer
;                   bra       MainLoop

;*******************************************************************************

MainLoop            proc                          ; reset robot
                    jsr       Stop                ; stop the robot in case of "reset"
                    mov       #2,time             ; wait two "seconds"
                    jsr       Timer               ; wait
                    brset     SFL,PORTA,MainLoop  ; jump if there is a line ahead
                    brset     SFR,PORTA,MainLoop  ; jump if there is a line ahead
;                   bra       Scanner             ; look for enemy

;*******************************************************************************
; "Scanner": search if there is a nearby enemy turning on itself 360 degrees
;*******************************************************************************

Scanner             proc
                    clr       count
                    mov       #$06,TSC            ; prescaler /64
                    lda       TCNTL               ; load timer counter
                    sta       random_number       ; save it in random number
                    cmpa      #$80
                    bhs       _1@@                ; rotate counterclockwise if less than 128
                                                  ; if not, hourly
                    jsr       RotateCCW
                    bra       Loop@@              ; follow

_1@@                jsr       RotateCW

Loop@@              brset     7,TSC,_2@@          ; ask if the timer is overflowing
                    brset     SPA,PORTB,Attack    ; check near enemy
                    brset     SFL,PORTA,MainLoop  ; reset with any sensed line
                    brset     SFR,PORTA,MainLoop
                    bra       Loop@@              ; if there is no enemy repeat

_2@@                inc       count               ; increment count
                    lda       #5                  ; charge 5
                    cbeq      count,Finder        ; compare and if count is 5, go to Finder
                    jsr       ResetFlag
                    bra       Loop@@              ; If not, keep counting

;*******************************************************************************
; "Attack". If it detects a line (wins), it is reset
; If you lose the enemy, call Scanner
;*******************************************************************************

Attack              proc
                    jsr       MoveAlong
                    jsr       ResetFlag           ; I do not remember
Loop@@              brset     7,TSC,Cont@@
                    brset     SFL,PORTA,Turn      ; win
                    brset     SFR,PORTA,Turn
          ;-------------------------------------- ; time delay
                    lda       #$FF
                    dbnza     *
          ;-------------------------------------- ; / end delay
                    brclr     SPA,PORTB,Scanner   ; lost the enemy, rummage
                    bra       Loop@@              ; keep pushing (without turning off engines)

Cont@@              jsr       ResetFlag
                    bra       Loop@@

;*******************************************************************************
; "Finder": Moves the robot forward, if it detects a line it turns randomly (using timer)
; If it detects an enemy it goes to "Attack".
;*******************************************************************************

Finder              proc
                    bsr       MoveAlong
          ;--------------------------------------
          ; go see if you come across the line forward; emptying TSC
          ;--------------------------------------
_1@@                brset     SFL,PORTA,Turn      ; ran into front line, turn
                    brset     SFR,PORTA,Turn      ; ran into front line, turn
                    brset     SPA,PORTB,Attack    ; ran into front line, attack
?2                  brset     7,TSC,_3@@          ; ask if the timer is overflowing
                    bra       _1@@                ; repeat

_3@@                jsr       ResetFlag           ; if the timer overflows, reset it. (we will use it to generate random number)
                    bra       _1@@

;*******************************************************************************

Turn                proc
                    lda       TCNTL               ; save 1st timer number
                    sta       random_number       ; at random_number
                    bsr       Stop
                    mov       #$03,TSC            ; prescaler /8
                    bsr       BackOff
                    mov       #$05,TSC            ; prescaler /32
                    mov       #1,time             ; hardly wait
                    bsr       Timer               ; wait
                    mov       #$06,TSC            ; prescaler /64
                    bsr       Stop

                    lda       random_number
                    cmpa      #128
                    bhs       _1@@                ; rotate counterclockwise if less than 128
                                                  ; if not, hourly
                    bsr       RotateCCW
                    bra       _2@@                ; follow

_1@@                bsr       RotateCW
                    lda       random_number
                    sub       #112                ; subtract 100 (so that it turns to one side or the other with the same limits)
                    sta       random_number       ; save the result
_2@@                mov       #0,TSC              ; change prescaler to / 1
                    lda       random_number
                    sta       time                ; and put it on time
          ;--------------------------------------
          ; wait time but checking enemy
          ;--------------------------------------
                    clr       count               ; empty count
                    bsr       ResetFlag           ; clean the Flag from overflow
_3@@                brset     7,TSC,_4@@          ; ask if the timer is overflowing
                    brset     SPA,PORTB,Attack    ; enemy? attack
                    brset     SFL,PORTA,Turn      ; line? go back and turn again
                    brset     SFR,PORTA,Turn      ; line? go back and turn again
                    bra       _3@@

_4@@                bsr       ResetFlag           ; clean the Flag from overflow
                    inc       count               ; increase overflows
                    lda       count
                    cbeq      time,_5@@           ; Is the right time reached? count = time?
                    bra       _3@@                ; if I don't get there, keep counting ...

_5@@                mov       #$06,TSC            ; reset prescaler to / 64
                    bsr       MoveAlong           ; advance again
                    bra       ?2                  ; return

;*******************************************************************************
; functions - calls (jsr / bsr)
;*******************************************************************************

RotateCCW           proc                          ; rotate counterclockwise
                    bset      M1B,PORTB           ; M1B
                    bclr      M1A,PORTB
                    bset      M2A,PORTB           ; M2A
                    bclr      M2B,PORTB
                    rts

;*******************************************************************************

RotateCW            proc                          ; rotate clockwise
                    bset      M1A,PORTB           ; M1A
                    bclr      M1B,PORTB
                    bset      M2B,PORTB           ; M2B
                    bclr      M2A,PORTB
                    rts

;*******************************************************************************

MoveAlong           proc
                    bset      M1A,PORTB           ; M1A
                    bclr      M1B,PORTB
                    bset      M2A,PORTB           ; M2A
                    bclr      M2B,PORTB
                    rts

;*******************************************************************************

BackOff             proc
                    bset      M1B,PORTB           ; M1B
                    bclr      M1A,PORTB
                    bset      M2B,PORTB           ; M2B
                    bclr      M2A,PORTB
                    rts

;*******************************************************************************

Stop                proc
                    bclr      M1A,PORTB           ; M10
                    bclr      M1B,PORTB
                    bclr      M2A,PORTB           ; M20
                    bclr      M2B,PORTB
                    rts

;*******************************************************************************

ResetFlag           proc
                    bclr      7,TSC               ; clean the Flag from overflow
                    rts

;*******************************************************************************

Timer               proc                          ; function that waits an amount of time depending on the time variable
                    clr       count               ; empty count
                    bclr      7,TSC               ; clean the Flag from overflow
Loop@@              brset     7,TSC,_1@@          ; ask if the timer is overflowing
                    bra       Loop@@
_1@@                bclr      7,TSC               ; clean the Flag from overflow
                    inc       count               ; increases amount of overflows
                    lda       count
                    cbeq      time,Done@@         ; Is the right time reached? count = time?
                    bra       Loop@@              ; if I don't get there, keep counting ...
Done@@              equ       :AnRTS              ; if I arrive, exit the timer function

;*******************************************************************************
                    #VECTORS
;*******************************************************************************
                    org       Vreset
                    dw        Start
