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
;*           : 21.04.18 Minor refactoring and optimizations
;*           : 21.04.30 Minor refactoring and optimizations
;*******************************************************************************

                    #ListOff
                    #Uses     qt4.inc
                    #ListOn
                    #MCF

;*******************************************************************************
; Definition of origins of the memories to be used
;*******************************************************************************

RAM                 def       $0080
ROM                 def       $EE00
Vreset              def       $FFFE

TMOD_VALUE          equ       65531

;*******************************************************************************
; Macros
;*******************************************************************************

ResetOverflowFlag   macro
                    bclr      7,TSC
                    endm

;*******************************************************************************
; Pin definitions
;*******************************************************************************

SFL                 @pin      PORTA,0
SFR                 @pin      PORTA,1

;STL                @pin      PORTA,2
;STR                @pin      PORTA,3

M1A                 @pin      PORTB,0
M1B                 @pin      PORTB,1
M2A                 @pin      PORTB,2
M2B                 @pin      PORTB,3

SPA                 @pin      PORTB,4

;*******************************************************************************
                    #RAM      RAM
;*******************************************************************************

count               rmb       2                   ;counts the amount of timer overflows
time                rmb       2                   ;used to indicate when to wait (how many overflows)
random_number       rmb       2                   ;this will be used to generate a random number

;*******************************************************************************
                    #ROM      ROM                 ;Specification of the start of the program, which is stored in the ROM, from the indicated address
;*******************************************************************************

;*******************************************************************************
; Configuration of registers, such as that of the OSC, the CONFIG1 - COP -
;*******************************************************************************

Start               proc
                    mov       #$01,CONFIG1        ;internal oscillator, turns off COP
          ;--------------------------------------
                    @Off      M1A,M1B,M2A,M2B     ;configure outputs (low initially)
                    @pullup   SPA                 ;pull up SPA
;                   @input    SFL,SFR,SPA         ;inputs (reset default)
          ;-------------------------------------- ;preparation of the TIM
                    clr       count               ;clean count

                    bset      5,TSC               ;turn off timer
                    mov       #$36,TSC            ;configure timer and prescaler / 64
                    mov       #]TMOD_VALUE,TMODH  ;upload TMOD comparison record
                    mov       #[TMOD_VALUE,TMODL  ;to get 5ms flag delay (251)
                    bclr      5,TSC               ;turn on timer
;                   bra       MainLoop

;*******************************************************************************

MainLoop            proc                          ;reset robot
Loop@@              jsr       Stop                ;stop the robot in case of "reset"
                    mov       #2,time             ;wait two "seconds"
                    jsr       Timer               ;wait
                    brset     SFL,Loop@@          ;jump if there is a line ahead
                    brset     SFR,Loop@@          ;jump if there is a line ahead
;                   bra       Scanner             ;look for enemy

;*******************************************************************************
; "Scanner": search if there is a nearby enemy turning on itself 360 degrees
;*******************************************************************************

Scanner             proc
                    clr       count
                    mov       #$06,TSC            ;prescaler /64
                    mov       TCNTL,random_number ;timer counter in random number
                    cmpa      #$80
                    bhs       _1@@                ;rotate counterclockwise if less than 128
                                                  ;if not, hourly
                    jsr       RotateCCW
                    bra       Loop@@              ;follow

_1@@                jsr       RotateCW

Loop@@              brset     7,TSC,_2@@          ;ask if the timer is overflowing
                    brset     SPA,Attack          ;check near enemy
                    brset     SFL,MainLoop        ;reset with any sensed line
                    brset     SFR,MainLoop
                    bra       Loop@@              ;if there is no enemy repeat

_2@@                inc       count               ;increment count
                    lda       #5                  ;charge 5
                    cbeq      count,Finder        ;if count is 5, go to Finder
                    @ResetOverflowFlag
                    bra       Loop@@              ;if not, keep counting

;*******************************************************************************
; "Attack". If it detects a line (wins), it is reset
; If you lose the enemy, call Scanner
;*******************************************************************************

Attack              proc
                    jsr       MoveAlong
                    @ResetOverflowFlag            ;(I do not remember)
Loop@@              brset     7,TSC,Cont@@
                    brset     SFL,Turn            ;win
                    brset     SFR,Turn
          ;-------------------------------------- ;time delay
                    clra
                    dbnza     *
          ;-------------------------------------- ;end delay
                    brclr     SPA,Scanner         ;lost the enemy, rummage
                    bra       Loop@@              ;keep pushing (without turning off engines)

Cont@@              @ResetOverflowFlag
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
Loop@@              brset     SFL,Turn            ;ran into front line, turn
                    brset     SFR,Turn            ;ran into front line, turn
                    brset     SPA,Attack          ;ran into front line, attack
?Finder             brclr     7,TSC,Loop@@        ;ask if the timer is overflowing
                    @ResetOverflowFlag            ;if the timer overflows, reset it. (we will use it to generate random number)
                    bra       Loop@@

;*******************************************************************************

Turn                proc
                    mov       TCNTL,random_number ;save 1st timer number at random_number
                    bsr       Stop
                    mov       #$03,TSC            ;prescaler /8
                    bsr       BackOff
                    mov       #$05,TSC            ;prescaler /32
                    mov       #1,time             ;hardly wait
                    bsr       Timer               ;wait
                    mov       #$06,TSC            ;prescaler /64
                    bsr       Stop

                    lda       random_number
                    cmpa      #128
                    bhs       _1@@                ;rotate counterclockwise if less than 128
                                                  ;if not, hourly
                    bsr       RotateCCW
                    bra       _2@@                ;follow

_1@@                bsr       RotateCW
                    lda       random_number
                    sub       #112                ;subtract 100 (so that it turns to one side or the other with the same limits)
                    sta       random_number       ;save the result
_2@@                clr       TSC                 ;change prescaler to /1
                    mov       random_number,time  ;and put it on time
          ;-------------------------------------- ;wait time but checking enemy
                    clr       count               ;empty count
                    @ResetOverflowFlag
_3@@                brset     7,TSC,_4@@          ;ask if the timer is overflowing
                    brset     SPA,Attack          ;enemy? attack
                    brset     SFL,Turn            ;line? go back and turn again
                    brset     SFR,Turn            ;line? go back and turn again
                    bra       _3@@

_4@@                @ResetOverflowFlag
                    inc       count               ;increase overflows
                    lda       count
                    cmpa      time                ;Is the right time reached? count = time?
                    bne       _3@@                ;if I don't get there, keep counting ...

                    mov       #$06,TSC            ;reset prescaler to / 64
                    bsr       MoveAlong           ;advance again
                    bra       ?Finder             ;return

;*******************************************************************************
; functions - calls (jsr / bsr)
;*******************************************************************************

RotateCCW           proc                          ;rotate counterclockwise
                    bset      M1B
                    bclr      M1A
                    bset      M2A
                    bclr      M2B
                    rts

;*******************************************************************************

RotateCW            proc                          ;rotate clockwise
                    bset      M1A
                    bclr      M1B
                    bset      M2B
                    bclr      M2A
                    rts

;*******************************************************************************

MoveAlong           proc
                    bset      M1A
                    bclr      M1B
                    bset      M2A
                    bclr      M2B
                    rts

;*******************************************************************************

BackOff             proc
                    bset      M1B
                    bclr      M1A
                    bset      M2B
                    bclr      M2A
                    rts

;*******************************************************************************

Stop                proc
                    bclr      M1A                 ;M10
                    bclr      M1B
                    bclr      M2A                 ;M20
                    bclr      M2B
                    rts

;*******************************************************************************

Timer               proc                          ;function that waits an amount of time depending on the time variable
                    clr       count               ;empty count
                    @ResetOverflowFlag
Loop@@              brclr     7,TSC,*             ;ask if the timer is overflowing
                    @ResetOverflowFlag
                    inc       count               ;increases amount of overflows
                    lda       count
                    cbeq      time,Done@@         ;Is the right time reached? count = time?
                    bra       Loop@@              ;if I don't get there, keep counting ...
Done@@              equ       :AnRTS              ;if I arrive, exit the timer function

;*******************************************************************************
                    #VECTORS
;*******************************************************************************
                    org       Vreset
                    dw        Start
