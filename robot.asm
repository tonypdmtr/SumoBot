;****************************************************************************************************************************
;  PROGRAMA : Lee dos sensores de linea y un sensor de proximidad de objeto.
;  OBJETIVO : (del robot): No irse del ring. Girar hasta encontrar oponente. Embestirlo. Si no hay, moverse y girar.
;
;  Autor adhsoft0@gmail.com
;  Licencia GPLv3
;
;****************************************************************************************************************************


;----------------------------------------------------------------------------------------------------------------------------
;       Incluir archivo con asociacion entre etiqueta / nombre  y las direcciones
;       de los registros asociados al microcontrolador
;----------------------------------------------------------------------------------------------------------------------------

$Include   'C:\pemicro\ics08qtqyz\qtqy_registers.inc ' ; se indica la ruta o PATH donde esta el archivo
                                                       ; ubicar el archivo .inc en la carpeta del.asm
;----------------------------------------------------------------------------------------------------------------------------
;               Definicion de origenes de las memorias a utilizar
;----------------------------------------------------------------------------------------------------------------------------

RAMStart                EQU     $0080
FLASHROMStart           EQU     $EE00
VECTORStart             EQU     $FFFE

;----------------------------------------------------------------------------------------------------------------------------
;              Definicion de Variables / Constantes  --  pueden ser bit o registros --
;----------------------------------------------------------------------------------------------------------------------------


;PTA bits:
SFL     EQU     0
SFR     EQU     1

STL     EQU     2
STR     EQU     3


;PTB bits:
M1A     EQU     0
M1B     EQU     1
M2A     EQU     2
M2B     EQU     3

SPA     EQU     4



;----------------------------------------------------------------------------------------------------------------------------
;              Especificacion del inicio de la RAM
;----------------------------------------------------------------------------------------------------------------------------

                        ORG     RAMStart
;----------------------------------------------------------------------------------------------------------------------------
;              Reservar espacios de memoria ram y asignar etiquetas o numeros
;----------------------------------------------------------------------------------------------------------------------------

count   dw 1    ; esta variable cuenta la cantidad de desbordes del timer
time    dw 1    ; esta se utilizara para indicar cuando tiempo se debe esperar (cuantos desbordes)
rndbas1 dw 1    ; esta se utilizara para generar un numero aleatorio


;----------------------------------------------------------------------------------------------------------------------------
;              Especificacion del inicio del programa ,que se almacena en la ROM, a partir de la direccion indicada
;----------------------------------------------------------------------------------------------------------------------------

                        ORG     FLASHROMStart                   ;

;----------------------------------------------------------------------------------------------------------------------------
;              Configuracion de registros, como el del OSC, EL CONFIG1 - COP -
;----------------------------------------------------------------------------------------------------------------------------

init:                   mov     #$01,config1    ; oscilador interno, apaga COP

;----------------------------------------------------------------------------------------------------------------------------
;              Configuracion de registros de los puertos y otros modulos
;
;               PTA0: SFL
;               PTA1: SFR
;
;               PTA2: STL
;               PTA3: STR
;
;
;               PTB0: M1A
;               PTB1: M1B
;               PTB2: M2A
;
;               PTB4: SPA
;
;----------------------------------------------------------------------------------------------------------------------------



                        mov     #%00001111,DDRB ; 4 entradas, 4 salidas
                        clr     DDRA            ; configuro todo el PTA como entrada
                        mov     #$00,PORTB      ; limpio el puerto
                        mov     #%00010000,ptbpue     ; pull up SPA

 ; ------------------  [preparacion del TIM]  --------------------------

                       clr     count      ; limpio count

                       bset    5,TSC      ; apago timer
                       mov     #$36,TSC   ; configura timer y prescaler /64
                       lda     #$FF       ;
                       sta     TMODH      ; cargo el registro de comparacion TMODH
                       lda     #$FB       ; para obtener 5ms de retardo de bandera (251)
                       sta     TMODL      ; cargo el registro de comparacion TMODL
                       bclr    5,TSC      ; enciendo timer



;----------------------------------------------------------------------------------------------------------------------------
;               Programa Principal
;----------------------------------------------------------------------------------------------------------------------------


start:                  ; resetear robot


                        jsr detenerse   ; detener al robot en caso de "reseteo"
                        mov #$02,time   ; esperar dos "segundos"
                        jsr timer       ; esperar.
                        brset   SFL,PORTA,start    ; saltar si hay linea adelante
                        brset   SFR,PORTA,start    ; saltar si hay linea adelante

                        jmp SUB0              ; buscar enemigo





;----------------------------------------------------------------------------------------------------------------------------
;               programa SUB0 "Escaner" : busca si hay un enemigo cercano girando sobre si mismo 360 grados
;----------------------------------------------------------------------------------------------------------------------------

SUB0:                   clr count
                        mov #$06,TSC            ; prescaler /64
                        lda TCNTL               ; cargar contador timer
                        sta rndbas1             ; guardarlo en num aleatorio
                        lda rndbas1            ; cargar num aleatorio
                        cmp #$80               
                        blo ab1                ; girar sobre si mismo en forma antihoraria si es menor que 128
                        jmp ac1                ; si no, en forma horaria
ab1:                    jsr gssah
                        jmp a2                 ; seguir

ac1:                    jsr gssh

a2:                     brset   7,TSC,des2      ; pregunta si desbordo el timer
                        brset   SPA,PORTB,SUB1  ; verificar enemigo cercano
                        brset   SFL,PORTA,back  ; resetear con cualquier sensado linea
                        brset   SFR,PORTA,back
                        jmp a2          ; si no hay enemigo repetir

des2:                   inc count       ; incremento count
                        lda #$05        ; cargo 5
                        cbeq count,SUB2 ; comparo y si count es 5, voy a SUB2
                        jsr bajarflag
                        jmp a2          ; si no, sigo contando

back:                   jmp start




;----------------------------------------------------------------------------------------------------------------------------
;               programa SUB1: "Atacar". Si detecta una linea (gana), se resetea.
;               Si pierde al enemigo llama a SUB0 (Escaner).
;----------------------------------------------------------------------------------------------------------------------------

SUB1:                  jsr     avanzar
                       jsr     bajarflag        ; no me acuerdo
d1a:                   brset   7,TSC,emmem
                       brset   SFL,PORTA,startrd  ; ganar
                       brset   SFR,PORTA,startrd
                       ;retardo

                       lda     #$FF
loop:                  dbnza   loop

                       ;/fin retardo
                       
                       brclr   SPA,PORTB,SUB0   ; perdio el enemigo, rebuscarlo
                       jmp     d1a              ; seguir empujando (sin apagar motores)

emmem:                 jsr bajarflag
                       jmp d1a

; redirecciones

startrd:               jmp girar

; fin rds



;----------------------------------------------------------------------------------------------------------------------------
;               programa SUB2 "Buscador": Mueve al robot adelante, si detecta una linea gira aleatoriamente (usando timer)
;               Si detecta un enemigo va a SUB1 "Atacar".
;----------------------------------------------------------------------------------------------------------------------------

SUB2:                   jsr  avanzar    ; avanzar

                        ; ir viendo si se topa con linea adelante; ir vaciando TSC

sss:                    brset   SFL,PORTA,girar    ; se topo con linea delantera, girar
                        brset   SFR,PORTA,girar    ; se topo con linea delantera, girar
                        brset   SPA,PORTB,SUB1     ; se topo con linea delantera, atacar
bb:                     brset   7,TSC,vaciar       ; pregunta si desbordo el timer
                        jmp sss                    ; repetir
vaciar:                 jsr bajarflag              ; si desbordo el timer, resetearlo. (lo usaremos para generar numero aleatorio)
                        jmp sss


girar:                  lda TCNTL             ; guardar 1er numero del timer
                        sta rndbas1           ; en rndbas1
                        jsr detenerse
                        mov #$03,TSC          ; prescaler /8
                        jsr retroceder
                        mov #$05,TSC          ; prescaler /32
                        mov #$01,time         ; esperar apenas
                        jsr timer             ; esperar.
                        mov #$06,TSC          ; prescaler /64
                        jsr detenerse

                        lda rndbas1           ; cargar num aleatorio
                        cmp #$80
                        blo ab                ; girar sobre si mismo en forma antihoraria si es menor que 128
                        jmp ac                ; si no, en forma horaria
ab:                     jsr gssah
                        jmp ad                ; seguir

ac:                     jsr gssh
                        lda rndbas1
                        sub #$70              ; restarle 100 (para que gire a un lado u otro con iguales limites)
                        sta rndbas1           ; guardar el resultado
ad:                     mov #$00,TSC          ; cambiar el prescaler a /1
                        lda rndbas1           ; cargar numero aleatorio
                        sta time              ; y ponerlo en time


                        ; esperar time pero verificando enemigo


                        clr     count        ; vaciar count
                        jsr bajarflag        ; limpia el Flag de desborde
aee:                    brset   7,TSC,desee  ; pregunta si desbordo el timer
                        brset   SPA,PORTB,SUB1rd ; enemigo? atacar.
                        brset   SFL,PORTA,girar ; linea? retroceder y girar de nuevo
                        brset   SFR,PORTA,girar ; linea? retroceder y girar de nuevo
                        jmp     aee

desee:                  jsr bajarflag        ; limpia el Flag de desborde
                        inc     count        ; incrementa cantidad de desbordes
                        lda     count
                        cbeq    time,bee     ; se llego al tiempo indicado?  count = time ?
                        jmp     aee          ; si no llego, seguir contando..


bee:                    mov #$06,TSC          ; reestablecer el prescaler en /64
                        jsr avanzar           ; volver a avanzar
                        jmp bb                ; volver

; rds

SUB1rd:                 jmp SUB1


;----------------------------------------------------------------------------------------------------------------------------

; funciones - calls (jsr/bsr)

;----------------------------------------------------------------------------------------------------------------------------

gssah:                  ; girar sobre si mismo en forma antihoraria
                        bset    M1B,PORTB         ; M1B
                        bclr    M1A,PORTB
                        bset    M2A,PORTB         ; M2A
                        bclr    M2B,PORTB
                        rts

gssh:                   ; girar sobre si mismo en forma horaria
                        bset    M1A,PORTB         ; M1A
                        bclr    M1B,PORTB
                        bset    M2B,PORTB         ; M2B
                        bclr    M2A,PORTB
                        rts


avanzar:                bset    M1A,PORTB         ; M1A
                        bclr    M1B,PORTB
                        bset    M2A,PORTB         ; M2A
                        bclr    M2B,PORTB
                        rts

retroceder:             bset    M1B,PORTB         ; M1B
                        bclr    M1A,PORTB
                        bset    M2B,PORTB         ; M2B
                        bclr    M2A,PORTB
                        rts

detenerse:              bclr    0,PORTB         ; M10
                        bclr    1,PORTB
                        bclr    2,PORTB         ; M20
                        bclr    3,PORTB
                        rts

bajarflag:              bclr    7,TSC      ; limpia el Flag de desborde
                        rts


; ----


timer:                 ; funcion que espera una cantidad de tiempo dependiendo de la variable time.
                       clr     count      ; vaciar count
                       bclr    7,TSC      ; limpia el Flag de desborde
a:                     brset   7,TSC,des  ; pregunta si desbordo el timer
                       jmp     a

des:                   bclr    7,TSC      ; limpia el Flag de desborde
                       inc     count      ; incrementa cantidad de desbordes
                       lda     count
                       cbeq    time,b     ; se llego al tiempo indicado?  count = time ?
                       jmp     a          ; si no llego, seguir contando..
b:                     rts                ; si llego, salir de la funcion timer.



;----------------------------------------------------------------------------------------------------------------------------
;               Indicacion del origen de los vectores de int.
;----------------------------------------------------------------------------------------------------------------------------

                        ORG     VECTORStart
;----------------------------------------------------------------------------------------------------------------------------
;                Asignacion de saltos para cada vector
;----------------------------------------------------------------------------------------------------------------------------

                        dw      init

;----------------------------------------------------------------------------------------------------------------------------

