;NOM: pixdel4.asm
;DESCRIPTION: version 4 du pixdel.
;             LED RGB control�e par un PIC10F200 ou PIC10F202
;             commandes re�ues sur GP3 en format UART 8 bits, pas de parit�, 1 stop.
;
;             format commande:
;             0xFF id_pixdel r_level g_level b_level
;               0xFF synchronisation, ne doit pas �tre utilis� comme id_pixdel ou niveau d'intensit�.
;               id_pixdel 0 = diffusion, id_unique = 1-254
;               r_level niveau de rouge 0-254
;               g_level niveau de vert 0-254
;               b_level niveau de bleu 0-254
;
;MCU: PIC10F200 ou 202
;DATE: 2013-03-05
;AUTEUR: Jacques Desch�nes
;REVISION: 2013-03-30
;          version 4, r��criture compl�te du firmware.


  include <P10F202.INC>

  __config _WDTE_OFF & _MCLRE_OFF

;#define DEBUG
;#define CALIB ; calibration de l'oscillateur

;;;;;;;;;;;;; constantes ;;;;;;;;;;;;;;
PIXDEL_ID EQU 1 ; 1 � 254 doit-�tre diff�rent pour chaque pixdel


BROADCAST EQU 0 ; identifiant message de diffusion

OPTION_CFG EQU B'11001000' ; configuration registre OPTION

RX_P EQU GP3 ; r�ception uart
#ifdef DEBUG
TX_P EQU GP1 ; transmission uart
#endif

CMD_SIZE EQU 4 ;  octets par commande, excluant l'octet de synchronisation

; bits couleurs rgb dans GPIO
GREEN   EQU GP0
BLUE    EQU GP1
RED     EQU GP2


; d�lais de bit en usec. pour les diff�rentes vitesse rs-232
; Valeur pour Fosc/4=1Mhz
; 9600 BAUD
BDLY_9600 EQU D'104' ;
HDLY_9600 EQU D'52' ; d�lais demi-bit
; 14400 BAUD
BDLY_14400 EQU D'69'
HDLY_14400 EQU D'35'
; 19200 BAUD
BDLY_19200 EQU D'52'
HDLY_19200 EQU D'26'
; 38400 BAUD
BDLY_38400 EQU  D'26'
HDLY_38400 EQU  D'13'

; vitesse utilis�e dans cette version, valeur d�lais pour oscillateur
; calibr� � ~= 4,619Mhz
BIT_DLY  EQU D'80'
HALF_DLY EQU D'40'
PWM_PERIOD EQU (~HALF_DLY) + 1 ; p�riode entre chaque appel de pwm_clock

; indicateurs bool�ens
F_RDBIT EQU 0  ; toggle lecture bit � tous les 2 cycles
F_STOP EQU 1  ; r�ception stop bit
F_BYTE EQU 2 ; octet re�u au complet
F_CMD  EQU 8 ; commande re�u et pr�te � �tre lue

OSC_CALIB EQU D'33' ; obtenu exp�rimentalement pour que Fosc/4 ~= 40*28800
                 ; muliple entier de la demi-p�riode de 14400 BAUD

;;;;;;;;;;;;; macros ;;;;;;;;;;;;;;;;;;

#define RX GPIO, RX_P ; r�ception des commande sur cette broche
#ifdef DEBUG
#define TX GPIO, TX_P ; transmission rs-232 pour d�boguage
#endif

;>>>>> cette macro n'est pas utilis� dans la version actuelle <<<<<<
; d�lais en micro-secondes bas� sur un Tcy de 1usec.
; d�lais maximal  3*255+2=767usec
delay_us macro usec
  local q=(usec-2)/3
  if q>0
    movlw q
    movwf delay_cntr
    decfsz delay_cntr,F
    goto $-1
    nop
    local r=(usec-2) % 3
    while r>1
      goto $+1
      local r=r-2
    endw
    if r>0
      nop
    endif
  else
    while usec>1
      goto $+1
      usec=usec-2
    endw
    if usec>0
      nop
    endif
  endif
  endm

next_task macro next ; 2 Tcy
    movlw next
    movwf task
    endm

init_state_idle macro ; 7 Tcy
    movlw CMD_SIZE
    movwf byte_cntr
    movlw rx_buff
    movwf FSR
    movlw task_wait_sync_start
    movwf task
    clrf flags
    endm

init_byte_rcv macro  ; 5 Tcy
    movlw H'80'
    movwf INDF
    movlw H'F0'
    andwf flags, F
    bsf flags, F_RDBIT
    endm

pwm_clock macro ; 12 Tcy
; contr�le l'intensit� des composantes rouge,verte,bleu
    incf pwm, F
    movfw pwm
    subwf dc_red, W
    rlf gpio_temp, F
    movfw pwm
    subwf dc_blue, W
    rlf gpio_temp, F
    movfw pwm
    subwf dc_green, W
    rlf gpio_temp, F
    comf gpio_temp, W
#ifdef DEBUG
    nop
#else
    movwf GPIO
#endif
    endm

;;;;;;;;;;;;; variables ;;;;;;;;;;;;;;;
  udata
task res 1 ; t�che en cours d'ex�cution
flags res 1 ; indicateurs bool�ens
byte_cntr res 1 ; compteur octets r�ception commande
rx_buff res CMD_SIZE ; m�moire tampon r�ception des commandes
delay_cntr res 1 ; compteur pour macro delay_us
pwm res 1  ; compteur pwm
dc_red res 1 ; rapport cyclique rouge
dc_blue res 1 ; rapport cyclique bleu
dc_green res 1 ; rappor cyclique vert
gpio_temp res 1 ; variable temporaire �tat GPIO utilis� par t�che pwm_clock.
#ifdef DEBUG
uart_byte res 1 ; octet envoy� part uart_tx
#endif

;;;;;;;;;;;;; code ;;;;;;;;;;;;;;;;;;;;

rst_vector org 0
#ifdef CALIB
    movlw (D'33'<<1)+1 ;ajust� pour que Fosc/4~=40*28800
    movwf OSCCAL
    goto $
#endif
  movlw OSC_CALIB<<1 ; valeur obtenu exp�rimentalement, doit-�tre ajust� pour chaque MCU
  movwf OSCCAL
  goto init

uart_rx
; r�ception RS-232 
    movlw 1<<F_RDBIT
    xorwf flags, F
    btfss flags, F_RDBIT
    return
    btfsc flags, F_STOP
    goto rcv_stop_bit
    setc
    btfss RX
    clrc
    rrf INDF, F
    skpc
    return
    bsf flags, F_STOP
    return
rcv_stop_bit
    btfsc RX
    bsf flags, F_BYTE
    return

#ifdef DEBUG
uart_tx
; transmet octet [uart_byte] , utilis� pour deboggage.
    bcf TX
    delay_us (BIT_DLY-D'6')
    setc
tx_bit_loop
    rrf uart_byte, F
    skpc
    bcf TX
    skpnc
    bsf TX
    delay_us (BIT_DLY - D'10')
    clrc
    movf uart_byte, F
    skpz
    goto tx_bit_loop
    return

echo ; transmet le contenu de rx_buff, pour d�boguage
    movlw rx_buff
    movwf FSR
    movlw CMD_SIZE
    movwf byte_cntr
echo_loop
    movfw INDF
    movwf uart_byte
    call uart_tx
    incf FSR
    decfsz byte_cntr, F
    goto echo_loop
    return

#endif


;;;;;;;;;;;; initialisation MCU ;;;;;;;
init
    movlw OPTION_CFG
    option
    clrf GPIO
    clrw
    tris GPIO
#ifdef DEBUG
    comf GPIO, F
    movlw D'255'
    movwf gpio_temp
delay_loop
    clrf TMR0
    movlw D'250'
    subwf TMR0, W
    skpc
    goto $-3
    decfsz gpio_temp, F
    goto delay_loop
    clrf GPIO
    movlw A'O'
    movwf uart_byte
    call uart_tx
    movlw A'K'
    movwf uart_byte
    movwf uart_byte
    call uart_tx
#endif
    clrf pwm
    movlw H'FF'
    movwf dc_red
    movwf dc_green
    movwf dc_blue
    init_state_idle
    clrf TMR0


;;;;;;;;;;;; boucle principale ;;;;;;;;
; la t�che pwm_clock s'ex�cute � chaque cycle du c�duleur
; les autres t�ches s'ex�cute � tour de r�le selon l'�tat
; du syst�me.
main
    movlw PWM_PERIOD + 2
    addwf TMR0
    pwm_clock ; 12 Tcy
    movfw task  ; task switch
    movwf PCL   ; +17 Tcy
task_wait_sync_start ; attend le bit de d�marrage r�ception octet SYNC (0xFF)
    btfsc RX
    goto idle_loop ; +20Tcy
    init_byte_rcv  ; 5 Tcy
    next_task task_sync
    goto idle_loop ; +28 Tcy
task_sync  ; r�ception octet SYNC
    call uart_rx   ; <= 15 Tcy
    btfss flags, F_BYTE
    goto idle_loop ; <=13 Tcy
    comf INDF,W ; z�ro si octet re�u = 0xFF
    skpz
    goto no_sync
    next_task task_wait_start_bit
    goto idle_loop
no_sync ; octet re�u n'est pas 0xFF
    next_task task_wait_sync_start
    goto idle_loop
task_wait_start_bit  ; attend bit de d�marrage r�ception des octets de commandes
    btfsc RX
    goto idle_loop
    init_byte_rcv
    next_task task_cmd_rcv
    goto idle_loop
task_cmd_rcv ; r�ception d'un octet de commande.
    call uart_rx
    btfss flags, F_BYTE
    goto idle_loop
    incf FSR
    next_task task_wait_start_bit
    decfsz byte_cntr, F
    goto idle_loop
#ifdef DEBUG
    call echo
    init_state_idle ; 7
    goto main
#endif
    movlw task_chk_id
    movwf task
    goto idle_loop
task_chk_id ; v�rifie l'identifiant avant d'accepter la commande
    movlw rx_buff
    movwf FSR
    movf INDF, W
    skpnz
    goto accept_cmd
    xorlw PIXDEL_ID
    skpz
    goto deny_cmd
accept_cmd  ; commande accept�e.
    next_task task_cmd 
    goto idle_loop
deny_cmd ; commande refus�e, mauvais pixdel_id
    init_state_idle ; 7
    goto idle_loop
task_cmd ; ex�cution de la commande re�ue.
    incf FSR, F
    comf INDF, W
    movwf dc_red
    incf FSR, F
    comf INDF, W
    movwf dc_green
    incf FSR, F
    comf INDF, W
    movwf dc_blue
    init_state_idle ; retour � l'�tat initial.
idle_loop ; compl�te le temps pour une p�riode PWM constante
    movlw PWM_PERIOD
    subwf TMR0, W
    skpnc
    goto idle_loop
    goto main
    end











