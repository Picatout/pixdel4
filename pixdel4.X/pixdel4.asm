;NOM: pixdel4.asm
;DESCRIPTION: version 3 du pixdel (version simplifiée).
;             LED RGB controlée par un PIC10F200 ou PIC10F202
;             commandes reçues sur GP3 en format UART 8 bits, pas de parité, 1 stop.
;
;             format commande:
;             0xFF id_pixdel r_level g_level b_level
;               0xFF synchronisation, ne doit pas être utilisé comme id_pixdel ou niveau d'intensité.
;               id_pixdel 0 = diffusion, id_unique 1-254
;               r_level niveau de rouge 0-254
;               g_level niveau de vert 0-254
;               b_level niveau de bleu 0-254
;
;MCU: PIC10F200 ou 202
;DATE: 2013-03-05
;AUTEUR: Jacques Deschênes
;REVISION: 2013-03-23
;          version 4, augmentation de la vitesse uart_rx à 38400 BAUD


  include <P10F202.INC>

  __config _WDTE_OFF & _MCLRE_OFF

;;;;;;;;;;;;; constantes ;;;;;;;;;;;;;;
;PIXDEL_ID EQU 1 ; 1-255 doit-être différent pour chaque pixdel
; pour des raisons pratique PIXDEL_ID est maintenant défini sur comme macro
; de ligne de commande mpasm.   mpasm -d PIXDEL_ID=n
;

BROADCAST EQU 0 ; identifiant message de diffusion

OPTION_CFG EQU B'11001000' ; configuration registre OPTION

RX_P EQU GP3 ; réception uart
TX_P EQU GP1 ; transmission uart

CMD_SIZE EQU 4 ;  octets par commande

; bits couleurs rgb dans GPIO
GREEN   EQU GP0
BLUE    EQU GP1
RED     EQU GP2


; délais de bit pour 9600 BAUD
BDLY_9600 EQU D'104' ;
HDLY_9600 EQU D'52' ; délais demi-bit
BDLY_14400 EQU D'69'
HDLY_14400 EQU D'35'
;délais de bit pour 19200 BAUD
BDLY_19200 EQU D'52'
HDLY_19200 EQU D'26'
; délais de bit pour 38400 BAUD
BDLY_38400 EQU  D'26'
HDLY_38400 EQU  D'13'

BIT_DLY  EQU BDLY_9600
HALF_DLY EQU HDLY_9600
PWM_PERIOD EQU (~HALF_DLY) + 1 ; période entre chaque appel de pwm_clock

; indicateurs booléens
F_RDBIT EQU 0  ; toggle lecture bit à tous les 2 cycles
F_STOP EQU 1  ; réception stop bit
F_BYTE EQU 2 ; octet reçu au complet
F_CMD  EQU 8 ; commande reçu et prête à être lue


;;;;;;;;;;;;; macros ;;;;;;;;;;;;;;;;;;
;#define DEBUG

#define RX GPIO, RX_P
#define TX GPIO, TX_P

; délais en micro-secondes basé sur un Tcy de 1usec.
; délais maximal  3*255+2=767usec
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
task res 1 ; tâche en cours d'exécution
flags res 1 ; indicateurs booléens
byte_cntr res 1 ; registre temporaire
rx_buff res CMD_SIZE ; mémoire tampon réception des commandes
delay_cntr res 1 ; compteur pour macro delay_us
pwm res 1  ; compteur pwm
dc_red res 1 ; rapport cyclique rouge
dc_blue res 1 ; rapport cyclique bleu
dc_green res 1 ; rappor cyclique vert
gpio_temp res 1 ; variable temporaire état GPIO utilisé par tâche PWM.
#ifdef DEBUG
uart_byte res 1 ; octet envoyé part uart_tx
#endif

;;;;;;;;;;;;; code ;;;;;;;;;;;;;;;;;;;;

rst_vector org 0
  goto init

uart_rx
; réception RS-232 
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
; transmet octet [uart_byte] , utilisé pur deboggage.
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

echo
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



read_cmd 
    movlw rx_buff
    movwf FSR
    movf INDF, W
    skpnz
    goto accept_cmd
    xorlw PIXDEL_ID
    skpz
    return  ; +11
accept_cmd
    incf FSR, F
    comf INDF, W
    movwf dc_red
    incf FSR, F
    comf INDF, W
    movwf dc_green
    incf FSR, F
    comf INDF, W
    movwf dc_blue
    return ; + 21


;;;;;;;;;;;; initialisation MCU ;;;;;;;
init
    movlw D'8' ; valeur obtenue expérimentalement par mesure de FOSC4 sur GP2
    movwf OSCCAL
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
main
    movlw PWM_PERIOD + 2
    addwf TMR0
    pwm_clock ; 12 Tcy
    movfw task  ; task switch
    movwf PCL
task_wait_sync_start
    btfsc RX
    goto idle_loop
    init_byte_rcv  ; 5 Tcy
    next_task task_sync
    goto idle_loop
task_sync
    call uart_rx   ; <= 15 Tcy
    btfss flags, F_BYTE
    goto idle_loop
    comf INDF,W
    skpz
    goto no_sync
    next_task task_wait_start_bit
    goto idle_loop
no_sync
    next_task task_wait_sync_start
    goto idle_loop
task_wait_start_bit
    btfsc RX
    goto idle_loop
    init_byte_rcv
    next_task task_cmd_rcv
    goto idle_loop
task_cmd_rcv
    call uart_rx
    btfss flags, F_BYTE
    goto idle_loop
    incf FSR
    next_task task_wait_start_bit
    decfsz byte_cntr, F
    goto idle_loop
#ifdef DEBUG
    call echo
#else
    call read_cmd ; 11 ou 21
#endif
    init_state_idle ; 7
    goto main
idle_loop
    movlw PWM_PERIOD
    subwf TMR0, W
    skpnc
    goto idle_loop
    goto main
    end











