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

CMD_SIZE EQU 4 ; 5 octets par commande

; bits couleurs rgb dans GPIO
GREEN   EQU GP0
BLUE    EQU GP1
RED     EQU GP2


; délais de bit pour 9600 BAUD
BDLY_9600 EQU D'104' ;
HDLY_9600 EQU D'52' ; délais demi-bit
;délais de bit pour 19200 BAUD
BDLY_19200 EQU D'52'
HDLY_19200 EQU D'26'
; délais de bit pour 38400 BAUD
BDLY_38400 EQU  D'26'
HDLY_38400 EQU  D'13'

BIT_DLY  EQU BDLY_19200
HALF_DLY EQU HDLY_19200
PWM_PERIOD EQU HALF_DLY ; période entre chaque appel de pwm_clock


;;;;;;;;;;;;; macros ;;;;;;;;;;;;;;;;;;
#define DEBUG

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

init_rcv_state macro
    movlw CMD_SIZE
    movwf byte_cntr
    movlw rx_buff
    movwf FSR
    clrf uart_byte
    endm


;;;;;;;;;;;;; variables ;;;;;;;;;;;;;;;
  udata
uart_byte res 1 ; octet reçu sur entrée uart
byte_cntr res 1 ; registre temporaire
rx_buff res CMD_SIZE ; mémoire tampon réception des commandes
delay_cntr res 1 ; compteur pour macro delay_us
pwm res 1  ; compteur pwm
dc_red res 1 ; rapport cyclique rouge
dc_blue res 1 ; rapport cyclique bleu
dc_green res 1 ; rappor cyclique vert
gpio_temp res 1 ; variable temporaire état GPIO utilisé par tâche PWM.

;;;;;;;;;;;;; code ;;;;;;;;;;;;;;;;;;;;

rst_vector org 0
  goto init

uart_rx
; réception RS-232 , 19200 BAUD
; sortie:
;   carry = 1 si octet reçu
;   octet reçu dans uart_byte
  btfsc RX
  return  ; 5uSec incluant call
  movlw H'80'
  movwf uart_byte ; +6uSec
  delay_us (PWM_PERIOD - D'6')
rx_bit_loop
  call pwm_clock ; +17uSec
  delay_us (PWM_PERIOD - D'17')
  call pwm_clock
  goto $+1
  setc
  btfss RX
  clrc
  rrf uart_byte, F
  skpc
  goto rx_bit_loop ; +7
  nop
  call pwm_clock
  setc
  return ; +3

#ifdef DEBUG
uart_tx
; transmet octet [uart_byte] , utilisé pur deboggage.
  bcf TX
  delay_us BIT_DLY
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
#endif

pwm_clock ; 17 uSec inclant call et return
    incf pwm, F
    clrf gpio_temp
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
#elif
    movwf GPIO
#endif
    return

read_cmd ;+4
    movlw rx_buff
    movwf FSR
#ifdef DEBUG
  movfw INDF
  movwf uart_byte
  call uart_tx
#endif
    call pwm_clock
    movf INDF, W
    skpnz
    goto broadcast_cmd
    xorlw PIXDEL_ID
    skpnz
    goto accept_cmd
    goto cmd_exit
broadcast_cmd
    nop
    goto $+1
accept_cmd
    goto $+1
    call pwm_clock
    incf FSR, F
    comf INDF, W
    movwf dc_red
    incf FSR, F
    comf INDF, W
    movwf dc_green
    incf FSR, F
    comf INDF, W
    movwf dc_blue
    call pwm_clock
    delay_us (PWM_PERIOD - D'17'- 1)
cmd_exit
    nop
    call pwm_clock
    init_rcv_state ;+6
    return ;+8


;;;;;;;;;;;; initialisation MCU ;;;;;;;
init
  movlw D'8' ; valeur obtenue expérimentalement par mesure de FOSC4 sur GP2
  movwf OSCCAL
  movlw OPTION_CFG
  option
  clrw
  tris GPIO
  bsf GPIO, RED
  bsf GPIO, GREEN
  bsf GPIO, BLUE
  movlw D'255'
  movwf gpio_temp
  clrf TMR0
  movlw D'250'
  subwf TMR0, W
  skpc
  goto $-3
  decfsz gpio_temp, F
  goto $-6
  clrf GPIO
  init_rcv_state
  clrf pwm
  movlw H'FF'
  movwf dc_red
  movwf dc_green
  movwf dc_blue

;;;;;;;;;;;; boucle principale ;;;;;;;;
main
;   attend octet de synchronisation
    call pwm_clock
    call uart_rx
    skpc
    goto main
chk_sync
    comf uart_byte, W
    skpz
    goto main
receiving_loop ; réception d'une commande
    call pwm_clock ; + 17uSec
    call uart_rx  ; 5uSec si rien reçu
    skpc  ; +2 si rien reçu
;    goto store_byte ;
    goto receiving_loop ; +2,  total boucle 26uSec
store_byte ; +6
    movfw uart_byte
    movwf INDF
    call pwm_clock
    incf  FSR, F
    decfsz byte_cntr, F
    goto receiving_loop
    call read_cmd
;    nop
;    call pwm_clock
;    delay_us (PWM_PERIOD - D'7')
    goto main

    end











