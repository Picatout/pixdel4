;NOM: pixdel4.asm
;DESCRIPTION: version 3 du pixdel (version simplifi�e).
;             LED RGB control�e par un PIC10F200 ou PIC10F202
;             commandes re�ues sur GP3 en format UART 8 bits, pas de parit�, 1 stop.
;
;             format commande:
;             0xAA id_pixdel r_level g_level b_level
;             0xAA octet de synchronisation
;             id_pixdel 0 = diffusion, id_unique 1-255
;             r_level niveau de rouge 0-255
;             g_level niveau de vert 0-255
;             b_level niveau de bleu 0-255
;
;MCU: PIC10F200 ou 202
;DATE: 2013-03-05
;AUTEUR: Jacques Desch�nes
;REVISION: 2013-03-23
;          version 4, augmentation de la vitesse uart_rx � 38400 BAUD


  include <P10F202.INC>

  __config _WDTE_OFF & _MCLRE_OFF

;;;;;;;;;;;;; constantes ;;;;;;;;;;;;;;
;PIXDEL_ID EQU 1 ; 1-255 doit-�tre diff�rent pour chaque pixdel
; pour des raisons pratique PIXDEL_ID est maintenant d�fini sur comme macro
; de ligne de commande mpasm.   mpasm -d PIXDEL_ID=n
;

BROADCAST EQU 0 ; identifiant message de diffusion

OPTION_CFG EQU B'11000001' ; configuration registre OPTION

RX_P EQU GP3 ; r�ception uart
TX_P EQU GP1 ; transmission uart

SYNC EQU H'AA' ; octet de synchronisation r�ception uart
CMD_SIZE EQU 4 ; 4 octets par commande

; bits couleurs rgb dans GPIO
GREEN   EQU GP0
BLUE    EQU GP1
RED     EQU GP2

; d�lais de bit pour 9600 BAUD
BDLY_9600 EQU D'104'-D'10'  ; -10 pour d�lais boucle induit.
HDLY_9600 EQU D'52' ; d�lais demi-bit
;d�lais de bit pour 19200 BAUD
BDLY_19200 EQU D'52'-D'10'
HDLY_19200 EQU D'26'
; d�lais de bit pour 38400 BAUD
BDLY_38400 EQU  D'26'-D'10'
HDLY_38400 EQU  D'12'

BIT_DLY  EQU BDLY_38400
HALF_DLY EQU HDLY_38400

;;;;;;;;;;;;; macros ;;;;;;;;;;;;;;;;;;
#define RX GPIO, RX_P
#define TX GPIO, TX_P

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

;;;;;;;;;;;;; variables ;;;;;;;;;;;;;;;
  udata
uart_byte res 1 ; octet re�u sur entr�e uart
delay_cntr res 1 ; compteur pour macro delay_us
pwm res 1  ; compteur pwm
dc_red res 1 ; rapport cyclique rouge
dc_blue res 1 ; rapport cyclique bleu
dc_green res 1 ; rappor cyclique vert
cmd_buff res 5 ; m�moire tampon r�ception des commandes
temp res 1 ; registre temporaire

;;;;;;;;;;;;; code ;;;;;;;;;;;;;;;;;;;;

rst_vector org 0
  goto init

uart_rx
; r�ception RS-232 TTL
; sortie: carry bit=1 indique qu'un octet a �t� re�u
;         octet re�u dans uart_byte
  clrc
  btfsc RX
  return
  movlw H'80'
  movwf uart_byte
  delay_us HALF_DLY
rx_bit_loop
  delay_us BIT_DLY
  setc
  btfss RX
  clrc
  rrf uart_byte, F
  goto $+1 ;pour avoir le m�me nombre de cycles dans la boucle la sous-routine tx
  skpc
  goto rx_bit_loop
  delay_us BIT_DLY + D'4'
  setc
  btfss RX
  clrc ;erreur r�ception devrait-�tre un stop bit.
  return

uart_tx
; transmet octet [uart_byte]
  bcf TX
  delay_us BIT_DLY
  setc
tx_bit_loop
  rrf uart_byte, F
  skpc
  bcf TX
  skpnc
  bsf TX
  delay_us BIT_DLY
  clrc
  movf uart_byte, F
  skpz
  goto tx_bit_loop
  return

rcv_cmd
   btfsc RX
   return
   call uart_rx
   skpc
   return
   movlw SYNC
   xorwf uart_byte, W
   skpz
   return
   movlw cmd_buff
   movwf FSR
   movlw CMD_SIZE ; nombre d'octets � recevoir
   movwf temp
rcv_next_byte
   btfsc RX  ; attend start bit
   goto $-1
   call uart_rx
   skpc
   return  ; erreur r�ception commande annul�e
   movfw uart_byte
   movwf INDF
   incf FSR, F
   decfsz temp, F
   goto rcv_next_byte
   call exec_cmd
   return

exec_cmd
; traite la commande re�u dans uart_byte
    movlw cmd_buff
    movwf FSR
    clrw
    xorwf INDF, W
    skpnz
    goto accept_cmd ; message de diffusion
    movlw PIXDEL_ID
    xorwf INDF, W
    skpz   ; pixdel_id correspondant � ce pixdel
    return ; pas concern�.
accept_cmd ; transfert des rapports cycliques dans les variables
    incf FSR, F
    movfw INDF
    movwf dc_red
    incf FSR, F
    movfw INDF
    movwf dc_green
    incf FSR, F
    movfw INDF
    movwf dc_blue
    clrf pwm
    return



;;;;;;;;;;;; initialisation MCU ;;;;;;;
init
  movlw D'8' ; valeur obtenue exp�rimentalement par mesure de FOSC4 sur GP2
  movwf OSCCAL
  movlw OPTION_CFG
  option
  clrw
  tris GPIO
  movlw 8
clear_ram
  movwf FSR
  clrf INDF
  incf FSR, F
  btfss FSR, 5
  goto clear_ram
  bsf GPIO, RED
  bsf GPIO, GREEN
  bsf GPIO, BLUE
  movlw D'255'
  movwf temp
  clrf TMR0
  movlw D'250'
  subwf TMR0, W
  skpc
  goto $-3
  decfsz temp, F
  goto $-6
  clrf GPIO

;;;;;;;;;;;; boucle principale ;;;;;;;;
main
    incf pwm, F
    clrf temp
red_channel
    movfw pwm
    skpz
    subwf dc_red, W
    skpnc
    bsf temp, RED
    btfss RX
    goto start_bit
green_channel
    movfw pwm
    skpz
    subwf dc_green, W
    skpnc
    bsf temp, GREEN
    btfss RX
    goto start_bit
blue_channel
    movfw pwm
    skpz
    subwf dc_blue, W
    skpnc
    bsf temp, BLUE
    movfw temp
    movwf GPIO
    btfsc RX
    goto main
start_bit
    call rcv_cmd
    goto main
    end











