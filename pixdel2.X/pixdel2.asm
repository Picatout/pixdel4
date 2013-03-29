; NOM: Pixdel
; DESCRIPTION: un PIC10F202 contrôle une DEL RGB et reçois des commande au format rs-232 sur GP3
;; Les Pixdels n'émettent pas de messages ils ne font qu'en recevoir.
;; chaque pixdel a un id unique de 8 bit, 253 pixdels peuvent êtrent connectés sur le même bus
;; le code 0 est réservé pour les commandes de groupe
;; le code 255 est réservé pour la diffusion.
;; les commandes sont envoyées sous la forme de messages de longueur fixé à 6 octets
;; Si un message nécessite moins de 6 octets, les octets non utilisées doivent-être envoyés avec la valeur 0. 
;; FORMAT DES MESSAGES: [ID|0x00|0xff] CMD ARGUMENTS
;; ID est l'identiant du pixdel ciblé par la commande
;;; un ID==0 indique que l'octet CMD contient le groupID dans ses 5 bits les plus significatifs
;; 0xff est un message de diffusion à tous les pixdels.
;;;; bits: 7643210
;;;;       |   |  |
;;;;       GID CMD
;; GID prend une valeur entre 1 et 31 et identifie le groupe cible de la commande
;; CMD est la commande à exécuter valeur entre 0 et 7
;; ARGUMENTS les paramètres de la commande.
;
; LISTE de commande
;;  0x00 : éteint la DEL    
;;         exemples:  
;;             1) 0x01 0x00 0x00 0x00 0x00 0x00  : éteint le pixdel dont le pixdelId = 1
;;             2) 0x00 0x11 0x00 0x00 0x00 0x00  : éteint les pixdels appartenenant au groupe 0x02, 0x02<<3|0x01=0x11
;;             3) 0xff 0x00 0x00 0x00 0x00 0x00  : éteint tous les pixdels   
;;  0x01 : allume la DEL
;;         exemples:
;;             1) 0x03 0x01 0x00 0x00 0x00 0x00  : allume le pixdel dont le pixdelId = 3
;;             2) 0x00 0x29 0x00 0x00 0x00 0x00  : allume les pixdels appartenenant au groupe 0x05, 0x05<<3|0x01=0x29
;;             3) 0xff 0x01 0x00 0x00 0x00 0x00  : allume tous les pixdels   
;;  0x02 : défini couleur
;;         exemples:
;;             1) 0x1f 0x02 0xff 0x00 0x00 0x00  : défini la couleur du pixdel dont le pixdelId = 0x1f avec la couleur ROUGE intensitée maximale.
;;             2) 0x00 0xfa 0x7f 0x00 0x7f 0x00  : défini la couleur des pixdels appartenenant au groupe 31 avec la couleur MAGENTA intensité 50%, 0x1f<<3|0x02=0xfa
;;             3) 0xff 0x02 0xff 0xff 0xff 0x00  : allume tous les pixdels en BLANC intensité maximale   
;;  0x03 : clignotement
;;         exemples:
;;             1) 0x1f 0x03 0xff 0x00 0x00 0x1f  : clignotement du pixdel dont le pixdelId = 0x1f avec la couleur ROUGE intensitée maximale,délais 0x1f*13.3msec.
;;             2) 0x00 0xa3 0x7f 0x7f 0x00 0x0f  : clignotement des pixdels appartenenant au groupe 0x14 avec la couleur JAUNE intensité 50%, délais 0xf*13.3msec.
;;             3) 0xff 0x03 0xff 0xff 0xff 0xff  : clignetement de tous les pixdels en BLANC intensité maximale, délais maximal 255*13.3msec.   
;;  0x04 : couleur au hazard
;;         exemples:
;;             1) 0x23 0x04 0x00 0x00 0x00 0x00  : défini une couleur au hazard pour le pixdel dont le pixdelId = 0x23
;;             2) 0x00 0x2c 0x00 0x00 0x00 0x00  : défini une couleur au hazard pour les pixdels appartenenant au groupe 0x05, 0x5<<3|0x4=0x2c
;;             3) 0xFF 0x04 0x00 0x00 0x00 0x00  : tous les pixdels choisirons une couleurs au hazard.   
;;  0x05 : défini le groupId
;;         exemple:
;;                0x1a 0x05 0x0c 0x00 0x00 0x00  : le pixdel 0x1a appartient maintenant au groupe 0x0c
;;  0x06 : initialise le PRG (pseudo random generator)
;;         exemple:
;;             1)  0x07 0x06 0x23 0x56 0xaa 0x00 : initialise le PRG du pixdel 0x07 avec la valeur 0x2356aa
;;             2)  0x00 0x26 0x50 0xe1 0xb2 0x00 : initialise le PRG du groupe 0x04 avec la valeur 0x50e1b2
;;             3)  0xff 0x06 0xab 0xcd 0xef 0x00 : initialise le PRG de tous les pixdels avec la valeur 0xabcdef
;;  0x07 : exécute la routine POST
;;         exemple:
;;             1)  0x04 0x07 0x00 0x00 0x00 0x00 : execute POST sur le pixdel avec l'id 4
;;             2)  0x00 0x27 0x00 0x00 0x00 0x00 : execute POST sur le groupe de pixdels avec l'id de groupe 4 
;;             3)  0xff 0x07 0x00 0x00 0x00 0x00 : execute POST sur tous les pixdel
;;      
;; 
; COULEUR
;; l'argument couleur est une triade d'octets dans l'ordre 'R'ouge, 'V'ert, 'B'leu  
;
; DELAIS
; délais de 0 à 255 en multiple MAIN_LOOP_USEC*256*delais pour MAIN_LOOP_USEC=52usec on a  13,3msec/unité
; 
; identifiant de groupe
; les groupId prennent une valeur entre 1 et 31 


 #include P10F202.INC
 errorlevel 2
 __config _WDTE_OFF & _MCLRE_OFF

;;;;;;;;;;;;  constantes ;;;;;;;;;;;;;;;;;;;;;;;;

PIXDEL_ID  EQU 1  ; identifiant du pixdel doit-être différent pour chaque pixdel

OPTION_INI EQU B'10001000' ; wake up on pin change désactivé, weak pullup désactivé, pré-scaler sur TMR0 désactivé
TRIS_INI EQU 0x8 ; GP0,GP1 et GP3 en mode sortie

UART_BIT_PERIOD EQU .1000000/.9600 ; 9600BAUD
MAIN_LOOP_USEC EQU ~((UART_BIT_PERIOD/2)-.10) ; boucle de 52 microsecondes pour le PWM

RAND_XOR_MASK EQU 0xE1 ; masque xor pour le générateur pseudo hasard.
;; GPIO contrôlant la DEL RGB
RED EQU GP0   ; contrôle électrode rouge  de la DEL
GREEN EQU GP2  ; contrôle électrode verte de la DEL
BLUE EQU GP1    ; contrôle électrode blue de la DEL 
;; indicateurs binaire
F_BLINK EQU 0 ; clignotement actif
F_LED_ON EQU 1 ; LED allumée
F_RX_BYTE EQU 2 ; réception octet en cours
F_ERR_FRAME EQU 3 ; erreur de réception
F_READ_BIT EQU 4 ; c'est le temps de lire le prochain bit

XOR_BLK_PHASE EQU 2 ; masque pour inverser l'indicateur F_LED_ON

MSG_LEN EQU 6  ; longueur en octets des messages RS-232

;;;;;;;;;; macro pré-processeur ;;;;;;;;;;;;;;;;;;;
 
 #define COMMON_CATHODE ; del rbg à cathode commune , si del à anode commune commentez cette constante
 #define RX_INP GPIO, GP3 ; entrée des commandes 

;;;;;;;;;;;;; macros ;;;;;;;;;;;;;;;;;;

 light_on macro color  ; allume une couleur
 #ifdef COMMON_CATHODE
   bsf GPIO, color
 #else
   bcf GPIO, color 
 #endif
 endm

 light_off macro color  ; eteint une couleur
 #ifdef COMMON_CATHODE
   bcf GPIO, color
 #else
   bsf GPIO, color
 #endif  
 endm

 all_off macro  ; eteint les 3 couleurs
 #ifdef COMMON_CATHODE
  clrf GPIO
 #else
  movlw 7
  movwf GPIO
 #endif
 endm

init_timer macro
 movlw MAIN_LOOP_USEC
 movwf TMR0
 endm


;;;;;;;;;;;;;;; variables ;;;;;;;;;;;;;;;;;;;;;;;;;;

  cblock 8
  delay : 1     ; compteur pour délais de cignotement ou autre
  delay_cnt : 1 ; conserve la valeur du délais entre les itérations 
  red_level : 1 ;
  green_level : 1;
  blue_level : 1;
  pwm_timer : 1 ; compteur de molation à largeur d'impulsion
  led_bits : 1 ; conserve l'état des bits qui contrôle les DEL
  flags : 1;
  temp : 1 ;
  rand : 3
  groupId : 1 ; 
  bit_cntr : 1 ; compteur de bit pour la réception sur GP3
  byte_cntr : 1 ; compteur d'octets reçus sur GP3
  cmdBuffer : MSG_LEN ; tampon de réception des commandes
  endc

;;;;;;;;;;;;;;;;  segment de code ;;;;;;;;;;;;;;;;;;;;;;

 org 0
 goto init

  #ifdef __DEBUG  ; code inclus seulement pendant le développement
  ;#define _UART
; envoie d'un octet par protocole UART
; utilise la sortie GP1 pour l'envoie UART
  #ifdef _UART
  uart_byte EQU cmdBuffer+MSG_LEN    ;octet à envoyer
uart_send
  movlw 9
  movwf bit_cntr
  movlw ~(UART_BIT_PERIOD-.19)
  movwf TMR0
  bcf GPIO, GP1
  goto bit_delay
send_bit_loop
  movlw ~(UART_BIT_PERIOD-.19)
  movwf TMR0
  decfsz bit_cntr,F
  goto send_next_bit
  bsf GPIO,GP1
  goto bit_delay
send_next_bit
  rrf uart_byte,F
  skpc  
  goto send_0
  bsf GPIO,GP1
  goto bit_delay
send_0
  bcf GPIO,GP1
bit_delay
  movlw 6
  subwf TMR0,W
  skpnc
  goto $-3
  movlw 0
  xorwf bit_cntr,W
  skpz
  goto send_bit_loop
  return

send_ok 
  movlw 'O'
  movwf uart_byte
  call uart_send
  movlw 'K'
  call uart_send
  return


send_echo
  decf FSR,F
  movfw INDF
  movwf uart_byte
  incf FSR,F
  call uart_send
  return

  #endif ;_UART

delay_ms
  movlw B'100000001'
  option
  movlw .5
  movwf TMR0
  movfw TMR0
  skpz
  goto $-2
  decfsz delay,F
  goto $-6
  movlw OPTION_INI
  option
  return

  #define _BLINK
  #ifdef _BLINK
  blink_byte EQU cmdBuffer+MSG_LEN 
; clignote valeur octet, bit plus significatif en premier, ROUGE=0, VERT=1
blink_binary 
  movlw B'100000111'
  option
  all_off
  movlw 8
  movwf bit_cntr
blink_loop
  rlf blink_byte,F 
  skpnc
  goto blink1
  light_on RED
  goto blink_delay
blink1
  light_on GREEN
blink_delay
  movlw 0x7F
  movlw .1
  movwf TMR0
  movfw TMR0
  skpz
  goto $-2
  clrf GPIO
  movlw .1
  movwf TMR0
  movfw TMR0
  skpz
  goto $-2
  decfsz bit_cntr,F
  goto blink_loop
  movlw OPTION_INI
  option
  return
  #endif ; _BLINK      
  #endif ; __DEBUG

;;;;;;;;;;;;;; random ;;;;;;;;;;;;;;;;;;;;;;;;
;; générateur de pseudo hasard utilisant la technique linear feedback shift register
;; REF: http://en.wikipedia.org/wiki/Linear_feedback_shift_register
;; variables:
;;  rand est le registre à décalage 24 bits
;;  temps d'exécution maximum 10Tcy
random 
  bcf STATUS, C
  rrf rand+2,F
  rrf rand+1,F
  rrf rand,F
  btfss STATUS , C
  return
  movlw RAND_XOR_MASK  
  xorwf rand+2, F
  return


;;;;;;;;;;;;;;;;;;    led_control  ;;;;;;;;;;;;;;;;;;;;;;;
;  controle la couleur de la del rgb par PWM
; variables:
;  pwm_timer est un compteur incrmenter à chaque boucle
;  red_level, green_level et blue_level détermine l'intensité pour cette couleur. 
;  La valeur de pwm_timer est comparée avec chacune des valeurs 'couleur'_level
;  et lorsque cette valeur atteint le seuil pwm cette couleur est éteinte.
;  durée d'exécution: maximum 20Tcy 
led_control
 btfss flags, F_LED_ON
 goto led_control_exit
 clrf led_bits 
 movfw green_level
 subwf pwm_timer, W ;comparaison avec le seuil vert
 skpc
 bsf led_bits, GREEN
 movfw blue_level
 subwf pwm_timer, W
 skpc
 bsf led_bits, BLUE
 movfw red_level
 subwf pwm_timer, W
 skpc
 bsf led_bits, RED
 movfw led_bits
 movwf GPIO
led_control_exit
 incf pwm_timer,F
 return

;;;;;;;;;;;;; load_color ;;;;;;;;;;;;;;;;;
; charge la couleur à partir du cmdBuffer
load_color
  incf FSR,F
  movfw INDF
  movwf red_level
  incf FSR, F
  movfw INDF
  movwf green_level
  incf FSR, F
  movfw INDF
  movwf blue_level
  bsf flags, F_LED_ON
  return


;;;;;;;;;;;;;;; process_command ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; traite la dernière commande recue
process_command
  btfsc flags, F_ERR_FRAME
  goto exit_processing
  movlw cmdBuffer
  movwf FSR
  movfw INDF
  skpz
  goto check_pixdel_id
  incf FSR, F
  movfw INDF
  andlw 0xF8
  xorwf groupId, W
  skpz
  goto exit_processing
  goto process_cmd01+1
check_pixdel_id
  xorlw PIXDEL_ID
  skpnz
  goto process_cmd01
  comf INDF,W ; vérifie si c'est 0xFF message diffusion
  skpz        ; si oui procède avec la commande
  goto exit_processing
process_cmd01
  incf FSR, F
  movfw INDF
  andlw 0x07
  addwf PCL, F
  goto cmd_led_off
  goto cmd_led_on
  goto cmd_set_color
  goto cmd_blink
  goto cmd_random
  goto cmd_group
  goto cmd_seed
  goto post
cmd_led_off
  bcf flags, F_LED_ON
  bcf flags, F_BLINK
  all_off
  goto exit_processing
cmd_led_on
  bsf flags, F_LED_ON
  bcf flags, F_BLINK
  goto exit_processing
cmd_set_color
  call load_color
  bcf flags, F_BLINK
  goto exit_processing
cmd_blink
  call load_color
  incf FSR,F
  movfw INDF
  movwf delay
  movwf delay_cnt
  bsf flags, F_BLINK
  goto exit_processing
cmd_random
  call random
  movfw rand
  movwf red_level
  movfw rand+1
  movwf green_level
  movfw rand+2 
  movwf blue_level
  bsf flags, F_LED_ON
  goto exit_processing
cmd_group
  incf FSR,F
  movfw INDF
  andlw 0x1F
  movwf groupId
  bcf STATUS, C
  rlf groupId,F
  rlf groupId,F 
  rlf groupId,F  ; groupId * 8
  goto exit_processing
cmd_seed
  incf FSR,F
  movfw INDF
  movwf rand
  incf FSR, F
  movfw INDF
  movwf rand+1
  incf FSR, F
  movfw INDF
  movwf rand+2
exit_processing
  movlw MSG_LEN
  movwf byte_cntr
  movlw cmdBuffer
  movwf FSR
  return

;;;;; post ;;;;;;;;;;;;;;;;
;; power on self test
post
  movlw .1
  movwf temp
post01
  movfw temp
  movwf GPIO
  movlw 0xFF
  movwf delay
  movlw .5
  movwf pwm_timer
post02
  movlw .6
  movwf TMR0
  subwf TMR0,W
  skpnc
  goto $-2
  decfsz delay,F 
  goto post02
  decfsz pwm_timer,F
  goto post02
  incf temp,F
  movlw 8
  xorwf temp,W
  skpz
  goto post01
  all_off
  clrf flags
  goto exit_processing


;;;;;;;;;;;;;;; receive_byte ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; réception des octets par protocole UART
;; variables:
;;  bit_cntr
;;  cmdBuffer
;;  byte_cntr
;;  indicateurs booléens:
;;   F_RX_BYTE
;;   F_READ_BIT
;;   F_ERR_FRAME

receive_byte
  btfsc flags, F_RX_BYTE     ;1,2
  goto receive_next_bit      ;3
  btfsc RX_INP               ;3,4
  return                     ;4
  ;initialisation pour réception d'un octet
init_rx_byte
 ; clrf recovery
  bsf flags, F_RX_BYTE       ;5
  bcf flags, F_READ_BIT      ;6
  bcf flags, F_ERR_FRAME
  movlw 8                    ;7
  movwf bit_cntr             ;8
  return                     ;10
receive_next_bit
  btfss flags, F_READ_BIT    ;4,5
  goto set_read_bit          ;7
  bcf flags, F_READ_BIT      ;6
  movf bit_cntr,F            ;7
  skpz                       ;8,9
  goto rotate_bit_in         ;10
  ; réception du bit d'arrêt
  bcf flags, F_RX_BYTE       ;10
  btfss RX_INP               ;11,12
  bsf flags, F_ERR_FRAME     ;12
  incf FSR,F                 ;13
  decfsz byte_cntr           ;14,15
  return                     ;15

 ; #define _SEND_MSG
  #ifdef _SEND_MSG
  movlw cmdBuffer
  movwf FSR
  movlw MSG_LEN
  movwf byte_cntr
blink_debug_loop
  movfw INDF
  movwf blink_byte
  call blink_binary
  decfsz byte_cntr
  goto $+2
  goto exit_processing
  incf FSR,F
  movlw 0x7F
  movwf delay
  call delay_ms
  goto blink_debug_loop
  #endif ; _SEND_MSG

  goto process_command       ;17+
rotate_bit_in  ; réception d'un bit
  bsf STATUS, C              ;11
  btfss RX_INP               ;12,13
  bcf STATUS, C              ;13
  rrf INDF, F                ;14
  decf bit_cntr,F            ;15
  return                     ;17
set_read_bit
  bsf flags, F_READ_BIT      ;8
  return                     ;10



;;;;;;;;;;;;;  initialisation ;;;;;;;;;;;;;;;;;;;;;;;
init
 movwf OSCCAL
 movwf rand ; je me sert de la contante de calibration de l'oscillateur pour initialiser le generateur pseudo-hazard
 movlw OPTION_INI
 option
 movlw TRIS_INI ; GP0, GP1, GP2 en sortie, GP3 entrée
 tris GPIO 
 ; éteint les 3 couleurs
 all_off
 clrf groupId
 clrf green_level
 clrf blue_level
 clrf red_level
 clrf flags
 clrf pwm_timer
 call post
 
 
;;;;;;;;;;;;;;;   procédure principale ;;;;;;;;;;;;;;;;;;;;;;;;;
;; l'ensemble des instructions entre 'init_timer' et 'wait_loop'
;; doit s'exécuter en un temps inférieur
;; au cycle du TIMER0 qui sert de référence de temps pour
;; la modulation PWM et pour la réception rs-232 sur GP3
main
  init_timer                   ;2
  call receive_byte            ;10|19|21|21+
main01
  call led_control             ;22|32|36|43|43+
  btfss flags, F_BLINK         ;23|24|19|35|36
  goto wait_loop               ;25|34   ;20|37
main02 ; clignotement actif
  movf pwm_timer,F             ;25|37
  skpz                         ;26|22|38|39
  goto wait_loop               ;28|37
  decfsz delay,F               ;28|24|40|41
  goto wait_loop               ;31|40
  movfw delay_cnt              ;31|42
  movwf delay                  ;32|43
  movlw XOR_BLK_PHASE          ;33|44
  xorwf flags,F                ;34|45
  all_off                      ;35|44
wait_loop     ;plus long parcour pour arrivé ici 46Tcy
  movlw .6
  subwf TMR0,W
  skpnc
  goto $-3
  goto main

  end
