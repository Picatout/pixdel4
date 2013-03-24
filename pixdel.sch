EESchema Schematic File Version 2  date 2013-03-07 19:42:18
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:special
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
EELAYER 43  0
EELAYER END
$Descr A4 11700 8267
Sheet 1 1
Title ""
Date "8 mar 2013"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text Notes 9350 2000 0    60   ~ 0
gnd\n
Text Notes 9350 1900 0    60   ~ 0
+5V
Text Notes 9350 1800 0    60   ~ 0
RS-232 TTL
Wire Wire Line
	7550 4500 7550 4700
Wire Wire Line
	6350 2550 7550 2550
Wire Wire Line
	7550 2550 7550 3100
Wire Wire Line
	4850 2350 4400 2350
Wire Wire Line
	4400 2350 4400 2900
Wire Wire Line
	4400 2900 7800 2900
Wire Wire Line
	7800 2900 7800 3100
Connection ~ 3800 2100
Wire Wire Line
	4850 2100 3800 2100
Connection ~ 8850 2100
Wire Wire Line
	8850 2200 8850 2000
Wire Wire Line
	8850 2100 6350 2100
Wire Wire Line
	8850 1900 8100 1900
Wire Wire Line
	8100 1900 8100 1300
Wire Wire Line
	8100 1300 3800 1300
Wire Wire Line
	3800 1300 3800 2150
Wire Wire Line
	3800 2550 3800 2650
Wire Wire Line
	7300 3100 4700 3100
Wire Wire Line
	4700 3100 4700 2550
Wire Wire Line
	4700 2550 4850 2550
Wire Wire Line
	8850 1800 7100 1800
Wire Wire Line
	7100 1800 7100 1850
Wire Wire Line
	7100 1850 6350 1850
$Comp
L GND #PWR1
U 1 1 51393347
P 3800 2650
F 0 "#PWR1" H 3800 2650 30  0001 C CNN
F 1 "GND" H 3800 2580 30  0001 C CNN
	1    3800 2650
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 5139331E
P 3800 2350
F 0 "C1" H 3850 2450 50  0000 L CNN
F 1 "100nF" H 3850 2250 50  0000 L CNN
	1    3800 2350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR2
U 1 1 51393306
P 7550 4700
F 0 "#PWR2" H 7550 4700 30  0001 C CNN
F 1 "GND" H 7550 4630 30  0001 C CNN
	1    7550 4700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR3
U 1 1 51393300
P 8850 2200
F 0 "#PWR3" H 8850 2200 30  0001 C CNN
F 1 "GND" H 8850 2130 30  0001 C CNN
	1    8850 2200
	1    0    0    -1  
$EndComp
$Comp
L CONN_3 K1
U 1 1 513932F2
P 9200 1900
F 0 "K1" V 9150 1900 50  0000 C CNN
F 1 "CONN_3" V 9250 1900 40  0000 C CNN
	1    9200 1900
	1    0    0    -1  
$EndComp
$Comp
L RGB_LED D1
U 1 1 513932C7
P 7550 3750
F 0 "D1" H 7000 4000 60  0000 C CNN
F 1 "RGB_LED" H 7000 3350 60  0000 C CNN
	1    7550 3750
	1    0    0    -1  
$EndComp
$Comp
L PIC10F202 U1
U 1 1 513932A0
P 4750 1100
F 0 "U1" H 5600 0   60  0000 C CNN
F 1 "PIC10F200" H 5550 600 60  0000 C CNN
	1    4750 1100
	1    0    0    -1  
$EndComp
$EndSCHEMATC
