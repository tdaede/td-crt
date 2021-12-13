EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 4
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Device:C C?
U 1 1 618EBD5D
P 2900 2550
AR Path="/618EBD5D" Ref="C?"  Part="1" 
AR Path="/618D9864/618EBD5D" Ref="C12"  Part="1" 
AR Path="/618F5F7A/618EBD5D" Ref="C16"  Part="1" 
AR Path="/618FC4A2/618EBD5D" Ref="C20"  Part="1" 
F 0 "C12" V 2648 2550 50  0000 C CNN
F 1 "1uF" V 2739 2550 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 2938 2400 50  0001 C CNN
F 3 "~" H 2900 2550 50  0001 C CNN
F 4 "1276-2926-1-ND" H 2900 2550 50  0001 C CNN "Digikey"
	1    2900 2550
	0    1    1    0   
$EndComp
Wire Wire Line
	2750 2550 2500 2550
$Comp
L Device:R R?
U 1 1 618EBD65
P 3050 2700
AR Path="/618EBD65" Ref="R?"  Part="1" 
AR Path="/618D9864/618EBD65" Ref="R9"  Part="1" 
AR Path="/618F5F7A/618EBD65" Ref="R20"  Part="1" 
AR Path="/618FC4A2/618EBD65" Ref="R24"  Part="1" 
F 0 "R9" H 3120 2746 50  0000 L CNN
F 1 "100k" H 3120 2655 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2980 2700 50  0001 C CNN
F 3 "~" H 3050 2700 50  0001 C CNN
F 4 "311-10.0KCRCT-ND" H 3050 2700 50  0001 C CNN "Digikey"
	1    3050 2700
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:2N7002 Q?
U 1 1 618EBD6C
P 3750 2750
AR Path="/618EBD6C" Ref="Q?"  Part="1" 
AR Path="/618D9864/618EBD6C" Ref="Q1"  Part="1" 
AR Path="/618F5F7A/618EBD6C" Ref="Q3"  Part="1" 
AR Path="/618FC4A2/618EBD6C" Ref="Q5"  Part="1" 
F 0 "Q1" H 3954 2796 50  0000 L CNN
F 1 "2N7002NXBKR" H 3650 2500 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 3950 2675 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/NDS7002A-D.PDF" H 3750 2750 50  0001 L CNN
F 4 "1727-8642-1-ND" H 3750 2750 50  0001 C CNN "Digikey"
	1    3750 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3850 2950 3050 2950
Wire Wire Line
	3050 2850 3050 2950
Wire Wire Line
	3850 2550 3050 2550
Connection ~ 3050 2550
$Comp
L td-rgbinput:AD8337 U?
U 1 1 618EBD78
P 5050 2650
AR Path="/618EBD78" Ref="U?"  Part="1" 
AR Path="/618D9864/618EBD78" Ref="U6"  Part="1" 
AR Path="/618F5F7A/618EBD78" Ref="U7"  Part="1" 
AR Path="/618FC4A2/618EBD78" Ref="U8"  Part="1" 
F 0 "U6" H 5150 2900 50  0000 L CNN
F 1 "AD8337" H 5150 2800 50  0000 L CNN
F 2 "Package_CSP:LFCSP-8-1EP_3x3mm_P0.5mm_EP1.45x1.74mm" H 5100 2700 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/ne5534.pdf" H 5100 2800 50  0001 C CNN
F 4 "AD8337BCPZ-WP" H 5050 2650 50  0001 C CNN "Digikey"
	1    5050 2650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 618EBD7E
P 4550 3450
AR Path="/618EBD7E" Ref="R?"  Part="1" 
AR Path="/618D9864/618EBD7E" Ref="R14"  Part="1" 
AR Path="/618F5F7A/618EBD7E" Ref="R21"  Part="1" 
AR Path="/618FC4A2/618EBD7E" Ref="R25"  Part="1" 
F 0 "R14" H 4480 3404 50  0000 R CNN
F 1 "470" H 4480 3495 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 4480 3450 50  0001 C CNN
F 3 "~" H 4550 3450 50  0001 C CNN
F 4 "YAG3733CT-ND" H 4550 3450 50  0001 C CNN "Digikey"
	1    4550 3450
	-1   0    0    1   
$EndComp
$Comp
L Device:R R?
U 1 1 618EBD84
P 4700 3250
AR Path="/618EBD84" Ref="R?"  Part="1" 
AR Path="/618D9864/618EBD84" Ref="R16"  Part="1" 
AR Path="/618F5F7A/618EBD84" Ref="R22"  Part="1" 
AR Path="/618FC4A2/618EBD84" Ref="R26"  Part="1" 
F 0 "R16" V 4493 3250 50  0000 C CNN
F 1 "470" V 4584 3250 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 4630 3250 50  0001 C CNN
F 3 "~" H 4700 3250 50  0001 C CNN
F 4 "YAG3733CT-ND" H 4700 3250 50  0001 C CNN "Digikey"
	1    4700 3250
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 618EBD8A
P 4550 3600
AR Path="/618EBD8A" Ref="#PWR?"  Part="1" 
AR Path="/618D9864/618EBD8A" Ref="#PWR030"  Part="1" 
AR Path="/618F5F7A/618EBD8A" Ref="#PWR044"  Part="1" 
AR Path="/618FC4A2/618EBD8A" Ref="#PWR055"  Part="1" 
F 0 "#PWR030" H 4550 3350 50  0001 C CNN
F 1 "GND" H 4555 3427 50  0000 C CNN
F 2 "" H 4550 3600 50  0001 C CNN
F 3 "" H 4550 3600 50  0001 C CNN
	1    4550 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 3300 4550 3250
Wire Wire Line
	4550 3250 4550 2750
Wire Wire Line
	4550 2750 4750 2750
Connection ~ 4550 3250
Wire Wire Line
	5150 2950 5150 3250
Wire Wire Line
	5150 3250 4850 3250
$Comp
L power:-5V #PWR?
U 1 1 618EBD96
P 4950 3000
AR Path="/618EBD96" Ref="#PWR?"  Part="1" 
AR Path="/618D9864/618EBD96" Ref="#PWR036"  Part="1" 
AR Path="/618F5F7A/618EBD96" Ref="#PWR047"  Part="1" 
AR Path="/618FC4A2/618EBD96" Ref="#PWR058"  Part="1" 
F 0 "#PWR036" H 4950 3100 50  0001 C CNN
F 1 "-5V" H 4965 3173 50  0000 C CNN
F 2 "" H 4950 3000 50  0001 C CNN
F 3 "" H 4950 3000 50  0001 C CNN
	1    4950 3000
	-1   0    0    1   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 618EBD9C
P 4950 2350
AR Path="/618EBD9C" Ref="#PWR?"  Part="1" 
AR Path="/618D9864/618EBD9C" Ref="#PWR035"  Part="1" 
AR Path="/618F5F7A/618EBD9C" Ref="#PWR046"  Part="1" 
AR Path="/618FC4A2/618EBD9C" Ref="#PWR057"  Part="1" 
F 0 "#PWR035" H 4950 2200 50  0001 C CNN
F 1 "+5V" H 4965 2523 50  0000 C CNN
F 2 "" H 4950 2350 50  0001 C CNN
F 3 "" H 4950 2350 50  0001 C CNN
	1    4950 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3850 2550 4750 2550
Connection ~ 3850 2550
$Comp
L power:GND #PWR?
U 1 1 618EBDA4
P 5050 2950
AR Path="/618EBDA4" Ref="#PWR?"  Part="1" 
AR Path="/618D9864/618EBDA4" Ref="#PWR037"  Part="1" 
AR Path="/618F5F7A/618EBDA4" Ref="#PWR048"  Part="1" 
AR Path="/618FC4A2/618EBDA4" Ref="#PWR059"  Part="1" 
F 0 "#PWR037" H 5050 2700 50  0001 C CNN
F 1 "GND" H 5055 2777 50  0000 C CNN
F 2 "" H 5050 2950 50  0001 C CNN
F 3 "" H 5050 2950 50  0001 C CNN
	1    5050 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 2650 5500 2650
Wire Wire Line
	4950 3000 4950 2950
$Comp
L Device:R_POT RV?
U 1 1 618EBDAC
P 4250 1950
AR Path="/618EBDAC" Ref="RV?"  Part="1" 
AR Path="/618D9864/618EBDAC" Ref="RV4"  Part="1" 
AR Path="/618F5F7A/618EBDAC" Ref="RV6"  Part="1" 
AR Path="/618FC4A2/618EBDAC" Ref="RV8"  Part="1" 
F 0 "RV4" H 4180 1996 50  0000 R CNN
F 1 "1k" H 4180 1905 50  0000 R CNN
F 2 "Potentiometer_THT:Potentiometer_Bourns_3386P_Vertical" H 4250 1950 50  0001 C CNN
F 3 "~" H 4250 1950 50  0001 C CNN
F 4 "3386P-102TLF-ND" H 4250 1950 50  0001 C CNN "Digikey"
	1    4250 1950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 618EBDB2
P 4500 2250
AR Path="/618EBDB2" Ref="#PWR?"  Part="1" 
AR Path="/618D9864/618EBDB2" Ref="#PWR029"  Part="1" 
AR Path="/618F5F7A/618EBDB2" Ref="#PWR043"  Part="1" 
AR Path="/618FC4A2/618EBDB2" Ref="#PWR054"  Part="1" 
F 0 "#PWR029" H 4500 2000 50  0001 C CNN
F 1 "GND" H 4505 2077 50  0000 C CNN
F 2 "" H 4500 2250 50  0001 C CNN
F 3 "" H 4500 2250 50  0001 C CNN
	1    4500 2250
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 618EBDB9
P 5650 2650
AR Path="/618EBDB9" Ref="R?"  Part="1" 
AR Path="/618D9864/618EBDB9" Ref="R17"  Part="1" 
AR Path="/618F5F7A/618EBDB9" Ref="R23"  Part="1" 
AR Path="/618FC4A2/618EBDB9" Ref="R27"  Part="1" 
F 0 "R17" V 5443 2650 50  0000 C CNN
F 1 "1k" V 5534 2650 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 5580 2650 50  0001 C CNN
F 3 "~" H 5650 2650 50  0001 C CNN
F 4 "311-1.00KCRCT-ND" H 5650 2650 50  0001 C CNN "Digikey"
	1    5650 2650
	0    1    1    0   
$EndComp
Wire Wire Line
	5800 2650 5850 2650
$Comp
L Transistor_FET:2N7002 Q?
U 1 1 618EBDC1
P 4700 4000
AR Path="/618EBDC1" Ref="Q?"  Part="1" 
AR Path="/618D9864/618EBDC1" Ref="Q2"  Part="1" 
AR Path="/618F5F7A/618EBDC1" Ref="Q4"  Part="1" 
AR Path="/618FC4A2/618EBDC1" Ref="Q6"  Part="1" 
F 0 "Q2" H 4904 4046 50  0000 L CNN
F 1 "2N7002NXBKR" H 4150 3850 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 4900 3925 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/NDS7002A-D.PDF" H 4700 4000 50  0001 L CNN
F 4 "1727-8642-1-ND" H 4700 4000 50  0001 C CNN "Digikey"
	1    4700 4000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 618EBDC7
P 4800 4200
AR Path="/618EBDC7" Ref="#PWR?"  Part="1" 
AR Path="/618D9864/618EBDC7" Ref="#PWR031"  Part="1" 
AR Path="/618F5F7A/618EBDC7" Ref="#PWR045"  Part="1" 
AR Path="/618FC4A2/618EBDC7" Ref="#PWR056"  Part="1" 
F 0 "#PWR031" H 4800 3950 50  0001 C CNN
F 1 "GND" H 4805 4027 50  0000 C CNN
F 2 "" H 4800 4200 50  0001 C CNN
F 3 "" H 4800 4200 50  0001 C CNN
	1    4800 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 3800 4800 3650
Wire Wire Line
	4800 3650 5450 3650
Wire Wire Line
	5450 3650 5450 2750
Wire Wire Line
	5450 2750 5850 2750
Wire Wire Line
	5850 2750 5850 2650
$Comp
L Device:C C?
U 1 1 618EBDD4
P 4500 2100
AR Path="/618EBDD4" Ref="C?"  Part="1" 
AR Path="/618D9864/618EBDD4" Ref="C14"  Part="1" 
AR Path="/618F5F7A/618EBDD4" Ref="C18"  Part="1" 
AR Path="/618FC4A2/618EBDD4" Ref="C22"  Part="1" 
F 0 "C14" H 4615 2146 50  0000 L CNN
F 1 "0.1uF" H 4615 2055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4538 1950 50  0001 C CNN
F 3 "~" H 4500 2100 50  0001 C CNN
F 4 "1276-2444-1-ND" H 4500 2100 50  0001 C CNN "Digikey"
	1    4500 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 1950 4400 1950
Wire Wire Line
	4500 2250 4250 2250
Wire Wire Line
	4250 2250 4250 2100
Connection ~ 4500 2250
Wire Wire Line
	5050 1950 5050 2350
Connection ~ 4500 1950
Wire Wire Line
	4500 1950 5050 1950
Wire Wire Line
	5850 2650 6150 2650
Connection ~ 5850 2650
$Comp
L Device:R_POT RV?
U 1 1 618EBDE3
P 6000 3450
AR Path="/618EBDE3" Ref="RV?"  Part="1" 
AR Path="/618D9864/618EBDE3" Ref="RV5"  Part="1" 
AR Path="/618F5F7A/618EBDE3" Ref="RV7"  Part="1" 
AR Path="/618FC4A2/618EBDE3" Ref="RV9"  Part="1" 
F 0 "RV5" H 5930 3496 50  0000 R CNN
F 1 "1k" H 5930 3405 50  0000 R CNN
F 2 "Potentiometer_THT:Potentiometer_Bourns_3386P_Vertical" H 6000 3450 50  0001 C CNN
F 3 "~" H 6000 3450 50  0001 C CNN
F 4 "3386P-102TLF-ND" H 6000 3450 50  0001 C CNN "Digikey"
	1    6000 3450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 618EBDEA
P 6000 3600
AR Path="/618EBDEA" Ref="#PWR?"  Part="1" 
AR Path="/618D9864/618EBDEA" Ref="#PWR038"  Part="1" 
AR Path="/618F5F7A/618EBDEA" Ref="#PWR049"  Part="1" 
AR Path="/618FC4A2/618EBDEA" Ref="#PWR060"  Part="1" 
F 0 "#PWR038" H 6000 3350 50  0001 C CNN
F 1 "GND" H 6005 3427 50  0000 C CNN
F 2 "" H 6000 3600 50  0001 C CNN
F 3 "" H 6000 3600 50  0001 C CNN
	1    6000 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	6150 3450 6150 2650
Connection ~ 6150 2650
Wire Wire Line
	6150 2650 6400 2650
Text Notes 5050 1550 0    50   ~ 0
note that with a very high bias it's possible to exceed the Vbe rating of the neck board's transistors
$Comp
L Device:C C?
U 1 1 618EBDF4
P 3350 2000
AR Path="/618EBDF4" Ref="C?"  Part="1" 
AR Path="/618D9864/618EBDF4" Ref="C13"  Part="1" 
AR Path="/618F5F7A/618EBDF4" Ref="C17"  Part="1" 
AR Path="/618FC4A2/618EBDF4" Ref="C21"  Part="1" 
F 0 "C13" H 3465 2046 50  0000 L CNN
F 1 "0.1uF" H 3465 1955 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3388 1850 50  0001 C CNN
F 3 "~" H 3350 2000 50  0001 C CNN
F 4 "1276-2444-1-ND" H 3350 2000 50  0001 C CNN "Digikey"
	1    3350 2000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 618EBDFA
P 3350 2150
AR Path="/618EBDFA" Ref="#PWR?"  Part="1" 
AR Path="/618D9864/618EBDFA" Ref="#PWR028"  Part="1" 
AR Path="/618F5F7A/618EBDFA" Ref="#PWR042"  Part="1" 
AR Path="/618FC4A2/618EBDFA" Ref="#PWR053"  Part="1" 
F 0 "#PWR028" H 3350 1900 50  0001 C CNN
F 1 "GND" H 3355 1977 50  0000 C CNN
F 2 "" H 3350 2150 50  0001 C CNN
F 3 "" H 3350 2150 50  0001 C CNN
	1    3350 2150
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 618EBE00
P 3350 1850
AR Path="/618EBE00" Ref="#PWR?"  Part="1" 
AR Path="/618D9864/618EBE00" Ref="#PWR027"  Part="1" 
AR Path="/618F5F7A/618EBE00" Ref="#PWR041"  Part="1" 
AR Path="/618FC4A2/618EBE00" Ref="#PWR052"  Part="1" 
F 0 "#PWR027" H 3350 1700 50  0001 C CNN
F 1 "+5V" H 3365 2023 50  0000 C CNN
F 2 "" H 3350 1850 50  0001 C CNN
F 3 "" H 3350 1850 50  0001 C CNN
	1    3350 1850
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 618EBE06
P 2550 2000
AR Path="/618EBE06" Ref="C?"  Part="1" 
AR Path="/618D9864/618EBE06" Ref="C10"  Part="1" 
AR Path="/618F5F7A/618EBE06" Ref="C15"  Part="1" 
AR Path="/618FC4A2/618EBE06" Ref="C19"  Part="1" 
F 0 "C10" H 2665 2046 50  0000 L CNN
F 1 "0.1uF" H 2665 1955 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 2588 1850 50  0001 C CNN
F 3 "~" H 2550 2000 50  0001 C CNN
F 4 "1276-2444-1-ND" H 2550 2000 50  0001 C CNN "Digikey"
	1    2550 2000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 618EBE0C
P 2550 2150
AR Path="/618EBE0C" Ref="#PWR?"  Part="1" 
AR Path="/618D9864/618EBE0C" Ref="#PWR026"  Part="1" 
AR Path="/618F5F7A/618EBE0C" Ref="#PWR040"  Part="1" 
AR Path="/618FC4A2/618EBE0C" Ref="#PWR051"  Part="1" 
F 0 "#PWR026" H 2550 1900 50  0001 C CNN
F 1 "GND" H 2555 1977 50  0000 C CNN
F 2 "" H 2550 2150 50  0001 C CNN
F 3 "" H 2550 2150 50  0001 C CNN
	1    2550 2150
	1    0    0    -1  
$EndComp
$Comp
L power:-5V #PWR?
U 1 1 618EBE12
P 2550 1850
AR Path="/618EBE12" Ref="#PWR?"  Part="1" 
AR Path="/618D9864/618EBE12" Ref="#PWR025"  Part="1" 
AR Path="/618F5F7A/618EBE12" Ref="#PWR039"  Part="1" 
AR Path="/618FC4A2/618EBE12" Ref="#PWR050"  Part="1" 
F 0 "#PWR025" H 2550 1950 50  0001 C CNN
F 1 "-5V" H 2565 2023 50  0000 C CNN
F 2 "" H 2550 1850 50  0001 C CNN
F 3 "" H 2550 1850 50  0001 C CNN
	1    2550 1850
	1    0    0    -1  
$EndComp
Text HLabel 2500 2550 0    50   Input ~ 0
input
Text HLabel 6400 2650 2    50   Input ~ 0
output
Text Label 3550 2550 0    50   ~ 0
ac
Text GLabel 3550 2750 0    50   Input ~ 0
clamp
Text GLabel 3050 2950 0    50   Input ~ 0
black_level
Text GLabel 4500 4000 0    50   Input ~ 0
blank
Text Notes 5050 1450 0    50   ~ 0
this design is very simple - offsets are not nulled along the chain
Text GLabel 4250 1800 0    50   Input ~ 0
gain_buffered
Text GLabel 6000 3300 0    50   Input ~ 0
bias_buffered
Text Notes 6050 4000 0    50   ~ 0
todo: bias impedance is not constant
$EndSCHEMATC
