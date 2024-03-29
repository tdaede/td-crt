EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Connection ~ 5450 2900
$Comp
L Transistor_FET:IPD50R3K0CE Q1
U 1 1 60CD0A49
P 5350 2700
F 0 "Q1" H 5554 2746 50  0000 L CNN
F 1 "IPD50R3K0CE" H 5554 2655 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:TO-252-2" H 5550 2625 50  0001 L CIN
F 3 "https://www.infineon.com/dgdl/IPx50R3K0CE_2_0.pdf?folderId=db3a3043163797a6011637d4bae7003b&fileId=db3a304339dcf4b10139e7e9ff592ce4" H 5350 2700 50  0001 L CNN
	1    5350 2700
	1    0    0    -1  
$EndComp
$Comp
L Device:R R29
U 1 1 5DA9EC34
P 2850 2600
F 0 "R29" H 2920 2646 50  0000 L CNN
F 1 "1M" H 2920 2555 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 2780 2600 50  0001 C CNN
F 3 "~" H 2850 2600 50  0001 C CNN
F 4 " 311-1.00MCRCT-ND" H 2850 2600 50  0001 C CNN "Digikey"
	1    2850 2600
	1    0    0    -1  
$EndComp
$Comp
L Device:R R30
U 1 1 5D9DC903
P 2850 2950
F 0 "R30" H 2920 2996 50  0000 L CNN
F 1 "37.4k" H 2920 2905 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 2780 2950 50  0001 C CNN
F 3 "~" H 2850 2950 50  0001 C CNN
F 4 " 	311-37.4KCRCT-ND " H 2850 2950 50  0001 C CNN "Digikey"
	1    2850 2950
	1    0    0    -1  
$EndComp
Connection ~ 2850 2800
Wire Wire Line
	2850 2750 2850 2800
Wire Wire Line
	4000 2800 2850 2800
$Comp
L power:GND #PWR0171
U 1 1 5D9DD502
P 4400 3600
F 0 "#PWR0171" H 4400 3350 50  0001 C CNN
F 1 "GND" H 4405 3427 50  0000 C CNN
F 2 "" H 4400 3600 50  0001 C CNN
F 3 "" H 4400 3600 50  0001 C CNN
	1    4400 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 3500 4400 3600
Connection ~ 4400 3500
Connection ~ 4400 3600
Wire Wire Line
	3750 3600 4400 3600
Wire Wire Line
	4400 3600 4900 3600
Wire Wire Line
	2850 2450 2850 2400
Connection ~ 2850 2400
Wire Wire Line
	2850 2400 3250 2400
Wire Wire Line
	2350 2400 2850 2400
$Comp
L Regulator_Switching:LT3757AEMSE U14
U 1 1 5D9DB5D2
P 4400 2900
F 0 "U14" H 4450 3581 50  0000 C CNN
F 1 "LT3758AEMSE" H 4450 3490 50  0000 C CNN
F 2 "Package_SO:MSOP-10-1EP_3x3mm_P0.5mm_EP1.68x1.88mm" H 4450 2350 50  0001 L CNN
F 3 "https://www.analog.com/media/en/technical-documentation/data-sheets/3757Afe.pdf" H 4400 2900 50  0001 C CNN
F 4 "LT3758AEMSE#TRPBFCT-ND" H 4400 2900 50  0001 C CNN "Digikey"
	1    4400 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 2900 3900 2900
Wire Wire Line
	3900 2900 3900 3500
Wire Wire Line
	3900 3500 4400 3500
$Comp
L Device:R R32
U 1 1 5D9DDAA1
P 3350 3300
F 0 "R32" V 3143 3300 50  0000 C CNN
F 1 "49.9k" V 3234 3300 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 3280 3300 50  0001 C CNN
F 3 "~" H 3350 3300 50  0001 C CNN
F 4 " 311-49.9KCRCT-ND" H 3350 3300 50  0001 C CNN "Digikey"
	1    3350 3300
	0    1    1    0   
$EndComp
Wire Wire Line
	3500 3300 3550 3300
Connection ~ 3550 3300
$Comp
L Device:C C21
U 1 1 5D9DE442
P 3550 3450
F 0 "C21" H 3665 3496 50  0000 L CNN
F 1 "100pF" H 3665 3405 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 3588 3300 50  0001 C CNN
F 3 "~" H 3550 3450 50  0001 C CNN
F 4 "10V" H 3550 3450 50  0001 C CNN "Voltage"
F 5 " 1276-2569-1-ND" H 3550 3450 50  0001 C CNN "Digikey"
	1    3550 3450
	1    0    0    -1  
$EndComp
Connection ~ 3550 3600
Wire Wire Line
	3200 3600 3550 3600
Wire Wire Line
	3550 3600 3750 3600
$Comp
L Device:C C20
U 1 1 5D9DEDB3
P 3200 3450
F 0 "C20" H 3315 3496 50  0000 L CNN
F 1 "0.01uF" H 3315 3405 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 3238 3300 50  0001 C CNN
F 3 "~" H 3200 3450 50  0001 C CNN
F 4 "10V" H 3200 3450 50  0001 C CNN "Voltage"
F 5 "1276-2434-1-ND" H 3200 3450 50  0001 C CNN "Digikey"
	1    3200 3450
	1    0    0    -1  
$EndComp
Connection ~ 3200 3600
Wire Wire Line
	3200 3600 3000 3600
Wire Wire Line
	2850 3600 2850 3100
Connection ~ 2850 3600
Wire Wire Line
	2350 3600 2850 3600
Wire Wire Line
	3000 3600 2850 3600
$Comp
L Device:C C22
U 1 1 5D9DFEAB
P 3750 3250
F 0 "C22" H 3865 3296 50  0000 L CNN
F 1 "10uF" H 3865 3205 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.33x1.80mm_HandSolder" H 3788 3100 50  0001 C CNN
F 3 "~" H 3750 3250 50  0001 C CNN
F 4 "10V" H 3750 3250 50  0001 C CNN "Voltage"
F 5 "1276-6641-1-ND " H 3750 3250 50  0001 C CNN "Digikey"
	1    3750 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 3100 4000 3100
Wire Wire Line
	3750 3400 3750 3600
Connection ~ 3750 3600
$Comp
L Device:R R31
U 1 1 5D9E07BB
P 3000 3450
F 0 "R31" H 3070 3496 50  0000 L CNN
F 1 "49.9k" H 3070 3405 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 2930 3450 50  0001 C CNN
F 3 "~" H 3000 3450 50  0001 C CNN
F 4 " 311-49.9KCRCT-ND" H 3000 3450 50  0001 C CNN "Digikey"
	1    3000 3450
	1    0    0    -1  
$EndComp
Connection ~ 3000 3600
Wire Wire Line
	3050 3300 3000 3300
Wire Wire Line
	4000 3000 3050 3000
Wire Wire Line
	3050 3000 3050 3300
$Comp
L Device:C C23
U 1 1 5D9E1097
P 4900 3450
F 0 "C23" H 5015 3496 50  0000 L CNN
F 1 "4.7uF" H 5015 3405 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 4938 3300 50  0001 C CNN
F 3 "~" H 4900 3450 50  0001 C CNN
F 4 "10V" H 4900 3450 50  0001 C CNN "Voltage"
F 5 "1276-3177-1-ND" H 4900 3450 50  0001 C CNN "Digikey"
	1    4900 3450
	1    0    0    -1  
$EndComp
Connection ~ 4900 3600
Wire Wire Line
	4900 3600 5450 3600
$Comp
L Device:R R37
U 1 1 5D9EA484
P 5450 3400
F 0 "R37" H 5520 3446 50  0000 L CNN
F 1 "0.01" H 5520 3355 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 5380 3400 50  0001 C CNN
F 3 "~" H 5450 3400 50  0001 C CNN
F 4 "311-0.01NUCT-ND" H 5450 3400 50  0001 C CNN "Digikey"
	1    5450 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5450 2900 4900 2900
Wire Wire Line
	5450 2900 5450 3250
Wire Wire Line
	5450 3600 5450 3550
Connection ~ 5450 3600
Wire Wire Line
	6250 3600 5450 3600
Wire Wire Line
	5450 2400 5450 2500
Wire Wire Line
	4900 2700 5150 2700
Wire Wire Line
	5600 2400 5450 2400
$Comp
L Device:R R39
U 1 1 5D9EC90B
P 6250 2550
F 0 "R39" H 6320 2596 50  0000 L CNN
F 1 "70K" H 6320 2505 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 6180 2550 50  0001 C CNN
F 3 "~" H 6250 2550 50  0001 C CNN
F 4 " 311-68.0KFRCT-ND" H 6250 2550 50  0001 C CNN "Digikey"
	1    6250 2550
	1    0    0    -1  
$EndComp
Connection ~ 6250 2400
Wire Wire Line
	5900 2400 6250 2400
Wire Wire Line
	6250 2400 6600 2400
$Comp
L Device:R R40
U 1 1 5D9ECEC9
P 6250 3300
F 0 "R40" H 6320 3346 50  0000 L CNN
F 1 "470" H 6320 3255 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 6180 3300 50  0001 C CNN
F 3 "~" H 6250 3300 50  0001 C CNN
F 4 " 311-470CRCT-ND" H 6250 3300 50  0001 C CNN "Digikey"
	1    6250 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 3450 6250 3600
Connection ~ 6250 3600
Wire Wire Line
	6600 3600 6250 3600
$Comp
L Device:CP C25
U 1 1 5D9ED334
P 6600 3050
F 0 "C25" H 6715 3096 50  0000 L CNN
F 1 "10uF" H 6715 3005 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D8.0mm_P3.50mm" H 6638 2900 50  0001 C CNN
F 3 "~" H 6600 3050 50  0001 C CNN
F 4 "200V" H 6600 3050 50  0001 C CNN "Voltage"
F 5 " UVK2D100MPD-ND " H 6600 3050 50  0001 C CNN "Digikey"
	1    6600 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	6600 2900 6600 2400
Connection ~ 6600 2400
Wire Wire Line
	6750 2400 6600 2400
Wire Wire Line
	6600 3200 6600 3600
Connection ~ 6600 3600
Wire Wire Line
	7000 3600 6600 3600
$Comp
L Device:C C26
U 1 1 5D9ED8A0
P 7000 3050
F 0 "C26" H 7115 3096 50  0000 L CNN
F 1 "0.1uF" H 7115 3005 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.33x1.80mm_HandSolder" H 7038 2900 50  0001 C CNN
F 3 "~" H 7000 3050 50  0001 C CNN
F 4 "200V" H 7000 3050 50  0001 C CNN "Voltage"
F 5 "732-12118-1-ND" H 7000 3050 50  0001 C CNN "Digikey"
	1    7000 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7000 3200 7000 3600
$Comp
L Device:C C18
U 1 1 5D9EE026
P 2350 2900
F 0 "C18" H 2465 2946 50  0000 L CNN
F 1 "22uF" H 2465 2855 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D8.0mm_P3.50mm" H 2388 2750 50  0001 C CNN
F 3 "~" H 2350 2900 50  0001 C CNN
F 4 "63V" H 2350 2900 50  0001 C CNN "Voltage"
F 5 "493-3872-1-ND " H 2350 2900 50  0001 C CNN "Digikey"
	1    2350 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 2750 2350 2400
Wire Wire Line
	2350 3050 2350 3600
$Comp
L Device:R_POT RV1
U 1 1 5D9EE8DD
P 6250 3000
F 0 "RV1" H 6180 2954 50  0000 R CNN
F 1 "1k" H 6180 3045 50  0000 R CNN
F 2 "Potentiometer_THT:Potentiometer_Bourns_3386P_Vertical" H 6250 3000 50  0001 C CNN
F 3 "~" H 6250 3000 50  0001 C CNN
F 4 "3386P-102LF-ND" H 6250 3000 50  0001 C CNN "Digikey"
	1    6250 3000
	-1   0    0    1   
$EndComp
Wire Wire Line
	6250 2850 6250 2700
Wire Wire Line
	6100 3050 6100 3000
Wire Wire Line
	6100 3050 4900 3050
Wire Wire Line
	4900 3050 4900 3100
Wire Wire Line
	7000 2400 6750 2400
Connection ~ 6750 2400
Wire Wire Line
	7000 2900 7000 2400
Wire Wire Line
	3550 3300 4000 3300
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 5DC1CAB2
P 6750 2400
F 0 "#FLG0101" H 6750 2475 50  0001 C CNN
F 1 "PWR_FLAG" H 6750 2573 50  0000 C CNN
F 2 "" H 6750 2400 50  0001 C CNN
F 3 "~" H 6750 2400 50  0001 C CNN
	1    6750 2400
	1    0    0    -1  
$EndComp
Connection ~ 5450 2400
Wire Wire Line
	5150 2400 5450 2400
$Comp
L Device:L L3
U 1 1 5D9EAD84
P 5000 2400
F 0 "L3" V 5190 2400 50  0000 C CNN
F 1 "33uH" V 5099 2400 50  0000 C CNN
F 2 "Inductor_SMD:L_Taiyo-Yuden_NR-60xx_HandSoldering" H 5000 2400 50  0001 C CNN
F 3 "~" H 5000 2400 50  0001 C CNN
F 4 "587-2631-1-ND" V 5000 2400 50  0001 C CNN "Digikey"
	1    5000 2400
	0    -1   -1   0   
$EndComp
$Comp
L Diode:C3D02060E D1
U 1 1 60E13622
P 5750 2400
F 0 "D1" H 5750 2183 50  0000 C CNN
F 1 "C3D02060E" H 5750 2274 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:TO-252-2_TabPin1" H 5750 2225 50  0001 C CNN
F 3 "https://www.wolfspeed.com/media/downloads/116/C3D02060E.pdf" H 5750 2400 50  0001 C CNN
F 4 "C3D02060E-ND" H 5750 2400 50  0001 C CNN "Digikey"
	1    5750 2400
	-1   0    0    1   
$EndComp
$Comp
L Mechanical:MountingHole H1
U 1 1 60E164B2
P 6850 4300
F 0 "H1" H 6950 4346 50  0000 L CNN
F 1 "MountingHole" H 6950 4255 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3" H 6850 4300 50  0001 C CNN
F 3 "~" H 6850 4300 50  0001 C CNN
	1    6850 4300
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H2
U 1 1 60E16BE7
P 7650 4250
F 0 "H2" H 7750 4296 50  0000 L CNN
F 1 "MountingHole" H 7750 4205 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3" H 7650 4250 50  0001 C CNN
F 3 "~" H 7650 4250 50  0001 C CNN
	1    7650 4250
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x05_Male J1
U 1 1 60E1A185
P 4000 4400
F 0 "J1" H 4108 4781 50  0000 C CNN
F 1 "Conn_01x05_Male" H 4108 4690 50  0000 C CNN
F 2 "Connector_JST:JST_XH_S5B-XH-A_1x05_P2.50mm_Horizontal" H 4000 4400 50  0001 C CNN
F 3 "~" H 4000 4400 50  0001 C CNN
F 4 "455-2242-ND" H 4000 4400 50  0001 C CNN "Digikey"
	1    4000 4400
	1    0    0    -1  
$EndComp
Text Label 4200 4200 0    50   ~ 0
160v_in
$Comp
L power:GND #PWR0103
U 1 1 60E1BD74
P 4600 4400
F 0 "#PWR0103" H 4600 4150 50  0001 C CNN
F 1 "GND" H 4605 4227 50  0000 C CNN
F 2 "" H 4600 4400 50  0001 C CNN
F 3 "" H 4600 4400 50  0001 C CNN
	1    4600 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 4400 4200 4400
Text Label 4200 4600 0    50   ~ 0
g1
Text Label 5550 4500 0    50   ~ 0
heater_out
$Comp
L Connector:Conn_01x05_Male J3
U 1 1 60E1DC4D
P 5350 4400
F 0 "J3" H 5458 4781 50  0000 C CNN
F 1 "Conn_01x05_Male" H 5458 4690 50  0000 C CNN
F 2 "Connector_JST:JST_XH_B5B-XH-A_1x05_P2.50mm_Vertical" H 5350 4400 50  0001 C CNN
F 3 "~" H 5350 4400 50  0001 C CNN
	1    5350 4400
	1    0    0    -1  
$EndComp
Text Label 5550 4200 0    50   ~ 0
180v_out
Text Label 6950 2400 0    50   ~ 0
180v_out
$Comp
L power:+12V #PWR0101
U 1 1 60E3F77B
P 4200 4300
F 0 "#PWR0101" H 4200 4150 50  0001 C CNN
F 1 "+12V" V 4215 4428 50  0000 L CNN
F 2 "" H 4200 4300 50  0001 C CNN
F 3 "" H 4200 4300 50  0001 C CNN
	1    4200 4300
	0    1    1    0   
$EndComp
NoConn ~ 5550 4300
Text Label 4200 4500 0    50   ~ 0
heater_in
Text Label 5550 4600 0    50   ~ 0
g1
$Comp
L power:GND #PWR0102
U 1 1 60E41271
P 6150 4400
F 0 "#PWR0102" H 6150 4150 50  0001 C CNN
F 1 "GND" H 6155 4227 50  0000 C CNN
F 2 "" H 6150 4400 50  0001 C CNN
F 3 "" H 6150 4400 50  0001 C CNN
	1    6150 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 4400 6150 4400
$Comp
L Jumper:SolderJumper_2_Bridged JP3
U 1 1 60E444DF
P 4850 1750
F 0 "JP3" V 4804 1818 50  0000 L CNN
F 1 "SolderJumper_2_Bridged" V 4895 1818 50  0000 L CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Bridged_Pad1.0x1.5mm" H 4850 1750 50  0001 C CNN
F 3 "~" H 4850 1750 50  0001 C CNN
	1    4850 1750
	0    1    1    0   
$EndComp
Text Label 4850 1600 1    50   ~ 0
160v_in
$Comp
L Jumper:SolderJumper_2_Open JP1
U 1 1 60E5DBCF
P 4350 2050
F 0 "JP1" H 4350 2255 50  0000 C CNN
F 1 "SolderJumper_2_Open" H 4350 2164 50  0000 C CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Open_Pad1.0x1.5mm" H 4350 2050 50  0001 C CNN
F 3 "~" H 4350 2050 50  0001 C CNN
	1    4350 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 1900 4850 2050
Wire Wire Line
	4500 2050 4850 2050
Connection ~ 4850 2050
Wire Wire Line
	4850 2050 4850 2400
Wire Wire Line
	4200 2050 3950 2050
Wire Wire Line
	3950 2050 3950 2400
Connection ~ 3950 2400
Wire Wire Line
	3950 2400 4400 2400
$Comp
L power:+12V #PWR0104
U 1 1 60E63C51
P 2350 2400
F 0 "#PWR0104" H 2350 2250 50  0001 C CNN
F 1 "+12V" H 2365 2573 50  0000 C CNN
F 2 "" H 2350 2400 50  0001 C CNN
F 3 "" H 2350 2400 50  0001 C CNN
	1    2350 2400
	1    0    0    -1  
$EndComp
Connection ~ 2350 2400
$Comp
L Jumper:SolderJumper_2_Bridged JP2
U 1 1 60E65B2F
P 4350 5450
F 0 "JP2" H 4350 5655 50  0000 C CNN
F 1 "SolderJumper_2_Bridged" H 4350 5564 50  0000 C CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Bridged_Pad1.0x1.5mm" H 4350 5450 50  0001 C CNN
F 3 "~" H 4350 5450 50  0001 C CNN
	1    4350 5450
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Linear:AMS1117CD U1
U 1 1 60E69238
P 2600 5150
F 0 "U1" H 2600 5392 50  0000 C CNN
F 1 "AMS1117CD" H 2600 5301 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:TO-252-3_TabPin2" H 2600 5350 50  0001 C CNN
F 3 "http://www.advanced-monolithic.com/pdf/ds1117.pdf" H 2700 4900 50  0001 C CNN
F 4 "NCP1117DT33T5GOSCT-ND" H 2600 5150 50  0001 C CNN "Digikey"
	1    2600 5150
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 60E696A0
P 3300 5300
F 0 "C1" H 3415 5346 50  0000 L CNN
F 1 "22uF" H 3415 5255 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D8.0mm_P3.50mm" H 3338 5150 50  0001 C CNN
F 3 "~" H 3300 5300 50  0001 C CNN
F 4 "63V" H 3300 5300 50  0001 C CNN "Voltage"
F 5 "493-3872-1-ND " H 3300 5300 50  0001 C CNN "Digikey"
	1    3300 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	2900 5150 2950 5150
$Comp
L Device:R R2
U 1 1 60E6B695
P 2950 5300
F 0 "R2" H 3020 5346 50  0000 L CNN
F 1 "R" H 3020 5255 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 2880 5300 50  0001 C CNN
F 3 "~" H 2950 5300 50  0001 C CNN
	1    2950 5300
	1    0    0    -1  
$EndComp
Connection ~ 2950 5150
Wire Wire Line
	2950 5150 3300 5150
Text Label 4500 5450 0    50   ~ 0
heater_out
Text Label 3300 5150 0    50   ~ 0
heater_out
Text Label 4200 5450 2    50   ~ 0
heater_in
$Comp
L power:+12V #PWR0105
U 1 1 60E6D372
P 2200 5150
F 0 "#PWR0105" H 2200 5000 50  0001 C CNN
F 1 "+12V" V 2215 5278 50  0000 L CNN
F 2 "" H 2200 5150 50  0001 C CNN
F 3 "" H 2200 5150 50  0001 C CNN
	1    2200 5150
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0106
U 1 1 60E6D88B
P 2600 5800
F 0 "#PWR0106" H 2600 5550 50  0001 C CNN
F 1 "GND" H 2605 5627 50  0000 C CNN
F 2 "" H 2600 5800 50  0001 C CNN
F 3 "" H 2600 5800 50  0001 C CNN
	1    2600 5800
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 60E6DFC1
P 2600 5600
F 0 "R1" H 2670 5646 50  0000 L CNN
F 1 "R" H 2670 5555 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" V 2530 5600 50  0001 C CNN
F 3 "~" H 2600 5600 50  0001 C CNN
	1    2600 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 5800 2600 5750
Wire Wire Line
	2950 5450 2600 5450
Connection ~ 2600 5450
Wire Wire Line
	2300 5150 2200 5150
$Comp
L power:GND #PWR0107
U 1 1 60E83083
P 3300 5450
F 0 "#PWR0107" H 3300 5200 50  0001 C CNN
F 1 "GND" H 3305 5277 50  0000 C CNN
F 2 "" H 3300 5450 50  0001 C CNN
F 3 "" H 3300 5450 50  0001 C CNN
	1    3300 5450
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 60E8380E
P 3250 2400
F 0 "#FLG0102" H 3250 2475 50  0001 C CNN
F 1 "PWR_FLAG" H 3250 2573 50  0000 C CNN
F 2 "" H 3250 2400 50  0001 C CNN
F 3 "~" H 3250 2400 50  0001 C CNN
	1    3250 2400
	1    0    0    -1  
$EndComp
Connection ~ 3250 2400
Wire Wire Line
	3250 2400 3950 2400
$Comp
L power:PWR_FLAG #FLG0103
U 1 1 60E8470E
P 3300 5450
F 0 "#FLG0103" H 3300 5525 50  0001 C CNN
F 1 "PWR_FLAG" V 3300 5578 50  0000 L CNN
F 2 "" H 3300 5450 50  0001 C CNN
F 3 "~" H 3300 5450 50  0001 C CNN
	1    3300 5450
	0    1    1    0   
$EndComp
Connection ~ 3300 5450
$EndSCHEMATC
