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
$Comp
L td-io:ADA4891-3 U3
U 1 1 600EAC98
P 9500 1400
F 0 "U3" H 9500 1767 50  0000 C CNN
F 1 "ADA4861-3" H 9500 1676 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 9500 1400 50  0001 C CNN
F 3 "https://www.analog.com/media/en/technical-documentation/data-sheets/ADA4891-1_4891-2_4891-3_4891-4.PDF" H 9500 1400 50  0001 C CNN
F 4 "ADA4861-3YRZ-RL7CT-ND" H 9500 1400 50  0001 C CNN "Digikey"
	1    9500 1400
	1    0    0    -1  
$EndComp
$Comp
L Device:R R15
U 1 1 60050CFF
P 9450 2000
F 0 "R15" V 9243 2000 50  0000 C CNN
F 1 "470" V 9334 2000 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 9380 2000 50  0001 C CNN
F 3 "~" H 9450 2000 50  0001 C CNN
F 4 "311-1.00KCRCT-ND" H 9450 2000 50  0001 C CNN "Digikey"
	1    9450 2000
	0    1    1    0   
$EndComp
Wire Wire Line
	9200 1500 9200 2000
Wire Wire Line
	9200 2000 9300 2000
Wire Wire Line
	9600 2000 9800 2000
Wire Wire Line
	9800 2000 9800 1400
$Comp
L Device:R R12
U 1 1 6005B872
P 9200 2150
F 0 "R12" H 9130 2104 50  0000 R CNN
F 1 "470" H 9130 2195 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 9130 2150 50  0001 C CNN
F 3 "~" H 9200 2150 50  0001 C CNN
F 4 "311-330CRCT-ND" H 9200 2150 50  0001 C CNN "Digikey"
	1    9200 2150
	-1   0    0    1   
$EndComp
Connection ~ 9200 2000
$Comp
L power:GND #PWR0128
U 1 1 60061134
P 9200 2400
F 0 "#PWR0128" H 9200 2150 50  0001 C CNN
F 1 "GND" H 9205 2227 50  0000 C CNN
F 2 "" H 9200 2400 50  0001 C CNN
F 3 "" H 9200 2400 50  0001 C CNN
	1    9200 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	9200 2400 9200 2300
$Comp
L Device:R R9
U 1 1 6006B3E7
P 5350 1350
F 0 "R9" H 5280 1304 50  0000 R CNN
F 1 "75" H 5280 1395 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 5280 1350 50  0001 C CNN
F 3 "~" H 5350 1350 50  0001 C CNN
F 4 "311-75.0CRCT-ND" H 5350 1350 50  0001 C CNN "Digikey"
	1    5350 1350
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0130
U 1 1 60070693
P 5600 1600
F 0 "#PWR0130" H 5600 1350 50  0001 C CNN
F 1 "GND" H 5605 1427 50  0000 C CNN
F 2 "" H 5600 1600 50  0001 C CNN
F 3 "" H 5600 1600 50  0001 C CNN
	1    5600 1600
	1    0    0    -1  
$EndComp
$Comp
L Device:R R18
U 1 1 60071155
P 10050 1400
F 0 "R18" V 9843 1400 50  0000 C CNN
F 1 "47" V 9934 1400 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 9980 1400 50  0001 C CNN
F 3 "~" H 10050 1400 50  0001 C CNN
F 4 "311-75.0CRCT-ND" H 10050 1400 50  0001 C CNN "Digikey"
	1    10050 1400
	0    1    1    0   
$EndComp
Wire Wire Line
	9900 1400 9800 1400
Connection ~ 9800 1400
Text Label 10200 1400 0    50   ~ 0
neck_red
Text Label 5350 1200 2    50   ~ 0
vga_red
$Comp
L td-io:ADA4891-3 U3
U 2 1 60095C91
P 10000 3050
F 0 "U3" H 10000 3417 50  0000 C CNN
F 1 "ADA4861-3" H 10000 3326 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 10000 3050 50  0001 C CNN
F 3 "https://www.analog.com/media/en/technical-documentation/data-sheets/ADA4891-1_4891-2_4891-3_4891-4.PDF" H 10000 3050 50  0001 C CNN
F 4 "ADA4861-3YRZ-RL7CT-ND" H 10000 3050 50  0001 C CNN "Digikey"
	2    10000 3050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R16
U 1 1 60095C97
P 9950 3650
F 0 "R16" V 9743 3650 50  0000 C CNN
F 1 "1k" V 9834 3650 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 9880 3650 50  0001 C CNN
F 3 "~" H 9950 3650 50  0001 C CNN
F 4 "311-1.00KCRCT-ND" H 9950 3650 50  0001 C CNN "Digikey"
	1    9950 3650
	0    1    1    0   
$EndComp
Wire Wire Line
	9700 3150 9700 3650
Wire Wire Line
	9700 3650 9800 3650
Wire Wire Line
	10100 3650 10300 3650
Wire Wire Line
	10300 3650 10300 3050
$Comp
L Device:R R13
U 1 1 60095CA1
P 9700 3800
F 0 "R13" H 9630 3754 50  0000 R CNN
F 1 "330" H 9630 3845 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 9630 3800 50  0001 C CNN
F 3 "~" H 9700 3800 50  0001 C CNN
F 4 "311-330CRCT-ND" H 9700 3800 50  0001 C CNN "Digikey"
	1    9700 3800
	-1   0    0    1   
$EndComp
Connection ~ 9700 3650
$Comp
L power:GND #PWR0131
U 1 1 60095CA8
P 9700 4050
F 0 "#PWR0131" H 9700 3800 50  0001 C CNN
F 1 "GND" H 9705 3877 50  0000 C CNN
F 2 "" H 9700 4050 50  0001 C CNN
F 3 "" H 9700 4050 50  0001 C CNN
	1    9700 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	9700 4050 9700 3950
$Comp
L Device:R R19
U 1 1 60095CC3
P 10550 3050
F 0 "R19" V 10343 3050 50  0000 C CNN
F 1 "75" V 10434 3050 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 10480 3050 50  0001 C CNN
F 3 "~" H 10550 3050 50  0001 C CNN
F 4 "311-75.0CRCT-ND" H 10550 3050 50  0001 C CNN "Digikey"
	1    10550 3050
	0    1    1    0   
$EndComp
Wire Wire Line
	10400 3050 10300 3050
Connection ~ 10300 3050
Text Label 10700 3050 0    50   ~ 0
jamma_green
$Comp
L td-io:ADA4891-3 U3
U 3 1 6009DE48
P 9950 4600
F 0 "U3" H 9950 4967 50  0000 C CNN
F 1 "ADA4861-3" H 9950 4876 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 9950 4600 50  0001 C CNN
F 3 "https://www.analog.com/media/en/technical-documentation/data-sheets/ADA4891-1_4891-2_4891-3_4891-4.PDF" H 9950 4600 50  0001 C CNN
F 4 "ADA4861-3YRZ-RL7CT-ND" H 9950 4600 50  0001 C CNN "Digikey"
	3    9950 4600
	1    0    0    -1  
$EndComp
$Comp
L Device:R R14
U 1 1 6009DE4E
P 9900 5200
F 0 "R14" V 9693 5200 50  0000 C CNN
F 1 "1k" V 9784 5200 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 9830 5200 50  0001 C CNN
F 3 "~" H 9900 5200 50  0001 C CNN
F 4 "311-1.00KCRCT-ND" H 9900 5200 50  0001 C CNN "Digikey"
	1    9900 5200
	0    1    1    0   
$EndComp
Wire Wire Line
	9650 4700 9650 5200
Wire Wire Line
	9650 5200 9750 5200
Wire Wire Line
	10050 5200 10250 5200
Wire Wire Line
	10250 5200 10250 4600
$Comp
L Device:R R11
U 1 1 6009DE58
P 9650 5350
F 0 "R11" H 9580 5304 50  0000 R CNN
F 1 "330" H 9580 5395 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 9580 5350 50  0001 C CNN
F 3 "~" H 9650 5350 50  0001 C CNN
F 4 "311-330CRCT-ND" H 9650 5350 50  0001 C CNN "Digikey"
	1    9650 5350
	-1   0    0    1   
$EndComp
Connection ~ 9650 5200
$Comp
L power:GND #PWR0134
U 1 1 6009DE5F
P 9650 5600
F 0 "#PWR0134" H 9650 5350 50  0001 C CNN
F 1 "GND" H 9655 5427 50  0000 C CNN
F 2 "" H 9650 5600 50  0001 C CNN
F 3 "" H 9650 5600 50  0001 C CNN
	1    9650 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	9650 5600 9650 5500
$Comp
L Device:R R17
U 1 1 6009DE7A
P 10500 4600
F 0 "R17" V 10293 4600 50  0000 C CNN
F 1 "75" V 10384 4600 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 10430 4600 50  0001 C CNN
F 3 "~" H 10500 4600 50  0001 C CNN
F 4 "311-75.0CRCT-ND" H 10500 4600 50  0001 C CNN "Digikey"
	1    10500 4600
	0    1    1    0   
$EndComp
Wire Wire Line
	10350 4600 10250 4600
Connection ~ 10250 4600
Text Label 10650 4600 0    50   ~ 0
jamma_blue
$Comp
L Memory_EEPROM:AT24CS01-SSHM U5
U 1 1 607C39AA
P 2700 6450
F 0 "U5" H 2800 6900 50  0000 C CNN
F 1 "AT24CS01-SSHM" H 3100 6750 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 2700 6450 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-8815-SEEPROM-AT24CS01-02-Datasheet.pdf" H 2700 6450 50  0001 C CNN
F 4 "AT24CS01-SSHM-TCT-ND" H 2700 6450 50  0001 C CNN "Digikey"
	1    2700 6450
	1    0    0    -1  
$EndComp
Wire Wire Line
	1850 6850 3300 6850
Wire Wire Line
	3300 6850 3300 6450
Wire Wire Line
	3300 6450 3100 6450
$Comp
L power:GND #PWR02
U 1 1 607D8BDF
P 2700 6950
F 0 "#PWR02" H 2700 6700 50  0001 C CNN
F 1 "GND" H 2705 6777 50  0000 C CNN
F 2 "" H 2700 6950 50  0001 C CNN
F 3 "" H 2700 6950 50  0001 C CNN
	1    2700 6950
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 6950 2700 6900
Wire Wire Line
	3100 6550 3100 6750
Wire Wire Line
	3100 6750 2700 6750
Connection ~ 2700 6750
Wire Wire Line
	2700 6750 2300 6750
Wire Wire Line
	2300 6750 2300 6550
Wire Wire Line
	2300 6550 2300 6450
Connection ~ 2300 6550
Wire Wire Line
	2300 6450 2300 6350
Connection ~ 2300 6450
Wire Wire Line
	3100 6350 3300 6350
Wire Wire Line
	3300 6350 3300 6100
Wire Wire Line
	3300 6100 2100 6100
Wire Wire Line
	2100 6100 2100 6250
Wire Wire Line
	2100 6250 1850 6250
$Comp
L power:PWR_FLAG #FLG01
U 1 1 60855E9C
P 2700 5600
F 0 "#FLG01" H 2700 5675 50  0001 C CNN
F 1 "PWR_FLAG" H 2700 5773 50  0000 C CNN
F 2 "" H 2700 5600 50  0001 C CNN
F 3 "~" H 2700 5600 50  0001 C CNN
	1    2700 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 6150 2700 5750
Wire Wire Line
	1250 6550 850  6550
Wire Wire Line
	850  6550 850  5750
Wire Wire Line
	850  5750 2700 5750
Connection ~ 2700 5750
Wire Wire Line
	2700 5750 2700 5600
Text Label 1850 6450 0    50   ~ 0
vga_hsync
Text Label 1850 6650 0    50   ~ 0
vga_vsync
Text Label 1250 6450 2    50   ~ 0
vga_blue
Text Label 1250 6250 2    50   ~ 0
vga_green
Text Label 1250 6050 2    50   ~ 0
vga_red
$Comp
L power:GND #PWR01
U 1 1 608837A8
P 750 7000
F 0 "#PWR01" H 750 6750 50  0001 C CNN
F 1 "GND" H 755 6827 50  0000 C CNN
F 2 "" H 750 7000 50  0001 C CNN
F 3 "" H 750 7000 50  0001 C CNN
	1    750  7000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 6850 750  6850
Wire Wire Line
	750  6850 750  7000
Wire Wire Line
	1250 5950 750  5950
Wire Wire Line
	750  5950 750  6150
Connection ~ 750  6850
Wire Wire Line
	1250 6750 750  6750
Connection ~ 750  6750
Wire Wire Line
	750  6750 750  6850
Wire Wire Line
	1250 6350 750  6350
Connection ~ 750  6350
Wire Wire Line
	750  6350 750  6750
Wire Wire Line
	1250 6150 750  6150
Connection ~ 750  6150
Wire Wire Line
	750  6150 750  6350
NoConn ~ 1250 6650
$Comp
L Device:C C7
U 1 1 6092E164
P 3600 6350
F 0 "C7" H 3715 6396 50  0000 L CNN
F 1 "0.1uF" H 3715 6305 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3638 6200 50  0001 C CNN
F 3 "~" H 3600 6350 50  0001 C CNN
F 4 "1276-2444-1-ND" H 3600 6350 50  0001 C CNN "Digikey"
	1    3600 6350
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 6900 3600 6900
Wire Wire Line
	3600 6900 3600 6500
Connection ~ 2700 6900
Wire Wire Line
	2700 6900 2700 6750
Wire Wire Line
	3600 6200 3600 5750
Wire Wire Line
	3600 5750 2700 5750
Wire Wire Line
	9500 1600 9850 1600
$Comp
L power:+5V #PWR0103
U 1 1 60B4034B
P 9850 1600
F 0 "#PWR0103" H 9850 1450 50  0001 C CNN
F 1 "+5V" H 9865 1773 50  0000 C CNN
F 2 "" H 9850 1600 50  0001 C CNN
F 3 "" H 9850 1600 50  0001 C CNN
	1    9850 1600
	0    1    1    0   
$EndComp
Wire Wire Line
	10000 3250 10350 3250
$Comp
L power:+5V #PWR0104
U 1 1 60BCD45D
P 10350 3250
F 0 "#PWR0104" H 10350 3100 50  0001 C CNN
F 1 "+5V" H 10365 3423 50  0000 C CNN
F 2 "" H 10350 3250 50  0001 C CNN
F 3 "" H 10350 3250 50  0001 C CNN
	1    10350 3250
	0    1    1    0   
$EndComp
Wire Wire Line
	9950 4800 10300 4800
$Comp
L power:+5V #PWR0105
U 1 1 60C555BE
P 10300 4800
F 0 "#PWR0105" H 10300 4650 50  0001 C CNN
F 1 "+5V" H 10315 4973 50  0000 C CNN
F 2 "" H 10300 4800 50  0001 C CNN
F 3 "" H 10300 4800 50  0001 C CNN
	1    10300 4800
	0    1    1    0   
$EndComp
NoConn ~ 1850 6050
$Comp
L Connector:DB15_Female_HighDensity_MountingHoles J5
U 1 1 60E92B60
P 1550 6450
F 0 "J5" H 1550 7317 50  0000 C CNN
F 1 "DB15_Female_HighDensity_MountingHoles" H 1550 7226 50  0000 C CNN
F 2 "td-io:DSUB-15-HD_Female_Horizontal_P2.29x2.5mm_EdgePinOffset8.35mm_Housed_MountingHolesOffset10.89mm" H 600 6850 50  0001 C CNN
F 3 " ~" H 600 6850 50  0001 C CNN
F 4 "Z13226-ND" H 1550 6450 50  0001 C CNN "Digikey"
	1    1550 6450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0114
U 1 1 60E94FC9
P 1550 7150
F 0 "#PWR0114" H 1550 6900 50  0001 C CNN
F 1 "GND" H 1555 6977 50  0000 C CNN
F 2 "" H 1550 7150 50  0001 C CNN
F 3 "" H 1550 7150 50  0001 C CNN
	1    1550 7150
	1    0    0    -1  
$EndComp
$Comp
L td-io:LM2664M4 U11
U 1 1 60EA99E4
P 4900 5850
F 0 "U11" H 4900 6217 50  0000 C CNN
F 1 "LM2664M4" H 4900 6126 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-6" H 4950 5500 50  0001 L CNN
F 3 "https://www.ti.com/lit/ds/symlink/lm2665.pdf" H 3050 7100 50  0001 C CNN
F 4 "LM2664M6/NOPBCT-ND" H 4900 5850 50  0001 C CNN "Digikey"
	1    4900 5850
	1    0    0    -1  
$EndComp
$Comp
L Device:C C30
U 1 1 60EAD422
P 5550 6000
F 0 "C30" H 5665 6046 50  0000 L CNN
F 1 "3.3uF" H 5665 5955 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5588 5850 50  0001 C CNN
F 3 "~" H 5550 6000 50  0001 C CNN
F 4 "1276-6461-1-ND" H 5550 6000 50  0001 C CNN "Digikey"
	1    5550 6000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 5850 5550 5850
Wire Wire Line
	5300 5950 5300 6150
Wire Wire Line
	5300 6150 5550 6150
$Comp
L Device:C C31
U 1 1 60F37814
P 6000 5850
F 0 "C31" H 6115 5896 50  0000 L CNN
F 1 "3.3uF" H 6115 5805 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 6038 5700 50  0001 C CNN
F 3 "~" H 6000 5850 50  0001 C CNN
F 4 "1276-6461-1-ND" H 6000 5850 50  0001 C CNN "Digikey"
	1    6000 5850
	1    0    0    -1  
$EndComp
$Comp
L Device:C C28
U 1 1 60F37C7E
P 4150 5900
F 0 "C28" H 4265 5946 50  0000 L CNN
F 1 "3.3uF" H 4265 5855 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4188 5750 50  0001 C CNN
F 3 "~" H 4150 5900 50  0001 C CNN
F 4 "1276-6461-1-ND" H 4150 5900 50  0001 C CNN "Digikey"
	1    4150 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 6050 4150 6150
Wire Wire Line
	4150 6150 4900 6150
$Comp
L power:GND #PWR0127
U 1 1 60F7DA3C
P 4900 6200
F 0 "#PWR0127" H 4900 5950 50  0001 C CNN
F 1 "GND" H 4905 6027 50  0000 C CNN
F 2 "" H 4900 6200 50  0001 C CNN
F 3 "" H 4900 6200 50  0001 C CNN
	1    4900 6200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4900 6200 6000 6200
Wire Wire Line
	6000 6200 6000 6000
Wire Wire Line
	6000 5700 5650 5700
Wire Wire Line
	5350 5700 5350 5750
Wire Wire Line
	5350 5750 5300 5750
$Comp
L power:-5V #PWR0129
U 1 1 61052A9C
P 5650 5700
F 0 "#PWR0129" H 5650 5800 50  0001 C CNN
F 1 "-5V" H 5665 5873 50  0000 C CNN
F 2 "" H 5650 5700 50  0001 C CNN
F 3 "" H 5650 5700 50  0001 C CNN
	1    5650 5700
	1    0    0    -1  
$EndComp
Connection ~ 5650 5700
Wire Wire Line
	5650 5700 5350 5700
Wire Wire Line
	4500 5750 4150 5750
$Comp
L power:+5V #PWR0132
U 1 1 6109B7C9
P 4150 5750
F 0 "#PWR0132" H 4150 5600 50  0001 C CNN
F 1 "+5V" H 4165 5923 50  0000 C CNN
F 2 "" H 4150 5750 50  0001 C CNN
F 3 "" H 4150 5750 50  0001 C CNN
	1    4150 5750
	1    0    0    -1  
$EndComp
Connection ~ 4150 5750
Wire Wire Line
	4500 5950 4500 5750
Connection ~ 4500 5750
Wire Wire Line
	4900 6200 4900 6150
Connection ~ 4900 6200
Connection ~ 4900 6150
$Comp
L Device:C C39
U 1 1 607EC225
P 5750 1200
F 0 "C39" V 5498 1200 50  0000 C CNN
F 1 "1uF" V 5589 1200 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5788 1050 50  0001 C CNN
F 3 "~" H 5750 1200 50  0001 C CNN
F 4 "1276-2926-1-ND" H 5750 1200 50  0001 C CNN "Digikey"
	1    5750 1200
	0    1    1    0   
$EndComp
Wire Wire Line
	5600 1200 5350 1200
$Comp
L Device:R R54
U 1 1 60892DF7
P 5900 1350
F 0 "R54" H 5970 1396 50  0000 L CNN
F 1 "10k" H 5970 1305 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 5830 1350 50  0001 C CNN
F 3 "~" H 5900 1350 50  0001 C CNN
F 4 "311-10.0KCRCT-ND" H 5900 1350 50  0001 C CNN "Digikey"
	1    5900 1350
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:2N7002 Q5
U 1 1 608E8C23
P 6600 1400
F 0 "Q5" H 6804 1446 50  0000 L CNN
F 1 "2N7002NXBKR" H 6500 1150 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 6800 1325 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/NDS7002A-D.PDF" H 6600 1400 50  0001 L CNN
F 4 "1727-8642-1-ND" H 6600 1400 50  0001 C CNN "Digikey"
	1    6600 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 1600 5900 1600
Text Label 6400 1400 2    50   ~ 0
clamp
Wire Wire Line
	5350 1500 5350 1600
Wire Wire Line
	5350 1600 5600 1600
Connection ~ 5600 1600
Wire Wire Line
	5900 1500 5900 1600
Connection ~ 5900 1600
Wire Wire Line
	5900 1600 5600 1600
Wire Wire Line
	6700 1200 5900 1200
Connection ~ 5900 1200
Wire Wire Line
	8750 1300 9200 1300
$Comp
L Device:R R10
U 1 1 60C5CFC5
P 5350 2900
F 0 "R10" H 5280 2854 50  0000 R CNN
F 1 "75" H 5280 2945 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 5280 2900 50  0001 C CNN
F 3 "~" H 5350 2900 50  0001 C CNN
F 4 "311-75.0CRCT-ND" H 5350 2900 50  0001 C CNN "Digikey"
	1    5350 2900
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR024
U 1 1 60C5CFCB
P 5600 3150
F 0 "#PWR024" H 5600 2900 50  0001 C CNN
F 1 "GND" H 5605 2977 50  0000 C CNN
F 2 "" H 5600 3150 50  0001 C CNN
F 3 "" H 5600 3150 50  0001 C CNN
	1    5600 3150
	1    0    0    -1  
$EndComp
Text Label 5350 2750 2    50   ~ 0
vga_green
$Comp
L Device:C C40
U 1 1 60C5CFD2
P 5750 2750
F 0 "C40" V 5498 2750 50  0000 C CNN
F 1 "1uF" V 5589 2750 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5788 2600 50  0001 C CNN
F 3 "~" H 5750 2750 50  0001 C CNN
F 4 "1276-2926-1-ND" H 5750 2750 50  0001 C CNN "Digikey"
	1    5750 2750
	0    1    1    0   
$EndComp
Wire Wire Line
	5600 2750 5350 2750
$Comp
L Device:R R59
U 1 1 60C5CFD9
P 5900 2900
F 0 "R59" H 5970 2946 50  0000 L CNN
F 1 "10k" H 5970 2855 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 5830 2900 50  0001 C CNN
F 3 "~" H 5900 2900 50  0001 C CNN
F 4 "311-10.0KCRCT-ND" H 5900 2900 50  0001 C CNN "Digikey"
	1    5900 2900
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:2N7002 Q6
U 1 1 60C5CFE0
P 6600 2950
F 0 "Q6" H 6804 2996 50  0000 L CNN
F 1 "2N7002NXBKR" H 6500 2700 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 6800 2875 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/NDS7002A-D.PDF" H 6600 2950 50  0001 L CNN
F 4 "1727-8642-1-ND" H 6600 2950 50  0001 C CNN "Digikey"
	1    6600 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 3150 5900 3150
Text Label 6400 2950 2    50   ~ 0
blank
Wire Wire Line
	5350 3050 5350 3150
Wire Wire Line
	5350 3150 5600 3150
Connection ~ 5600 3150
Wire Wire Line
	5900 3050 5900 3150
Connection ~ 5900 3150
Wire Wire Line
	5900 3150 5600 3150
Wire Wire Line
	6700 2750 5900 2750
Connection ~ 5900 2750
Wire Wire Line
	9250 2950 9700 2950
$Comp
L Device:R R8
U 1 1 60D0F878
P 5300 4450
F 0 "R8" H 5230 4404 50  0000 R CNN
F 1 "75" H 5230 4495 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 5230 4450 50  0001 C CNN
F 3 "~" H 5300 4450 50  0001 C CNN
F 4 "311-75.0CRCT-ND" H 5300 4450 50  0001 C CNN "Digikey"
	1    5300 4450
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR023
U 1 1 60D0F87E
P 5550 4700
F 0 "#PWR023" H 5550 4450 50  0001 C CNN
F 1 "GND" H 5555 4527 50  0000 C CNN
F 2 "" H 5550 4700 50  0001 C CNN
F 3 "" H 5550 4700 50  0001 C CNN
	1    5550 4700
	1    0    0    -1  
$EndComp
Text Label 5300 4300 2    50   ~ 0
vga_blue
$Comp
L Device:C C38
U 1 1 60D0F885
P 5700 4300
F 0 "C38" V 5448 4300 50  0000 C CNN
F 1 "1uF" V 5539 4300 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5738 4150 50  0001 C CNN
F 3 "~" H 5700 4300 50  0001 C CNN
F 4 "1276-2926-1-ND" H 5700 4300 50  0001 C CNN "Digikey"
	1    5700 4300
	0    1    1    0   
$EndComp
Wire Wire Line
	5550 4300 5300 4300
$Comp
L Device:R R53
U 1 1 60D0F88C
P 5850 4450
F 0 "R53" H 5920 4496 50  0000 L CNN
F 1 "10k" H 5920 4405 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 5780 4450 50  0001 C CNN
F 3 "~" H 5850 4450 50  0001 C CNN
F 4 "311-10.0KCRCT-ND" H 5850 4450 50  0001 C CNN "Digikey"
	1    5850 4450
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:2N7002 Q4
U 1 1 60D0F893
P 6550 4500
F 0 "Q4" H 6754 4546 50  0000 L CNN
F 1 "2N7002NXBKR" H 6450 4250 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 6750 4425 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/NDS7002A-D.PDF" H 6550 4500 50  0001 L CNN
F 4 "1727-8642-1-ND" H 6550 4500 50  0001 C CNN "Digikey"
	1    6550 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	6650 4700 5850 4700
Text Label 6350 4500 2    50   ~ 0
blank
Wire Wire Line
	5300 4600 5300 4700
Wire Wire Line
	5300 4700 5550 4700
Connection ~ 5550 4700
Wire Wire Line
	5850 4600 5850 4700
Connection ~ 5850 4700
Wire Wire Line
	5850 4700 5550 4700
Wire Wire Line
	6650 4300 5850 4300
Connection ~ 5850 4300
Wire Wire Line
	9200 4500 9650 4500
Text Label 6250 1200 0    50   ~ 0
red_ac
Text Label 6250 2750 0    50   ~ 0
green_ac
Text Label 6250 4300 0    50   ~ 0
blue_ac
Text Label 9650 5100 1    50   ~ 0
blue_fb
Text Label 9700 3550 1    50   ~ 0
green_fb
Text Label 9200 1850 1    50   ~ 0
red_fb
$Comp
L power:PWR_FLAG #FLG0104
U 1 1 60B4AAF6
P 6000 5700
F 0 "#FLG0104" H 6000 5775 50  0001 C CNN
F 1 "PWR_FLAG" H 6000 5873 50  0000 C CNN
F 2 "" H 6000 5700 50  0001 C CNN
F 3 "~" H 6000 5700 50  0001 C CNN
	1    6000 5700
	1    0    0    -1  
$EndComp
Connection ~ 6000 5700
$Comp
L td-rgbinput:AD8337 U?
U 1 1 612E0D2A
P 7900 1300
F 0 "U?" H 8000 1550 50  0000 L CNN
F 1 "AD8337" H 8000 1450 50  0000 L CNN
F 2 "" H 7950 1350 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/ne5534.pdf" H 7950 1450 50  0001 C CNN
	1    7900 1300
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 61307C28
P 7400 2100
F 0 "R?" H 7330 2054 50  0000 R CNN
F 1 "470" H 7330 2145 50  0000 R CNN
F 2 "" V 7330 2100 50  0001 C CNN
F 3 "~" H 7400 2100 50  0001 C CNN
	1    7400 2100
	-1   0    0    1   
$EndComp
$Comp
L Device:R R?
U 1 1 61308D53
P 7550 1900
F 0 "R?" V 7343 1900 50  0000 C CNN
F 1 "470" V 7434 1900 50  0000 C CNN
F 2 "" V 7480 1900 50  0001 C CNN
F 3 "~" H 7550 1900 50  0001 C CNN
	1    7550 1900
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 6130A29D
P 7400 2250
F 0 "#PWR?" H 7400 2000 50  0001 C CNN
F 1 "GND" H 7405 2077 50  0000 C CNN
F 2 "" H 7400 2250 50  0001 C CNN
F 3 "" H 7400 2250 50  0001 C CNN
	1    7400 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	7400 1950 7400 1900
Wire Wire Line
	7400 1900 7400 1400
Wire Wire Line
	7400 1400 7600 1400
Connection ~ 7400 1900
Wire Wire Line
	8000 1600 8000 1900
Wire Wire Line
	8000 1900 7700 1900
$Comp
L power:-5V #PWR?
U 1 1 61317194
P 7800 1650
F 0 "#PWR?" H 7800 1750 50  0001 C CNN
F 1 "-5V" H 7815 1823 50  0000 C CNN
F 2 "" H 7800 1650 50  0001 C CNN
F 3 "" H 7800 1650 50  0001 C CNN
	1    7800 1650
	-1   0    0    1   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 6131797C
P 7800 1000
F 0 "#PWR?" H 7800 850 50  0001 C CNN
F 1 "+5V" H 7815 1173 50  0000 C CNN
F 2 "" H 7800 1000 50  0001 C CNN
F 3 "" H 7800 1000 50  0001 C CNN
	1    7800 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 1200 7600 1200
Connection ~ 6700 1200
$Comp
L power:GND #PWR?
U 1 1 6131D370
P 7900 1600
F 0 "#PWR?" H 7900 1350 50  0001 C CNN
F 1 "GND" H 7905 1427 50  0000 C CNN
F 2 "" H 7900 1600 50  0001 C CNN
F 3 "" H 7900 1600 50  0001 C CNN
	1    7900 1600
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 6131F02F
P 8500 1300
F 0 "R?" V 8293 1300 50  0000 C CNN
F 1 "1k" V 8384 1300 50  0000 C CNN
F 2 "" V 8430 1300 50  0001 C CNN
F 3 "~" H 8500 1300 50  0001 C CNN
	1    8500 1300
	0    1    1    0   
$EndComp
Wire Wire Line
	8200 1300 8350 1300
$Comp
L Device:R_POT RV?
U 1 1 61325DD1
P 8550 1700
F 0 "RV?" H 8480 1746 50  0000 R CNN
F 1 "1k" H 8480 1655 50  0000 R CNN
F 2 "" H 8550 1700 50  0001 C CNN
F 3 "~" H 8550 1700 50  0001 C CNN
	1    8550 1700
	1    0    0    -1  
$EndComp
Text Label 8550 1550 2    50   ~ 0
bias_buffered
$Comp
L power:GND #PWR?
U 1 1 61327AD1
P 8550 1850
F 0 "#PWR?" H 8550 1600 50  0001 C CNN
F 1 "GND" H 8555 1677 50  0000 C CNN
F 2 "" H 8550 1850 50  0001 C CNN
F 3 "" H 8550 1850 50  0001 C CNN
	1    8550 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	8650 1300 8750 1300
Wire Wire Line
	8750 1300 8750 1700
Wire Wire Line
	8750 1700 8700 1700
Connection ~ 8750 1300
Wire Wire Line
	7800 1650 7800 1600
$Comp
L Device:R_POT RV?
U 1 1 6134073E
P 7350 700
F 0 "RV?" H 7280 746 50  0000 R CNN
F 1 "1k" H 7280 655 50  0000 R CNN
F 2 "" H 7350 700 50  0001 C CNN
F 3 "~" H 7350 700 50  0001 C CNN
	1    7350 700 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 61341BD2
P 7350 850
F 0 "#PWR?" H 7350 600 50  0001 C CNN
F 1 "GND" H 7355 677 50  0000 C CNN
F 2 "" H 7350 850 50  0001 C CNN
F 3 "" H 7350 850 50  0001 C CNN
	1    7350 850 
	1    0    0    -1  
$EndComp
Text Label 7350 550  2    50   ~ 0
gain_buffered
Wire Wire Line
	7500 700  7900 700 
Wire Wire Line
	7900 700  7900 1000
$Comp
L Switch:SW_DIP_x04 SW?
U 1 1 613F6891
P 1750 4850
F 0 "SW?" H 1750 5317 50  0000 C CNN
F 1 "SW_DIP_x04" H 1750 5226 50  0000 C CNN
F 2 "Button_Switch_SMD:SW_DIP_SPSTx04_Slide_9.78x12.34mm_W8.61mm_P2.54mm" H 1750 4850 50  0001 C CNN
F 3 "~" H 1750 4850 50  0001 C CNN
	1    1750 4850
	1    0    0    -1  
$EndComp
Text Label 1450 4650 2    50   ~ 0
vga_red
Text Label 1450 4750 2    50   ~ 0
vga_green
Text Label 1450 4850 2    50   ~ 0
vga_blue
Text Label 1450 4950 2    50   ~ 0
vga_hsync
$Comp
L power:GND #PWR?
U 1 1 613FC223
P 2350 4950
F 0 "#PWR?" H 2350 4700 50  0001 C CNN
F 1 "GND" H 2355 4777 50  0000 C CNN
F 2 "" H 2350 4950 50  0001 C CNN
F 3 "" H 2350 4950 50  0001 C CNN
	1    2350 4950
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 613FAAB3
P 2200 4650
F 0 "R?" V 2200 4450 50  0000 C CNN
F 1 "75" V 2200 4300 50  0000 C CNN
F 2 "" V 2130 4650 50  0001 C CNN
F 3 "~" H 2200 4650 50  0001 C CNN
	1    2200 4650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2350 4650 2350 4750
$Comp
L Device:R R?
U 1 1 6140AC4B
P 2200 4750
F 0 "R?" V 2200 4550 50  0000 C CNN
F 1 "75" V 2200 4400 50  0000 C CNN
F 2 "" V 2130 4750 50  0001 C CNN
F 3 "~" H 2200 4750 50  0001 C CNN
	1    2200 4750
	0    -1   -1   0   
$EndComp
Connection ~ 2350 4750
Wire Wire Line
	2350 4750 2350 4850
$Comp
L Device:R R?
U 1 1 6140B035
P 2200 4850
F 0 "R?" V 2200 4650 50  0000 C CNN
F 1 "75" V 2200 4500 50  0000 C CNN
F 2 "" V 2130 4850 50  0001 C CNN
F 3 "~" H 2200 4850 50  0001 C CNN
	1    2200 4850
	0    -1   -1   0   
$EndComp
Connection ~ 2350 4850
Wire Wire Line
	2350 4850 2350 4950
$Comp
L Device:R R?
U 1 1 6140B3B2
P 2200 4950
F 0 "R?" V 2200 4750 50  0000 C CNN
F 1 "75" V 2200 4600 50  0000 C CNN
F 2 "" V 2130 4950 50  0001 C CNN
F 3 "~" H 2200 4950 50  0001 C CNN
	1    2200 4950
	0    -1   -1   0   
$EndComp
Connection ~ 2350 4950
$Comp
L Amplifier_Operational:TLC272 U?
U 1 1 6140D9C4
P 2250 1350
F 0 "U?" H 2250 1717 50  0000 C CNN
F 1 "TLC272" H 2250 1626 50  0000 C CNN
F 2 "" H 2250 1350 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/tlc272.pdf" H 2250 1350 50  0001 C CNN
	1    2250 1350
	1    0    0    -1  
$EndComp
$Comp
L Device:R_POT RV?
U 1 1 6140F5BC
P 1400 1250
F 0 "RV?" H 1330 1296 50  0000 R CNN
F 1 "1k" H 1330 1205 50  0000 R CNN
F 2 "" H 1400 1250 50  0001 C CNN
F 3 "~" H 1400 1250 50  0001 C CNN
	1    1400 1250
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 61410BD8
P 2250 1800
F 0 "C?" V 1998 1800 50  0000 C CNN
F 1 "0.01uF" V 2089 1800 50  0000 C CNN
F 2 "" H 2288 1650 50  0001 C CNN
F 3 "~" H 2250 1800 50  0001 C CNN
	1    2250 1800
	0    1    1    0   
$EndComp
Wire Wire Line
	2100 1800 1950 1800
Wire Wire Line
	1950 1800 1950 1450
Wire Wire Line
	2400 1800 2550 1800
Wire Wire Line
	2550 1800 2550 1350
$Comp
L Device:R R?
U 1 1 6141E954
P 2700 1350
F 0 "R?" V 2493 1350 50  0000 C CNN
F 1 "10" V 2584 1350 50  0000 C CNN
F 2 "" V 2630 1350 50  0001 C CNN
F 3 "~" H 2700 1350 50  0001 C CNN
	1    2700 1350
	0    1    1    0   
$EndComp
Connection ~ 2550 1350
$Comp
L Device:R R?
U 1 1 6141FA4D
P 2250 2200
F 0 "R?" V 2043 2200 50  0000 C CNN
F 1 "R" V 2134 2200 50  0000 C CNN
F 2 "" V 2180 2200 50  0001 C CNN
F 3 "~" H 2250 2200 50  0001 C CNN
	1    2250 2200
	0    1    1    0   
$EndComp
$Comp
L Device:C C?
U 1 1 61420868
P 3000 1700
F 0 "C?" H 3115 1746 50  0000 L CNN
F 1 "0.1uF" H 3115 1655 50  0000 L CNN
F 2 "" H 3038 1550 50  0001 C CNN
F 3 "~" H 3000 1700 50  0001 C CNN
	1    3000 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 2200 2850 2200
Wire Wire Line
	2850 2200 2850 1350
Wire Wire Line
	2850 1350 3000 1350
Wire Wire Line
	3000 1350 3000 1550
Connection ~ 2850 1350
$Comp
L power:GND #PWR?
U 1 1 61429C6B
P 3000 1850
F 0 "#PWR?" H 3000 1600 50  0001 C CNN
F 1 "GND" H 3005 1677 50  0000 C CNN
F 2 "" H 3000 1850 50  0001 C CNN
F 3 "" H 3000 1850 50  0001 C CNN
	1    3000 1850
	1    0    0    -1  
$EndComp
Text Label 3000 1350 0    50   ~ 0
bias_buffered
Wire Wire Line
	1950 1250 1550 1250
$Comp
L power:GND #PWR?
U 1 1 61438CE5
P 1400 1400
F 0 "#PWR?" H 1400 1150 50  0001 C CNN
F 1 "GND" H 1405 1227 50  0000 C CNN
F 2 "" H 1400 1400 50  0001 C CNN
F 3 "" H 1400 1400 50  0001 C CNN
	1    1400 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 1800 1950 2200
Wire Wire Line
	1950 2200 2100 2200
Connection ~ 1950 1800
Text Notes 750  1100 0    50   ~ 0
global bias
$Comp
L power:+5V #PWR?
U 1 1 61451AE4
P 1400 1100
F 0 "#PWR?" H 1400 950 50  0001 C CNN
F 1 "+5V" H 1415 1273 50  0000 C CNN
F 2 "" H 1400 1100 50  0001 C CNN
F 3 "" H 1400 1100 50  0001 C CNN
	1    1400 1100
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:2N7002 Q?
U 1 1 6145200F
P 8650 2250
F 0 "Q?" H 8854 2296 50  0000 L CNN
F 1 "2N7002NXBKR" H 8550 2000 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 8850 2175 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/NDS7002A-D.PDF" H 8650 2250 50  0001 L CNN
F 4 "1727-8642-1-ND" H 8650 2250 50  0001 C CNN "Digikey"
	1    8650 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	8750 2050 8750 1700
Connection ~ 8750 1700
$Comp
L power:GND #PWR?
U 1 1 614606C8
P 8750 2450
F 0 "#PWR?" H 8750 2200 50  0001 C CNN
F 1 "GND" H 8755 2277 50  0000 C CNN
F 2 "" H 8750 2450 50  0001 C CNN
F 3 "" H 8750 2450 50  0001 C CNN
	1    8750 2450
	1    0    0    -1  
$EndComp
Text Label 8450 2250 2    50   ~ 0
blank
$Comp
L td-deflect:LMH1980 U?
U 1 1 61470C26
P 5100 7100
F 0 "U?" H 5125 7565 50  0000 C CNN
F 1 "LMH1980" H 5125 7474 50  0000 C CNN
F 2 "Package_SO:VSSOP-10_3x3mm_P0.5mm" H 4700 7450 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lmh1980.pdf" H 4700 7450 50  0001 C CNN
	1    5100 7100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 61472123
P 4400 7300
F 0 "#PWR?" H 4400 7050 50  0001 C CNN
F 1 "GND" H 4405 7127 50  0000 C CNN
F 2 "" H 4400 7300 50  0001 C CNN
F 3 "" H 4400 7300 50  0001 C CNN
	1    4400 7300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 7300 4400 7000
Wire Wire Line
	4400 7000 4700 7000
$Comp
L Device:C C?
U 1 1 61478724
P 3900 7150
F 0 "C?" H 4015 7196 50  0000 L CNN
F 1 "C" H 4015 7105 50  0000 L CNN
F 2 "" H 3938 7000 50  0001 C CNN
F 3 "~" H 3900 7150 50  0001 C CNN
	1    3900 7150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 61479181
P 3900 7300
F 0 "#PWR?" H 3900 7050 50  0001 C CNN
F 1 "GND" H 3905 7127 50  0000 C CNN
F 2 "" H 3900 7300 50  0001 C CNN
F 3 "" H 3900 7300 50  0001 C CNN
	1    3900 7300
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 61479E6C
P 3900 7000
F 0 "#PWR?" H 3900 6850 50  0001 C CNN
F 1 "+5V" H 3915 7173 50  0000 C CNN
F 2 "" H 3900 7000 50  0001 C CNN
F 3 "" H 3900 7000 50  0001 C CNN
	1    3900 7000
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 6147B04C
P 4450 6700
F 0 "#PWR?" H 4450 6550 50  0001 C CNN
F 1 "+5V" H 4465 6873 50  0000 C CNN
F 2 "" H 4450 6700 50  0001 C CNN
F 3 "" H 4450 6700 50  0001 C CNN
	1    4450 6700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4700 7100 4450 7100
Wire Wire Line
	4450 7100 4450 6700
$Comp
L Jumper:Jumper_2_Bridged JP?
U 1 1 61481920
P 6200 7000
F 0 "JP?" H 6200 7195 50  0000 C CNN
F 1 "Jumper_2_Bridged" H 6200 7104 50  0000 C CNN
F 2 "" H 6200 7000 50  0001 C CNN
F 3 "~" H 6200 7000 50  0001 C CNN
	1    6200 7000
	1    0    0    -1  
$EndComp
Text Label 6400 7000 0    50   ~ 0
clamp
Wire Wire Line
	5550 7000 6000 7000
Text Label 5700 7000 0    50   ~ 0
blank
Text Label 5550 7200 0    50   ~ 0
vsync
Text Label 5550 7300 0    50   ~ 0
hsync
Text Label 5550 7100 0    50   ~ 0
csync
Text Label 5550 6900 0    50   ~ 0
oddeven
$Comp
L Amplifier_Operational:TLC272 U?
U 2 1 61499CD7
P 2200 2800
F 0 "U?" H 2200 3167 50  0000 C CNN
F 1 "TLC272" H 2200 3076 50  0000 C CNN
F 2 "" H 2200 2800 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/tlc272.pdf" H 2200 2800 50  0001 C CNN
	2    2200 2800
	1    0    0    -1  
$EndComp
$Comp
L Device:R_POT RV?
U 1 1 61499CDD
P 1350 2700
F 0 "RV?" H 1280 2746 50  0000 R CNN
F 1 "1k" H 1280 2655 50  0000 R CNN
F 2 "" H 1350 2700 50  0001 C CNN
F 3 "~" H 1350 2700 50  0001 C CNN
	1    1350 2700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 61499CE3
P 2200 3250
F 0 "C?" V 1948 3250 50  0000 C CNN
F 1 "0.01uF" V 2039 3250 50  0000 C CNN
F 2 "" H 2238 3100 50  0001 C CNN
F 3 "~" H 2200 3250 50  0001 C CNN
	1    2200 3250
	0    1    1    0   
$EndComp
Wire Wire Line
	2050 3250 1900 3250
Wire Wire Line
	1900 3250 1900 2900
Wire Wire Line
	2350 3250 2500 3250
Wire Wire Line
	2500 3250 2500 2800
$Comp
L Device:R R?
U 1 1 61499CED
P 2650 2800
F 0 "R?" V 2443 2800 50  0000 C CNN
F 1 "10" V 2534 2800 50  0000 C CNN
F 2 "" V 2580 2800 50  0001 C CNN
F 3 "~" H 2650 2800 50  0001 C CNN
	1    2650 2800
	0    1    1    0   
$EndComp
Connection ~ 2500 2800
$Comp
L Device:R R?
U 1 1 61499CF4
P 2200 3650
F 0 "R?" V 1993 3650 50  0000 C CNN
F 1 "R" V 2084 3650 50  0000 C CNN
F 2 "" V 2130 3650 50  0001 C CNN
F 3 "~" H 2200 3650 50  0001 C CNN
	1    2200 3650
	0    1    1    0   
$EndComp
$Comp
L Device:C C?
U 1 1 61499CFA
P 2950 3150
F 0 "C?" H 3065 3196 50  0000 L CNN
F 1 "0.1uF" H 3065 3105 50  0000 L CNN
F 2 "" H 2988 3000 50  0001 C CNN
F 3 "~" H 2950 3150 50  0001 C CNN
	1    2950 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 3650 2800 3650
Wire Wire Line
	2800 3650 2800 2800
Wire Wire Line
	2800 2800 2950 2800
Wire Wire Line
	2950 2800 2950 3000
Connection ~ 2800 2800
$Comp
L power:GND #PWR?
U 1 1 61499D05
P 2950 3300
F 0 "#PWR?" H 2950 3050 50  0001 C CNN
F 1 "GND" H 2955 3127 50  0000 C CNN
F 2 "" H 2950 3300 50  0001 C CNN
F 3 "" H 2950 3300 50  0001 C CNN
	1    2950 3300
	1    0    0    -1  
$EndComp
Text Label 2950 2800 0    50   ~ 0
gain_buffered
Wire Wire Line
	1900 2700 1500 2700
$Comp
L power:GND #PWR?
U 1 1 61499D0D
P 1350 2850
F 0 "#PWR?" H 1350 2600 50  0001 C CNN
F 1 "GND" H 1355 2677 50  0000 C CNN
F 2 "" H 1350 2850 50  0001 C CNN
F 3 "" H 1350 2850 50  0001 C CNN
	1    1350 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 3250 1900 3650
Wire Wire Line
	1900 3650 2050 3650
Connection ~ 1900 3250
Text Notes 700  2550 0    50   ~ 0
global gain
$Comp
L power:+5V #PWR?
U 1 1 61499D17
P 1350 2550
F 0 "#PWR?" H 1350 2400 50  0001 C CNN
F 1 "+5V" H 1365 2723 50  0000 C CNN
F 2 "" H 1350 2550 50  0001 C CNN
F 3 "" H 1350 2550 50  0001 C CNN
	1    1350 2550
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Linear:LM1117-5.0 U?
U 1 1 614B26D6
P 7300 5850
F 0 "U?" H 7300 6092 50  0000 C CNN
F 1 "LM1117-5.0" H 7300 6001 50  0000 C CNN
F 2 "" H 7300 5850 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm1117.pdf" H 7300 5850 50  0001 C CNN
	1    7300 5850
	1    0    0    -1  
$EndComp
$EndSCHEMATC