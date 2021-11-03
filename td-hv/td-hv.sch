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
L Transistor_FET:C2M1000170J Q1
U 1 1 60FCFDD9
P 4400 3550
F 0 "Q1" H 4604 3596 50  0000 L CNN
F 1 "IMBF170R450M1XTMA1" H 3950 3850 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:TO-263-7_TabPin8" H 4400 3550 50  0001 C CIN
F 3 "https://www.wolfspeed.com/media/downloads/820/C2M1000170J.pdf" H 4400 3550 50  0001 L CNN
F 4 "448-IMBF170R450M1XTMA1CT-ND" H 4400 3550 50  0001 C CNN "Digikey"
	1    4400 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 3350 4750 3350
Wire Wire Line
	4500 3950 4500 3850
Wire Wire Line
	6000 3150 6350 3150
$Comp
L Device:C C5
U 1 1 60FE0D45
P 5750 1000
F 0 "C5" H 5865 1046 50  0000 L CNN
F 1 "0.1uF 200V" H 5865 955 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 5788 850 50  0001 C CNN
F 3 "~" H 5750 1000 50  0001 C CNN
F 4 "732-12081-1-ND" H 5750 1000 50  0001 C CNN "Digikey"
	1    5750 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 700  6000 800 
Wire Wire Line
	5750 850  5750 800 
Wire Wire Line
	5750 800  6000 800 
Connection ~ 6000 800 
$Comp
L Device:CP C4
U 1 1 60FE1739
P 5400 1000
F 0 "C4" H 5518 1046 50  0000 L CNN
F 1 "12uF 250V" H 5518 955 50  0000 L CNN
F 2 "Capacitor_SMD:C_Elec_8x10.2" H 5438 850 50  0001 C CNN
F 3 "~" H 5400 1000 50  0001 C CNN
F 4 "493-6735-1-ND" H 5400 1000 50  0001 C CNN "Digikey"
	1    5400 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5750 800  5400 800 
Wire Wire Line
	5400 800  5400 850 
Connection ~ 5750 800 
Wire Wire Line
	5750 1150 5400 1150
$Comp
L power:GND #PWR0102
U 1 1 60FE204A
P 5750 1150
F 0 "#PWR0102" H 5750 900 50  0001 C CNN
F 1 "GND" H 5755 977 50  0000 C CNN
F 2 "" H 5750 1150 50  0001 C CNN
F 3 "" H 5750 1150 50  0001 C CNN
	1    5750 1150
	1    0    0    -1  
$EndComp
Connection ~ 5750 1150
$Comp
L power:GND #PWR0103
U 1 1 60FE2DF9
P 3850 1350
F 0 "#PWR0103" H 3850 1100 50  0001 C CNN
F 1 "GND" H 3855 1177 50  0000 C CNN
F 2 "" H 3850 1350 50  0001 C CNN
F 3 "" H 3850 1350 50  0001 C CNN
	1    3850 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 4350 4500 4250
Wire Wire Line
	4200 3850 4500 3850
Connection ~ 4500 3850
Wire Wire Line
	4500 3850 4500 3750
$Comp
L Device:Fuse F1
U 1 1 60FE49A7
P 4650 1550
F 0 "F1" V 4453 1550 50  0000 C CNN
F 1 "Fuse" V 4544 1550 50  0000 C CNN
F 2 "Fuse:Fuse_Schurter_UMT250" V 4580 1550 50  0001 C CNN
F 3 "~" H 4650 1550 50  0001 C CNN
F 4 "F1889-ND" H 4650 1550 50  0001 C CNN "Digikey"
	1    4650 1550
	0    1    1    0   
$EndComp
$Comp
L power:+12V #PWR0105
U 1 1 60FE68DF
P 3700 1200
F 0 "#PWR0105" H 3700 1050 50  0001 C CNN
F 1 "+12V" H 3715 1373 50  0000 C CNN
F 2 "" H 3700 1200 50  0001 C CNN
F 3 "" H 3700 1200 50  0001 C CNN
	1    3700 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3850 1350 3250 1350
Wire Wire Line
	3700 1200 3700 1450
Wire Wire Line
	3700 1450 3250 1450
Wire Wire Line
	4200 3850 4200 3950
$Comp
L Device:R R3
U 1 1 60FEB6CA
P 3900 3550
F 0 "R3" V 3693 3550 50  0000 C CNN
F 1 "1" V 3784 3550 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 3830 3550 50  0001 C CNN
F 3 "~" H 3900 3550 50  0001 C CNN
F 4 "YAG3705CT-ND" H 3900 3550 50  0001 C CNN "Digikey"
	1    3900 3550
	0    1    1    0   
$EndComp
Wire Wire Line
	4050 3550 4200 3550
Wire Wire Line
	3750 3550 3650 3550
$Comp
L power:GND #PWR0107
U 1 1 60FECC27
P 3150 5000
F 0 "#PWR0107" H 3150 4750 50  0001 C CNN
F 1 "GND" H 3155 4827 50  0000 C CNN
F 2 "" H 3150 5000 50  0001 C CNN
F 3 "" H 3150 5000 50  0001 C CNN
	1    3150 5000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0108
U 1 1 60FED63F
P 4500 4350
F 0 "#PWR0108" H 4500 4100 50  0001 C CNN
F 1 "GND" H 4505 4177 50  0000 C CNN
F 2 "" H 4500 4350 50  0001 C CNN
F 3 "" H 4500 4350 50  0001 C CNN
	1    4500 4350
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 60FEDC18
P 6950 4650
F 0 "R5" H 7020 4696 50  0000 L CNN
F 1 "100k" H 7020 4605 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 6880 4650 50  0001 C CNN
F 3 "~" H 6950 4650 50  0001 C CNN
F 4 "YAG3688CT-ND" H 6950 4650 50  0001 C CNN "Digikey"
	1    6950 4650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 60FEF10B
P 2150 3500
F 0 "R1" H 2220 3546 50  0000 L CNN
F 1 "10k" H 2220 3455 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2080 3500 50  0001 C CNN
F 3 "~" H 2150 3500 50  0001 C CNN
F 4 "311-10.0KCRCT-ND" H 2150 3500 50  0001 C CNN "Digikey"
	1    2150 3500
	1    0    0    -1  
$EndComp
Connection ~ 2150 3650
Wire Wire Line
	2150 3650 2650 3650
$Comp
L Device:C C1
U 1 1 60FEF84C
P 1650 3500
F 0 "C1" H 1765 3546 50  0000 L CNN
F 1 "10000pF" H 1765 3455 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 1688 3350 50  0001 C CNN
F 3 "~" H 1650 3500 50  0001 C CNN
F 4 "732-12139-1-ND" H 1650 3500 50  0001 C CNN "Digikey"
	1    1650 3500
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 60FF1553
P 1700 4000
F 0 "C2" H 1815 4046 50  0000 L CNN
F 1 "0.1uF" H 1815 3955 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 1738 3850 50  0001 C CNN
F 3 "~" H 1700 4000 50  0001 C CNN
F 4 "478-10836-1-ND" H 1700 4000 50  0001 C CNN "Digikey"
	1    1700 4000
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 60FF299E
P 2150 3950
F 0 "R2" H 2220 3996 50  0000 L CNN
F 1 "100k" H 2220 3905 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2080 3950 50  0001 C CNN
F 3 "~" H 2150 3950 50  0001 C CNN
F 4 "YAG3688CT-ND" H 2150 3950 50  0001 C CNN "Digikey"
	1    2150 3950
	0    1    1    0   
$EndComp
$Comp
L Device:C C3
U 1 1 60FF2FE4
P 2550 4100
F 0 "C3" H 2665 4146 50  0000 L CNN
F 1 "560pF" H 2665 4055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 2588 3950 50  0001 C CNN
F 3 "~" H 2550 4100 50  0001 C CNN
F 4 "399-1134-1-ND" H 2550 4100 50  0001 C CNN "Digikey"
	1    2550 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 3950 2550 3950
Wire Wire Line
	2550 3950 2300 3950
Connection ~ 2550 3950
$Comp
L Comparator:LM2903 U3
U 1 1 60FF7B44
P 9250 4750
F 0 "U3" H 9250 5117 50  0000 C CNN
F 1 "LM2903" H 9250 5026 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 9250 4750 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm393.pdf" H 9250 4750 50  0001 C CNN
F 4 "LM2903EDR2GOSCT-ND" H 9250 4750 50  0001 C CNN "Digikey"
	1    9250 4750
	1    0    0    -1  
$EndComp
$Comp
L Diode:BAV99 D1
U 1 1 60FFB08C
P 8050 4000
F 0 "D1" H 8050 4216 50  0000 C CNN
F 1 "BAV99" H 8050 4125 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 8050 3500 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/BAV99_SER.pdf" H 8050 4000 50  0001 C CNN
F 4 "BAV99RWT1GOSCT-ND" H 8050 4000 50  0001 C CNN "Digikey"
	1    8050 4000
	1    0    0    -1  
$EndComp
$Comp
L Device:R_POT_TRIM RV1
U 1 1 60FFDC4B
P 8050 4850
F 0 "RV1" H 7980 4896 50  0000 R CNN
F 1 "1k" H 7980 4805 50  0000 R CNN
F 2 "Potentiometer_SMD:Potentiometer_Bourns_3314G_Vertical" H 8050 4850 50  0001 C CNN
F 3 "~" H 8050 4850 50  0001 C CNN
F 4 "3314G-1-102E" H 8050 4850 50  0001 C CNN "Digikey"
	1    8050 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	8050 4700 8050 4550
$Comp
L Device:C C6
U 1 1 60FFF436
P 7500 4800
F 0 "C6" H 7615 4846 50  0000 L CNN
F 1 "0.1uF" H 7615 4755 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 7538 4650 50  0001 C CNN
F 3 "~" H 7500 4800 50  0001 C CNN
F 4 "478-10836-1-ND" H 7500 4800 50  0001 C CNN "Digikey"
	1    7500 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	7500 4650 7500 4550
Connection ~ 8050 4550
Wire Wire Line
	8050 4550 8050 4500
Wire Wire Line
	7500 5050 8050 5050
Wire Wire Line
	8050 5050 8050 5000
Wire Wire Line
	7500 4950 7500 5050
$Comp
L power:GND #PWR0110
U 1 1 610025BE
P 8050 5050
F 0 "#PWR0110" H 8050 4800 50  0001 C CNN
F 1 "GND" H 8055 4877 50  0000 C CNN
F 2 "" H 8050 5050 50  0001 C CNN
F 3 "" H 8050 5050 50  0001 C CNN
	1    8050 5050
	1    0    0    -1  
$EndComp
Connection ~ 8050 5050
Wire Wire Line
	8950 4850 8800 4850
Text Label 2150 3850 0    50   ~ 0
ucc3801_ref
Text Label 8450 4650 0    50   ~ 0
ucc3801_ref
Text Label 2300 3050 0    50   ~ 0
comp
Text Label 9550 4750 0    50   ~ 0
comp
Text Notes 8950 5050 0    50   ~ 0
x-ray protect
Text Label 3650 3950 0    50   ~ 0
cs
Text Label 950  3350 0    50   ~ 0
fb
$Comp
L Amplifier_Operational:TLC272 U2
U 1 1 6100C694
P 8450 3450
F 0 "U2" H 8794 3496 50  0000 L CNN
F 1 "TLC272" H 8794 3405 50  0000 L CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 8500 3500 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/tlc272.pdf" H 8600 3600 50  0001 C CNN
F 4 "296-7204-5-ND" H 8450 3450 50  0001 C CNN "Digikey"
	1    8450 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	8750 3450 8750 3050
Wire Wire Line
	8750 3050 8000 3050
Wire Wire Line
	8000 3050 8000 3550
Wire Wire Line
	8000 3550 8150 3550
NoConn ~ 8350 4000
Wire Wire Line
	9300 3450 8750 3450
Connection ~ 8750 3450
$Comp
L Comparator:LM2903 U3
U 2 1 6101961E
P 9250 5550
F 0 "U3" H 9250 5917 50  0000 C CNN
F 1 "LM2903" H 9250 5826 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 9250 5550 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm393.pdf" H 9250 5550 50  0001 C CNN
F 4 "LM2903EDR2GOSCT-ND" H 9250 5550 50  0001 C CNN "Digikey"
	2    9250 5550
	1    0    0    -1  
$EndComp
Text Label 9300 3450 0    50   ~ 0
fb
Wire Wire Line
	8950 5450 8800 5450
Wire Wire Line
	8700 4650 8700 5650
Wire Wire Line
	8700 5650 8950 5650
Connection ~ 8700 4650
Wire Wire Line
	8700 4650 8450 4650
$Comp
L Device:LED D3
U 1 1 6101D8ED
P 10150 5400
F 0 "D3" V 10189 5282 50  0000 R CNN
F 1 "LED" V 10098 5282 50  0000 R CNN
F 2 "LED_SMD:LED_0805_2012Metric_Castellated" H 10150 5400 50  0001 C CNN
F 3 "~" H 10150 5400 50  0001 C CNN
F 4 "732-4984-1-ND" V 10150 5400 50  0001 C CNN "Digikey"
	1    10150 5400
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9550 5550 10150 5550
$Comp
L Device:R R8
U 1 1 61020D05
P 10150 5050
F 0 "R8" H 10220 5096 50  0000 L CNN
F 1 "10k" H 10220 5005 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 10080 5050 50  0001 C CNN
F 3 "~" H 10150 5050 50  0001 C CNN
F 4 "311-10.0KCRCT-ND" H 10150 5050 50  0001 C CNN "Digikey"
	1    10150 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	10150 5200 10150 5250
$Comp
L power:+12V #PWR0114
U 1 1 6102249D
P 10150 4900
F 0 "#PWR0114" H 10150 4750 50  0001 C CNN
F 1 "+12V" H 10165 5073 50  0000 C CNN
F 2 "" H 10150 4900 50  0001 C CNN
F 3 "" H 10150 4900 50  0001 C CNN
	1    10150 4900
	1    0    0    -1  
$EndComp
$Comp
L Device:R R7
U 1 1 61023605
P 8350 5250
F 0 "R7" H 8420 5296 50  0000 L CNN
F 1 "10k" H 8420 5205 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 8280 5250 50  0001 C CNN
F 3 "~" H 8350 5250 50  0001 C CNN
F 4 "311-10.0KCRCT-ND" H 8350 5250 50  0001 C CNN "Digikey"
	1    8350 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	7500 4550 8050 4550
Wire Wire Line
	8050 4550 8350 4550
Wire Wire Line
	8350 4550 8350 5100
$Comp
L Device:LED D2
U 1 1 61026861
P 8350 5650
F 0 "D2" V 8389 5532 50  0000 R CNN
F 1 "‎150080GS75000‎" V 8298 5532 50  0000 R CNN
F 2 "LED_SMD:LED_0805_2012Metric_Castellated" H 8350 5650 50  0001 C CNN
F 3 "~" H 8350 5650 50  0001 C CNN
F 4 " ‎732-4983-1-ND‎ " V 8350 5650 50  0001 C CNN "Digikey"
F 5 " Würth Elektronik" V 8350 5650 50  0001 C CNN "Manufacturer"
	1    8350 5650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8350 5400 8350 5500
$Comp
L power:GND #PWR0115
U 1 1 61028641
P 8350 5800
F 0 "#PWR0115" H 8350 5550 50  0001 C CNN
F 1 "GND" H 8355 5627 50  0000 C CNN
F 2 "" H 8350 5800 50  0001 C CNN
F 3 "" H 8350 5800 50  0001 C CNN
	1    8350 5800
	1    0    0    -1  
$EndComp
Text Label 9600 5550 0    50   ~ 0
~xray_protect
Text Label 3250 1650 0    50   ~ 0
~xray_protect
Text Label 3250 1750 0    50   ~ 0
eht_current
Text Label 3250 1850 0    50   ~ 0
fb
NoConn ~ 6350 4150
$Comp
L Comparator:LM2903 U3
U 3 1 6103ED6A
P 11050 5050
F 0 "U3" H 11008 5096 50  0000 L CNN
F 1 "LM2903" H 11008 5005 50  0000 L CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 11050 5050 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm393.pdf" H 11050 5050 50  0001 C CNN
F 4 "LM2903EDR2GOSCT-ND" H 11050 5050 50  0001 C CNN "Digikey"
	3    11050 5050
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR0118
U 1 1 6103F2EF
P 10950 4750
F 0 "#PWR0118" H 10950 4600 50  0001 C CNN
F 1 "+12V" H 10965 4923 50  0000 C CNN
F 2 "" H 10950 4750 50  0001 C CNN
F 3 "" H 10950 4750 50  0001 C CNN
	1    10950 4750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0119
U 1 1 6103F5A1
P 10950 5350
F 0 "#PWR0119" H 10950 5100 50  0001 C CNN
F 1 "GND" H 10955 5177 50  0000 C CNN
F 2 "" H 10950 5350 50  0001 C CNN
F 3 "" H 10950 5350 50  0001 C CNN
	1    10950 5350
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 61040EE7
P 3700 1200
F 0 "#FLG0101" H 3700 1275 50  0001 C CNN
F 1 "PWR_FLAG" V 3700 1328 50  0000 L CNN
F 2 "" H 3700 1200 50  0001 C CNN
F 3 "~" H 3700 1200 50  0001 C CNN
	1    3700 1200
	0    1    1    0   
$EndComp
Connection ~ 3700 1200
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 61041313
P 4800 1550
F 0 "#FLG0102" H 4800 1625 50  0001 C CNN
F 1 "PWR_FLAG" V 4800 1678 50  0000 L CNN
F 2 "" H 4800 1550 50  0001 C CNN
F 3 "~" H 4800 1550 50  0001 C CNN
	1    4800 1550
	-1   0    0    1   
$EndComp
$Comp
L power:PWR_FLAG #FLG01
U 1 1 6104179A
P 3850 1350
F 0 "#FLG01" H 3850 1425 50  0001 C CNN
F 1 "PWR_FLAG" V 3850 1478 50  0000 L CNN
F 2 "" H 3850 1350 50  0001 C CNN
F 3 "~" H 3850 1350 50  0001 C CNN
	1    3850 1350
	0    1    1    0   
$EndComp
Connection ~ 3850 1350
Wire Wire Line
	3250 1550 4500 1550
$Comp
L power:PWR_FLAG #FLG0103
U 1 1 6104633C
P 1700 3850
F 0 "#FLG0103" H 1700 3925 50  0001 C CNN
F 1 "PWR_FLAG" V 1700 3977 50  0000 L CNN
F 2 "" H 1700 3850 50  0001 C CNN
F 3 "~" H 1700 3850 50  0001 C CNN
	1    1700 3850
	0    -1   -1   0   
$EndComp
Connection ~ 1700 3850
NoConn ~ 6950 3950
$Comp
L Device:C C7
U 1 1 61066736
P 4950 2550
F 0 "C7" H 5065 2596 50  0000 L CNN
F 1 "47pF 2000V NP0" H 5065 2505 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 4988 2400 50  0001 C CNN
F 3 "~" H 4950 2550 50  0001 C CNN
F 4 "1276-3272-1-ND‎" H 4950 2550 50  0001 C CNN "Digikey"
	1    4950 2550
	1    0    0    -1  
$EndComp
$Comp
L Device:R R10
U 1 1 61066EB0
P 5500 2300
F 0 "R10" H 5570 2346 50  0000 L CNN
F 1 "3k" H 5570 2255 50  0000 L CNN
F 2 "Resistor_SMD:R_2512_6332Metric" V 5430 2300 50  0001 C CNN
F 3 "~" H 5500 2300 50  0001 C CNN
F 4 "13-RC2512JK-073KLCT-ND‎" H 5500 2300 50  0001 C CNN "Digikey"
	1    5500 2300
	0    1    1    0   
$EndComp
$Comp
L Device:R R9
U 1 1 61068292
P 5000 1000
F 0 "R9" H 5070 1046 50  0000 L CNN
F 1 "100k" H 5070 955 50  0000 L CNN
F 2 "Resistor_SMD:R_1210_3225Metric" V 4930 1000 50  0001 C CNN
F 3 "~" H 5000 1000 50  0001 C CNN
F 4 "13-RC1210JR-07100KLCT-ND" H 5000 1000 50  0001 C CNN "Digikey"
	1    5000 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5000 850  5000 800 
Wire Wire Line
	5000 800  5400 800 
Connection ~ 5400 800 
Wire Wire Line
	5400 1150 5000 1150
Connection ~ 5400 1150
Wire Wire Line
	7500 3350 8150 3350
Wire Wire Line
	6250 5350 7750 5350
Wire Wire Line
	1650 3650 2150 3650
Wire Wire Line
	1700 3850 2000 3850
Text Notes 4700 4400 0    50   ~ 0
Note: SiC MOSFET with 12V gate drive
Text Label 4800 3350 0    50   ~ 0
sw
$Comp
L power:VBUS #PWR0101
U 1 1 60FFA7FA
P 6000 700
F 0 "#PWR0101" H 6000 550 50  0001 C CNN
F 1 "VBUS" H 6015 873 50  0000 C CNN
F 2 "" H 6000 700 50  0001 C CNN
F 3 "" H 6000 700 50  0001 C CNN
	1    6000 700 
	1    0    0    -1  
$EndComp
$Comp
L power:VBUS #PWR0104
U 1 1 60FFB334
P 4800 1550
F 0 "#PWR0104" H 4800 1400 50  0001 C CNN
F 1 "VBUS" H 4815 1723 50  0000 C CNN
F 2 "" H 4800 1550 50  0001 C CNN
F 3 "" H 4800 1550 50  0001 C CNN
	1    4800 1550
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Controller:UCC3801 U1
U 1 1 60FE9DE4
P 3150 3750
F 0 "U1" H 3150 4331 50  0000 C CNN
F 1 "UCC2801" H 3150 4240 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 3150 3750 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/ucc3800.pdf" H 3150 3750 50  0001 C CNN
F 4 "296-27157-1-ND" H 3150 3750 50  0001 C CNN "Digikey"
	1    3150 3750
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR0106
U 1 1 60FEC4E1
P 3150 2450
F 0 "#PWR0106" H 3150 2300 50  0001 C CNN
F 1 "+12V" H 3165 2623 50  0000 C CNN
F 2 "" H 3150 2450 50  0001 C CNN
F 3 "" H 3150 2450 50  0001 C CNN
	1    3150 2450
	1    0    0    -1  
$EndComp
$Comp
L Device:R R13
U 1 1 61001216
P 3150 2600
F 0 "R13" H 3220 2646 50  0000 L CNN
F 1 "1k" H 3220 2555 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 3080 2600 50  0001 C CNN
F 3 "~" H 3150 2600 50  0001 C CNN
	1    3150 2600
	1    0    0    -1  
$EndComp
$Comp
L Device:C C8
U 1 1 61001B21
P 2800 2900
F 0 "C8" H 2915 2946 50  0000 L CNN
F 1 "0.1uF" H 2915 2855 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 2838 2750 50  0001 C CNN
F 3 "~" H 2800 2900 50  0001 C CNN
	1    2800 2900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0121
U 1 1 6100494E
P 2800 3050
F 0 "#PWR0121" H 2800 2800 50  0001 C CNN
F 1 "GND" H 2805 2877 50  0000 C CNN
F 2 "" H 2800 3050 50  0001 C CNN
F 3 "" H 2800 3050 50  0001 C CNN
	1    2800 3050
	1    0    0    -1  
$EndComp
$Comp
L Device:C C9
U 1 1 61007B85
P 9250 2800
F 0 "C9" H 9365 2846 50  0000 L CNN
F 1 "0.1uF" H 9365 2755 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 9288 2650 50  0001 C CNN
F 3 "~" H 9250 2800 50  0001 C CNN
	1    9250 2800
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR0122
U 1 1 61007DFB
P 9750 2550
F 0 "#PWR0122" H 9750 2400 50  0001 C CNN
F 1 "+12V" H 9765 2723 50  0000 C CNN
F 2 "" H 9750 2550 50  0001 C CNN
F 3 "" H 9750 2550 50  0001 C CNN
	1    9750 2550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0123
U 1 1 6100804D
P 9750 3150
F 0 "#PWR0123" H 9750 2900 50  0001 C CNN
F 1 "GND" H 9755 2977 50  0000 C CNN
F 2 "" H 9750 3150 50  0001 C CNN
F 3 "" H 9750 3150 50  0001 C CNN
	1    9750 3150
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H1
U 1 1 61043056
P 1250 1050
F 0 "H1" H 1350 1099 50  0000 L CNN
F 1 "MountingHole_Pad" H 1350 1008 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_Pad_Via" H 1250 1050 50  0001 C CNN
F 3 "~" H 1250 1050 50  0001 C CNN
	1    1250 1050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0124
U 1 1 6104352B
P 1250 1150
F 0 "#PWR0124" H 1250 900 50  0001 C CNN
F 1 "GND" H 1255 977 50  0000 C CNN
F 2 "" H 1250 1150 50  0001 C CNN
F 3 "" H 1250 1150 50  0001 C CNN
	1    1250 1150
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H2
U 1 1 61046C60
P 1250 1500
F 0 "H2" H 1350 1549 50  0000 L CNN
F 1 "MountingHole_Pad" H 1350 1458 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_Pad_Via" H 1250 1500 50  0001 C CNN
F 3 "~" H 1250 1500 50  0001 C CNN
	1    1250 1500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0125
U 1 1 61046C66
P 1250 1600
F 0 "#PWR0125" H 1250 1350 50  0001 C CNN
F 1 "GND" H 1255 1427 50  0000 C CNN
F 2 "" H 1250 1600 50  0001 C CNN
F 3 "" H 1250 1600 50  0001 C CNN
	1    1250 1600
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H3
U 1 1 61048E4D
P 1250 1950
F 0 "H3" H 1350 1999 50  0000 L CNN
F 1 "MountingHole_Pad" H 1350 1908 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_Pad_Via" H 1250 1950 50  0001 C CNN
F 3 "~" H 1250 1950 50  0001 C CNN
	1    1250 1950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0126
U 1 1 61048E53
P 1250 2050
F 0 "#PWR0126" H 1250 1800 50  0001 C CNN
F 1 "GND" H 1255 1877 50  0000 C CNN
F 2 "" H 1250 2050 50  0001 C CNN
F 3 "" H 1250 2050 50  0001 C CNN
	1    1250 2050
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H4
U 1 1 6104B27C
P 1250 2400
F 0 "H4" H 1350 2449 50  0000 L CNN
F 1 "MountingHole_Pad" H 1350 2358 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_Pad_Via" H 1250 2400 50  0001 C CNN
F 3 "~" H 1250 2400 50  0001 C CNN
	1    1250 2400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0127
U 1 1 6104B282
P 1250 2500
F 0 "#PWR0127" H 1250 2250 50  0001 C CNN
F 1 "GND" H 1255 2327 50  0000 C CNN
F 2 "" H 1250 2500 50  0001 C CNN
F 3 "" H 1250 2500 50  0001 C CNN
	1    1250 2500
	1    0    0    -1  
$EndComp
$Comp
L Device:SPARK_GAP E1
U 1 1 61064BB0
P 4750 3550
F 0 "E1" V 4704 3603 50  0000 L CNN
F 1 "SPARK_GAP" V 4795 3603 50  0000 L CNN
F 2 "Resistor_SMD:R_1218_3246Metric_Pad1.22x4.75mm_HandSolder" H 4750 3480 50  0001 C CNN
F 3 "~" V 4750 3550 50  0001 C CNN
	1    4750 3550
	0    1    1    0   
$EndComp
Connection ~ 4750 3350
$Comp
L power:GND #PWR0128
U 1 1 61066187
P 4750 3750
F 0 "#PWR0128" H 4750 3500 50  0001 C CNN
F 1 "GND" H 4755 3577 50  0000 C CNN
F 2 "" H 4750 3750 50  0001 C CNN
F 3 "" H 4750 3750 50  0001 C CNN
	1    4750 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 3950 2000 3850
Connection ~ 2000 3850
Wire Wire Line
	2000 3850 2650 3850
$Comp
L Device:R R12
U 1 1 611646F0
P 2550 4700
F 0 "R12" H 2620 4746 50  0000 L CNN
F 1 "50" H 2620 4655 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2480 4700 50  0001 C CNN
F 3 "~" H 2550 4700 50  0001 C CNN
	1    2550 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 4850 3150 4850
Wire Wire Line
	3150 4850 3150 5000
Wire Wire Line
	3150 4150 3150 4850
Connection ~ 3150 4850
Text Label 2100 4550 2    50   ~ 0
sync_in
$Comp
L Device:R R11
U 1 1 6116D74D
P 2400 4550
F 0 "R11" V 2193 4550 50  0000 C CNN
F 1 "120" V 2284 4550 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2330 4550 50  0001 C CNN
F 3 "~" H 2400 4550 50  0001 C CNN
	1    2400 4550
	0    1    1    0   
$EndComp
Wire Wire Line
	2250 4550 2100 4550
Wire Wire Line
	1700 4850 2550 4850
Wire Wire Line
	1700 4150 1700 4850
Connection ~ 2550 4850
Wire Wire Line
	2550 4250 2550 4550
Connection ~ 2550 4550
$Comp
L Connector:Conn_01x07_Male J1
U 1 1 6103A046
P 3050 1650
F 0 "J1" H 3158 2031 50  0000 C CNN
F 1 "Conn_01x07_Male" H 3158 1940 50  0000 C CNN
F 2 "Connector_JST:JST_XH_B7B-XH-AM_1x07_P2.50mm_Vertical" H 3050 1650 50  0001 C CNN
F 3 "~" H 3050 1650 50  0001 C CNN
F 4 "455-2238-ND" H 3050 1650 50  0001 C CNN "Digikey"
	1    3050 1650
	1    0    0    -1  
$EndComp
Text Label 3250 1950 0    50   ~ 0
sync_in
$Comp
L power:PWR_FLAG #FLG0104
U 1 1 611B4433
P 3150 2950
F 0 "#FLG0104" H 3150 3025 50  0001 C CNN
F 1 "PWR_FLAG" V 3150 3077 50  0000 L CNN
F 2 "" H 3150 2950 50  0001 C CNN
F 3 "~" H 3150 2950 50  0001 C CNN
	1    3150 2950
	0    1    1    0   
$EndComp
Connection ~ 3150 2950
Wire Wire Line
	3150 2950 3150 3350
Wire Wire Line
	2800 2750 3150 2750
Connection ~ 3150 2750
Wire Wire Line
	3150 2750 3150 2950
$Comp
L Device:C C10
U 1 1 611BED53
P 10600 5100
F 0 "C10" H 10715 5146 50  0000 L CNN
F 1 "0.1uF" H 10715 5055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 10638 4950 50  0001 C CNN
F 3 "~" H 10600 5100 50  0001 C CNN
	1    10600 5100
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR0129
U 1 1 611BED59
P 10600 4950
F 0 "#PWR0129" H 10600 4800 50  0001 C CNN
F 1 "+12V" H 10615 5123 50  0000 C CNN
F 2 "" H 10600 4950 50  0001 C CNN
F 3 "" H 10600 4950 50  0001 C CNN
	1    10600 4950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0130
U 1 1 611BED5F
P 10600 5250
F 0 "#PWR0130" H 10600 5000 50  0001 C CNN
F 1 "GND" H 10605 5077 50  0000 C CNN
F 2 "" H 10600 5250 50  0001 C CNN
F 3 "" H 10600 5250 50  0001 C CNN
	1    10600 5250
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:TLC272 U2
U 3 1 611FE131
P 9850 2850
F 0 "U2" H 9808 2896 50  0000 L CNN
F 1 "TLC272" H 9808 2805 50  0000 L CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 9850 2850 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/tlc272.pdf" H 9850 2850 50  0001 C CNN
	3    9850 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	9750 2550 9250 2550
Wire Wire Line
	9250 2550 9250 2650
Connection ~ 9750 2550
Wire Wire Line
	9750 3150 9250 3150
Wire Wire Line
	9250 2950 9250 3150
Connection ~ 9750 3150
Text Label 4100 1550 0    50   ~ 0
vbus_in
$Comp
L Device:R_POT_TRIM RV2
U 1 1 61008162
P 6950 4350
F 0 "RV2" H 6880 4396 50  0000 R CNN
F 1 "10k" H 6880 4305 50  0000 R CNN
F 2 "Potentiometer_SMD:Potentiometer_Bourns_3314G_Vertical" H 6950 4350 50  0001 C CNN
F 3 "~" H 6950 4350 50  0001 C CNN
F 4 "3314G-1-102E" H 6950 4350 50  0001 C CNN "Digikey"
	1    6950 4350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0109
U 1 1 60FEDDEA
P 6950 4800
F 0 "#PWR0109" H 6950 4550 50  0001 C CNN
F 1 "GND" H 6955 4627 50  0000 C CNN
F 2 "" H 6950 4800 50  0001 C CNN
F 3 "" H 6950 4800 50  0001 C CNN
	1    6950 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	6950 4500 7100 4500
Wire Wire Line
	7100 4500 7100 4350
Connection ~ 6950 4500
Wire Wire Line
	7100 4500 7500 4500
Wire Wire Line
	7500 3350 7500 4500
Connection ~ 7100 4500
Wire Wire Line
	8700 4650 8950 4650
Wire Wire Line
	8800 5450 8800 4850
Connection ~ 8800 4850
Wire Wire Line
	8800 4850 8200 4850
Wire Wire Line
	6950 4150 6950 4200
Text Label 7000 3350 0    50   ~ 0
eht_current
$Comp
L power:GND #PWR0117
U 1 1 61034B7D
P 7250 3650
F 0 "#PWR0117" H 7250 3400 50  0001 C CNN
F 1 "GND" H 7255 3477 50  0000 C CNN
F 2 "" H 7250 3650 50  0001 C CNN
F 3 "" H 7250 3650 50  0001 C CNN
	1    7250 3650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 61032E6F
P 7250 3500
F 0 "R6" H 7320 3546 50  0000 L CNN
F 1 "100" H 7320 3455 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 7180 3500 50  0001 C CNN
F 3 "~" H 7250 3500 50  0001 C CNN
F 4 "311-100CRCT-ND" H 7250 3500 50  0001 C CNN "Digikey"
	1    7250 3500
	1    0    0    -1  
$EndComp
$Comp
L td-hv:053X0528-001 L1
U 1 1 60FCCF60
P 6550 3300
F 0 "L1" H 6550 3645 50  0000 C CNN
F 1 "053X0528-001" H 6550 3554 50  0000 C CNN
F 2 "td-hv:053X0528-001" V 6550 3350 50  0001 C CNN
F 3 "~" V 6550 3350 50  0001 C CNN
	1    6550 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6950 3350 7250 3350
$Comp
L Device:R R4
U 1 1 60FD3507
P 4500 4100
F 0 "R4" H 4570 4146 50  0000 L CNN
F 1 "4.7" H 4570 4055 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 4430 4100 50  0001 C CNN
F 3 "~" H 4500 4100 50  0001 C CNN
F 4 "RNCP1206FTD4R70" H 4500 4100 50  0001 C CNN "Digikey"
	1    4500 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 3550 6350 3550
Wire Wire Line
	6250 3550 6250 5350
$Comp
L power:GND #PWR01
U 1 1 6182760E
P 5950 3950
F 0 "#PWR01" H 5950 3700 50  0001 C CNN
F 1 "GND" H 5955 3777 50  0000 C CNN
F 2 "" H 5950 3950 50  0001 C CNN
F 3 "" H 5950 3950 50  0001 C CNN
	1    5950 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	6350 3950 5950 3950
NoConn ~ 6350 3750
Wire Wire Line
	5650 2300 6000 2300
Wire Wire Line
	6000 2300 6000 3150
Wire Wire Line
	4750 3350 4950 3350
Wire Wire Line
	5350 2300 4950 2300
Wire Wire Line
	4950 2300 4950 2400
Wire Wire Line
	4950 2700 4950 3350
Connection ~ 4950 3350
Wire Wire Line
	4950 3350 6350 3350
$Comp
L Device:R R16
U 1 1 6183E9D5
P 8050 4350
F 0 "R16" H 8120 4396 50  0000 L CNN
F 1 "10k" H 8120 4305 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 7980 4350 50  0001 C CNN
F 3 "~" H 8050 4350 50  0001 C CNN
	1    8050 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 4000 7750 5350
$Comp
L Device:R R14
U 1 1 61843A83
P 1200 3350
F 0 "R14" V 993 3350 50  0000 C CNN
F 1 "10k" V 1084 3350 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1130 3350 50  0001 C CNN
F 3 "~" H 1200 3350 50  0001 C CNN
	1    1200 3350
	0    1    1    0   
$EndComp
Wire Wire Line
	1350 3350 1650 3350
$Comp
L Device:C C11
U 1 1 61846BD3
P 2150 3200
F 0 "C11" H 2265 3246 50  0000 L CNN
F 1 "10000pF" H 2265 3155 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 2188 3050 50  0001 C CNN
F 3 "~" H 2150 3200 50  0001 C CNN
F 4 "732-12139-1-ND" H 2150 3200 50  0001 C CNN "Digikey"
	1    2150 3200
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 3050 2150 3050
Wire Wire Line
	2650 3050 2650 3550
Wire Wire Line
	2150 3050 1650 3050
Wire Wire Line
	1650 3050 1650 3350
Connection ~ 2150 3050
Connection ~ 1650 3350
Wire Wire Line
	1050 3350 950  3350
$Comp
L Device:L L2
U 1 1 61855892
P 6000 1600
F 0 "L2" H 6053 1646 50  0000 L CNN
F 1 "L" H 6053 1555 50  0000 L CNN
F 2 "Inductor_SMD:L_Bourns_SRR1260" H 6000 1600 50  0001 C CNN
F 3 "~" H 6000 1600 50  0001 C CNN
	1    6000 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 1750 6000 2300
Connection ~ 6000 2300
$Comp
L Device:R R15
U 1 1 618596A1
P 5700 1600
F 0 "R15" H 5770 1646 50  0000 L CNN
F 1 "R" H 5770 1555 50  0000 L CNN
F 2 "Resistor_THT:R_Radial_Power_L13.0mm_W9.0mm_P5.00mm" V 5630 1600 50  0001 C CNN
F 3 "~" H 5700 1600 50  0001 C CNN
	1    5700 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 800  6000 1450
Wire Wire Line
	5700 1450 6000 1450
Connection ~ 6000 1450
Wire Wire Line
	5700 1750 6000 1750
Connection ~ 6000 1750
Text Notes 6150 1500 0    50   ~ 0
turn-on damping
Text Notes 5050 2200 0    50   ~ 0
turn-off damping
NoConn ~ 4200 3650
$Comp
L Device:R R17
U 1 1 61870C99
P 4050 3950
F 0 "R17" V 3843 3950 50  0000 C CNN
F 1 "10k" V 3934 3950 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 3980 3950 50  0001 C CNN
F 3 "~" H 4050 3950 50  0001 C CNN
	1    4050 3950
	0    1    1    0   
$EndComp
Wire Wire Line
	3900 3950 3800 3950
$Comp
L Device:C C12
U 1 1 61874CDB
P 3800 4100
F 0 "C12" H 3915 4146 50  0000 L CNN
F 1 "0.01uF" H 3915 4055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3838 3950 50  0001 C CNN
F 3 "~" H 3800 4100 50  0001 C CNN
	1    3800 4100
	1    0    0    -1  
$EndComp
Connection ~ 3800 3950
Wire Wire Line
	3800 3950 3650 3950
$Comp
L power:GND #PWR0111
U 1 1 61874EAD
P 3800 4250
F 0 "#PWR0111" H 3800 4000 50  0001 C CNN
F 1 "GND" H 3805 4077 50  0000 C CNN
F 2 "" H 3800 4250 50  0001 C CNN
F 3 "" H 3800 4250 50  0001 C CNN
	1    3800 4250
	1    0    0    -1  
$EndComp
Connection ~ 4800 1550
$EndSCHEMATC
