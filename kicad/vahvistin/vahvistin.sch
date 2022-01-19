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
L Device:R R1
U 1 1 61938E81
P 3950 2200
F 0 "R1" V 3743 2200 50  0000 C CNN
F 1 "3.3k" V 3834 2200 50  0000 C CNN
F 2 "" V 3880 2200 50  0001 C CNN
F 3 "~" H 3950 2200 50  0001 C CNN
	1    3950 2200
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 61939C45
P 3750 2650
F 0 "#PWR?" H 3750 2400 50  0001 C CNN
F 1 "GND" H 3755 2477 50  0000 C CNN
F 2 "" H 3750 2650 50  0001 C CNN
F 3 "" H 3750 2650 50  0001 C CNN
	1    3750 2650
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR?
U 1 1 6193A143
P 3750 1550
F 0 "#PWR?" H 3750 1400 50  0001 C CNN
F 1 "VCC" H 3765 1723 50  0000 C CNN
F 2 "" H 3750 1550 50  0001 C CNN
F 3 "" H 3750 1550 50  0001 C CNN
	1    3750 1550
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 6193B068
P 3950 1600
F 0 "R3" V 3743 1600 50  0000 C CNN
F 1 "390k" V 3834 1600 50  0000 C CNN
F 2 "" V 3880 1600 50  0001 C CNN
F 3 "~" H 3950 1600 50  0001 C CNN
	1    3950 1600
	0    1    1    0   
$EndComp
$Comp
L Device:R R4
U 1 1 6193BCAC
P 4150 1800
F 0 "R4" H 4220 1846 50  0000 L CNN
F 1 "150k" H 4220 1755 50  0000 L CNN
F 2 "" V 4080 1800 50  0001 C CNN
F 3 "~" H 4150 1800 50  0001 C CNN
	1    4150 1800
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 61938867
P 3950 2600
F 0 "R2" V 3835 2600 50  0000 C CNN
F 1 "1.2k" V 3744 2600 50  0000 C CNN
F 2 "" V 3880 2600 50  0001 C CNN
F 3 "~" H 3950 2600 50  0001 C CNN
	1    3950 2600
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4100 2200 4150 2200
Wire Wire Line
	4150 2200 4150 2250
Wire Wire Line
	4150 1650 4150 1600
Wire Wire Line
	4150 1600 4100 1600
$Comp
L power:GND #PWR?
U 1 1 6194E1FF
P 4150 2000
F 0 "#PWR?" H 4150 1750 50  0001 C CNN
F 1 "GND" H 4238 1963 50  0000 L CNN
F 2 "" H 4150 2000 50  0001 C CNN
F 3 "" H 4150 2000 50  0001 C CNN
	1    4150 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 1950 4150 2000
Wire Wire Line
	3750 1550 3750 1600
Wire Wire Line
	3750 1600 3800 1600
Wire Wire Line
	3750 1600 3750 2200
Wire Wire Line
	3750 2200 3800 2200
Connection ~ 3750 1600
$Comp
L Amplifier_Operational:LM324 U1
U 1 1 619538E6
P 4700 1500
F 0 "U1" H 4700 1700 50  0000 C CNN
F 1 "LM324" H 4700 1800 50  0000 C CNN
F 2 "" H 4650 1600 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2902-n.pdf" H 4750 1700 50  0001 C CNN
	1    4700 1500
	1    0    0    1   
$EndComp
$Comp
L Amplifier_Operational:LM324 U2
U 1 1 619559D5
P 4700 2300
F 0 "U2" H 4700 2667 50  0000 C CNN
F 1 "LM324" H 4700 2576 50  0000 C CNN
F 2 "" H 4650 2400 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2902-n.pdf" H 4750 2500 50  0001 C CNN
	1    4700 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 2650 3750 2600
Wire Wire Line
	3750 2600 3800 2600
Wire Wire Line
	4100 2600 4150 2600
Wire Wire Line
	4150 2600 4150 2550
Wire Wire Line
	4150 2200 4400 2200
Connection ~ 4150 2200
Wire Wire Line
	4150 1600 4400 1600
Connection ~ 4150 1600
Wire Wire Line
	4400 1400 4350 1400
Wire Wire Line
	5050 1500 5000 1500
Wire Wire Line
	4400 2400 4350 2400
Wire Wire Line
	5050 2300 5000 2300
$Comp
L Sensor_Temperature:PT100 TH1
U 1 1 61937B0F
P 4150 2400
F 0 "TH1" H 3997 2446 50  0000 R CNN
F 1 "PT100" H 3997 2355 50  0000 R CNN
F 2 "" H 4150 2450 50  0001 C CNN
F 3 "https://www.heraeus.com/media/media/group/doc_group/products_1/hst/sot_to/de_15/to_92_d.pdf" H 4150 2450 50  0001 C CNN
	1    4150 2400
	1    0    0    -1  
$EndComp
Wire Notes Line
	3650 2900 3650 1300
Wire Notes Line
	3650 1300 4300 1300
Text Notes 3650 3000 0    50   ~ 0
Wheastonen silta
Text Notes 4750 3000 0    50   ~ 0
Instrumentointivahvistin
Text Notes 4300 850  0    50   ~ 0
LM324N:n VCC ja GND pinnin välillä on 22uF suodatuskondensaattori
$Comp
L Device:R R7
U 1 1 61EAC77B
P 5050 1900
F 0 "R7" H 4980 1854 50  0000 R CNN
F 1 "680" H 4980 1945 50  0000 R CNN
F 2 "" V 4980 1900 50  0001 C CNN
F 3 "~" H 5050 1900 50  0001 C CNN
	1    5050 1900
	-1   0    0    1   
$EndComp
Connection ~ 5050 1500
Connection ~ 5050 2300
$Comp
L Device:R R5
U 1 1 61F0FCAC
P 4700 1200
F 0 "R5" V 4907 1200 50  0000 C CNN
F 1 "5.6k" V 4816 1200 50  0000 C CNN
F 2 "" V 4630 1200 50  0001 C CNN
F 3 "~" H 4700 1200 50  0001 C CNN
	1    4700 1200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4550 1200 4350 1200
Wire Wire Line
	4850 1200 5050 1200
$Comp
L Device:R R6
U 1 1 61F10D29
P 4700 2600
F 0 "R6" V 4585 2600 50  0000 C CNN
F 1 "5.6k" V 4494 2600 50  0000 C CNN
F 2 "" V 4630 2600 50  0001 C CNN
F 3 "~" H 4700 2600 50  0001 C CNN
	1    4700 2600
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4550 2600 4350 2600
Wire Wire Line
	4850 2600 5050 2600
Wire Wire Line
	5050 2300 5050 2600
Wire Wire Line
	4350 2400 4350 2600
Wire Wire Line
	4350 1200 4350 1400
Wire Wire Line
	5050 1200 5050 1500
Wire Wire Line
	5050 2300 5050 2200
Wire Wire Line
	5050 1750 5050 1600
Wire Wire Line
	5450 2200 5450 2250
Connection ~ 5450 2200
Wire Wire Line
	5400 2200 5450 2200
Wire Wire Line
	5450 2000 5450 2200
Wire Wire Line
	6150 1600 6150 1900
Wire Wire Line
	5450 1600 5450 1800
Connection ~ 6150 1900
Wire Wire Line
	6150 1900 6200 1900
Wire Wire Line
	5450 2550 5450 2600
$Comp
L power:GND #PWR?
U 1 1 619B2F28
P 5450 2600
F 0 "#PWR?" H 5450 2350 50  0001 C CNN
F 1 "GND" H 5455 2427 50  0000 C CNN
F 2 "" H 5450 2600 50  0001 C CNN
F 3 "" H 5450 2600 50  0001 C CNN
	1    5450 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	5450 2000 5500 2000
Wire Wire Line
	6150 1900 6100 1900
Wire Wire Line
	5950 1600 6150 1600
Wire Wire Line
	5450 1800 5500 1800
Wire Wire Line
	5650 1600 5450 1600
$Comp
L Device:R R11
U 1 1 6198C724
P 5450 2400
F 0 "R11" H 5380 2354 50  0000 R CNN
F 1 "33k" H 5380 2445 50  0000 R CNN
F 2 "" V 5380 2400 50  0001 C CNN
F 3 "~" H 5450 2400 50  0001 C CNN
	1    5450 2400
	-1   0    0    1   
$EndComp
$Comp
L Device:R R10
U 1 1 6198C0CA
P 5800 1600
F 0 "R10" V 6007 1600 50  0000 C CNN
F 1 "33k" V 5916 1600 50  0000 C CNN
F 2 "" V 5730 1600 50  0001 C CNN
F 3 "~" H 5800 1600 50  0001 C CNN
	1    5800 1600
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R9
U 1 1 6198BA98
P 5250 2200
F 0 "R9" V 5135 2200 50  0000 C CNN
F 1 "8.2k" V 5044 2200 50  0000 C CNN
F 2 "" V 5180 2200 50  0001 C CNN
F 3 "~" H 5250 2200 50  0001 C CNN
	1    5250 2200
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R8
U 1 1 6198B647
P 5250 1600
F 0 "R8" V 5043 1600 50  0000 C CNN
F 1 "8.2k" V 5134 1600 50  0000 C CNN
F 2 "" V 5180 1600 50  0001 C CNN
F 3 "~" H 5250 1600 50  0001 C CNN
	1    5250 1600
	0    1    1    0   
$EndComp
$Comp
L Amplifier_Operational:LM324 U3
U 1 1 61989D09
P 5800 1900
F 0 "U3" H 5800 2100 50  0000 C CNN
F 1 "LM324" H 5800 2200 50  0000 C CNN
F 2 "" H 5750 2000 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2902-n.pdf" H 5850 2100 50  0001 C CNN
	1    5800 1900
	1    0    0    1   
$EndComp
Wire Wire Line
	5400 1600 5450 1600
Connection ~ 5450 1600
Wire Wire Line
	5050 1600 5100 1600
Connection ~ 5050 1600
Wire Wire Line
	5050 1600 5050 1500
Wire Wire Line
	5050 2200 5100 2200
Connection ~ 5050 2200
Wire Wire Line
	5050 2200 5050 2050
Text Notes 6450 2650 0    50   ~ 0
Sallen-Key Alipäästösuodin
Wire Notes Line
	7700 2550 7700 1200
Wire Notes Line
	6200 2550 7700 2550
Text GLabel 7750 1800 2    50   Input ~ 0
VAHVISTIN_LÄHTÖ
$Comp
L Device:R R12
U 1 1 61AA5678
P 6350 1900
F 0 "R12" V 6557 1900 50  0000 C CNN
F 1 "1M" V 6466 1900 50  0000 C CNN
F 2 "" V 6280 1900 50  0001 C CNN
F 3 "~" H 6350 1900 50  0001 C CNN
	1    6350 1900
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R13
U 1 1 61AA5D37
P 6750 1900
F 0 "R13" V 6957 1900 50  0000 C CNN
F 1 "1M" V 6866 1900 50  0000 C CNN
F 2 "" V 6680 1900 50  0001 C CNN
F 3 "~" H 6750 1900 50  0001 C CNN
	1    6750 1900
	0    -1   -1   0   
$EndComp
Connection ~ 6550 1900
Wire Wire Line
	6500 1900 6550 1900
Wire Wire Line
	6550 1900 6600 1900
Wire Wire Line
	6550 1550 6550 1900
Connection ~ 6950 1550
Wire Wire Line
	6900 1550 6950 1550
Wire Wire Line
	6600 1550 6550 1550
Wire Wire Line
	7650 1800 7750 1800
Connection ~ 7650 1800
Wire Wire Line
	6950 1700 7000 1700
Wire Wire Line
	6950 1550 6950 1700
Wire Wire Line
	7650 1550 6950 1550
Wire Wire Line
	7650 1800 7650 1550
Wire Wire Line
	7600 1800 7650 1800
Wire Wire Line
	6950 2250 6950 2300
$Comp
L power:GND #PWR?
U 1 1 61AD3AA5
P 6950 2300
F 0 "#PWR?" H 6950 2050 50  0001 C CNN
F 1 "GND" H 6955 2127 50  0000 C CNN
F 2 "" H 6950 2300 50  0001 C CNN
F 3 "" H 6950 2300 50  0001 C CNN
	1    6950 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6950 1900 7000 1900
Connection ~ 6950 1900
Wire Wire Line
	6950 1950 6950 1900
$Comp
L Device:C C2
U 1 1 61AA2B56
P 6950 2100
F 0 "C2" H 6835 2146 50  0000 R CNN
F 1 "10n" H 6835 2055 50  0000 R CNN
F 2 "" H 6988 1950 50  0001 C CNN
F 3 "~" H 6950 2100 50  0001 C CNN
	1    6950 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	6900 1900 6950 1900
$Comp
L Device:C C1
U 1 1 61AA3D1E
P 6750 1550
F 0 "C1" V 7002 1550 50  0000 C CNN
F 1 "20n" V 6911 1550 50  0000 C CNN
F 2 "" H 6788 1400 50  0001 C CNN
F 3 "~" H 6750 1550 50  0001 C CNN
	1    6750 1550
	0    -1   -1   0   
$EndComp
$Comp
L Amplifier_Operational:LM324 U4
U 1 1 61AA05F8
P 7300 1800
F 0 "U4" H 7300 2000 50  0000 C CNN
F 1 "LM324" H 7300 2100 50  0000 C CNN
F 2 "" H 7250 1900 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2902-n.pdf" H 7350 2000 50  0001 C CNN
	1    7300 1800
	1    0    0    1   
$EndComp
Wire Notes Line
	4300 900  6200 900 
Wire Notes Line
	7700 1200 6200 1200
Wire Notes Line
	4300 900  4300 2900
Wire Notes Line
	3650 2900 6200 2900
Wire Notes Line
	6200 900  6200 2900
$EndSCHEMATC
