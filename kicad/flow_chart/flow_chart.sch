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
L flow_chart-rescue:ATMega328P-flow_chart-noname-rescue-flow_chart-rescue ATMega328P
U 1 1 61535601
P 6400 3500
F 0 "ATMega328P" H 6250 3400 50  0000 L CNN
F 1 "Mikrokontrolleri" H 6200 3500 50  0000 L CNN
F 2 "" H 6400 3500 50  0001 C CNN
F 3 "" H 6400 3500 50  0001 C CNN
	1    6400 3500
	1    0    0    -1  
$EndComp
$Comp
L flow_chart-rescue:PowerSource-flow_chart-noname-rescue-flow_chart-rescue 12V
U 1 1 615362DB
P 4100 3750
F 0 "12V" H 4100 3650 50  0000 C CNN
F 1 "Virtalähde" H 4100 3750 50  0000 C CNN
F 2 "" H 4100 3750 50  0001 C CNN
F 3 "" H 4100 3750 50  0001 C CNN
	1    4100 3750
	1    0    0    -1  
$EndComp
$Comp
L flow_chart-rescue:Switch-flow_chart-noname-rescue-flow_chart-rescue U?
U 1 1 61536F6F
P 4950 3750
F 0 "U?" H 4950 4015 50  0001 C CNN
F 1 "Virtakytkin" H 4950 3700 50  0000 C CNN
F 2 "" H 4950 3750 50  0001 C CNN
F 3 "" H 4950 3750 50  0001 C CNN
	1    4950 3750
	1    0    0    -1  
$EndComp
$Comp
L flow_chart-rescue:VoltageRegulator-flow_chart-noname-rescue-flow_chart-rescue MCP2011
U 1 1 6153ABFC
P 5400 3050
F 0 "MCP2011" H 5400 2900 50  0000 C CNN
F 1 "Regulaattori" H 5400 3000 50  0000 C CNN
F 2 "" H 5400 3050 50  0001 C CNN
F 3 "" H 5400 3050 50  0001 C CNN
	1    5400 3050
	1    0    0    -1  
$EndComp
Text Label 4550 3800 0    50   ~ 0
12V
Text Label 5400 3600 2    50   ~ 0
12V
Text Label 5850 3150 0    50   ~ 0
5V
$Comp
L flow_chart-rescue:ADC-flow_chart-noname-rescue-flow_chart-rescue AD-Muunnin
U 1 1 6154363F
P 5700 4300
F 0 "AD-Muunnin" H 5700 4300 50  0000 C CNN
F 1 "MCP3202-C" H 5700 4200 50  0000 C CNN
F 2 "" H 5700 4300 50  0001 C CNN
F 3 "" H 5700 4300 50  0001 C CNN
	1    5700 4300
	1    0    0    -1  
$EndComp
$Comp
L flow_chart-rescue:pt100_wheatstone-flow_chart-rescue PT100
U 1 1 6155524C
P 3850 4300
F 0 "PT100" H 3850 4300 50  0000 C CNN
F 1 "(Wheatstone)" H 3850 4200 50  0000 C CNN
F 2 "" H 3850 4300 50  0001 C CNN
F 3 "" H 3850 4300 50  0001 C CNN
	1    3850 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6050 4350 6350 4350
Wire Wire Line
	6350 4350 6350 3950
Text Label 6100 4350 0    50   ~ 0
SPI
$Comp
L flow_chart-rescue:Buzzer-flow_chart-noname-rescue-flow_chart-rescue U?
U 1 1 61559377
P 6650 4350
F 0 "U?" H 6878 4454 50  0001 L CNN
F 1 "Summeri" H 6500 4350 50  0000 L CNN
F 2 "" H 6650 4350 50  0001 C CNN
F 3 "" H 6650 4350 50  0001 C CNN
	1    6650 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	6650 3950 6650 4150
Text Label 6650 4050 0    50   ~ 0
5V
$Comp
L flow_chart-rescue:LCD-flow_chart-noname-rescue-flow_chart-rescue MC21605H6W
U 1 1 6155C6F0
P 6850 2700
F 0 "MC21605H6W" H 6600 2650 50  0000 L CNN
F 1 "LCD Näyttö" H 6650 2750 50  0000 L CNN
F 2 "" H 6850 2700 50  0001 C CNN
F 3 "" H 6850 2700 50  0001 C CNN
	1    6850 2700
	1    0    0    -1  
$EndComp
Text Label 6700 3150 0    50   ~ 0
Hitachi_HD44780
$Comp
L flow_chart-rescue:Button-flow_chart-noname-rescue-flow_chart-rescue U?
U 1 1 6155FD0E
P 7350 3350
F 0 "U?" H 7578 3346 50  0001 L CNN
F 1 "Nappi" H 7200 3300 50  0000 L CNN
F 2 "" H 7350 3350 50  0001 C CNN
F 3 "" H 7350 3350 50  0001 C CNN
	1    7350 3350
	1    0    0    -1  
$EndComp
$Comp
L flow_chart-rescue:Button-flow_chart-noname-rescue-flow_chart-rescue U?
U 1 1 61560A43
P 7350 3650
F 0 "U?" H 7578 3646 50  0001 L CNN
F 1 "Nappi" H 7200 3600 50  0000 L CNN
F 2 "" H 7350 3650 50  0001 C CNN
F 3 "" H 7350 3650 50  0001 C CNN
	1    7350 3650
	1    0    0    -1  
$EndComp
$Comp
L flow_chart-rescue:Amplifier-flow_chart-noname-rescue-flow_chart-rescue Instrumenta-
U 1 1 615651FF
P 4800 4350
F 0 "Instrumenta-" H 4800 4400 50  0000 C CNN
F 1 "atiovahvistin" H 4800 4300 50  0000 C CNN
F 2 "" H 4800 4350 50  0001 C CNN
F 3 "" H 4800 4350 50  0001 C CNN
	1    4800 4350
	1    0    0    -1  
$EndComp
$Comp
L flow_chart-rescue:NVRAM-flow_chart-noname-rescue-flow_chart-rescue 47C16-I
U 1 1 615E3FA7
P 6100 2700
F 0 "47C16-I" H 5950 2650 50  0000 L CNN
F 1 "NVRAM" H 5950 2750 50  0000 L CNN
F 2 "" H 6100 2700 50  0001 C CNN
F 3 "" H 6100 2700 50  0001 C CNN
	1    6100 2700
	1    0    0    -1  
$EndComp
Text Label 6300 3150 0    50   ~ 0
I2C
Text GLabel 5950 3150 2    50   Input ~ 0
VCC
Wire Wire Line
	5800 3150 5950 3150
Wire Wire Line
	6900 3400 7050 3400
Wire Wire Line
	6900 3700 7050 3700
Text Label 6950 3700 0    50   ~ 0
5V
Text Label 6950 3400 0    50   ~ 0
5V
$Comp
L flow_chart-rescue:OSCILLATOR-flow_chart-noname-rescue-flow_chart-rescue 16MHz
U 1 1 616016E3
P 5700 3900
F 0 "16MHz" H 5550 3850 50  0000 L CNN
F 1 "Oskillaattori" H 5450 3950 50  0000 L CNN
F 2 "" H 5700 3900 50  0001 C CNN
F 3 "" H 5700 3900 50  0001 C CNN
	1    5700 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 3700 5700 3550
Wire Wire Line
	5700 3550 6100 3550
Wire Wire Line
	6300 2950 6300 3150
Wire Wire Line
	6700 2950 6700 3150
Text Label 5750 3550 0    50   ~ 0
5V_kello
Wire Wire Line
	4500 3800 4650 3800
Wire Wire Line
	5250 3800 5400 3800
Wire Wire Line
	5400 3800 5400 3400
Wire Wire Line
	6300 2950 6100 2950
Wire Wire Line
	6700 2950 6850 2950
Wire Wire Line
	4250 4350 4400 4350
Wire Wire Line
	5200 4350 5350 4350
$EndSCHEMATC
