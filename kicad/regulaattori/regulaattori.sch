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
L Regulator_Switching:MAX5035AUPA U?
U 1 1 61B85338
P 5250 3450
F 0 "U?" H 5250 3917 50  0001 C CNN
F 1 "MAX5033B" H 5250 3825 50  0000 C CNN
F 2 "Package_DIP:DIP-8_W7.62mm" H 5400 3100 50  0001 L CIN
F 3 "http://datasheets.maximintegrated.com/en/ds/MAX5035.pdf" H 5250 3400 50  0001 C CNN
	1    5250 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 3850 5150 4050
Wire Wire Line
	5150 4050 5350 4050
Wire Wire Line
	5350 4050 5350 3850
$Comp
L Device:L L1
U 1 1 61B862C0
P 6050 3650
F 0 "L1" H 6102 3696 50  0000 L CNN
F 1 "220u" H 6102 3605 50  0000 L CNN
F 2 "" H 6050 3650 50  0001 C CNN
F 3 "~" H 6050 3650 50  0001 C CNN
	1    6050 3650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 61B88351
P 4050 3350
F 0 "R2" V 3935 3350 50  0000 C CNN
F 1 "1k" V 3844 3350 50  0000 C CNN
F 2 "" V 3980 3350 50  0001 C CNN
F 3 "~" H 4050 3350 50  0001 C CNN
	1    4050 3350
	0    -1   -1   0   
$EndComp
$Comp
L Diode:1N4148 D1
U 1 1 61B8A269
P 6250 3450
F 0 "D1" H 6250 3667 50  0000 C CNN
F 1 "1N4148" H 6250 3576 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 6250 3275 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/1N4148_1N4448.pdf" H 6250 3450 50  0001 C CNN
	1    6250 3450
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 61B8D486
P 4800 3850
F 0 "C1" H 4600 3850 50  0000 L CNN
F 1 "100n" H 4500 3750 50  0000 L CNN
F 2 "" H 4838 3700 50  0001 C CNN
F 3 "~" H 4800 3850 50  0001 C CNN
	1    4800 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 4000 4800 4050
Wire Wire Line
	4800 4050 5150 4050
Connection ~ 5150 4050
$Comp
L power:GND #PWR?
U 1 1 61B94241
P 6450 3500
F 0 "#PWR?" H 6450 3250 50  0001 C CNN
F 1 "GND" H 6455 3327 50  0000 C CNN
F 2 "" H 6450 3500 50  0001 C CNN
F 3 "" H 6450 3500 50  0001 C CNN
	1    6450 3500
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 61B97661
P 6050 4050
F 0 "C2" H 6165 4096 50  0000 L CNN
F 1 "1m" H 6165 4005 50  0000 L CNN
F 2 "" H 6088 3900 50  0001 C CNN
F 3 "~" H 6050 4050 50  0001 C CNN
	1    6050 4050
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR?
U 1 1 61B98F13
P 6600 3800
F 0 "#PWR?" H 6600 3650 50  0001 C CNN
F 1 "VCC" H 6615 3973 50  0000 C CNN
F 2 "" H 6600 3800 50  0001 C CNN
F 3 "" H 6600 3800 50  0001 C CNN
	1    6600 3800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 61B9A4B2
P 6050 4250
F 0 "#PWR?" H 6050 4000 50  0001 C CNN
F 1 "GND" H 6055 4077 50  0000 C CNN
F 2 "" H 6050 4250 50  0001 C CNN
F 3 "" H 6050 4250 50  0001 C CNN
	1    6050 4250
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 61B9B3F0
P 5850 3250
F 0 "C3" V 5598 3250 50  0000 C CNN
F 1 "100n" V 5689 3250 50  0000 C CNN
F 2 "" H 5888 3100 50  0001 C CNN
F 3 "~" H 5850 3250 50  0001 C CNN
	1    5850 3250
	0    1    1    0   
$EndComp
Wire Wire Line
	5650 3250 5700 3250
Wire Wire Line
	5650 3450 6050 3450
Wire Wire Line
	6050 3450 6050 3500
Wire Wire Line
	6000 3250 6050 3250
Wire Wire Line
	6050 3250 6050 3450
Connection ~ 6050 3450
Wire Wire Line
	6050 3450 6100 3450
Wire Wire Line
	6450 3450 6400 3450
Wire Wire Line
	6450 3500 6450 3450
Wire Wire Line
	6050 3850 5650 3850
Wire Wire Line
	5650 3850 5650 3650
Wire Wire Line
	6050 3800 6050 3850
Wire Wire Line
	6050 3900 6050 3850
Connection ~ 6050 3850
Wire Wire Line
	6050 4200 6050 4250
Wire Wire Line
	6600 3850 6600 3800
Wire Wire Line
	6050 3850 6600 3850
$Comp
L power:GND #PWR?
U 1 1 61C0227C
P 5350 4100
F 0 "#PWR?" H 5350 3850 50  0001 C CNN
F 1 "GND" H 5355 3927 50  0000 C CNN
F 2 "" H 5350 4100 50  0001 C CNN
F 3 "" H 5350 4100 50  0001 C CNN
	1    5350 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 4100 5350 4050
Connection ~ 5350 4050
Wire Wire Line
	3850 3250 3850 3350
Wire Wire Line
	3850 3350 3900 3350
$Comp
L Device:C C4
U 1 1 61C18D39
P 3850 3550
F 0 "C4" H 3965 3504 50  0000 L CNN
F 1 "47u" H 3965 3595 50  0000 L CNN
F 2 "" H 3888 3400 50  0001 C CNN
F 3 "~" H 3850 3550 50  0001 C CNN
	1    3850 3550
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 61C198B0
P 3850 3750
F 0 "#PWR?" H 3850 3500 50  0001 C CNN
F 1 "GND" H 3855 3577 50  0000 C CNN
F 2 "" H 3850 3750 50  0001 C CNN
F 3 "" H 3850 3750 50  0001 C CNN
	1    3850 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3850 3750 3850 3700
Wire Wire Line
	3850 3400 3850 3350
Connection ~ 3850 3350
$Comp
L Switch:SW_SPST SW1
U 1 1 61C20E87
P 4300 3700
F 0 "SW1" V 4300 3798 50  0000 L CNN
F 1 "SW_SPST" H 4300 3844 50  0001 C CNN
F 2 "" H 4300 3700 50  0001 C CNN
F 3 "~" H 4300 3700 50  0001 C CNN
	1    4300 3700
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3850 3250 4850 3250
$Comp
L Simulation_SPICE:VDC V1
U 1 1 61C3816B
P 3450 3500
F 0 "V1" H 3320 3546 50  0000 R CNN
F 1 "12V" H 3320 3455 50  0000 R CNN
F 2 "" H 3450 3500 50  0001 C CNN
F 3 "~" H 3450 3500 50  0001 C CNN
F 4 "Y" H 3450 3500 50  0001 L CNN "Spice_Netlist_Enabled"
F 5 "V" H 3450 3500 50  0001 L CNN "Spice_Primitive"
F 6 "dc(1)" H 3320 3409 50  0001 R CNN "Spice_Model"
	1    3450 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 3250 3850 3250
Connection ~ 3850 3250
Wire Wire Line
	3450 3250 3450 3300
$Comp
L power:GND #PWR?
U 1 1 61C3FF38
P 3450 3750
F 0 "#PWR?" H 3450 3500 50  0001 C CNN
F 1 "GND" H 3455 3577 50  0000 C CNN
F 2 "" H 3450 3750 50  0001 C CNN
F 3 "" H 3450 3750 50  0001 C CNN
	1    3450 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 3750 3450 3700
Wire Wire Line
	4800 3700 4800 3650
Wire Wire Line
	4800 3650 4850 3650
Wire Wire Line
	4200 3350 4300 3350
$Comp
L Device:R R1
U 1 1 61B86DA6
P 4450 3700
F 0 "R1" H 4520 3746 50  0000 L CNN
F 1 "1k" H 4520 3655 50  0000 L CNN
F 2 "" V 4380 3700 50  0001 C CNN
F 3 "~" H 4450 3700 50  0001 C CNN
	1    4450 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 4050 4300 4050
Wire Wire Line
	4300 3900 4300 3950
Connection ~ 4800 4050
Wire Wire Line
	4450 3850 4450 3950
Wire Wire Line
	4450 3950 4300 3950
Connection ~ 4300 3950
Wire Wire Line
	4300 3950 4300 4050
Wire Wire Line
	4300 3500 4300 3450
Connection ~ 4300 3350
Wire Wire Line
	4300 3350 4850 3350
Wire Wire Line
	4450 3550 4450 3450
Wire Wire Line
	4450 3450 4300 3450
Connection ~ 4300 3450
Wire Wire Line
	4300 3450 4300 3350
$EndSCHEMATC
