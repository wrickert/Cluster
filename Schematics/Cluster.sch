EESchema Schematic File Version 4
LIBS:Cluster-cache
EELAYER 29 0
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
Text GLabel 1050 1400 2    50   Input ~ 0
Neutral
Text GLabel 1050 1500 2    50   Input ~ 0
Fuel
Text GLabel 1050 1600 2    50   Input ~ 0
Oil
Text GLabel 1050 1900 2    50   Input ~ 0
OverDrive
Text GLabel 1050 2000 2    50   Input ~ 0
LeftTurn
Text GLabel 1050 2100 2    50   Input ~ 0
RightTurn
Text GLabel 1050 2200 2    50   Input ~ 0
HighBeam
$Comp
L power:+12V #PWR03
U 1 1 5BF6B374
P 1050 2500
F 0 "#PWR03" H 1050 2350 50  0001 C CNN
F 1 "+12V" V 1065 2628 50  0000 L CNN
F 2 "" H 1050 2500 50  0001 C CNN
F 3 "" H 1050 2500 50  0001 C CNN
	1    1050 2500
	0    1    1    0   
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J2
U 1 1 5BF76E42
P 850 3050
F 0 "J2" H 770 2725 50  0000 C CNN
F 1 "Serial" H 770 2816 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 850 3050 50  0001 C CNN
F 3 "~" H 850 3050 50  0001 C CNN
	1    850  3050
	-1   0    0    1   
$EndComp
Text GLabel 1050 2950 2    50   Input ~ 0
TX
Text GLabel 1050 3050 2    50   Input ~ 0
RX
$Comp
L MCU_ST_STM32F4:STM32F411RETx U3
U 1 1 5CA0E699
P 8400 3450
F 0 "U3" H 7900 1650 50  0000 C CNN
F 1 "STM32F411RETx" H 8850 1500 50  0000 C CNN
F 2 "Housings_QFP:LQFP-64_10x10mm_Pitch0.5mm" H 7800 1750 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00115249.pdf" H 8400 3450 50  0001 C CNN
	1    8400 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	8600 1750 8500 1750
Wire Wire Line
	8200 1750 8200 1700
Connection ~ 8200 1750
Connection ~ 8300 1750
Wire Wire Line
	8300 1750 8200 1750
Connection ~ 8400 1750
Wire Wire Line
	8400 1750 8300 1750
Connection ~ 8500 1750
Wire Wire Line
	8500 1750 8400 1750
Wire Wire Line
	8600 5250 8500 5250
Wire Wire Line
	8200 5250 8200 5300
Connection ~ 8200 5250
Connection ~ 8300 5250
Wire Wire Line
	8300 5250 8200 5250
Connection ~ 8400 5250
Wire Wire Line
	8400 5250 8300 5250
Connection ~ 8500 5250
Wire Wire Line
	8500 5250 8400 5250
$Comp
L power:VSS #PWR0101
U 1 1 5CA0FF4A
P 8200 5300
F 0 "#PWR0101" H 8200 5150 50  0001 C CNN
F 1 "VSS" H 8218 5473 50  0000 C CNN
F 2 "" H 8200 5300 50  0001 C CNN
F 3 "" H 8200 5300 50  0001 C CNN
	1    8200 5300
	-1   0    0    1   
$EndComp
Text GLabel 9100 4250 2    50   Input ~ 0
SCL
Text GLabel 9100 4350 2    50   Input ~ 0
SDA
Text GLabel 7700 4350 0    50   Input ~ 0
RightTurn
Text GLabel 7700 4250 0    50   Input ~ 0
LeftTurn
Text GLabel 7700 4150 0    50   Input ~ 0
Oil
Text GLabel 9100 5050 2    50   Input ~ 0
HighBeam
Text GLabel 9100 4950 2    50   Input ~ 0
Neutral
Text GLabel 9100 4850 2    50   Input ~ 0
LowFuel
Text GLabel 9100 1950 2    50   Input ~ 0
Fuel
Text GLabel 9100 2050 2    50   Input ~ 0
Temp
Text GLabel 7450 1850 2    50   Input ~ 0
Reset
$Comp
L power:VCC #PWR0102
U 1 1 5CA10461
P 8200 1700
F 0 "#PWR0102" H 8200 1550 50  0001 C CNN
F 1 "VCC" H 8217 1873 50  0000 C CNN
F 2 "" H 8200 1700 50  0001 C CNN
F 3 "" H 8200 1700 50  0001 C CNN
	1    8200 1700
	1    0    0    -1  
$EndComp
Text GLabel 9100 2950 2    50   Input ~ 0
RX
Text GLabel 9100 2850 2    50   Input ~ 0
TX
$Comp
L Regulator_Switching:LM2596S-ADJ U2
U 1 1 5CA10785
P 4450 6400
F 0 "U2" H 4450 6767 50  0000 C CNN
F 1 "LM2596S-ADJ" H 4450 6676 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:TO-263-5_TabPin3" H 4500 6150 50  0001 L CIN
F 3 "http://www.ti.com/lit/ds/symlink/lm2596.pdf" H 4450 6400 50  0001 C CNN
	1    4450 6400
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0103
U 1 1 5CA109EF
P 1050 2400
F 0 "#PWR0103" H 1050 2250 50  0001 C CNN
F 1 "VCC" V 1067 2528 50  0000 L CNN
F 2 "" H 1050 2400 50  0001 C CNN
F 3 "" H 1050 2400 50  0001 C CNN
	1    1050 2400
	0    1    1    0   
$EndComp
$Comp
L Device:CP1 C1
U 1 1 5CA10B7D
P 3500 6450
F 0 "C1" H 3615 6496 50  0000 L CNN
F 1 "100uF" H 3615 6405 50  0000 L CNN
F 2 "Capacitors_SMD:CP_Elec_8x10.5" H 3500 6450 50  0001 C CNN
F 3 "~" H 3500 6450 50  0001 C CNN
	1    3500 6450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3350 6300 3500 6300
Connection ~ 3500 6300
Wire Wire Line
	3500 6300 3950 6300
$Comp
L power:+12V #PWR0104
U 1 1 5CA10D2D
P 3350 6300
F 0 "#PWR0104" H 3350 6150 50  0001 C CNN
F 1 "+12V" V 3365 6428 50  0000 L CNN
F 2 "" H 3350 6300 50  0001 C CNN
F 3 "" H 3350 6300 50  0001 C CNN
	1    3350 6300
	0    -1   -1   0   
$EndComp
$Comp
L power:VSS #PWR0105
U 1 1 5CA10FFE
P 3300 6800
F 0 "#PWR0105" H 3300 6650 50  0001 C CNN
F 1 "VSS" V 3318 6928 50  0000 L CNN
F 2 "" H 3300 6800 50  0001 C CNN
F 3 "" H 3300 6800 50  0001 C CNN
	1    3300 6800
	0    -1   -1   0   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J10
U 1 1 5CA113CA
P 4150 7000
F 0 "J10" H 4229 7042 50  0000 L CNN
F 1 "PowerReg" H 4229 6951 50  0000 L CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01_Pitch2.54mm" H 4150 7000 50  0001 C CNN
F 3 "~" H 4150 7000 50  0001 C CNN
	1    4150 7000
	1    0    0    -1  
$EndComp
$Comp
L Device:L_Small L1
U 1 1 5CA11743
P 5400 6500
F 0 "L1" V 5585 6500 50  0000 C CNN
F 1 "33uH" V 5494 6500 50  0000 C CNN
F 2 "Inductors_SMD:L_6.3x6.3_H3" H 5400 6500 50  0001 C CNN
F 3 "~" H 5400 6500 50  0001 C CNN
	1    5400 6500
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4950 6500 5150 6500
Connection ~ 5150 6500
Wire Wire Line
	5150 6500 5300 6500
$Comp
L Device:CP1 C4
U 1 1 5CA11A8E
P 5700 6650
F 0 "C4" H 5815 6696 50  0000 L CNN
F 1 "220uF" H 5815 6605 50  0000 L CNN
F 2 "Capacitors_SMD:CP_Elec_8x10.5" H 5700 6650 50  0001 C CNN
F 3 "~" H 5700 6650 50  0001 C CNN
	1    5700 6650
	1    0    0    -1  
$EndComp
Connection ~ 5150 6800
Wire Wire Line
	5150 6800 4450 6800
Connection ~ 5700 6800
Wire Wire Line
	5700 6800 5450 6800
Wire Wire Line
	3500 6600 3500 6800
Connection ~ 3500 6800
Wire Wire Line
	3500 6800 3300 6800
Wire Wire Line
	4450 6700 4450 6800
Connection ~ 4450 6800
Wire Wire Line
	4450 6800 3500 6800
Wire Wire Line
	3950 6500 3950 7000
$Comp
L Device:R_Small R1
U 1 1 5CA12D78
P 5600 6400
F 0 "R1" H 5659 6446 50  0000 L CNN
F 1 "3.5k" H 5659 6355 50  0000 L CNN
F 2 "Resistors_SMD:R_0805" H 5600 6400 50  0001 C CNN
F 3 "~" H 5600 6400 50  0001 C CNN
	1    5600 6400
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R2
U 1 1 5CA12E1A
P 6050 6300
F 0 "R2" V 5854 6300 50  0000 C CNN
F 1 "2k" V 5945 6300 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" H 6050 6300 50  0001 C CNN
F 3 "~" H 6050 6300 50  0001 C CNN
	1    6050 6300
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C5
U 1 1 5CA12F62
P 5850 6400
F 0 "C5" H 5942 6446 50  0000 L CNN
F 1 "15nF" H 5942 6355 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 5850 6400 50  0001 C CNN
F 3 "~" H 5850 6400 50  0001 C CNN
	1    5850 6400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4950 6300 5600 6300
Connection ~ 5600 6300
Wire Wire Line
	5600 6300 5850 6300
Connection ~ 5850 6300
Wire Wire Line
	5850 6300 5950 6300
Wire Wire Line
	5500 6500 5600 6500
Connection ~ 5600 6500
Wire Wire Line
	5600 6500 5700 6500
Connection ~ 5700 6500
Wire Wire Line
	5700 6500 5850 6500
Connection ~ 5850 6500
Wire Wire Line
	5850 6500 6550 6500
Wire Wire Line
	6150 6300 6250 6300
Wire Wire Line
	6250 6300 6250 6800
Wire Wire Line
	5700 6800 6250 6800
$Comp
L power:VSS #PWR0106
U 1 1 5CA1436E
P 5450 6800
F 0 "#PWR0106" H 5450 6650 50  0001 C CNN
F 1 "VSS" H 5468 6973 50  0000 C CNN
F 2 "" H 5450 6800 50  0001 C CNN
F 3 "" H 5450 6800 50  0001 C CNN
	1    5450 6800
	-1   0    0    1   
$EndComp
Connection ~ 5450 6800
Wire Wire Line
	5450 6800 5150 6800
$Comp
L power:VCC #PWR0107
U 1 1 5CA14453
P 6550 6500
F 0 "#PWR0107" H 6550 6350 50  0001 C CNN
F 1 "VCC" V 6567 6628 50  0000 L CNN
F 2 "" H 6550 6500 50  0001 C CNN
F 3 "" H 6550 6500 50  0001 C CNN
	1    6550 6500
	0    1    1    0   
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J3
U 1 1 5CA14813
P 850 3750
F 0 "J3" H 770 3425 50  0000 C CNN
F 1 "i2c" H 770 3516 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 850 3750 50  0001 C CNN
F 3 "~" H 850 3750 50  0001 C CNN
	1    850  3750
	-1   0    0    1   
$EndComp
Text GLabel 1050 3650 2    50   Input ~ 0
SCL
Text GLabel 1050 3750 2    50   Input ~ 0
SDA
$Comp
L Device:R_Small R5
U 1 1 5CA17DA8
P 7350 2150
F 0 "R5" V 7154 2150 50  0000 C CNN
F 1 "100k" V 7245 2150 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" H 7350 2150 50  0001 C CNN
F 3 "~" H 7350 2150 50  0001 C CNN
	1    7350 2150
	0    1    1    0   
$EndComp
Wire Wire Line
	7450 2150 7700 2150
$Comp
L power:VSS #PWR0108
U 1 1 5CA18C96
P 6950 2150
F 0 "#PWR0108" H 6950 2000 50  0001 C CNN
F 1 "VSS" V 6968 2278 50  0000 L CNN
F 2 "" H 6950 2150 50  0001 C CNN
F 3 "" H 6950 2150 50  0001 C CNN
	1    6950 2150
	0    -1   -1   0   
$EndComp
$Comp
L Device:C_Small C8
U 1 1 5CA1972A
P 7150 1950
F 0 "C8" V 6921 1950 50  0000 C CNN
F 1 "100nF" V 7012 1950 50  0000 C CNN
F 2 "Capacitors_SMD:C_0805" H 7150 1950 50  0001 C CNN
F 3 "~" H 7150 1950 50  0001 C CNN
	1    7150 1950
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R6
U 1 1 5CA19799
P 7450 1700
F 0 "R6" H 7391 1654 50  0000 R CNN
F 1 "100k" H 7391 1745 50  0000 R CNN
F 2 "Resistors_SMD:R_0805" H 7450 1700 50  0001 C CNN
F 3 "~" H 7450 1700 50  0001 C CNN
	1    7450 1700
	-1   0    0    1   
$EndComp
Wire Wire Line
	7250 1950 7450 1950
Wire Wire Line
	7450 1800 7450 1950
Connection ~ 7450 1950
Wire Wire Line
	7450 1950 7700 1950
$Comp
L Device:C_Small C9
U 1 1 5CA1ABF0
P 7150 2350
F 0 "C9" V 6921 2350 50  0000 C CNN
F 1 "4.7uF" V 7012 2350 50  0000 C CNN
F 2 "Capacitors_SMD:C_0805" H 7150 2350 50  0001 C CNN
F 3 "~" H 7150 2350 50  0001 C CNN
	1    7150 2350
	0    1    1    0   
$EndComp
Wire Wire Line
	7050 1950 7050 2150
Wire Wire Line
	6950 2150 7050 2150
Connection ~ 7050 2150
Wire Wire Line
	7050 2150 7050 2350
Wire Wire Line
	7050 2150 7250 2150
Wire Wire Line
	7450 1600 7450 1550
$Comp
L power:VCC #PWR0109
U 1 1 5CA1CECD
P 7450 1550
F 0 "#PWR0109" H 7450 1400 50  0001 C CNN
F 1 "VCC" H 7467 1723 50  0000 C CNN
F 2 "" H 7450 1550 50  0001 C CNN
F 3 "" H 7450 1550 50  0001 C CNN
	1    7450 1550
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C10
U 1 1 5CA1D024
P 8700 1600
F 0 "C10" H 8792 1646 50  0000 L CNN
F 1 "100nF" H 8792 1555 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 8700 1600 50  0001 C CNN
F 3 "~" H 8700 1600 50  0001 C CNN
	1    8700 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	8700 1700 8700 1750
Wire Wire Line
	8700 1500 8700 1450
$Comp
L power:VCC #PWR0110
U 1 1 5CA1E887
P 8700 1450
F 0 "#PWR0110" H 8700 1300 50  0001 C CNN
F 1 "VCC" H 8717 1623 50  0000 C CNN
F 2 "" H 8700 1450 50  0001 C CNN
F 3 "" H 8700 1450 50  0001 C CNN
	1    8700 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	7250 2350 7700 2350
$Comp
L Device:Crystal Y1
U 1 1 5CA1F74F
P 7250 3000
F 0 "Y1" V 7204 3131 50  0000 L CNN
F 1 "8MHz" V 7295 3131 50  0000 L CNN
F 2 "Crystals:Crystal_SMD_5032-2pin_5.0x3.2mm" H 7250 3000 50  0001 C CNN
F 3 "~" H 7250 3000 50  0001 C CNN
	1    7250 3000
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C7
U 1 1 5CA1F826
P 6900 3150
F 0 "C7" V 6671 3150 50  0000 C CNN
F 1 "20pF" V 6762 3150 50  0000 C CNN
F 2 "Capacitors_SMD:C_0805" H 6900 3150 50  0001 C CNN
F 3 "~" H 6900 3150 50  0001 C CNN
	1    6900 3150
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C6
U 1 1 5CA1F930
P 6900 2850
F 0 "C6" V 6671 2850 50  0000 C CNN
F 1 "20pF" V 6762 2850 50  0000 C CNN
F 2 "Capacitors_SMD:C_0805" H 6900 2850 50  0001 C CNN
F 3 "~" H 6900 2850 50  0001 C CNN
	1    6900 2850
	0    1    1    0   
$EndComp
Wire Wire Line
	7700 3050 7600 3050
Wire Wire Line
	7600 3050 7600 2850
Wire Wire Line
	7600 2850 7250 2850
Connection ~ 7250 2850
Wire Wire Line
	7250 2850 7000 2850
Wire Wire Line
	6800 2850 6800 3100
Wire Wire Line
	7000 3150 7250 3150
Connection ~ 7250 3150
Wire Wire Line
	7250 3150 7700 3150
$Comp
L power:VSS #PWR0111
U 1 1 5CA2260E
P 6800 3100
F 0 "#PWR0111" H 6800 2950 50  0001 C CNN
F 1 "VSS" V 6818 3228 50  0000 L CNN
F 2 "" H 6800 3100 50  0001 C CNN
F 3 "" H 6800 3100 50  0001 C CNN
	1    6800 3100
	0    -1   -1   0   
$EndComp
Connection ~ 6800 3100
Wire Wire Line
	6800 3100 6800 3150
$Comp
L power:VSS #PWR0112
U 1 1 5CA22F14
P 1050 2300
F 0 "#PWR0112" H 1050 2150 50  0001 C CNN
F 1 "VSS" V 1067 2428 50  0000 L CNN
F 2 "" H 1050 2300 50  0001 C CNN
F 3 "" H 1050 2300 50  0001 C CNN
	1    1050 2300
	0    1    1    0   
$EndComp
Text GLabel 9100 3250 2    50   Input ~ 0
SWDIO
Text GLabel 9100 3350 2    50   Input ~ 0
SWCLK
Text GLabel 9100 3950 2    50   Input ~ 0
SWO
Text GLabel 9100 4050 2    50   Input ~ 0
JRST
Text GLabel 1050 4650 2    50   Input ~ 0
SWO
Text GLabel 1050 4750 2    50   Input ~ 0
JRST
Text GLabel 1050 4450 2    50   Input ~ 0
SWDIO
Text GLabel 1050 4550 2    50   Input ~ 0
SWCLK
$Comp
L power:VSS #PWR0113
U 1 1 5CA274D8
P 1050 4850
F 0 "#PWR0113" H 1050 4700 50  0001 C CNN
F 1 "VSS" V 1067 4978 50  0000 L CNN
F 2 "" H 1050 4850 50  0001 C CNN
F 3 "" H 1050 4850 50  0001 C CNN
	1    1050 4850
	0    1    1    0   
$EndComp
$Comp
L power:VCC #PWR0114
U 1 1 5CA2751F
P 1050 4950
F 0 "#PWR0114" H 1050 4800 50  0001 C CNN
F 1 "VCC" V 1067 5078 50  0000 L CNN
F 2 "" H 1050 4950 50  0001 C CNN
F 3 "" H 1050 4950 50  0001 C CNN
	1    1050 4950
	0    1    1    0   
$EndComp
$Comp
L power:VSS #PWR0115
U 1 1 5CA2BD9C
P 1050 3850
F 0 "#PWR0115" H 1050 3700 50  0001 C CNN
F 1 "VSS" V 1067 3978 50  0000 L CNN
F 2 "" H 1050 3850 50  0001 C CNN
F 3 "" H 1050 3850 50  0001 C CNN
	1    1050 3850
	0    1    1    0   
$EndComp
$Comp
L power:VSS #PWR0116
U 1 1 5CA2BDE5
P 1050 3150
F 0 "#PWR0116" H 1050 3000 50  0001 C CNN
F 1 "VSS" V 1067 3278 50  0000 L CNN
F 2 "" H 1050 3150 50  0001 C CNN
F 3 "" H 1050 3150 50  0001 C CNN
	1    1050 3150
	0    1    1    0   
$EndComp
Text GLabel 1050 1700 2    50   Input ~ 0
Temp
Text GLabel 1050 1800 2    50   Input ~ 0
LowFuel
$Comp
L Connector_Generic:Conn_01x12 J1
U 1 1 5CA2C7D3
P 850 2000
F 0 "J1" H 770 1175 50  0000 C CNN
F 1 "FairingConnector" H 770 1266 50  0000 C CNN
F 2 "TerminalBlock_RND:TerminalBlock_RND_205-00022_Pitch5.00mm" H 850 2000 50  0001 C CNN
F 3 "~" H 850 2000 50  0001 C CNN
	1    850  2000
	-1   0    0    1   
$EndComp
Text GLabel 9100 2150 2    50   Input ~ 0
GPIO1
Text GLabel 9100 2250 2    50   Input ~ 0
GPIO2
Text GLabel 9100 2350 2    50   Input ~ 0
GPIO3
Text GLabel 9100 2450 2    50   Input ~ 0
GPIO4
Text GLabel 9100 2550 2    50   Input ~ 0
GPIO5
Text GLabel 9100 2650 2    50   Input ~ 0
GPIO6
Text GLabel 1050 5350 2    50   Input ~ 0
GPIO1
Text GLabel 1050 5450 2    50   Input ~ 0
GPIO2
Text GLabel 1050 5550 2    50   Input ~ 0
GPIO3
Text GLabel 1050 5650 2    50   Input ~ 0
GPIO4
Text GLabel 1050 5750 2    50   Input ~ 0
GPIO5
Text GLabel 1050 5850 2    50   Input ~ 0
GPIO6
$Comp
L Connector_Generic:Conn_01x06 J5
U 1 1 5CA32F8F
P 850 5650
F 0 "J5" H 770 5125 50  0000 C CNN
F 1 "GPIO" H 770 5216 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06_Pitch2.54mm" H 850 5650 50  0001 C CNN
F 3 "~" H 850 5650 50  0001 C CNN
	1    850  5650
	-1   0    0    1   
$EndComp
Text GLabel 5150 7550 2    50   Input ~ 0
SCL
Text GLabel 6250 7550 2    50   Input ~ 0
SDA
$Comp
L Device:R_Small R3
U 1 1 5CA35359
P 4950 7550
F 0 "R3" V 4754 7550 50  0000 C CNN
F 1 "10k" V 4845 7550 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" H 4950 7550 50  0001 C CNN
F 3 "~" H 4950 7550 50  0001 C CNN
	1    4950 7550
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R4
U 1 1 5CA3546B
P 6050 7550
F 0 "R4" V 6246 7550 50  0000 C CNN
F 1 "10k" V 6155 7550 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" H 6050 7550 50  0001 C CNN
F 3 "~" H 6050 7550 50  0001 C CNN
	1    6050 7550
	0    -1   -1   0   
$EndComp
$Comp
L power:VCC #PWR0117
U 1 1 5CA3557E
P 4750 7550
F 0 "#PWR0117" H 4750 7400 50  0001 C CNN
F 1 "VCC" V 4768 7677 50  0000 L CNN
F 2 "" H 4750 7550 50  0001 C CNN
F 3 "" H 4750 7550 50  0001 C CNN
	1    4750 7550
	0    -1   -1   0   
$EndComp
$Comp
L power:VCC #PWR0118
U 1 1 5CA3563A
P 5850 7550
F 0 "#PWR0118" H 5850 7400 50  0001 C CNN
F 1 "VCC" V 5868 7677 50  0000 L CNN
F 2 "" H 5850 7550 50  0001 C CNN
F 3 "" H 5850 7550 50  0001 C CNN
	1    5850 7550
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4750 7550 4850 7550
Wire Wire Line
	5050 7550 5150 7550
Wire Wire Line
	5850 7550 5950 7550
Wire Wire Line
	6150 7550 6250 7550
$Comp
L Cluster-rescue:LIS3DHTR-LIS3DHTR U1
U 1 1 5CA3D50B
P 3700 2550
F 0 "U1" H 3700 3217 50  0000 C CNN
F 1 "LIS3DHTR" H 3700 3126 50  0000 C CNN
F 2 "LIS3DHTR:LGA16R50P5X3_300X300X100N" H 3700 2550 50  0001 L BNN
F 3 "LIS3DHTR" H 3700 2550 50  0001 L BNN
F 4 "https://www.digikey.com/product-detail/en/stmicroelectronics/LIS3DHTR/497-10613-1-ND/2334355?utm_source=snapeda&utm_medium=aggregator&utm_campaign=symbol" H 3700 2550 50  0001 L BNN "Field4"
F 5 "MEMS digital output motion sensor ultra low-power high performance 3-axes nano accelerometer" H 3700 2550 50  0001 L BNN "Field5"
F 6 "STMicroelectronics" H 3700 2550 50  0001 L BNN "Field6"
F 7 "LGA-16 STMicroelectronics" H 3700 2550 50  0001 L BNN "Field7"
F 8 "497-10613-1-ND" H 3700 2550 50  0001 L BNN "Field8"
	1    3700 2550
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C3
U 1 1 5CA3D7C2
P 5050 2250
F 0 "C3" H 5142 2296 50  0000 L CNN
F 1 "100nF" H 5142 2205 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 5050 2250 50  0001 C CNN
F 3 "~" H 5050 2250 50  0001 C CNN
	1    5050 2250
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C2
U 1 1 5CA3D83A
P 4700 2250
F 0 "C2" H 4792 2296 50  0000 L CNN
F 1 "10uF" H 4792 2205 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 4700 2250 50  0001 C CNN
F 3 "~" H 4700 2250 50  0001 C CNN
	1    4700 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 2150 4700 2150
Connection ~ 4700 2150
Wire Wire Line
	4700 2150 4450 2150
$Comp
L power:VSS #PWR0119
U 1 1 5CA41665
P 4950 2350
F 0 "#PWR0119" H 4950 2200 50  0001 C CNN
F 1 "VSS" H 4968 2523 50  0000 C CNN
F 2 "" H 4950 2350 50  0001 C CNN
F 3 "" H 4950 2350 50  0001 C CNN
	1    4950 2350
	-1   0    0    1   
$EndComp
Connection ~ 4950 2350
Wire Wire Line
	4950 2350 5050 2350
$Comp
L power:VSS #PWR0120
U 1 1 5CA4170D
P 4300 3050
F 0 "#PWR0120" H 4300 2900 50  0001 C CNN
F 1 "VSS" H 4318 3223 50  0000 C CNN
F 2 "" H 4300 3050 50  0001 C CNN
F 3 "" H 4300 3050 50  0001 C CNN
	1    4300 3050
	-1   0    0    1   
$EndComp
Text GLabel 3100 2750 0    50   Input ~ 0
SCL
Text GLabel 3100 2950 0    50   Input ~ 0
SDA
$Comp
L power:VCC #PWR0121
U 1 1 5CA43336
P 5050 2150
F 0 "#PWR0121" H 5050 2000 50  0001 C CNN
F 1 "VCC" H 5067 2323 50  0000 C CNN
F 2 "" H 5050 2150 50  0001 C CNN
F 3 "" H 5050 2150 50  0001 C CNN
	1    5050 2150
	1    0    0    -1  
$EndComp
Connection ~ 5050 2150
$Comp
L power:VCC #PWR0122
U 1 1 5CA43392
P 3100 2150
F 0 "#PWR0122" H 3100 2000 50  0001 C CNN
F 1 "VCC" H 3117 2323 50  0000 C CNN
F 2 "" H 3100 2150 50  0001 C CNN
F 3 "" H 3100 2150 50  0001 C CNN
	1    3100 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 2250 4450 2250
Wire Wire Line
	4450 2250 4450 2150
Connection ~ 4450 2150
Wire Wire Line
	4450 2150 4300 2150
Wire Wire Line
	4300 2350 4700 2350
Connection ~ 4700 2350
Wire Wire Line
	4700 2350 4950 2350
Text GLabel 9100 3750 2    50   Input ~ 0
LightsOutFront
Text GLabel 9100 3650 2    50   Input ~ 0
LightsOutBack
$Comp
L power:VSS #PWR0123
U 1 1 5CA4FD7A
P 1050 6600
F 0 "#PWR0123" H 1050 6450 50  0001 C CNN
F 1 "VSS" V 1067 6728 50  0000 L CNN
F 2 "" H 1050 6600 50  0001 C CNN
F 3 "" H 1050 6600 50  0001 C CNN
	1    1050 6600
	0    1    1    0   
$EndComp
$Comp
L power:VCC #PWR0124
U 1 1 5CA52CF6
P 1050 6700
F 0 "#PWR0124" H 1050 6550 50  0001 C CNN
F 1 "VCC" V 1067 6828 50  0000 L CNN
F 2 "" H 1050 6700 50  0001 C CNN
F 3 "" H 1050 6700 50  0001 C CNN
	1    1050 6700
	0    1    1    0   
$EndComp
$Comp
L power:+12V #PWR0125
U 1 1 5CA52DAF
P 1050 6800
F 0 "#PWR0125" H 1050 6650 50  0001 C CNN
F 1 "+12V" V 1065 6928 50  0000 L CNN
F 2 "" H 1050 6800 50  0001 C CNN
F 3 "" H 1050 6800 50  0001 C CNN
	1    1050 6800
	0    1    1    0   
$EndComp
Text GLabel 7700 3750 0    50   Input ~ 0
MISO
Text GLabel 7700 3850 0    50   Input ~ 0
MOSI
Text GLabel 9100 4650 2    50   Input ~ 0
SCK
Text GLabel 2100 6650 2    50   Input ~ 0
SCK
Text GLabel 2100 6250 2    50   Input ~ 0
MISO
Text GLabel 2100 6350 2    50   Input ~ 0
MOSI
$Comp
L Connector_Generic:Conn_01x40 J7
U 1 1 5CA57C7F
P 1900 5650
F 0 "J7" H 1700 3400 50  0000 C CNN
F 1 "Conn_01x40" H 1800 3300 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x40_Pitch1.00mm" H 1900 5650 50  0001 C CNN
F 3 "~" H 1900 5650 50  0001 C CNN
	1    1900 5650
	-1   0    0    1   
$EndComp
$Comp
L power:VSS #PWR0126
U 1 1 5CA57E42
P 2100 7150
F 0 "#PWR0126" H 2100 7000 50  0001 C CNN
F 1 "VSS" V 2117 7278 50  0000 L CNN
F 2 "" H 2100 7150 50  0001 C CNN
F 3 "" H 2100 7150 50  0001 C CNN
	1    2100 7150
	0    1    1    0   
$EndComp
$Comp
L power:VCC #PWR0127
U 1 1 5CA57F75
P 2100 7050
F 0 "#PWR0127" H 2100 6900 50  0001 C CNN
F 1 "VCC" V 2117 7178 50  0000 L CNN
F 2 "" H 2100 7050 50  0001 C CNN
F 3 "" H 2100 7050 50  0001 C CNN
	1    2100 7050
	0    1    1    0   
$EndComp
$Comp
L power:VSS #PWR0128
U 1 1 5CA5802B
P 2100 6750
F 0 "#PWR0128" H 2100 6600 50  0001 C CNN
F 1 "VSS" V 2117 6878 50  0000 L CNN
F 2 "" H 2100 6750 50  0001 C CNN
F 3 "" H 2100 6750 50  0001 C CNN
	1    2100 6750
	0    1    1    0   
$EndComp
$Comp
L power:VCC #PWR0129
U 1 1 5CA5823A
P 2100 7050
F 0 "#PWR0129" H 2100 6900 50  0001 C CNN
F 1 "VCC" V 2117 7178 50  0000 L CNN
F 2 "" H 2100 7050 50  0001 C CNN
F 3 "" H 2100 7050 50  0001 C CNN
	1    2100 7050
	0    1    1    0   
$EndComp
Connection ~ 2100 7050
$Comp
L power:VCC #PWR0130
U 1 1 5CA58353
P 2100 4350
F 0 "#PWR0130" H 2100 4200 50  0001 C CNN
F 1 "VCC" V 2117 4478 50  0000 L CNN
F 2 "" H 2100 4350 50  0001 C CNN
F 3 "" H 2100 4350 50  0001 C CNN
	1    2100 4350
	0    1    1    0   
$EndComp
Wire Wire Line
	2100 4050 2150 4050
Connection ~ 2100 4050
$Comp
L power:VSS #PWR0131
U 1 1 5CA59CD7
P 2150 4050
F 0 "#PWR0131" H 2150 3900 50  0001 C CNN
F 1 "VSS" V 2167 4178 50  0000 L CNN
F 2 "" H 2150 4050 50  0001 C CNN
F 3 "" H 2150 4050 50  0001 C CNN
	1    2150 4050
	0    1    1    0   
$EndComp
$Comp
L power:VSS #PWR0132
U 1 1 5CA59FDC
P 2100 6050
F 0 "#PWR0132" H 2100 5900 50  0001 C CNN
F 1 "VSS" V 2117 6178 50  0000 L CNN
F 2 "" H 2100 6050 50  0001 C CNN
F 3 "" H 2100 6050 50  0001 C CNN
	1    2100 6050
	0    1    1    0   
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J9
U 1 1 5CA5A15F
P 2300 3750
F 0 "J9" H 2380 3792 50  0000 L CNN
F 1 "Conn_01x03" H 2380 3701 50  0000 L CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 2300 3750 50  0001 C CNN
F 3 "~" H 2300 3750 50  0001 C CNN
	1    2300 3750
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x40 J8
U 1 1 5CA5A259
P 1900 5650
F 0 "J8" H 1820 3425 50  0000 C CNN
F 1 "Conn_01x40" H 1820 3516 50  0000 C CNN
F 2 "Connectors:XF2M" H 1900 5650 50  0001 C CNN
F 3 "~" H 1900 5650 50  0001 C CNN
	1    1900 5650
	-1   0    0    1   
$EndComp
Connection ~ 2100 6050
Connection ~ 2100 4350
Connection ~ 2100 3850
Connection ~ 2100 3750
Connection ~ 2100 3650
Connection ~ 2100 7150
Connection ~ 2100 6750
NoConn ~ 7700 4550
NoConn ~ 7700 4650
NoConn ~ 7700 4750
NoConn ~ 7700 4850
NoConn ~ 7700 4950
NoConn ~ 7700 5050
NoConn ~ 9100 4550
NoConn ~ 9100 4450
NoConn ~ 9100 4150
NoConn ~ 9100 3450
NoConn ~ 9100 3050
NoConn ~ 9100 3150
NoConn ~ 7700 3350
$Comp
L Connector_Generic:Conn_01x03 J12
U 1 1 5CA78B24
P 4500 2650
F 0 "J12" H 4580 2692 50  0000 L CNN
F 1 "Conn_01x03" H 4580 2601 50  0000 L CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 4500 2650 50  0001 C CNN
F 3 "~" H 4500 2650 50  0001 C CNN
	1    4500 2650
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J11
U 1 1 5CA78CAF
P 2900 2550
F 0 "J11" H 2820 2225 50  0000 C CNN
F 1 "Conn_01x02" H 2820 2316 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 2900 2550 50  0001 C CNN
F 3 "~" H 2900 2550 50  0001 C CNN
	1    2900 2550
	-1   0    0    1   
$EndComp
NoConn ~ 3100 2850
Text GLabel 9100 4750 2    50   Input ~ 0
OverDrive
Text GLabel 1050 4350 2    50   Input ~ 0
Reset
$Comp
L Connector_Generic:Conn_01x07 J4
U 1 1 5CA7AD41
P 850 4650
F 0 "J4" H 770 4125 50  0000 C CNN
F 1 "Programming" H 770 4216 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x07_Pitch2.54mm" H 850 4650 50  0001 C CNN
F 3 "~" H 850 4650 50  0001 C CNN
	1    850  4650
	-1   0    0    1   
$EndComp
Text GLabel 7700 4050 0    50   Input ~ 0
LightsOutAux
Text GLabel 7700 3950 0    50   Input ~ 0
LightsOutAuxTwo
Text GLabel 1050 6200 2    50   Input ~ 0
LightsOutFront
Text GLabel 1050 6300 2    50   Input ~ 0
LightsOutBack
Text GLabel 1050 6400 2    50   Input ~ 0
LightsOutAux
Text GLabel 1050 6500 2    50   Input ~ 0
LightsOutAuxTwo
$Comp
L Connector_Generic:Conn_01x07 J6
U 1 1 5CA37061
P 850 6500
F 0 "J6" H 768 5975 50  0000 C CNN
F 1 "Conn_01x07" H 768 6066 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x07_Pitch2.54mm" H 850 6500 50  0001 C CNN
F 3 "~" H 850 6500 50  0001 C CNN
	1    850  6500
	-1   0    0    1   
$EndComp
$Comp
L Diode:1N5822 D1
U 1 1 5CA1159F
P 5150 6650
F 0 "D1" V 5104 6729 50  0000 L CNN
F 1 "1N5822" V 5195 6729 50  0000 L CNN
F 2 "Diodes_SMD:D_SMA" H 5150 6475 50  0001 C CNN
F 3 "http://www.vishay.com/docs/88526/1n5820.pdf" H 5150 6650 50  0001 C CNN
	1    5150 6650
	0    1    1    0   
$EndComp
NoConn ~ 9100 3850
NoConn ~ 7700 3550
NoConn ~ 7700 3650
$Comp
L Cluster:AiP8563 U4
U 1 1 5CA417E5
P 4600 4200
F 0 "U4" H 4600 4565 50  0000 C CNN
F 1 "AiP8563" H 4600 4474 50  0000 C CNN
F 2 "" H 4000 3700 50  0001 C CNN
F 3 "" H 4000 3700 50  0001 C CNN
	1    4600 4200
	1    0    0    -1  
$EndComp
$Comp
L Device:Crystal_Small Y2
U 1 1 5CA5BD2C
P 3800 4100
F 0 "Y2" V 3754 4188 50  0000 L CNN
F 1 " 32.768kHz" V 3950 3900 50  0000 L CNN
F 2 "" H 3800 4100 50  0001 C CNN
F 3 "~" H 3800 4100 50  0001 C CNN
	1    3800 4100
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C11
U 1 1 5CA5CE75
P 3600 4000
F 0 "C11" V 3371 4000 50  0000 C CNN
F 1 "20pF" V 3462 4000 50  0000 C CNN
F 2 "Capacitors_SMD:C_0805" H 3600 4000 50  0001 C CNN
F 3 "~" H 3600 4000 50  0001 C CNN
	1    3600 4000
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C12
U 1 1 5CA5D4DA
P 3600 4200
F 0 "C12" V 3371 4200 50  0000 C CNN
F 1 "20pF" V 3462 4200 50  0000 C CNN
F 2 "Capacitors_SMD:C_0805" H 3600 4200 50  0001 C CNN
F 3 "~" H 3600 4200 50  0001 C CNN
	1    3600 4200
	0    1    1    0   
$EndComp
Wire Wire Line
	3800 4000 4000 4000
Wire Wire Line
	4000 4000 4000 4100
Wire Wire Line
	4000 4100 4050 4100
Wire Wire Line
	3700 4000 3800 4000
Connection ~ 3800 4000
Wire Wire Line
	3700 4200 3800 4200
Connection ~ 3800 4200
Wire Wire Line
	3800 4200 4050 4200
Wire Wire Line
	3500 4000 3450 4000
Wire Wire Line
	3450 4000 3450 4200
Wire Wire Line
	3450 4200 3500 4200
$Comp
L power:VSS #PWR01
U 1 1 5CA71B7C
P 3450 4200
F 0 "#PWR01" H 3450 4050 50  0001 C CNN
F 1 "VSS" V 3468 4328 50  0000 L CNN
F 2 "" H 3450 4200 50  0001 C CNN
F 3 "" H 3450 4200 50  0001 C CNN
	1    3450 4200
	0    -1   -1   0   
$EndComp
Connection ~ 3450 4200
$Comp
L Device:C_Small C14
U 1 1 5CA72D9A
P 5250 4000
F 0 "C14" H 5342 4046 50  0000 L CNN
F 1 "10uF" H 5342 3955 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 5250 4000 50  0001 C CNN
F 3 "~" H 5250 4000 50  0001 C CNN
	1    5250 4000
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C16
U 1 1 5CA7327D
P 5600 4000
F 0 "C16" H 5692 4046 50  0000 L CNN
F 1 "0.1uF" H 5692 3955 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 5600 4000 50  0001 C CNN
F 3 "~" H 5600 4000 50  0001 C CNN
	1    5600 4000
	1    0    0    -1  
$EndComp
Text GLabel 5150 4300 2    50   Input ~ 0
SCL
Text GLabel 5150 4400 2    50   Input ~ 0
SDA
$Comp
L power:VSS #PWR05
U 1 1 5CA75677
P 4050 4400
F 0 "#PWR05" H 4050 4250 50  0001 C CNN
F 1 "VSS" H 4068 4573 50  0000 C CNN
F 2 "" H 4050 4400 50  0001 C CNN
F 3 "" H 4050 4400 50  0001 C CNN
	1    4050 4400
	-1   0    0    1   
$EndComp
NoConn ~ 4050 4300
$Comp
L Device:D_ALT D3
U 1 1 5CA7D920
P 6100 4100
F 0 "D3" H 6100 4316 50  0000 C CNN
F 1 "D_ALT" H 6100 4225 50  0000 C CNN
F 2 "" H 6100 4100 50  0001 C CNN
F 3 "~" H 6100 4100 50  0001 C CNN
	1    6100 4100
	1    0    0    -1  
$EndComp
$Comp
L Device:D_ALT D2
U 1 1 5CA7DD6A
P 5850 4300
F 0 "D2" V 5804 4379 50  0000 L CNN
F 1 "D_ALT" V 5895 4379 50  0000 L CNN
F 2 "" H 5850 4300 50  0001 C CNN
F 3 "~" H 5850 4300 50  0001 C CNN
	1    5850 4300
	0    1    1    0   
$EndComp
Wire Wire Line
	6000 4100 5950 4100
Connection ~ 5250 4100
Wire Wire Line
	5250 4100 5150 4100
Connection ~ 5600 4100
Wire Wire Line
	5600 4100 5250 4100
Connection ~ 5950 4100
Wire Wire Line
	5950 4100 5850 4100
Wire Wire Line
	5850 4150 5850 4100
Connection ~ 5850 4100
Wire Wire Line
	5850 4100 5600 4100
$Comp
L Device:Battery_Cell BT1
U 1 1 5CA8509D
P 5850 4750
F 0 "BT1" H 5968 4846 50  0000 L CNN
F 1 "Battery_Cell" H 5968 4755 50  0000 L CNN
F 2 "" V 5850 4810 50  0001 C CNN
F 3 "~" V 5850 4810 50  0001 C CNN
	1    5850 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	5850 4550 5850 4500
$Comp
L power:VSS #PWR010
U 1 1 5CA891E1
P 5850 4900
F 0 "#PWR010" H 5850 4750 50  0001 C CNN
F 1 "VSS" H 5868 5073 50  0000 C CNN
F 2 "" H 5850 4900 50  0001 C CNN
F 3 "" H 5850 4900 50  0001 C CNN
	1    5850 4900
	-1   0    0    1   
$EndComp
Wire Wire Line
	5850 4900 5850 4850
$Comp
L power:VCC #PWR011
U 1 1 5CA8CF43
P 6300 4100
F 0 "#PWR011" H 6300 3950 50  0001 C CNN
F 1 "VCC" V 6317 4228 50  0000 L CNN
F 2 "" H 6300 4100 50  0001 C CNN
F 3 "" H 6300 4100 50  0001 C CNN
	1    6300 4100
	0    1    1    0   
$EndComp
Wire Wire Line
	6300 4100 6250 4100
$Comp
L badge-cache:badge-rescue_HX4056 U5
U 1 1 5CA91910
P 4700 4850
F 0 "U5" H 4650 5175 50  0000 C CNN
F 1 "badge-rescue_HX4056" H 4650 5084 50  0000 C CNN
F 2 "" H 4600 4850 50  0001 C CNN
F 3 "" H 4600 4850 50  0001 C CNN
	1    4700 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 4750 5450 4750
Wire Wire Line
	5450 4750 5450 4500
Wire Wire Line
	5450 4500 5550 4500
Connection ~ 5850 4500
Wire Wire Line
	5850 4500 5850 4450
$Comp
L Device:R_Small R7
U 1 1 5CA9B6E5
P 5250 4850
F 0 "R7" V 5054 4850 50  0000 C CNN
F 1 "1k" V 5145 4850 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" H 5250 4850 50  0001 C CNN
F 3 "~" H 5250 4850 50  0001 C CNN
	1    5250 4850
	0    1    1    0   
$EndComp
Wire Wire Line
	5100 4850 5150 4850
$Comp
L power:VSS #PWR08
U 1 1 5CA9F6CB
P 5450 4850
F 0 "#PWR08" H 5450 4700 50  0001 C CNN
F 1 "VSS" H 5468 5023 50  0000 C CNN
F 2 "" H 5450 4850 50  0001 C CNN
F 3 "" H 5450 4850 50  0001 C CNN
	1    5450 4850
	-1   0    0    1   
$EndComp
Wire Wire Line
	5450 4850 5350 4850
$Comp
L power:VSS #PWR07
U 1 1 5CAA3577
P 5150 5050
F 0 "#PWR07" H 5150 4900 50  0001 C CNN
F 1 "VSS" H 5168 5223 50  0000 C CNN
F 2 "" H 5150 5050 50  0001 C CNN
F 3 "" H 5150 5050 50  0001 C CNN
	1    5150 5050
	-1   0    0    1   
$EndComp
Wire Wire Line
	5150 5050 5100 5050
$Comp
L power:VSS #PWR06
U 1 1 5CAA6F8D
P 4150 5050
F 0 "#PWR06" H 4150 4900 50  0001 C CNN
F 1 "VSS" H 4168 5223 50  0000 C CNN
F 2 "" H 4150 5050 50  0001 C CNN
F 3 "" H 4150 5050 50  0001 C CNN
	1    4150 5050
	-1   0    0    1   
$EndComp
Wire Wire Line
	4150 5050 4200 5050
$Comp
L Device:C_Small C13
U 1 1 5CAAB789
P 3950 4850
F 0 "C13" H 3858 4804 50  0000 R CNN
F 1 "10uF" H 3858 4895 50  0000 R CNN
F 2 "Capacitors_SMD:C_0805" H 3950 4850 50  0001 C CNN
F 3 "~" H 3950 4850 50  0001 C CNN
	1    3950 4850
	-1   0    0    1   
$EndComp
$Comp
L Device:C_Small C15
U 1 1 5CAABC70
P 5550 4700
F 0 "C15" H 5458 4654 50  0000 R CNN
F 1 "10uF" H 5458 4745 50  0000 R CNN
F 2 "Capacitors_SMD:C_0805" H 5550 4700 50  0001 C CNN
F 3 "~" H 5550 4700 50  0001 C CNN
	1    5550 4700
	-1   0    0    1   
$EndComp
Wire Wire Line
	5550 4600 5550 4500
Connection ~ 5550 4500
Wire Wire Line
	5550 4500 5850 4500
$Comp
L power:VSS #PWR09
U 1 1 5CAAFD31
P 5550 4850
F 0 "#PWR09" H 5550 4700 50  0001 C CNN
F 1 "VSS" H 5568 5023 50  0000 C CNN
F 2 "" H 5550 4850 50  0001 C CNN
F 3 "" H 5550 4850 50  0001 C CNN
	1    5550 4850
	-1   0    0    1   
$EndComp
Wire Wire Line
	5550 4850 5550 4800
$Comp
L power:VSS #PWR04
U 1 1 5CAB3B7F
P 3950 5000
F 0 "#PWR04" H 3950 4850 50  0001 C CNN
F 1 "VSS" H 3968 5173 50  0000 C CNN
F 2 "" H 3950 5000 50  0001 C CNN
F 3 "" H 3950 5000 50  0001 C CNN
	1    3950 5000
	-1   0    0    1   
$EndComp
Wire Wire Line
	3950 5000 3950 4950
Connection ~ 3950 4750
Wire Wire Line
	3950 4750 4200 4750
$Comp
L power:VCC #PWR02
U 1 1 5CABBFAA
P 3800 4750
F 0 "#PWR02" H 3800 4600 50  0001 C CNN
F 1 "VCC" V 3818 4877 50  0000 L CNN
F 2 "" H 3800 4750 50  0001 C CNN
F 3 "" H 3800 4750 50  0001 C CNN
	1    3800 4750
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3800 4750 3950 4750
NoConn ~ 4200 4850
NoConn ~ 4200 4950
$Comp
L Connector_Generic:Conn_01x01 J13
U 1 1 5CAC98B1
P 5350 4200
F 0 "J13" H 5430 4242 50  0000 L CNN
F 1 "ClkOUT" H 5430 4151 50  0000 L CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01_Pitch2.54mm" H 5350 4200 50  0001 C CNN
F 3 "~" H 5350 4200 50  0001 C CNN
	1    5350 4200
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J14
U 1 1 5CACCAA1
P 2050 1450
F 0 "J14" H 1968 1125 50  0000 C CNN
F 1 "Conn_01x02" H 1968 1216 50  0000 C CNN
F 2 "" H 2050 1450 50  0001 C CNN
F 3 "~" H 2050 1450 50  0001 C CNN
	1    2050 1450
	-1   0    0    1   
$EndComp
Text GLabel 2250 1350 2    50   Input ~ 0
LeftIN
Text GLabel 2250 1450 2    50   Input ~ 0
RightIN
Text GLabel 7700 4450 0    50   Input ~ 0
LeftIN
Text GLabel 9100 2750 2    50   Input ~ 0
RightIN
$EndSCHEMATC
