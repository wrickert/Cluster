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
L Cluster-rescue:+12V-power #PWR03
U 1 1 5BF6B374
P 2250 1950
F 0 "#PWR03" H 2250 1800 50  0001 C CNN
F 1 "+12V" V 2265 2078 50  0000 L CNN
F 2 "" H 2250 1950 50  0001 C CNN
F 3 "" H 2250 1950 50  0001 C CNN
	1    2250 1950
	0    1    1    0   
$EndComp
$Comp
L Cluster-rescue:Conn_01x03-Connector_Generic J2
U 1 1 5BF76E42
P 850 3050
F 0 "J2" H 770 2725 50  0000 C CNN
F 1 "Serial" H 770 2816 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 850 3050 50  0001 C CNN
F 3 "~" H 850 3050 50  0001 C CNN
	1    850  3050
	-1   0    0    1   
$EndComp
Text GLabel 1050 2950 2    50   Input ~ 0
TX
Text GLabel 1050 3050 2    50   Input ~ 0
RX
$Comp
L Cluster-rescue:STM32F411RETx-MCU_ST_STM32F4 U5
U 1 1 5CA0E699
P 8950 3050
F 0 "U5" H 8400 1250 50  0000 C CNN
F 1 "STM32F411RETx" H 9500 1250 50  0000 C CNN
F 2 "Housings_QFP:LQFP-64_10x10mm_Pitch0.5mm" H 8350 1350 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00115249.pdf" H 8950 3050 50  0001 C CNN
	1    8950 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	9150 1350 9050 1350
Wire Wire Line
	8750 1350 8750 1300
Connection ~ 8750 1350
Connection ~ 8850 1350
Wire Wire Line
	8850 1350 8750 1350
Connection ~ 8950 1350
Wire Wire Line
	8950 1350 8850 1350
Connection ~ 9050 1350
Wire Wire Line
	9050 1350 8950 1350
Wire Wire Line
	9150 4850 9050 4850
Wire Wire Line
	8750 4850 8750 4900
Connection ~ 8750 4850
Connection ~ 8850 4850
Wire Wire Line
	8850 4850 8750 4850
Connection ~ 8950 4850
Wire Wire Line
	8950 4850 8850 4850
Connection ~ 9050 4850
Wire Wire Line
	9050 4850 8950 4850
$Comp
L Cluster-rescue:VSS-power #PWR068
U 1 1 5CA0FF4A
P 8750 4900
F 0 "#PWR068" H 8750 4750 50  0001 C CNN
F 1 "VSS" H 8768 5073 50  0000 C CNN
F 2 "" H 8750 4900 50  0001 C CNN
F 3 "" H 8750 4900 50  0001 C CNN
	1    8750 4900
	-1   0    0    1   
$EndComp
Text GLabel 9650 3850 2    50   Input ~ 0
SCL
Text GLabel 9650 3950 2    50   Input ~ 0
SDA
Text GLabel 8250 3950 0    50   Input ~ 0
RightTurn
Text GLabel 8250 3850 0    50   Input ~ 0
LeftTurn
Text GLabel 8250 3750 0    50   Input ~ 0
Oil
Text GLabel 9650 4650 2    50   Input ~ 0
HighBeam
Text GLabel 9650 4550 2    50   Input ~ 0
Neutral
Text GLabel 9650 4450 2    50   Input ~ 0
LowFuel
Text GLabel 9650 1550 2    50   Input ~ 0
Fuel
Text GLabel 9650 1650 2    50   Input ~ 0
Temp
Text GLabel 8050 1550 1    50   Input ~ 0
Reset
$Comp
L Cluster-rescue:VCC-power #PWR067
U 1 1 5CA10461
P 8750 1300
F 0 "#PWR067" H 8750 1150 50  0001 C CNN
F 1 "VCC" H 8767 1473 50  0000 C CNN
F 2 "" H 8750 1300 50  0001 C CNN
F 3 "" H 8750 1300 50  0001 C CNN
	1    8750 1300
	1    0    0    -1  
$EndComp
Text GLabel 9650 2550 2    50   Input ~ 0
RX
Text GLabel 9650 2450 2    50   Input ~ 0
TX
$Comp
L Cluster-rescue:LM2596S-ADJ-Regulator_Switching U2
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
L Cluster-rescue:VCC-power #PWR02
U 1 1 5CA109EF
P 1050 2400
F 0 "#PWR02" H 1050 2250 50  0001 C CNN
F 1 "VCC" V 1067 2528 50  0000 L CNN
F 2 "" H 1050 2400 50  0001 C CNN
F 3 "" H 1050 2400 50  0001 C CNN
	1    1050 2400
	0    1    1    0   
$EndComp
$Comp
L Cluster-rescue:CP1-Device C1
U 1 1 5CA10B7D
P 3500 6450
F 0 "C1" H 3615 6496 50  0000 L CNN
F 1 "100uF" H 3615 6405 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_8x10.5" H 3500 6450 50  0001 C CNN
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
L Cluster-rescue:+12V-power #PWR028
U 1 1 5CA10D2D
P 3350 6300
F 0 "#PWR028" H 3350 6150 50  0001 C CNN
F 1 "+12V" V 3365 6428 50  0000 L CNN
F 2 "" H 3350 6300 50  0001 C CNN
F 3 "" H 3350 6300 50  0001 C CNN
	1    3350 6300
	0    -1   -1   0   
$EndComp
$Comp
L Cluster-rescue:VSS-power #PWR026
U 1 1 5CA10FFE
P 3300 6800
F 0 "#PWR026" H 3300 6650 50  0001 C CNN
F 1 "VSS" V 3318 6928 50  0000 L CNN
F 2 "" H 3300 6800 50  0001 C CNN
F 3 "" H 3300 6800 50  0001 C CNN
	1    3300 6800
	0    -1   -1   0   
$EndComp
$Comp
L Cluster-rescue:Conn_01x01-Connector_Generic J12
U 1 1 5CA113CA
P 4150 7000
F 0 "J12" H 4229 7042 50  0000 L CNN
F 1 "PowerReg" H 4229 6951 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 4150 7000 50  0001 C CNN
F 3 "~" H 4150 7000 50  0001 C CNN
	1    4150 7000
	1    0    0    -1  
$EndComp
$Comp
L Cluster-rescue:L_Small-Device L1
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
L Cluster-rescue:CP1-Device C18
U 1 1 5CA11A8E
P 5700 6650
F 0 "C18" H 5815 6696 50  0000 L CNN
F 1 "220uF" H 5815 6605 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_8x10.5" H 5700 6650 50  0001 C CNN
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
L Cluster-rescue:R_Small-Device R4
U 1 1 5CA12D78
P 5600 6400
F 0 "R4" H 5659 6446 50  0000 L CNN
F 1 "3.5k" H 5659 6355 50  0000 L CNN
F 2 "Resistors_SMD:R_0805" H 5600 6400 50  0001 C CNN
F 3 "~" H 5600 6400 50  0001 C CNN
	1    5600 6400
	1    0    0    -1  
$EndComp
$Comp
L Cluster-rescue:R_Small-Device R5
U 1 1 5CA12E1A
P 6050 6300
F 0 "R5" V 5854 6300 50  0000 C CNN
F 1 "2k" V 5945 6300 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" H 6050 6300 50  0001 C CNN
F 3 "~" H 6050 6300 50  0001 C CNN
	1    6050 6300
	0    1    1    0   
$EndComp
$Comp
L Cluster-rescue:C_Small-Device C19
U 1 1 5CA12F62
P 5850 6400
F 0 "C19" H 5942 6446 50  0000 L CNN
F 1 "15nF" H 5942 6355 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5850 6400 50  0001 C CNN
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
	5850 6500 6400 6500
Wire Wire Line
	6150 6300 6250 6300
Wire Wire Line
	6250 6300 6250 6800
Wire Wire Line
	5700 6800 6250 6800
$Comp
L Cluster-rescue:VSS-power #PWR046
U 1 1 5CA1436E
P 5450 6800
F 0 "#PWR046" H 5450 6650 50  0001 C CNN
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
L Cluster-rescue:VCC-power #PWR057
U 1 1 5CA14453
P 6550 6500
F 0 "#PWR057" H 6550 6350 50  0001 C CNN
F 1 "VCC" V 6567 6628 50  0000 L CNN
F 2 "" H 6550 6500 50  0001 C CNN
F 3 "" H 6550 6500 50  0001 C CNN
	1    6550 6500
	0    1    1    0   
$EndComp
$Comp
L Cluster-rescue:Conn_01x03-Connector_Generic J3
U 1 1 5CA14813
P 850 3750
F 0 "J3" H 770 3425 50  0000 C CNN
F 1 "i2c" H 770 3516 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 850 3750 50  0001 C CNN
F 3 "~" H 850 3750 50  0001 C CNN
	1    850  3750
	-1   0    0    1   
$EndComp
Text GLabel 1050 3650 2    50   Input ~ 0
SCL
Text GLabel 1050 3750 2    50   Input ~ 0
SDA
$Comp
L Cluster-rescue:VSS-power #PWR063
U 1 1 5CA18C96
P 7500 1750
F 0 "#PWR063" H 7500 1600 50  0001 C CNN
F 1 "VSS" V 7518 1878 50  0000 L CNN
F 2 "" H 7500 1750 50  0001 C CNN
F 3 "" H 7500 1750 50  0001 C CNN
	1    7500 1750
	0    -1   -1   0   
$EndComp
$Comp
L Cluster-rescue:C_Small-Device C25
U 1 1 5CA1972A
P 7700 1550
F 0 "C25" V 7471 1550 50  0000 C CNN
F 1 "100nF" V 7562 1550 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 7700 1550 50  0001 C CNN
F 3 "~" H 7700 1550 50  0001 C CNN
	1    7700 1550
	0    1    1    0   
$EndComp
$Comp
L Cluster-rescue:C_Small-Device C26
U 1 1 5CA1ABF0
P 7700 1950
F 0 "C26" V 7471 1950 50  0000 C CNN
F 1 "4.7uF" V 7562 1950 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 7700 1950 50  0001 C CNN
F 3 "~" H 7700 1950 50  0001 C CNN
	1    7700 1950
	0    1    1    0   
$EndComp
Wire Wire Line
	7600 1550 7600 1750
Wire Wire Line
	7500 1750 7600 1750
Connection ~ 7600 1750
Wire Wire Line
	7600 1750 7600 1950
Wire Wire Line
	9250 1300 9250 1350
Wire Wire Line
	7800 1950 8250 1950
$Comp
L Cluster-rescue:Crystal-Device Y2
U 1 1 5CA1F74F
P 7800 2600
F 0 "Y2" V 7754 2731 50  0000 L CNN
F 1 "8MHz" V 7845 2731 50  0000 L CNN
F 2 "Crystals:Crystal_SMD_5032-2pin_5.0x3.2mm" H 7800 2600 50  0001 C CNN
F 3 "~" H 7800 2600 50  0001 C CNN
	1    7800 2600
	0    1    1    0   
$EndComp
$Comp
L Cluster-rescue:C_Small-Device C22
U 1 1 5CA1F826
P 7450 2750
F 0 "C22" V 7221 2750 50  0000 C CNN
F 1 "20pF" V 7312 2750 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 7450 2750 50  0001 C CNN
F 3 "~" H 7450 2750 50  0001 C CNN
	1    7450 2750
	0    1    1    0   
$EndComp
$Comp
L Cluster-rescue:C_Small-Device C21
U 1 1 5CA1F930
P 7450 2450
F 0 "C21" V 7221 2450 50  0000 C CNN
F 1 "20pF" V 7312 2450 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 7450 2450 50  0001 C CNN
F 3 "~" H 7450 2450 50  0001 C CNN
	1    7450 2450
	0    1    1    0   
$EndComp
Wire Wire Line
	8250 2650 8150 2650
Wire Wire Line
	8150 2650 8150 2450
Wire Wire Line
	8150 2450 7800 2450
Connection ~ 7800 2450
Wire Wire Line
	7800 2450 7550 2450
Wire Wire Line
	7350 2450 7350 2700
Wire Wire Line
	7550 2750 7800 2750
Connection ~ 7800 2750
Wire Wire Line
	7800 2750 8250 2750
$Comp
L Cluster-rescue:VSS-power #PWR061
U 1 1 5CA2260E
P 7350 2700
F 0 "#PWR061" H 7350 2550 50  0001 C CNN
F 1 "VSS" V 7368 2828 50  0000 L CNN
F 2 "" H 7350 2700 50  0001 C CNN
F 3 "" H 7350 2700 50  0001 C CNN
	1    7350 2700
	0    -1   -1   0   
$EndComp
Connection ~ 7350 2700
Wire Wire Line
	7350 2700 7350 2750
$Comp
L Cluster-rescue:VSS-power #PWR01
U 1 1 5CA22F14
P 1050 2300
F 0 "#PWR01" H 1050 2150 50  0001 C CNN
F 1 "VSS" V 1067 2428 50  0000 L CNN
F 2 "" H 1050 2300 50  0001 C CNN
F 3 "" H 1050 2300 50  0001 C CNN
	1    1050 2300
	0    1    1    0   
$EndComp
Text GLabel 9650 2850 2    50   Input ~ 0
SWDIO
Text GLabel 9650 2950 2    50   Input ~ 0
SWCLK
Text GLabel 9650 3550 2    50   Input ~ 0
SWO
Text GLabel 9650 3650 2    50   Input ~ 0
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
L Cluster-rescue:VSS-power #PWR06
U 1 1 5CA274D8
P 1050 4850
F 0 "#PWR06" H 1050 4700 50  0001 C CNN
F 1 "VSS" V 1067 4978 50  0000 L CNN
F 2 "" H 1050 4850 50  0001 C CNN
F 3 "" H 1050 4850 50  0001 C CNN
	1    1050 4850
	0    1    1    0   
$EndComp
$Comp
L Cluster-rescue:VCC-power #PWR07
U 1 1 5CA2751F
P 1050 4950
F 0 "#PWR07" H 1050 4800 50  0001 C CNN
F 1 "VCC" V 1067 5078 50  0000 L CNN
F 2 "" H 1050 4950 50  0001 C CNN
F 3 "" H 1050 4950 50  0001 C CNN
	1    1050 4950
	0    1    1    0   
$EndComp
$Comp
L Cluster-rescue:VSS-power #PWR05
U 1 1 5CA2BD9C
P 1050 3850
F 0 "#PWR05" H 1050 3700 50  0001 C CNN
F 1 "VSS" V 1067 3978 50  0000 L CNN
F 2 "" H 1050 3850 50  0001 C CNN
F 3 "" H 1050 3850 50  0001 C CNN
	1    1050 3850
	0    1    1    0   
$EndComp
$Comp
L Cluster-rescue:VSS-power #PWR04
U 1 1 5CA2BDE5
P 1050 3150
F 0 "#PWR04" H 1050 3000 50  0001 C CNN
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
Text GLabel 9650 1750 2    50   Input ~ 0
GPIO1
Text GLabel 9650 1850 2    50   Input ~ 0
GPIO2
Text GLabel 9650 1950 2    50   Input ~ 0
GPIO3
Text GLabel 9650 2050 2    50   Input ~ 0
GPIO4
Text GLabel 9650 2150 2    50   Input ~ 0
GPIO5
Text GLabel 9650 2250 2    50   Input ~ 0
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
L Cluster-rescue:Conn_01x06-Connector_Generic J5
U 1 1 5CA32F8F
P 850 5650
F 0 "J5" H 770 5125 50  0000 C CNN
F 1 "GPIO" H 770 5216 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 850 5650 50  0001 C CNN
F 3 "~" H 850 5650 50  0001 C CNN
	1    850  5650
	-1   0    0    1   
$EndComp
Text GLabel 5150 7550 2    50   Input ~ 0
SCL
Text GLabel 6250 7550 2    50   Input ~ 0
SDA
$Comp
L Cluster-rescue:R_Small-Device R2
U 1 1 5CA35359
P 4950 7550
F 0 "R2" V 4754 7550 50  0000 C CNN
F 1 "10k" V 4845 7550 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" H 4950 7550 50  0001 C CNN
F 3 "~" H 4950 7550 50  0001 C CNN
	1    4950 7550
	0    1    1    0   
$EndComp
$Comp
L Cluster-rescue:R_Small-Device R6
U 1 1 5CA3546B
P 6050 7550
F 0 "R6" V 6246 7550 50  0000 C CNN
F 1 "10k" V 6155 7550 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" H 6050 7550 50  0001 C CNN
F 3 "~" H 6050 7550 50  0001 C CNN
	1    6050 7550
	0    -1   -1   0   
$EndComp
$Comp
L Cluster-rescue:VCC-power #PWR040
U 1 1 5CA3557E
P 4750 7550
F 0 "#PWR040" H 4750 7400 50  0001 C CNN
F 1 "VCC" V 4768 7677 50  0000 L CNN
F 2 "" H 4750 7550 50  0001 C CNN
F 3 "" H 4750 7550 50  0001 C CNN
	1    4750 7550
	0    -1   -1   0   
$EndComp
$Comp
L Cluster-rescue:VCC-power #PWR051
U 1 1 5CA3563A
P 5850 7550
F 0 "#PWR051" H 5850 7400 50  0001 C CNN
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
L Cluster-rescue:C_Small-Device C12
U 1 1 5CA3D7C2
P 5050 2250
F 0 "C12" H 5142 2296 50  0000 L CNN
F 1 "100nF" H 5142 2205 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5050 2250 50  0001 C CNN
F 3 "~" H 5050 2250 50  0001 C CNN
	1    5050 2250
	1    0    0    -1  
$EndComp
$Comp
L Cluster-rescue:C_Small-Device C10
U 1 1 5CA3D83A
P 4700 2250
F 0 "C10" H 4792 2296 50  0000 L CNN
F 1 "10uF" H 4792 2205 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4700 2250 50  0001 C CNN
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
L Cluster-rescue:VSS-power #PWR041
U 1 1 5CA41665
P 4950 2350
F 0 "#PWR041" H 4950 2200 50  0001 C CNN
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
L Cluster-rescue:VSS-power #PWR037
U 1 1 5CA4170D
P 4300 3050
F 0 "#PWR037" H 4300 2900 50  0001 C CNN
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
L Cluster-rescue:VCC-power #PWR042
U 1 1 5CA43336
P 5050 2150
F 0 "#PWR042" H 5050 2000 50  0001 C CNN
F 1 "VCC" H 5067 2323 50  0000 C CNN
F 2 "" H 5050 2150 50  0001 C CNN
F 3 "" H 5050 2150 50  0001 C CNN
	1    5050 2150
	1    0    0    -1  
$EndComp
Connection ~ 5050 2150
$Comp
L Cluster-rescue:VCC-power #PWR025
U 1 1 5CA43392
P 3100 2150
F 0 "#PWR025" H 3100 2000 50  0001 C CNN
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
Text GLabel 9650 3350 2    50   Input ~ 0
LightsOutFront
Text GLabel 9650 3250 2    50   Input ~ 0
LightsOutBack
$Comp
L Cluster-rescue:VSS-power #PWR08
U 1 1 5CA4FD7A
P 1050 6600
F 0 "#PWR08" H 1050 6450 50  0001 C CNN
F 1 "VSS" V 1067 6728 50  0000 L CNN
F 2 "" H 1050 6600 50  0001 C CNN
F 3 "" H 1050 6600 50  0001 C CNN
	1    1050 6600
	0    1    1    0   
$EndComp
$Comp
L Cluster-rescue:VCC-power #PWR09
U 1 1 5CA52CF6
P 1050 6700
F 0 "#PWR09" H 1050 6550 50  0001 C CNN
F 1 "VCC" V 1067 6828 50  0000 L CNN
F 2 "" H 1050 6700 50  0001 C CNN
F 3 "" H 1050 6700 50  0001 C CNN
	1    1050 6700
	0    1    1    0   
$EndComp
Text GLabel 8250 3350 0    50   Input ~ 0
MISO
Text GLabel 8250 3450 0    50   Input ~ 0
MOSI
Text GLabel 9650 4250 2    50   Input ~ 0
SCK
Text GLabel 2400 6650 2    50   Input ~ 0
SCK
Text GLabel 2400 6250 2    50   Input ~ 0
MISO
Text GLabel 2400 6350 2    50   Input ~ 0
MOSI
$Comp
L Cluster-rescue:VSS-power #PWR022
U 1 1 5CA57E42
P 2400 7150
F 0 "#PWR022" H 2400 7000 50  0001 C CNN
F 1 "VSS" V 2417 7278 50  0000 L CNN
F 2 "" H 2400 7150 50  0001 C CNN
F 3 "" H 2400 7150 50  0001 C CNN
	1    2400 7150
	0    1    1    0   
$EndComp
$Comp
L Cluster-rescue:VCC-power #PWR020
U 1 1 5CA57F75
P 2400 7050
F 0 "#PWR020" H 2400 6900 50  0001 C CNN
F 1 "VCC" V 2417 7178 50  0000 L CNN
F 2 "" H 2400 7050 50  0001 C CNN
F 3 "" H 2400 7050 50  0001 C CNN
	1    2400 7050
	0    1    1    0   
$EndComp
$Comp
L Cluster-rescue:VSS-power #PWR019
U 1 1 5CA5802B
P 2400 6750
F 0 "#PWR019" H 2400 6600 50  0001 C CNN
F 1 "VSS" V 2417 6878 50  0000 L CNN
F 2 "" H 2400 6750 50  0001 C CNN
F 3 "" H 2400 6750 50  0001 C CNN
	1    2400 6750
	0    1    1    0   
$EndComp
$Comp
L Cluster-rescue:VCC-power #PWR021
U 1 1 5CA5823A
P 2400 7050
F 0 "#PWR021" H 2400 6900 50  0001 C CNN
F 1 "VCC" V 2417 7178 50  0000 L CNN
F 2 "" H 2400 7050 50  0001 C CNN
F 3 "" H 2400 7050 50  0001 C CNN
	1    2400 7050
	0    1    1    0   
$EndComp
Connection ~ 2400 7050
$Comp
L Cluster-rescue:VCC-power #PWR017
U 1 1 5CA58353
P 2400 4350
F 0 "#PWR017" H 2400 4200 50  0001 C CNN
F 1 "VCC" V 2417 4478 50  0000 L CNN
F 2 "" H 2400 4350 50  0001 C CNN
F 3 "" H 2400 4350 50  0001 C CNN
	1    2400 4350
	0    1    1    0   
$EndComp
Wire Wire Line
	2400 4050 2450 4050
$Comp
L Cluster-rescue:VSS-power #PWR023
U 1 1 5CA59CD7
P 2450 4050
F 0 "#PWR023" H 2450 3900 50  0001 C CNN
F 1 "VSS" V 2467 4178 50  0000 L CNN
F 2 "" H 2450 4050 50  0001 C CNN
F 3 "" H 2450 4050 50  0001 C CNN
	1    2450 4050
	0    1    1    0   
$EndComp
$Comp
L Cluster-rescue:VSS-power #PWR018
U 1 1 5CA59FDC
P 2400 6050
F 0 "#PWR018" H 2400 5900 50  0001 C CNN
F 1 "VSS" V 2417 6178 50  0000 L CNN
F 2 "" H 2400 6050 50  0001 C CNN
F 3 "" H 2400 6050 50  0001 C CNN
	1    2400 6050
	0    1    1    0   
$EndComp
$Comp
L Cluster-rescue:Conn_01x03-Connector_Generic J10
U 1 1 5CA5A15F
P 2800 3750
F 0 "J10" H 2880 3792 50  0000 L CNN
F 1 "Conn_01x03" H 2880 3701 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 2800 3750 50  0001 C CNN
F 3 "~" H 2800 3750 50  0001 C CNN
	1    2800 3750
	1    0    0    -1  
$EndComp
NoConn ~ 8250 4150
NoConn ~ 8250 4250
NoConn ~ 8250 4350
NoConn ~ 8250 4450
NoConn ~ 9650 4150
NoConn ~ 9650 4050
NoConn ~ 9650 3750
NoConn ~ 9650 3050
NoConn ~ 9650 2650
NoConn ~ 9650 2750
NoConn ~ 8250 2950
$Comp
L Cluster-rescue:Conn_01x03-Connector_Generic J13
U 1 1 5CA78B24
P 4500 2650
F 0 "J13" H 4580 2692 50  0000 L CNN
F 1 "Conn_01x03" H 4580 2601 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 4500 2650 50  0001 C CNN
F 3 "~" H 4500 2650 50  0001 C CNN
	1    4500 2650
	1    0    0    -1  
$EndComp
$Comp
L Cluster-rescue:Conn_01x02-Connector_Generic J11
U 1 1 5CA78CAF
P 2900 2550
F 0 "J11" H 2820 2225 50  0000 C CNN
F 1 "Conn_01x02" H 2820 2316 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 2900 2550 50  0001 C CNN
F 3 "~" H 2900 2550 50  0001 C CNN
	1    2900 2550
	-1   0    0    1   
$EndComp
NoConn ~ 3100 2850
Text GLabel 9650 4350 2    50   Input ~ 0
OverDrive
Text GLabel 1050 4350 2    50   Input ~ 0
Reset
Text GLabel 8250 3650 0    50   Input ~ 0
LightsOutAux
Text GLabel 8250 3550 0    50   Input ~ 0
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
L Cluster-rescue:1N5822-Diode D1
U 1 1 5CA1159F
P 5150 6650
F 0 "D1" V 5104 6729 50  0000 L CNN
F 1 "1N5822" V 5195 6729 50  0000 L CNN
F 2 "Diodes_SMD:D_SMA" H 5150 6475 50  0001 C CNN
F 3 "http://www.vishay.com/docs/88526/1n5820.pdf" H 5150 6650 50  0001 C CNN
	1    5150 6650
	0    1    1    0   
$EndComp
NoConn ~ 8250 3150
NoConn ~ 8250 3250
$Comp
L Cluster-rescue:AiP8563-Cluster U3
U 1 1 5CA417E5
P 4600 4200
F 0 "U3" H 4600 4565 50  0000 C CNN
F 1 "AiP8563" H 4600 4474 50  0000 C CNN
F 2 "Housings_SSOP:MSOP-8_3x3mm_Pitch0.65mm" H 4000 3700 50  0001 C CNN
F 3 "" H 4000 3700 50  0001 C CNN
	1    4600 4200
	1    0    0    -1  
$EndComp
$Comp
L Cluster-rescue:Crystal_Small-Device Y1
U 1 1 5CA5BD2C
P 3800 4100
F 0 "Y1" V 3754 4188 50  0000 L CNN
F 1 " 32.768kHz" V 3950 3900 50  0000 L CNN
F 2 "Crystals:Crystal_SMD_3215-2pin_3.2x1.5mm" H 3800 4100 50  0001 C CNN
F 3 "~" H 3800 4100 50  0001 C CNN
	1    3800 4100
	0    1    1    0   
$EndComp
$Comp
L Cluster-rescue:C_Small-Device C2
U 1 1 5CA5CE75
P 3600 4000
F 0 "C2" V 3371 4000 50  0000 C CNN
F 1 "20pF" V 3462 4000 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3600 4000 50  0001 C CNN
F 3 "~" H 3600 4000 50  0001 C CNN
	1    3600 4000
	0    1    1    0   
$EndComp
$Comp
L Cluster-rescue:C_Small-Device C3
U 1 1 5CA5D4DA
P 3600 4200
F 0 "C3" V 3371 4200 50  0000 C CNN
F 1 "20pF" V 3462 4200 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3600 4200 50  0001 C CNN
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
L Cluster-rescue:VSS-power #PWR029
U 1 1 5CA71B7C
P 3450 4200
F 0 "#PWR029" H 3450 4050 50  0001 C CNN
F 1 "VSS" V 3468 4328 50  0000 L CNN
F 2 "" H 3450 4200 50  0001 C CNN
F 3 "" H 3450 4200 50  0001 C CNN
	1    3450 4200
	0    -1   -1   0   
$EndComp
Connection ~ 3450 4200
$Comp
L Cluster-rescue:C_Small-Device C13
U 1 1 5CA72D9A
P 5250 4000
F 0 "C13" H 5342 4046 50  0000 L CNN
F 1 "10uF" H 5342 3955 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5250 4000 50  0001 C CNN
F 3 "~" H 5250 4000 50  0001 C CNN
	1    5250 4000
	1    0    0    -1  
$EndComp
$Comp
L Cluster-rescue:C_Small-Device C16
U 1 1 5CA7327D
P 5600 4000
F 0 "C16" H 5692 4046 50  0000 L CNN
F 1 "0.1uF" H 5692 3955 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5600 4000 50  0001 C CNN
F 3 "~" H 5600 4000 50  0001 C CNN
	1    5600 4000
	1    0    0    -1  
$EndComp
Text GLabel 5150 4300 2    50   Input ~ 0
SCL
Text GLabel 5150 4400 2    50   Input ~ 0
SDA
$Comp
L Cluster-rescue:VSS-power #PWR035
U 1 1 5CA75677
P 4050 4400
F 0 "#PWR035" H 4050 4250 50  0001 C CNN
F 1 "VSS" H 4068 4573 50  0000 C CNN
F 2 "" H 4050 4400 50  0001 C CNN
F 3 "" H 4050 4400 50  0001 C CNN
	1    4050 4400
	-1   0    0    1   
$EndComp
NoConn ~ 4050 4300
$Comp
L Cluster-rescue:D_ALT-Device D3
U 1 1 5CA7D920
P 6150 4100
F 0 "D3" H 6150 4316 50  0000 C CNN
F 1 "D_ALT" H 6150 4225 50  0000 C CNN
F 2 "Diodes_SMD:D_0603" H 6150 4100 50  0001 C CNN
F 3 "~" H 6150 4100 50  0001 C CNN
	1    6150 4100
	1    0    0    -1  
$EndComp
$Comp
L Cluster-rescue:D_ALT-Device D2
U 1 1 5CA7DD6A
P 5850 4300
F 0 "D2" V 5804 4379 50  0000 L CNN
F 1 "D_ALT" V 5895 4379 50  0000 L CNN
F 2 "Diodes_SMD:D_0603" H 5850 4300 50  0001 C CNN
F 3 "~" H 5850 4300 50  0001 C CNN
	1    5850 4300
	0    1    1    0   
$EndComp
Connection ~ 5250 4100
Wire Wire Line
	5250 4100 5150 4100
Connection ~ 5600 4100
Wire Wire Line
	5600 4100 5250 4100
Wire Wire Line
	5850 4150 5850 4100
Connection ~ 5850 4100
Wire Wire Line
	5850 4100 5600 4100
$Comp
L Cluster-rescue:Battery_Cell-Device BT1
U 1 1 5CA8509D
P 5850 4750
F 0 "BT1" H 5968 4846 50  0000 L CNN
F 1 "Battery_Cell" H 5968 4755 50  0000 L CNN
F 2 "Battery_Holders:Keystone_3008_1x2450-CoinCell" V 5850 4810 50  0001 C CNN
F 3 "~" V 5850 4810 50  0001 C CNN
	1    5850 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	5850 4550 5850 4500
$Comp
L Cluster-rescue:VSS-power #PWR050
U 1 1 5CA891E1
P 5850 4900
F 0 "#PWR050" H 5850 4750 50  0001 C CNN
F 1 "VSS" H 5868 5073 50  0000 C CNN
F 2 "" H 5850 4900 50  0001 C CNN
F 3 "" H 5850 4900 50  0001 C CNN
	1    5850 4900
	-1   0    0    1   
$EndComp
Wire Wire Line
	5850 4900 5850 4850
$Comp
L Cluster-rescue:VCC-power #PWR055
U 1 1 5CA8CF43
P 6300 4100
F 0 "#PWR055" H 6300 3950 50  0001 C CNN
F 1 "VCC" V 6317 4228 50  0000 L CNN
F 2 "" H 6300 4100 50  0001 C CNN
F 3 "" H 6300 4100 50  0001 C CNN
	1    6300 4100
	0    1    1    0   
$EndComp
Wire Wire Line
	6300 4100 6250 4100
$Comp
L Cluster-rescue:badge-rescue_HX4056-badge-cache U4
U 1 1 5CA91910
P 4700 4850
F 0 "U4" H 4650 5175 50  0000 C CNN
F 1 "badge-rescue_HX4056" H 4650 5084 50  0000 C CNN
F 2 "Badge:HX4056" H 4600 4850 50  0001 C CNN
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
L Cluster-rescue:R_Small-Device R3
U 1 1 5CA9B6E5
P 5250 4850
F 0 "R3" V 5054 4850 50  0000 C CNN
F 1 "1k" V 5145 4850 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" H 5250 4850 50  0001 C CNN
F 3 "~" H 5250 4850 50  0001 C CNN
	1    5250 4850
	0    1    1    0   
$EndComp
Wire Wire Line
	5100 4850 5150 4850
$Comp
L Cluster-rescue:VSS-power #PWR045
U 1 1 5CA9F6CB
P 5400 4850
F 0 "#PWR045" H 5400 4700 50  0001 C CNN
F 1 "VSS" H 5418 5023 50  0000 C CNN
F 2 "" H 5400 4850 50  0001 C CNN
F 3 "" H 5400 4850 50  0001 C CNN
	1    5400 4850
	-1   0    0    1   
$EndComp
$Comp
L Cluster-rescue:VSS-power #PWR043
U 1 1 5CAA3577
P 5150 5050
F 0 "#PWR043" H 5150 4900 50  0001 C CNN
F 1 "VSS" H 5168 5223 50  0000 C CNN
F 2 "" H 5150 5050 50  0001 C CNN
F 3 "" H 5150 5050 50  0001 C CNN
	1    5150 5050
	-1   0    0    1   
$EndComp
Wire Wire Line
	5150 5050 5100 5050
$Comp
L Cluster-rescue:VSS-power #PWR036
U 1 1 5CAA6F8D
P 4150 5050
F 0 "#PWR036" H 4150 4900 50  0001 C CNN
F 1 "VSS" H 4168 5223 50  0000 C CNN
F 2 "" H 4150 5050 50  0001 C CNN
F 3 "" H 4150 5050 50  0001 C CNN
	1    4150 5050
	-1   0    0    1   
$EndComp
Wire Wire Line
	4150 5050 4200 5050
$Comp
L Cluster-rescue:C_Small-Device C6
U 1 1 5CAAB789
P 3950 4850
F 0 "C6" H 3858 4804 50  0000 R CNN
F 1 "10uF" H 3858 4895 50  0000 R CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3950 4850 50  0001 C CNN
F 3 "~" H 3950 4850 50  0001 C CNN
	1    3950 4850
	-1   0    0    1   
$EndComp
$Comp
L Cluster-rescue:C_Small-Device C15
U 1 1 5CAABC70
P 5550 4700
F 0 "C15" H 5458 4654 50  0000 R CNN
F 1 "10uF" H 5458 4745 50  0000 R CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5550 4700 50  0001 C CNN
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
L Cluster-rescue:VSS-power #PWR047
U 1 1 5CAAFD31
P 5550 4850
F 0 "#PWR047" H 5550 4700 50  0001 C CNN
F 1 "VSS" H 5568 5023 50  0000 C CNN
F 2 "" H 5550 4850 50  0001 C CNN
F 3 "" H 5550 4850 50  0001 C CNN
	1    5550 4850
	-1   0    0    1   
$EndComp
Wire Wire Line
	5550 4850 5550 4800
$Comp
L Cluster-rescue:VSS-power #PWR034
U 1 1 5CAB3B7F
P 3950 5250
F 0 "#PWR034" H 3950 5100 50  0001 C CNN
F 1 "VSS" H 3968 5423 50  0000 C CNN
F 2 "" H 3950 5250 50  0001 C CNN
F 3 "" H 3950 5250 50  0001 C CNN
	1    3950 5250
	-1   0    0    1   
$EndComp
Wire Wire Line
	3950 5250 3950 5200
Connection ~ 3950 4750
Wire Wire Line
	3950 4750 4200 4750
$Comp
L Cluster-rescue:VCC-power #PWR033
U 1 1 5CABBFAA
P 3800 4750
F 0 "#PWR033" H 3800 4600 50  0001 C CNN
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
L Cluster-rescue:Conn_01x01-Connector_Generic J14
U 1 1 5CAC98B1
P 5350 4200
F 0 "J14" H 5430 4242 50  0000 L CNN
F 1 "ClkOUT" H 5430 4151 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 5350 4200 50  0001 C CNN
F 3 "~" H 5350 4200 50  0001 C CNN
	1    5350 4200
	1    0    0    -1  
$EndComp
$Comp
L Cluster-rescue:Conn_01x02-Connector_Generic J8
U 1 1 5CACCAA1
P 2050 1450
F 0 "J8" H 1968 1125 50  0000 C CNN
F 1 "Conn_01x02" H 1968 1216 50  0000 C CNN
F 2 "TerminalBlock_RND:TerminalBlock_RND_205-00001_Pitch5.00mm" H 2050 1450 50  0001 C CNN
F 3 "~" H 2050 1450 50  0001 C CNN
	1    2050 1450
	-1   0    0    1   
$EndComp
Text GLabel 2250 1350 2    50   Input ~ 0
LeftIN
Text GLabel 2250 1450 2    50   Input ~ 0
RightIN
Text GLabel 8250 4050 0    50   Input ~ 0
LeftIN
Text GLabel 9650 2350 2    50   Input ~ 0
RightIN
$Comp
L Cluster-rescue:SK6812-LED D5
U 1 1 5CA58DBC
P 6850 5900
F 0 "D5" H 7194 5946 50  0000 L CNN
F 1 "SK6812" H 7194 5855 50  0000 L CNN
F 2 "LEDs:LED_WS2812B-PLCC4" H 6900 5600 50  0001 L TNN
F 3 "https://cdn-shop.adafruit.com/product-files/1138/SK6812+LED+datasheet+.pdf" H 6950 5525 50  0001 L TNN
	1    6850 5900
	1    0    0    -1  
$EndComp
$Comp
L Cluster-rescue:VSS-power #PWR059
U 1 1 5CA5AC5C
P 6850 6250
F 0 "#PWR059" H 6850 6100 50  0001 C CNN
F 1 "VSS" H 6868 6423 50  0000 C CNN
F 2 "" H 6850 6250 50  0001 C CNN
F 3 "" H 6850 6250 50  0001 C CNN
	1    6850 6250
	-1   0    0    1   
$EndComp
Wire Wire Line
	6850 6250 6850 6200
$Comp
L Cluster-rescue:C_Small-Device C20
U 1 1 5CA5FAFE
P 7000 5550
F 0 "C20" V 6771 5550 50  0000 C CNN
F 1 "100nF" V 6862 5550 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 7000 5550 50  0001 C CNN
F 3 "~" H 7000 5550 50  0001 C CNN
	1    7000 5550
	0    1    1    0   
$EndComp
$Comp
L Cluster-rescue:VCC-power #PWR058
U 1 1 5CA60CA1
P 6850 5400
F 0 "#PWR058" H 6850 5250 50  0001 C CNN
F 1 "VCC" H 6867 5573 50  0000 C CNN
F 2 "" H 6850 5400 50  0001 C CNN
F 3 "" H 6850 5400 50  0001 C CNN
	1    6850 5400
	1    0    0    -1  
$EndComp
Wire Wire Line
	6850 5600 6850 5550
Wire Wire Line
	6850 5550 6900 5550
Connection ~ 6850 5550
Wire Wire Line
	6850 5550 6850 5400
$Comp
L Cluster-rescue:VSS-power #PWR060
U 1 1 5CA68FDE
P 7150 5550
F 0 "#PWR060" H 7150 5400 50  0001 C CNN
F 1 "VSS" V 7167 5678 50  0000 L CNN
F 2 "" H 7150 5550 50  0001 C CNN
F 3 "" H 7150 5550 50  0001 C CNN
	1    7150 5550
	0    1    1    0   
$EndComp
Wire Wire Line
	7100 5550 7150 5550
$Comp
L Cluster-rescue:SK6812-LED D6
U 1 1 5CA6DB6E
P 7800 5900
F 0 "D6" H 8144 5946 50  0000 L CNN
F 1 "SK6812" H 8144 5855 50  0000 L CNN
F 2 "LEDs:LED_WS2812B-PLCC4" H 7850 5600 50  0001 L TNN
F 3 "https://cdn-shop.adafruit.com/product-files/1138/SK6812+LED+datasheet+.pdf" H 7900 5525 50  0001 L TNN
	1    7800 5900
	1    0    0    -1  
$EndComp
$Comp
L Cluster-rescue:VSS-power #PWR065
U 1 1 5CA6DB78
P 7800 6250
F 0 "#PWR065" H 7800 6100 50  0001 C CNN
F 1 "VSS" H 7818 6423 50  0000 C CNN
F 2 "" H 7800 6250 50  0001 C CNN
F 3 "" H 7800 6250 50  0001 C CNN
	1    7800 6250
	-1   0    0    1   
$EndComp
Wire Wire Line
	7800 6250 7800 6200
$Comp
L Cluster-rescue:C_Small-Device C27
U 1 1 5CA6DB83
P 7950 5550
F 0 "C27" V 7721 5550 50  0000 C CNN
F 1 "100nF" V 7812 5550 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 7950 5550 50  0001 C CNN
F 3 "~" H 7950 5550 50  0001 C CNN
	1    7950 5550
	0    1    1    0   
$EndComp
$Comp
L Cluster-rescue:VCC-power #PWR064
U 1 1 5CA6DB8D
P 7800 5400
F 0 "#PWR064" H 7800 5250 50  0001 C CNN
F 1 "VCC" H 7817 5573 50  0000 C CNN
F 2 "" H 7800 5400 50  0001 C CNN
F 3 "" H 7800 5400 50  0001 C CNN
	1    7800 5400
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 5600 7800 5550
Wire Wire Line
	7800 5550 7850 5550
Connection ~ 7800 5550
Wire Wire Line
	7800 5550 7800 5400
$Comp
L Cluster-rescue:VSS-power #PWR066
U 1 1 5CA6DB9B
P 8100 5550
F 0 "#PWR066" H 8100 5400 50  0001 C CNN
F 1 "VSS" V 8117 5678 50  0000 L CNN
F 2 "" H 8100 5550 50  0001 C CNN
F 3 "" H 8100 5550 50  0001 C CNN
	1    8100 5550
	0    1    1    0   
$EndComp
Wire Wire Line
	8050 5550 8100 5550
$Comp
L Cluster-rescue:SK6812-LED D7
U 1 1 5CA75E50
P 8750 5900
F 0 "D7" H 9094 5946 50  0000 L CNN
F 1 "SK6812" H 9094 5855 50  0000 L CNN
F 2 "LEDs:LED_WS2812B-PLCC4" H 8800 5600 50  0001 L TNN
F 3 "https://cdn-shop.adafruit.com/product-files/1138/SK6812+LED+datasheet+.pdf" H 8850 5525 50  0001 L TNN
	1    8750 5900
	1    0    0    -1  
$EndComp
$Comp
L Cluster-rescue:VSS-power #PWR070
U 1 1 5CA75E5A
P 8750 6250
F 0 "#PWR070" H 8750 6100 50  0001 C CNN
F 1 "VSS" H 8768 6423 50  0000 C CNN
F 2 "" H 8750 6250 50  0001 C CNN
F 3 "" H 8750 6250 50  0001 C CNN
	1    8750 6250
	-1   0    0    1   
$EndComp
Wire Wire Line
	8750 6250 8750 6200
$Comp
L Cluster-rescue:C_Small-Device C28
U 1 1 5CA75E65
P 8900 5550
F 0 "C28" V 8671 5550 50  0000 C CNN
F 1 "100nF" V 8762 5550 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 8900 5550 50  0001 C CNN
F 3 "~" H 8900 5550 50  0001 C CNN
	1    8900 5550
	0    1    1    0   
$EndComp
$Comp
L Cluster-rescue:VCC-power #PWR069
U 1 1 5CA75E6F
P 8750 5400
F 0 "#PWR069" H 8750 5250 50  0001 C CNN
F 1 "VCC" H 8767 5573 50  0000 C CNN
F 2 "" H 8750 5400 50  0001 C CNN
F 3 "" H 8750 5400 50  0001 C CNN
	1    8750 5400
	1    0    0    -1  
$EndComp
Wire Wire Line
	8750 5600 8750 5550
Wire Wire Line
	8750 5550 8800 5550
Connection ~ 8750 5550
Wire Wire Line
	8750 5550 8750 5400
$Comp
L Cluster-rescue:VSS-power #PWR071
U 1 1 5CA75E7D
P 9050 5550
F 0 "#PWR071" H 9050 5400 50  0001 C CNN
F 1 "VSS" V 9067 5678 50  0000 L CNN
F 2 "" H 9050 5550 50  0001 C CNN
F 3 "" H 9050 5550 50  0001 C CNN
	1    9050 5550
	0    1    1    0   
$EndComp
Wire Wire Line
	9000 5550 9050 5550
$Comp
L Cluster-rescue:SK6812-LED D8
U 1 1 5CA75E88
P 9700 5900
F 0 "D8" H 10044 5946 50  0000 L CNN
F 1 "SK6812" H 10044 5855 50  0000 L CNN
F 2 "LEDs:LED_WS2812B-PLCC4" H 9750 5600 50  0001 L TNN
F 3 "https://cdn-shop.adafruit.com/product-files/1138/SK6812+LED+datasheet+.pdf" H 9800 5525 50  0001 L TNN
	1    9700 5900
	1    0    0    -1  
$EndComp
$Comp
L Cluster-rescue:VSS-power #PWR074
U 1 1 5CA75E92
P 9700 6250
F 0 "#PWR074" H 9700 6100 50  0001 C CNN
F 1 "VSS" H 9718 6423 50  0000 C CNN
F 2 "" H 9700 6250 50  0001 C CNN
F 3 "" H 9700 6250 50  0001 C CNN
	1    9700 6250
	-1   0    0    1   
$EndComp
Wire Wire Line
	9700 6250 9700 6200
$Comp
L Cluster-rescue:C_Small-Device C29
U 1 1 5CA75E9D
P 9850 5550
F 0 "C29" V 9621 5550 50  0000 C CNN
F 1 "100nF" V 9712 5550 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 9850 5550 50  0001 C CNN
F 3 "~" H 9850 5550 50  0001 C CNN
	1    9850 5550
	0    1    1    0   
$EndComp
$Comp
L Cluster-rescue:VCC-power #PWR073
U 1 1 5CA75EA7
P 9700 5400
F 0 "#PWR073" H 9700 5250 50  0001 C CNN
F 1 "VCC" H 9717 5573 50  0000 C CNN
F 2 "" H 9700 5400 50  0001 C CNN
F 3 "" H 9700 5400 50  0001 C CNN
	1    9700 5400
	1    0    0    -1  
$EndComp
Wire Wire Line
	9700 5600 9700 5550
Wire Wire Line
	9700 5550 9750 5550
Connection ~ 9700 5550
Wire Wire Line
	9700 5550 9700 5400
$Comp
L Cluster-rescue:VSS-power #PWR075
U 1 1 5CA75EB5
P 10000 5550
F 0 "#PWR075" H 10000 5400 50  0001 C CNN
F 1 "VSS" V 10017 5678 50  0000 L CNN
F 2 "" H 10000 5550 50  0001 C CNN
F 3 "" H 10000 5550 50  0001 C CNN
	1    10000 5550
	0    1    1    0   
$EndComp
Wire Wire Line
	9950 5550 10000 5550
$Comp
L Cluster-rescue:SK6812-LED D9
U 1 1 5CA80F2A
P 10650 5900
F 0 "D9" H 10994 5946 50  0000 L CNN
F 1 "SK6812" H 10994 5855 50  0000 L CNN
F 2 "LEDs:LED_WS2812B-PLCC4" H 10700 5600 50  0001 L TNN
F 3 "https://cdn-shop.adafruit.com/product-files/1138/SK6812+LED+datasheet+.pdf" H 10750 5525 50  0001 L TNN
	1    10650 5900
	1    0    0    -1  
$EndComp
$Comp
L Cluster-rescue:VSS-power #PWR077
U 1 1 5CA80F34
P 10650 6250
F 0 "#PWR077" H 10650 6100 50  0001 C CNN
F 1 "VSS" H 10668 6423 50  0000 C CNN
F 2 "" H 10650 6250 50  0001 C CNN
F 3 "" H 10650 6250 50  0001 C CNN
	1    10650 6250
	-1   0    0    1   
$EndComp
Wire Wire Line
	10650 6250 10650 6200
$Comp
L Cluster-rescue:C_Small-Device C30
U 1 1 5CA80F3F
P 10800 5550
F 0 "C30" V 10571 5550 50  0000 C CNN
F 1 "100nF" V 10662 5550 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 10800 5550 50  0001 C CNN
F 3 "~" H 10800 5550 50  0001 C CNN
	1    10800 5550
	0    1    1    0   
$EndComp
$Comp
L Cluster-rescue:VCC-power #PWR076
U 1 1 5CA80F49
P 10650 5400
F 0 "#PWR076" H 10650 5250 50  0001 C CNN
F 1 "VCC" H 10667 5573 50  0000 C CNN
F 2 "" H 10650 5400 50  0001 C CNN
F 3 "" H 10650 5400 50  0001 C CNN
	1    10650 5400
	1    0    0    -1  
$EndComp
Wire Wire Line
	10650 5600 10650 5550
Wire Wire Line
	10650 5550 10700 5550
Connection ~ 10650 5550
Wire Wire Line
	10650 5550 10650 5400
$Comp
L Cluster-rescue:VSS-power #PWR078
U 1 1 5CA80F57
P 10950 5550
F 0 "#PWR078" H 10950 5400 50  0001 C CNN
F 1 "VSS" V 10967 5678 50  0000 L CNN
F 2 "" H 10950 5550 50  0001 C CNN
F 3 "" H 10950 5550 50  0001 C CNN
	1    10950 5550
	0    1    1    0   
$EndComp
Wire Wire Line
	10900 5550 10950 5550
Wire Wire Line
	7150 5900 7500 5900
Wire Wire Line
	8100 5900 8450 5900
Wire Wire Line
	9050 5900 9400 5900
Wire Wire Line
	10000 5900 10350 5900
NoConn ~ 10950 5900
Text GLabel 6500 5900 0    50   Input ~ 0
Neo
Wire Wire Line
	6500 5900 6550 5900
Text GLabel 9650 3450 2    50   Input ~ 0
Neo
$Comp
L Cluster-rescue:R_Small-Device R1
U 1 1 5CACBB63
P 3950 5100
F 0 "R1" H 3800 5150 50  0000 L CNN
F 1 "1.5" H 3750 5050 50  0000 L CNN
F 2 "Resistors_SMD:R_0805" H 3950 5100 50  0001 C CNN
F 3 "~" H 3950 5100 50  0001 C CNN
	1    3950 5100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 5000 3950 4950
Wire Wire Line
	5400 4850 5350 4850
$Comp
L Cluster-rescue:VSS-power #PWR044
U 1 1 5CB2F64B
P 5250 3850
F 0 "#PWR044" H 5250 3700 50  0001 C CNN
F 1 "VSS" H 5268 4023 50  0000 C CNN
F 2 "" H 5250 3850 50  0001 C CNN
F 3 "" H 5250 3850 50  0001 C CNN
	1    5250 3850
	1    0    0    -1  
$EndComp
$Comp
L Cluster-rescue:VSS-power #PWR048
U 1 1 5CB300AD
P 5600 3850
F 0 "#PWR048" H 5600 3700 50  0001 C CNN
F 1 "VSS" H 5618 4023 50  0000 C CNN
F 2 "" H 5600 3850 50  0001 C CNN
F 3 "" H 5600 3850 50  0001 C CNN
	1    5600 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 3850 5600 3900
Wire Wire Line
	5250 3900 5250 3850
Wire Wire Line
	5850 4100 6000 4100
Connection ~ 6300 4100
$Comp
L Cluster-rescue:Conn_01x40-Connector_Generic J7
U 1 1 5CA5A259
P 1800 5650
F 0 "J7" H 1720 3425 50  0000 C CNN
F 1 "Conn_01x40" H 1720 3516 50  0000 C CNN
F 2 "Cluster:XF2M-4015" H 1800 5650 50  0001 C CNN
F 3 "~" H 1800 5650 50  0001 C CNN
	1    1800 5650
	-1   0    0    1   
$EndComp
$Comp
L Cluster-rescue:Conn_01x40-Connector_Generic J9
U 1 1 5CA57C7F
P 2200 5650
F 0 "J9" H 2000 3400 50  0000 C CNN
F 1 "Conn_01x40" H 2100 3300 50  0000 C CNN
F 2 "Connector_PinHeader_1.00mm:PinHeader_1x40_P1.00mm_Vertical" H 2200 5650 50  0001 C CNN
F 3 "~" H 2200 5650 50  0001 C CNN
	1    2200 5650
	-1   0    0    1   
$EndComp
Wire Wire Line
	2400 6650 2000 6650
Wire Wire Line
	2400 6350 2000 6350
Wire Wire Line
	2400 6250 2000 6250
Wire Wire Line
	2400 7150 2000 7150
Connection ~ 2400 7150
Wire Wire Line
	2400 7050 2000 7050
Wire Wire Line
	2400 6950 2000 6950
NoConn ~ 2400 7550
NoConn ~ 2400 7450
NoConn ~ 2400 7350
NoConn ~ 2400 7250
NoConn ~ 2400 6850
$Comp
L Cluster-rescue:VCC-power #PWR016
U 1 1 5DB85CCD
P 2000 6550
F 0 "#PWR016" H 2000 6400 50  0001 C CNN
F 1 "VCC" V 2017 6678 50  0000 L CNN
F 2 "" H 2000 6550 50  0001 C CNN
F 3 "" H 2000 6550 50  0001 C CNN
	1    2000 6550
	0    1    1    0   
$EndComp
$Comp
L Cluster-rescue:VCC-power #PWR015
U 1 1 5DB86415
P 2000 6450
F 0 "#PWR015" H 2000 6300 50  0001 C CNN
F 1 "VCC" V 2017 6578 50  0000 L CNN
F 2 "" H 2000 6450 50  0001 C CNN
F 3 "" H 2000 6450 50  0001 C CNN
	1    2000 6450
	0    1    1    0   
$EndComp
$Comp
L Cluster-rescue:VSS-power #PWR013
U 1 1 5DB87B94
P 2000 6050
F 0 "#PWR013" H 2000 5900 50  0001 C CNN
F 1 "VSS" V 2017 6178 50  0000 L CNN
F 2 "" H 2000 6050 50  0001 C CNN
F 3 "" H 2000 6050 50  0001 C CNN
	1    2000 6050
	0    1    1    0   
$EndComp
$Comp
L Cluster-rescue:VCC-power #PWR014
U 1 1 5DB87F6B
P 2000 6150
F 0 "#PWR014" H 2000 6000 50  0001 C CNN
F 1 "VCC" V 2017 6278 50  0000 L CNN
F 2 "" H 2000 6150 50  0001 C CNN
F 3 "" H 2000 6150 50  0001 C CNN
	1    2000 6150
	0    1    1    0   
$EndComp
Wire Wire Line
	2000 6050 2000 5950
Connection ~ 2000 6050
Connection ~ 2000 4550
Wire Wire Line
	2000 4550 2000 4450
Connection ~ 2000 4650
Wire Wire Line
	2000 4650 2000 4550
Connection ~ 2000 4750
Wire Wire Line
	2000 4750 2000 4650
Connection ~ 2000 4850
Wire Wire Line
	2000 4850 2000 4750
Connection ~ 2000 4950
Wire Wire Line
	2000 4950 2000 4850
Connection ~ 2000 5050
Wire Wire Line
	2000 5050 2000 4950
Connection ~ 2000 5150
Wire Wire Line
	2000 5150 2000 5050
Connection ~ 2000 5250
Wire Wire Line
	2000 5250 2000 5150
Connection ~ 2000 5350
Wire Wire Line
	2000 5350 2000 5250
Connection ~ 2000 5450
Wire Wire Line
	2000 5450 2000 5350
Connection ~ 2000 5550
Wire Wire Line
	2000 5550 2000 5450
Connection ~ 2000 5650
Wire Wire Line
	2000 5650 2000 5550
Connection ~ 2000 5750
Wire Wire Line
	2000 5750 2000 5650
Connection ~ 2000 5850
Wire Wire Line
	2000 5850 2000 5750
Connection ~ 2000 5950
Wire Wire Line
	2000 5950 2000 5850
$Comp
L Cluster-rescue:VCC-power #PWR012
U 1 1 5DB91F31
P 2000 4350
F 0 "#PWR012" H 2000 4200 50  0001 C CNN
F 1 "VCC" V 2017 4478 50  0000 L CNN
F 2 "" H 2000 4350 50  0001 C CNN
F 3 "" H 2000 4350 50  0001 C CNN
	1    2000 4350
	0    1    1    0   
$EndComp
$Comp
L Cluster-rescue:VSS-power #PWR011
U 1 1 5DB92312
P 2000 4050
F 0 "#PWR011" H 2000 3900 50  0001 C CNN
F 1 "VSS" V 2017 4178 50  0000 L CNN
F 2 "" H 2000 4050 50  0001 C CNN
F 3 "" H 2000 4050 50  0001 C CNN
	1    2000 4050
	0    1    1    0   
$EndComp
Wire Wire Line
	2000 4250 2000 4150
Connection ~ 2000 4050
Connection ~ 2000 4150
Wire Wire Line
	2000 4150 2000 4050
Wire Wire Line
	2000 3950 2000 4050
Wire Wire Line
	2400 3650 2000 3650
Wire Wire Line
	2000 3750 2400 3750
Wire Wire Line
	2400 3850 2000 3850
Connection ~ 2400 3850
$Comp
L Cluster-rescue:VCC-power #PWR024
U 1 1 5DBBD78A
P 2500 3500
F 0 "#PWR024" H 2500 3350 50  0001 C CNN
F 1 "VCC" V 2517 3628 50  0000 L CNN
F 2 "" H 2500 3500 50  0001 C CNN
F 3 "" H 2500 3500 50  0001 C CNN
	1    2500 3500
	0    1    1    0   
$EndComp
Wire Wire Line
	2400 3850 2500 3850
Wire Wire Line
	2400 3750 2550 3750
Connection ~ 2400 3750
Wire Wire Line
	2400 3650 2500 3650
Connection ~ 2400 3650
Wire Wire Line
	2500 3500 2500 3650
Connection ~ 2500 3650
Wire Wire Line
	2500 3650 2600 3650
Wire Wire Line
	2500 3650 2500 3850
Connection ~ 2500 3850
Wire Wire Line
	2500 3850 2600 3850
Wire Wire Line
	2550 3750 2550 3950
Wire Wire Line
	2550 3950 2450 3950
Wire Wire Line
	2450 3950 2450 4050
Connection ~ 2550 3750
Wire Wire Line
	2550 3750 2600 3750
Connection ~ 2450 4050
$Comp
L Cluster-rescue:VSS-power #PWR027
U 1 1 5DC01710
P 3300 7400
F 0 "#PWR027" H 3300 7250 50  0001 C CNN
F 1 "VSS" V 3317 7528 50  0000 L CNN
F 2 "" H 3300 7400 50  0001 C CNN
F 3 "" H 3300 7400 50  0001 C CNN
	1    3300 7400
	1    0    0    -1  
$EndComp
$Comp
L Cluster-rescue:VCC-power #PWR030
U 1 1 5DC01CFB
P 3550 7400
F 0 "#PWR030" H 3550 7250 50  0001 C CNN
F 1 "VCC" V 3567 7528 50  0000 L CNN
F 2 "" H 3550 7400 50  0001 C CNN
F 3 "" H 3550 7400 50  0001 C CNN
	1    3550 7400
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG01
U 1 1 5DC0B85A
P 3300 7400
F 0 "#FLG01" H 3300 7475 50  0001 C CNN
F 1 "PWR_FLAG" H 3300 7573 50  0000 C CNN
F 2 "" H 3300 7400 50  0001 C CNN
F 3 "~" H 3300 7400 50  0001 C CNN
	1    3300 7400
	-1   0    0    1   
$EndComp
$Comp
L power:PWR_FLAG #FLG02
U 1 1 5DC0BD0B
P 3550 7400
F 0 "#FLG02" H 3550 7475 50  0001 C CNN
F 1 "PWR_FLAG" H 3550 7573 50  0000 C CNN
F 2 "" H 3550 7400 50  0001 C CNN
F 3 "~" H 3550 7400 50  0001 C CNN
	1    3550 7400
	-1   0    0    1   
$EndComp
NoConn ~ 2000 7550
NoConn ~ 2000 7450
NoConn ~ 2000 7350
NoConn ~ 2000 7250
NoConn ~ 2000 6850
NoConn ~ 2000 6750
NoConn ~ 2400 6550
NoConn ~ 2400 6450
NoConn ~ 2400 6150
NoConn ~ 2400 5950
NoConn ~ 2400 5850
NoConn ~ 2400 5750
NoConn ~ 2400 5650
NoConn ~ 2400 5550
NoConn ~ 2400 5450
NoConn ~ 2400 5350
NoConn ~ 2400 5250
NoConn ~ 2400 5150
NoConn ~ 2400 5050
NoConn ~ 2400 4950
NoConn ~ 2400 4850
NoConn ~ 2400 4750
NoConn ~ 2400 4650
NoConn ~ 2400 4550
NoConn ~ 2400 4450
NoConn ~ 2400 4250
NoConn ~ 2400 4150
NoConn ~ 2400 3950
$Comp
L power:VDDA #PWR072
U 1 1 5DD16176
P 9250 1300
F 0 "#PWR072" H 9250 1150 50  0001 C CNN
F 1 "VDDA" H 9267 1473 50  0000 C CNN
F 2 "" H 9250 1300 50  0001 C CNN
F 3 "" H 9250 1300 50  0001 C CNN
	1    9250 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 1550 8250 1550
$Comp
L Device:C_Small C4
U 1 1 5DD1C081
P 3700 1100
F 0 "C4" H 3792 1146 50  0000 L CNN
F 1 "100nF" H 3792 1055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3700 1100 50  0001 C CNN
F 3 "~" H 3700 1100 50  0001 C CNN
	1    3700 1100
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C7
U 1 1 5DD1E386
P 4100 1100
F 0 "C7" H 4192 1146 50  0000 L CNN
F 1 "100nF" H 4192 1055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4100 1100 50  0001 C CNN
F 3 "~" H 4100 1100 50  0001 C CNN
	1    4100 1100
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C9
U 1 1 5DD1E865
P 4500 1100
F 0 "C9" H 4592 1146 50  0000 L CNN
F 1 "100nF" H 4592 1055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4500 1100 50  0001 C CNN
F 3 "~" H 4500 1100 50  0001 C CNN
	1    4500 1100
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C11
U 1 1 5DD1EB44
P 4900 1100
F 0 "C11" H 4992 1146 50  0000 L CNN
F 1 "100nF" H 4992 1055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4900 1100 50  0001 C CNN
F 3 "~" H 4900 1100 50  0001 C CNN
	1    4900 1100
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C14
U 1 1 5DD1EDFE
P 5300 1100
F 0 "C14" H 5392 1146 50  0000 L CNN
F 1 "100nF" H 5392 1055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5300 1100 50  0001 C CNN
F 3 "~" H 5300 1100 50  0001 C CNN
	1    5300 1100
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C17
U 1 1 5DD1F15C
P 5700 1100
F 0 "C17" H 5792 1146 50  0000 L CNN
F 1 "4.7uF" H 5792 1055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5700 1100 50  0001 C CNN
F 3 "~" H 5700 1100 50  0001 C CNN
	1    5700 1100
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C5
U 1 1 5DD1F5B4
P 3700 1450
F 0 "C5" H 3792 1496 50  0000 L CNN
F 1 "100nF" H 3792 1405 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3700 1450 50  0001 C CNN
F 3 "~" H 3700 1450 50  0001 C CNN
	1    3700 1450
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C8
U 1 1 5DD1F8E4
P 4100 1450
F 0 "C8" H 4192 1496 50  0000 L CNN
F 1 "1uF" H 4192 1405 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4100 1450 50  0001 C CNN
F 3 "~" H 4100 1450 50  0001 C CNN
	1    4100 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 1000 4100 1000
Connection ~ 4100 1000
Wire Wire Line
	4100 1000 4500 1000
Connection ~ 4500 1000
Wire Wire Line
	4500 1000 4900 1000
Connection ~ 4900 1000
Wire Wire Line
	4900 1000 5300 1000
Connection ~ 5300 1000
Wire Wire Line
	5300 1000 5700 1000
Connection ~ 5700 1000
Wire Wire Line
	5700 1000 6100 1000
Wire Wire Line
	3700 1200 4100 1200
Connection ~ 4100 1200
Wire Wire Line
	4100 1200 4500 1200
Connection ~ 4500 1200
Wire Wire Line
	4500 1200 4900 1200
Connection ~ 4900 1200
Wire Wire Line
	4900 1200 5300 1200
Connection ~ 5300 1200
Wire Wire Line
	5300 1200 5700 1200
Connection ~ 5700 1200
Wire Wire Line
	5700 1200 6100 1200
$Comp
L Cluster-rescue:VCC-power #PWR053
U 1 1 5DD34ACD
P 6100 1000
F 0 "#PWR053" H 6100 850 50  0001 C CNN
F 1 "VCC" H 6117 1173 50  0000 C CNN
F 2 "" H 6100 1000 50  0001 C CNN
F 3 "" H 6100 1000 50  0001 C CNN
	1    6100 1000
	0    1    1    0   
$EndComp
$Comp
L Cluster-rescue:VSS-power #PWR054
U 1 1 5DD35403
P 6100 1200
F 0 "#PWR054" H 6100 1050 50  0001 C CNN
F 1 "VSS" V 6118 1328 50  0000 L CNN
F 2 "" H 6100 1200 50  0001 C CNN
F 3 "" H 6100 1200 50  0001 C CNN
	1    6100 1200
	0    1    1    0   
$EndComp
Wire Wire Line
	3700 1350 4100 1350
Connection ~ 4100 1350
Wire Wire Line
	4100 1350 4400 1350
Wire Wire Line
	3700 1550 4100 1550
Connection ~ 4100 1550
Wire Wire Line
	4100 1550 4400 1550
$Comp
L Cluster-rescue:VSS-power #PWR039
U 1 1 5DD4A726
P 4400 1550
F 0 "#PWR039" H 4400 1400 50  0001 C CNN
F 1 "VSS" V 4418 1678 50  0000 L CNN
F 2 "" H 4400 1550 50  0001 C CNN
F 3 "" H 4400 1550 50  0001 C CNN
	1    4400 1550
	0    1    1    0   
$EndComp
$Comp
L power:VDDA #PWR038
U 1 1 5DD4B3A8
P 4400 1350
F 0 "#PWR038" H 4400 1200 50  0001 C CNN
F 1 "VDDA" V 4417 1478 50  0000 L CNN
F 2 "" H 4400 1350 50  0001 C CNN
F 3 "" H 4400 1350 50  0001 C CNN
	1    4400 1350
	0    1    1    0   
$EndComp
$Comp
L Device:L_Small L2
U 1 1 5DD4D5BF
P 5850 1600
F 0 "L2" V 5669 1600 50  0000 C CNN
F 1 "L_Small" V 5760 1600 50  0000 C CNN
F 2 "Inductor_SMD:L_0603_1608Metric" H 5850 1600 50  0001 C CNN
F 3 "~" H 5850 1600 50  0001 C CNN
	1    5850 1600
	0    1    1    0   
$EndComp
$Comp
L Cluster-rescue:VCC-power #PWR049
U 1 1 5DD4D97D
P 5650 1600
F 0 "#PWR049" H 5650 1450 50  0001 C CNN
F 1 "VCC" H 5667 1773 50  0000 C CNN
F 2 "" H 5650 1600 50  0001 C CNN
F 3 "" H 5650 1600 50  0001 C CNN
	1    5650 1600
	0    -1   -1   0   
$EndComp
$Comp
L power:VDDA #PWR052
U 1 1 5DD4DE61
P 6050 1600
F 0 "#PWR052" H 6050 1450 50  0001 C CNN
F 1 "VDDA" V 6067 1728 50  0000 L CNN
F 2 "" H 6050 1600 50  0001 C CNN
F 3 "" H 6050 1600 50  0001 C CNN
	1    6050 1600
	0    1    1    0   
$EndComp
Wire Wire Line
	5950 1600 6050 1600
Wire Wire Line
	5650 1600 5750 1600
$Comp
L Cluster-rescue:Crystal_Small-Device Y3
U 1 1 5DD68936
P 7900 4650
F 0 "Y3" V 7854 4738 50  0000 L CNN
F 1 "32.768kHz" V 7945 4738 50  0000 L CNN
F 2 "Crystal:Crystal_SMD_2012-2Pin_2.0x1.2mm" H 7900 4650 50  0001 C CNN
F 3 "" H 7900 4650 50  0001 C CNN
	1    7900 4650
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C23
U 1 1 5DD69389
P 7600 4550
F 0 "C23" V 7371 4550 50  0000 C CNN
F 1 "1.5pF" V 7462 4550 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 7600 4550 50  0001 C CNN
F 3 "~" H 7600 4550 50  0001 C CNN
	1    7600 4550
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C24
U 1 1 5DD69783
P 7600 4750
F 0 "C24" V 7371 4750 50  0000 C CNN
F 1 "1.5pF" V 7462 4750 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 7600 4750 50  0001 C CNN
F 3 "~" H 7600 4750 50  0001 C CNN
	1    7600 4750
	0    1    1    0   
$EndComp
Wire Wire Line
	8250 4550 7900 4550
Connection ~ 7900 4550
Wire Wire Line
	7900 4550 7700 4550
Wire Wire Line
	8250 4650 8150 4650
Wire Wire Line
	8150 4650 8150 4750
Wire Wire Line
	8150 4750 7900 4750
Connection ~ 7900 4750
Wire Wire Line
	7900 4750 7700 4750
Wire Wire Line
	7500 4550 7400 4550
Wire Wire Line
	7400 4550 7400 4650
Wire Wire Line
	7400 4750 7500 4750
$Comp
L Cluster-rescue:VSS-power #PWR062
U 1 1 5DD8C915
P 7400 4650
F 0 "#PWR062" H 7400 4500 50  0001 C CNN
F 1 "VSS" H 7418 4823 50  0000 C CNN
F 2 "" H 7400 4650 50  0001 C CNN
F 3 "" H 7400 4650 50  0001 C CNN
	1    7400 4650
	0    -1   -1   0   
$EndComp
Connection ~ 7400 4650
Wire Wire Line
	7400 4650 7400 4750
$Comp
L Device:LED_Small D4
U 1 1 5DD9C6C0
P 6400 6900
F 0 "D4" V 6446 6832 50  0000 R CNN
F 1 "Pwr Ind" V 6355 6832 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" V 6400 6900 50  0001 C CNN
F 3 "~" V 6400 6900 50  0001 C CNN
	1    6400 6900
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_Small R7
U 1 1 5DD9C058
P 6400 6650
F 0 "R7" H 6459 6696 50  0000 L CNN
F 1 "330" H 6459 6605 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 6400 6650 50  0001 C CNN
F 3 "~" H 6400 6650 50  0001 C CNN
	1    6400 6650
	1    0    0    -1  
$EndComp
$Comp
L Cluster-rescue:VSS-power #PWR056
U 1 1 5DDA0BC8
P 6400 7050
F 0 "#PWR056" H 6400 6900 50  0001 C CNN
F 1 "VSS" H 6418 7223 50  0000 C CNN
F 2 "" H 6400 7050 50  0001 C CNN
F 3 "" H 6400 7050 50  0001 C CNN
	1    6400 7050
	-1   0    0    1   
$EndComp
Wire Wire Line
	6400 7050 6400 7000
Wire Wire Line
	6400 6800 6400 6750
Wire Wire Line
	6400 6550 6400 6500
Connection ~ 6400 6500
Wire Wire Line
	6400 6500 6550 6500
$Comp
L power:VDDA #PWR031
U 1 1 5DDCF714
P 3800 7400
F 0 "#PWR031" H 3800 7250 50  0001 C CNN
F 1 "VDDA" H 3817 7573 50  0000 C CNN
F 2 "" H 3800 7400 50  0001 C CNN
F 3 "" H 3800 7400 50  0001 C CNN
	1    3800 7400
	1    0    0    -1  
$EndComp
Connection ~ 3550 7400
$Comp
L Cluster-rescue:+12V-power #PWR032
U 1 1 5DDD80CB
P 4000 7400
F 0 "#PWR032" H 4000 7250 50  0001 C CNN
F 1 "+12V" V 4015 7528 50  0000 L CNN
F 2 "" H 4000 7400 50  0001 C CNN
F 3 "" H 4000 7400 50  0001 C CNN
	1    4000 7400
	1    0    0    -1  
$EndComp
Text GLabel 8250 1750 0    50   Input ~ 0
Boot0
Text GLabel 1050 4250 2    50   Input ~ 0
Boot0
$Comp
L Connector_Generic:Conn_01x08 J4
U 1 1 5DDE2D1A
P 850 4650
F 0 "J4" H 768 4025 50  0000 C CNN
F 1 "Programming" H 768 4116 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x08_P2.54mm_Vertical" H 850 4650 50  0001 C CNN
F 3 "~" H 850 4650 50  0001 C CNN
	1    850  4650
	-1   0    0    1   
$EndComp
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 5DE29893
P 3800 7400
F 0 "#FLG0101" H 3800 7475 50  0001 C CNN
F 1 "PWR_FLAG" H 3800 7573 50  0000 C CNN
F 2 "" H 3800 7400 50  0001 C CNN
F 3 "~" H 3800 7400 50  0001 C CNN
	1    3800 7400
	-1   0    0    1   
$EndComp
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 5DE29A89
P 4000 7400
F 0 "#FLG0102" H 4000 7475 50  0001 C CNN
F 1 "PWR_FLAG" H 4000 7573 50  0000 C CNN
F 2 "" H 4000 7400 50  0001 C CNN
F 3 "~" H 4000 7400 50  0001 C CNN
	1    4000 7400
	-1   0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x11 J1
U 1 1 5DEDE3ED
P 850 1900
F 0 "J1" H 768 1175 50  0000 C CNN
F 1 "FairingConnector" H 768 1266 50  0000 C CNN
F 2 "TerminalBlock_RND:TerminalBlock_RND_205-00021_1x11_P5.00mm_Horizontal" H 850 1900 50  0001 C CNN
F 3 "~" H 850 1900 50  0001 C CNN
	1    850  1900
	-1   0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J15
U 1 1 5DEF3A21
P 2050 1950
F 0 "J15" H 1968 1625 50  0000 C CNN
F 1 "Conn_01x02" H 1968 1716 50  0000 C CNN
F 2 "TerminalBlock_RND:TerminalBlock_RND_205-00012_1x02_P5.00mm_Horizontal" H 2050 1950 50  0001 C CNN
F 3 "~" H 2050 1950 50  0001 C CNN
	1    2050 1950
	-1   0    0    1   
$EndComp
$Comp
L Cluster-rescue:VSS-power #PWR0101
U 1 1 5DEFA079
P 2250 1850
F 0 "#PWR0101" H 2250 1700 50  0001 C CNN
F 1 "VSS" V 2267 1978 50  0000 L CNN
F 2 "" H 2250 1850 50  0001 C CNN
F 3 "" H 2250 1850 50  0001 C CNN
	1    2250 1850
	0    1    1    0   
$EndComp
$Comp
L Connector_Generic:Conn_01x06 J6
U 1 1 5DF02103
P 850 6500
F 0 "J6" H 768 5975 50  0000 C CNN
F 1 "Conn_01x06" H 768 6066 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 850 6500 50  0001 C CNN
F 3 "~" H 850 6500 50  0001 C CNN
	1    850  6500
	-1   0    0    1   
$EndComp
$EndSCHEMATC
