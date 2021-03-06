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
Text Notes 7100 6950 0    100  ~ 0
CAN FD Physical-layer expansion board\nfor NUCLEO-G474RE
Wire Wire Line
	7800 1900 7100 1900
Wire Wire Line
	7800 1800 7800 1900
Wire Wire Line
	7800 2100 7800 2050
Wire Wire Line
	7800 2050 7100 2050
$Comp
L Device:R R6
U 1 1 5E5922A9
P 9000 1750
F 0 "R6" H 9070 1796 50  0000 L CNN
F 1 "62R" H 9070 1705 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 8930 1750 50  0001 C CNN
F 3 "~" H 9000 1750 50  0001 C CNN
	1    9000 1750
	1    0    0    -1  
$EndComp
$Comp
L Device:R R7
U 1 1 5E592547
P 9000 2150
F 0 "R7" H 9070 2196 50  0000 L CNN
F 1 "62R" H 9070 2105 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 8930 2150 50  0001 C CNN
F 3 "~" H 9000 2150 50  0001 C CNN
	1    9000 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	8650 2300 9000 2300
Wire Wire Line
	8650 1800 8650 1600
Wire Wire Line
	8650 1600 9000 1600
Wire Wire Line
	8650 2100 8650 2300
Text GLabel 9750 2300 3    50   BiDi ~ 0
CAN1_L
Text GLabel 9750 1600 1    50   BiDi ~ 0
CAN1_H
$Comp
L power:+3.3V #PWR023
U 1 1 5E5A6B3A
P 7250 2200
F 0 "#PWR023" H 7250 2050 50  0001 C CNN
F 1 "+3.3V" V 7265 2328 50  0000 L CNN
F 2 "" H 7250 2200 50  0001 C CNN
F 3 "" H 7250 2200 50  0001 C CNN
	1    7250 2200
	0    1    1    0   
$EndComp
$Comp
L Device:C C7
U 1 1 5E5AD813
P 7200 2350
F 0 "C7" H 7315 2396 50  0000 L CNN
F 1 "100n" H 7315 2305 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 7238 2200 50  0001 C CNN
F 3 "~" H 7200 2350 50  0001 C CNN
	1    7200 2350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5E5B15EA
P 4500 1800
F 0 "C1" H 4615 1846 50  0000 L CNN
F 1 "100n" H 4615 1755 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 4538 1650 50  0001 C CNN
F 3 "~" H 4500 1800 50  0001 C CNN
	1    4500 1800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR09
U 1 1 5E5B374E
P 4500 2000
F 0 "#PWR09" H 4500 1750 50  0001 C CNN
F 1 "GND" H 4505 1827 50  0000 C CNN
F 2 "" H 4500 2000 50  0001 C CNN
F 3 "" H 4500 2000 50  0001 C CNN
	1    4500 2000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR020
U 1 1 5E5B4BCF
P 7200 2500
F 0 "#PWR020" H 7200 2250 50  0001 C CNN
F 1 "GND" H 7205 2327 50  0000 C CNN
F 2 "" H 7200 2500 50  0001 C CNN
F 3 "" H 7200 2500 50  0001 C CNN
	1    7200 2500
	1    0    0    -1  
$EndComp
Text GLabel 5250 2200 0    50   Output ~ 0
RX1
Text GLabel 5250 2050 0    50   Input ~ 0
TX1
$Comp
L power:+5V #PWR08
U 1 1 5E5A8D56
P 4500 1650
F 0 "#PWR08" H 4500 1500 50  0001 C CNN
F 1 "+5V" H 4515 1823 50  0000 C CNN
F 2 "" H 4500 1650 50  0001 C CNN
F 3 "" H 4500 1650 50  0001 C CNN
	1    4500 1650
	1    0    0    -1  
$EndComp
Connection ~ 4500 1650
Wire Wire Line
	4500 1950 5000 1950
Wire Wire Line
	4500 1950 4500 2000
Connection ~ 4500 1950
Wire Wire Line
	7200 2200 7100 2200
Wire Wire Line
	7200 2200 7250 2200
Connection ~ 7200 2200
Connection ~ 9000 1950
Wire Wire Line
	9000 1950 9000 2000
Wire Wire Line
	9000 1900 9000 1950
$Comp
L power:GND #PWR029
U 1 1 5E59789F
P 9750 2000
F 0 "#PWR029" H 9750 1750 50  0001 C CNN
F 1 "GND" V 9755 1872 50  0000 R CNN
F 2 "" H 9750 2000 50  0001 C CNN
F 3 "" H 9750 2000 50  0001 C CNN
	1    9750 2000
	1    0    0    -1  
$EndComp
$Comp
L Device:C C13
U 1 1 5E593691
P 9500 1950
F 0 "C13" V 9752 1950 50  0000 C CNN
F 1 "4.7n" V 9661 1950 50  0000 C CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 9538 1800 50  0001 C CNN
F 3 "~" H 9500 1950 50  0001 C CNN
	1    9500 1950
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9000 1950 9350 1950
Text GLabel 6950 1350 0    50   Input ~ 0
STANDBY_1
Wire Wire Line
	4500 1650 5000 1650
Wire Wire Line
	5500 1900 5000 1900
Wire Wire Line
	5000 1900 5000 1950
Wire Wire Line
	5500 1750 5000 1750
Wire Wire Line
	5000 1750 5000 1650
Wire Wire Line
	5250 2050 5500 2050
Text GLabel 1350 2400 2    50   Output ~ 0
TX1
Text GLabel 1350 2550 2    50   Input ~ 0
RX2
Text GLabel 1350 2650 2    50   Output ~ 0
TX2
Text GLabel 1350 2800 2    50   Input ~ 0
RX3
Text GLabel 1350 2900 2    50   Output ~ 0
TX3
Text GLabel 1350 3150 2    50   BiDi ~ 0
D-
Text GLabel 1350 3050 2    50   BiDi ~ 0
D+
Wire Wire Line
	7800 3800 7100 3800
Wire Wire Line
	7800 3700 7800 3800
Wire Wire Line
	7800 4000 7800 3950
Wire Wire Line
	7800 3950 7100 3950
$Comp
L Device:R R8
U 1 1 5E635F1A
P 9000 3650
F 0 "R8" H 9070 3696 50  0000 L CNN
F 1 "62R" H 9070 3605 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 8930 3650 50  0001 C CNN
F 3 "~" H 9000 3650 50  0001 C CNN
	1    9000 3650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R9
U 1 1 5E635F20
P 9000 4050
F 0 "R9" H 9070 4096 50  0000 L CNN
F 1 "62R" H 9070 4005 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 8930 4050 50  0001 C CNN
F 3 "~" H 9000 4050 50  0001 C CNN
	1    9000 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	8650 4200 9000 4200
Wire Wire Line
	8650 3700 8650 3500
Wire Wire Line
	8650 3500 9000 3500
Wire Wire Line
	8650 4000 8650 4200
Text GLabel 9750 4200 3    50   BiDi ~ 0
CAN2_L
Text GLabel 9750 3500 1    50   BiDi ~ 0
CAN2_H
$Comp
L power:+3.3V #PWR024
U 1 1 5E635F2E
P 7250 4100
F 0 "#PWR024" H 7250 3950 50  0001 C CNN
F 1 "+3.3V" V 7265 4228 50  0000 L CNN
F 2 "" H 7250 4100 50  0001 C CNN
F 3 "" H 7250 4100 50  0001 C CNN
	1    7250 4100
	0    1    1    0   
$EndComp
$Comp
L Device:C C8
U 1 1 5E635F34
P 7200 4250
F 0 "C8" H 7315 4296 50  0000 L CNN
F 1 "100n" H 7315 4205 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 7238 4100 50  0001 C CNN
F 3 "~" H 7200 4250 50  0001 C CNN
	1    7200 4250
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 5E635F3A
P 4650 3700
F 0 "C2" H 4765 3746 50  0000 L CNN
F 1 "100n" H 4765 3655 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 4688 3550 50  0001 C CNN
F 3 "~" H 4650 3700 50  0001 C CNN
	1    4650 3700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR011
U 1 1 5E635F40
P 4650 3900
F 0 "#PWR011" H 4650 3650 50  0001 C CNN
F 1 "GND" H 4655 3727 50  0000 C CNN
F 2 "" H 4650 3900 50  0001 C CNN
F 3 "" H 4650 3900 50  0001 C CNN
	1    4650 3900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR021
U 1 1 5E635F46
P 7200 4400
F 0 "#PWR021" H 7200 4150 50  0001 C CNN
F 1 "GND" H 7205 4227 50  0000 C CNN
F 2 "" H 7200 4400 50  0001 C CNN
F 3 "" H 7200 4400 50  0001 C CNN
	1    7200 4400
	1    0    0    -1  
$EndComp
Text GLabel 5250 4100 0    50   Output ~ 0
RX2
Text GLabel 5250 3950 0    50   Input ~ 0
TX2
$Comp
L power:+5V #PWR010
U 1 1 5E635F4E
P 4650 3550
F 0 "#PWR010" H 4650 3400 50  0001 C CNN
F 1 "+5V" H 4665 3723 50  0000 C CNN
F 2 "" H 4650 3550 50  0001 C CNN
F 3 "" H 4650 3550 50  0001 C CNN
	1    4650 3550
	1    0    0    -1  
$EndComp
Connection ~ 4650 3550
Wire Wire Line
	4650 3850 5000 3850
Wire Wire Line
	4650 3850 4650 3900
Connection ~ 4650 3850
Wire Wire Line
	7200 4100 7100 4100
Wire Wire Line
	7200 4100 7250 4100
Connection ~ 7200 4100
Connection ~ 9000 3850
Wire Wire Line
	9000 3850 9000 3900
Wire Wire Line
	9000 3800 9000 3850
$Comp
L power:GND #PWR030
U 1 1 5E635F5E
P 9750 3850
F 0 "#PWR030" H 9750 3600 50  0001 C CNN
F 1 "GND" V 9755 3722 50  0000 R CNN
F 2 "" H 9750 3850 50  0001 C CNN
F 3 "" H 9750 3850 50  0001 C CNN
	1    9750 3850
	1    0    0    -1  
$EndComp
$Comp
L Device:C C14
U 1 1 5E635F64
P 9500 3850
F 0 "C14" V 9752 3850 50  0000 C CNN
F 1 "4.7n" V 9661 3850 50  0000 C CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 9538 3700 50  0001 C CNN
F 3 "~" H 9500 3850 50  0001 C CNN
	1    9500 3850
	0    -1   -1   0   
$EndComp
Connection ~ 9000 3500
Wire Wire Line
	9000 3850 9350 3850
Text GLabel 6950 3250 0    50   Input ~ 0
STANDBY_2
Wire Wire Line
	4650 3550 5000 3550
Wire Wire Line
	5500 3800 5000 3800
Wire Wire Line
	5000 3800 5000 3850
Wire Wire Line
	5500 3650 5000 3650
Wire Wire Line
	5000 3650 5000 3550
$Comp
L canfd-usb-nucleo:ATA6561-GAQW-N U3
U 1 1 5E635F75
P 5500 3700
F 0 "U3" H 6300 4050 60  0000 C CNN
F 1 "ATA6561-GAQW-N" H 6300 3950 60  0000 C CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 6300 3940 60  0001 C CNN
F 3 "" H 5500 3700 60  0000 C CNN
	1    5500 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5250 3950 5500 3950
Wire Wire Line
	7800 5500 7100 5500
Wire Wire Line
	7800 5400 7800 5500
Wire Wire Line
	7800 5700 7800 5650
Wire Wire Line
	7800 5650 7100 5650
$Comp
L Device:R R10
U 1 1 5E64424A
P 9000 5350
F 0 "R10" H 9070 5396 50  0000 L CNN
F 1 "62R" H 9070 5305 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 8930 5350 50  0001 C CNN
F 3 "~" H 9000 5350 50  0001 C CNN
	1    9000 5350
	1    0    0    -1  
$EndComp
$Comp
L Device:R R11
U 1 1 5E644250
P 9000 5750
F 0 "R11" H 9070 5796 50  0000 L CNN
F 1 "62R" H 9070 5705 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 8930 5750 50  0001 C CNN
F 3 "~" H 9000 5750 50  0001 C CNN
	1    9000 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	8650 5900 9000 5900
Wire Wire Line
	8650 5400 8650 5200
Wire Wire Line
	8650 5200 9000 5200
Wire Wire Line
	8650 5700 8650 5900
Text GLabel 9750 5900 3    50   BiDi ~ 0
CAN3_L
Text GLabel 9750 5200 1    50   BiDi ~ 0
CAN3_H
$Comp
L power:+3.3V #PWR025
U 1 1 5E64425E
P 7250 5800
F 0 "#PWR025" H 7250 5650 50  0001 C CNN
F 1 "+3.3V" V 7265 5928 50  0000 L CNN
F 2 "" H 7250 5800 50  0001 C CNN
F 3 "" H 7250 5800 50  0001 C CNN
	1    7250 5800
	0    1    1    0   
$EndComp
$Comp
L Device:C C9
U 1 1 5E644264
P 7200 5950
F 0 "C9" H 7315 5996 50  0000 L CNN
F 1 "100n" H 7315 5905 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 7238 5800 50  0001 C CNN
F 3 "~" H 7200 5950 50  0001 C CNN
	1    7200 5950
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5E64426A
P 4500 5400
F 0 "C3" H 4615 5446 50  0000 L CNN
F 1 "100n" H 4615 5355 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 4538 5250 50  0001 C CNN
F 3 "~" H 4500 5400 50  0001 C CNN
	1    4500 5400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR013
U 1 1 5E644270
P 4500 5600
F 0 "#PWR013" H 4500 5350 50  0001 C CNN
F 1 "GND" H 4505 5427 50  0000 C CNN
F 2 "" H 4500 5600 50  0001 C CNN
F 3 "" H 4500 5600 50  0001 C CNN
	1    4500 5600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR022
U 1 1 5E644276
P 7200 6100
F 0 "#PWR022" H 7200 5850 50  0001 C CNN
F 1 "GND" H 7205 5927 50  0000 C CNN
F 2 "" H 7200 6100 50  0001 C CNN
F 3 "" H 7200 6100 50  0001 C CNN
	1    7200 6100
	1    0    0    -1  
$EndComp
Text GLabel 5250 5800 0    50   Output ~ 0
RX3
Text GLabel 5250 5650 0    50   Input ~ 0
TX3
$Comp
L power:+5V #PWR012
U 1 1 5E64427E
P 4500 5250
F 0 "#PWR012" H 4500 5100 50  0001 C CNN
F 1 "+5V" H 4515 5423 50  0000 C CNN
F 2 "" H 4500 5250 50  0001 C CNN
F 3 "" H 4500 5250 50  0001 C CNN
	1    4500 5250
	1    0    0    -1  
$EndComp
Connection ~ 4500 5250
Wire Wire Line
	4500 5550 5000 5550
Wire Wire Line
	4500 5550 4500 5600
Connection ~ 4500 5550
Wire Wire Line
	7200 5800 7100 5800
Wire Wire Line
	7200 5800 7250 5800
Connection ~ 7200 5800
Connection ~ 9000 5550
Wire Wire Line
	9000 5550 9000 5600
Wire Wire Line
	9000 5500 9000 5550
$Comp
L power:GND #PWR031
U 1 1 5E64428E
P 9750 5600
F 0 "#PWR031" H 9750 5350 50  0001 C CNN
F 1 "GND" V 9755 5472 50  0000 R CNN
F 2 "" H 9750 5600 50  0001 C CNN
F 3 "" H 9750 5600 50  0001 C CNN
	1    9750 5600
	1    0    0    -1  
$EndComp
$Comp
L Device:C C15
U 1 1 5E644294
P 9500 5550
F 0 "C15" V 9752 5550 50  0000 C CNN
F 1 "4.7n" V 9661 5550 50  0000 C CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 9538 5400 50  0001 C CNN
F 3 "~" H 9500 5550 50  0001 C CNN
	1    9500 5550
	0    -1   -1   0   
$EndComp
Connection ~ 9000 5200
Connection ~ 9000 5900
Wire Wire Line
	9000 5550 9350 5550
Text GLabel 6950 4950 0    50   Input ~ 0
STANDBY_3
Wire Wire Line
	4500 5250 5000 5250
Wire Wire Line
	5500 5500 5000 5500
Wire Wire Line
	5000 5500 5000 5550
Wire Wire Line
	5500 5350 5000 5350
Wire Wire Line
	5000 5350 5000 5250
$Comp
L canfd-usb-nucleo:ATA6561-GAQW-N U4
U 1 1 5E6442A5
P 5500 5400
F 0 "U4" H 6300 5750 60  0000 C CNN
F 1 "ATA6561-GAQW-N" H 6300 5650 60  0000 C CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 6300 5640 60  0001 C CNN
F 3 "" H 5500 5400 60  0000 C CNN
	1    5500 5400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5250 5650 5500 5650
Wire Wire Line
	1150 1700 1300 1700
$Comp
L power:+3.3V #PWR06
U 1 1 5E697522
P 1300 900
F 0 "#PWR06" H 1300 750 50  0001 C CNN
F 1 "+3.3V" V 1291 1000 50  0000 L CNN
F 2 "" H 1300 900 50  0001 C CNN
F 3 "" H 1300 900 50  0001 C CNN
	1    1300 900 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR027
U 1 1 5E6CB4E2
P 7750 3300
F 0 "#PWR027" H 7750 3050 50  0001 C CNN
F 1 "GND" H 7838 3263 50  0000 L CNN
F 2 "" H 7750 3300 50  0001 C CNN
F 3 "" H 7750 3300 50  0001 C CNN
	1    7750 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 3250 7750 3300
Text Notes 7850 1050 0    50   ~ 0
Solder jumper or 0R resistor\nfor STANDBY mode
$Comp
L power:+5V #PWR02
U 1 1 5E7BB094
P 2000 4950
F 0 "#PWR02" H 2000 4800 50  0001 C CNN
F 1 "+5V" H 2015 5123 50  0000 C CNN
F 2 "" H 2000 4950 50  0001 C CNN
F 3 "" H 2000 4950 50  0001 C CNN
	1    2000 4950
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push_Dual_x2 SW1
U 1 1 5E5E7AB8
P 3850 1750
F 0 "SW1" H 3950 1950 50  0000 R CNN
F 1 "SW_Push_Dual_x2" H 3850 1944 50  0001 C CNN
F 2 "Buttons_Switches_THT:SW_PUSH_6mm" H 3850 1950 50  0001 C CNN
F 3 "~" H 3850 1950 50  0001 C CNN
	1    3850 1750
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Push_Dual_x2 SW2
U 1 1 5E5F1893
P 3100 1750
F 0 "SW2" H 3200 1900 50  0000 R CNN
F 1 "SW_Push_Dual_x2" H 3100 1944 50  0001 C CNN
F 2 "Buttons_Switches_THT:SW_PUSH_6mm" H 3100 1950 50  0001 C CNN
F 3 "~" H 3100 1950 50  0001 C CNN
	1    3100 1750
	0    -1   -1   0   
$EndComp
$Comp
L canfd-usb-nucleo:Conn_02x19_Odd_Even CN7
U 4 1 5E6737E0
P 900 1600
F 0 "CN7" H 979 1915 50  0000 C CNN
F 1 "POWER" H 979 1824 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x19_Pitch2.54mm" H 400 1150 50  0001 C CNN
F 3 "" H 400 1150 50  0001 C CNN
	4    900  1600
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push_Dual_x2 SW3
U 1 1 5E6B0662
P 2400 1750
F 0 "SW3" H 2500 1950 50  0000 R CNN
F 1 "SW_Push_Dual_x2" H 2400 1944 50  0001 C CNN
F 2 "Buttons_Switches_THT:SW_PUSH_6mm" H 2400 1950 50  0001 C CNN
F 3 "~" H 2400 1950 50  0001 C CNN
	1    2400 1750
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 5E7E626A
P 3100 2400
F 0 "#PWR0101" H 3100 2150 50  0001 C CNN
F 1 "GND" H 3105 2227 50  0000 C CNN
F 2 "" H 3100 2400 50  0001 C CNN
F 3 "" H 3100 2400 50  0001 C CNN
	1    3100 2400
	1    0    0    -1  
$EndComp
$Comp
L Device:R R14
U 1 1 5E7C3EB8
P 3100 2200
F 0 "R14" H 3170 2246 50  0000 L CNN
F 1 "10k" H 3170 2155 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3030 2200 50  0001 C CNN
F 3 "~" H 3100 2200 50  0001 C CNN
	1    3100 2200
	1    0    0    -1  
$EndComp
$Comp
L Device:R R13
U 1 1 5E720659
P 2400 2200
F 0 "R13" H 2470 2246 50  0000 L CNN
F 1 "10k" H 2470 2155 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2330 2200 50  0001 C CNN
F 3 "~" H 2400 2200 50  0001 C CNN
	1    2400 2200
	1    0    0    -1  
$EndComp
$Comp
L Device:R R19
U 1 1 5E714CFE
P 3850 2200
F 0 "R19" H 3920 2246 50  0000 L CNN
F 1 "10k" H 3920 2155 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3780 2200 50  0001 C CNN
F 3 "~" H 3850 2200 50  0001 C CNN
	1    3850 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3850 2350 3850 2400
Wire Wire Line
	3100 2350 3100 2400
Wire Wire Line
	2400 2350 2400 2400
Wire Wire Line
	3100 1550 3100 1200
Wire Wire Line
	2400 1550 2400 1200
$Comp
L Device:C C17
U 1 1 5E810FED
P 3300 1700
F 0 "C17" H 3415 1746 50  0000 L CNN
F 1 "100n" H 3415 1655 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 3338 1550 50  0001 C CNN
F 3 "~" H 3300 1700 50  0001 C CNN
	1    3300 1700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C16
U 1 1 5E80B465
P 2600 1700
F 0 "C16" H 2715 1746 50  0000 L CNN
F 1 "100n" H 2715 1655 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 2638 1550 50  0001 C CNN
F 3 "~" H 2600 1700 50  0001 C CNN
	1    2600 1700
	1    0    0    -1  
$EndComp
Text GLabel 1350 2300 2    50   Input ~ 0
RX1
Text GLabel 3350 1950 2    50   Output ~ 0
SW2
Text GLabel 4100 1950 2    50   Output ~ 0
SW3
Wire Wire Line
	2400 1950 2400 2050
Wire Wire Line
	3100 1950 3100 2050
Wire Wire Line
	3850 1950 3850 2050
Wire Wire Line
	2600 1950 2600 1850
Connection ~ 2400 1950
Wire Wire Line
	4050 1850 4050 1950
Wire Wire Line
	4050 1950 3850 1950
Connection ~ 3850 1950
Wire Wire Line
	3300 1850 3300 1950
Wire Wire Line
	3300 1950 3100 1950
Connection ~ 3100 1950
Wire Wire Line
	4100 1950 4050 1950
Connection ~ 4050 1950
Wire Wire Line
	3350 1950 3300 1950
Connection ~ 3300 1950
Wire Wire Line
	2600 1950 2400 1950
Wire Wire Line
	2650 1950 2600 1950
Connection ~ 2600 1950
Wire Wire Line
	2400 1200 2600 1200
Connection ~ 3100 1200
$Comp
L Device:C C18
U 1 1 5E816B78
P 4050 1700
F 0 "C18" H 4165 1746 50  0000 L CNN
F 1 "100n" H 4165 1655 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 4088 1550 50  0001 C CNN
F 3 "~" H 4050 1700 50  0001 C CNN
	1    4050 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3850 1200 3850 1550
Wire Wire Line
	3100 1200 3300 1200
Wire Wire Line
	3100 2400 3850 2400
Connection ~ 3100 2400
Wire Wire Line
	2400 2400 3100 2400
$Comp
L Device:R R15
U 1 1 5ECBC8B5
P 3250 3600
F 0 "R15" H 3320 3646 50  0000 L CNN
F 1 "510R" H 3320 3555 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3180 3600 50  0001 C CNN
F 3 "~" H 3250 3600 50  0001 C CNN
	1    3250 3600
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R16
U 1 1 5ECC4436
P 3250 3950
F 0 "R16" H 3320 3996 50  0000 L CNN
F 1 "510R" H 3320 3905 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3180 3950 50  0001 C CNN
F 3 "~" H 3250 3950 50  0001 C CNN
	1    3250 3950
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R17
U 1 1 5ECCC03E
P 3250 4350
F 0 "R17" H 3320 4396 50  0000 L CNN
F 1 "510R" H 3320 4305 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3180 4350 50  0001 C CNN
F 3 "~" H 3250 4350 50  0001 C CNN
	1    3250 4350
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R18
U 1 1 5ECD3AAA
P 3250 4750
F 0 "R18" H 3320 4796 50  0000 L CNN
F 1 "510R" H 3320 4705 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3180 4750 50  0001 C CNN
F 3 "~" H 3250 4750 50  0001 C CNN
	1    3250 4750
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 5ECFD823
P 3850 4850
F 0 "#PWR0102" H 3850 4600 50  0001 C CNN
F 1 "GND" H 3855 4677 50  0000 C CNN
F 2 "" H 3850 4850 50  0001 C CNN
F 3 "" H 3850 4850 50  0001 C CNN
	1    3850 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 3600 3850 3600
Wire Wire Line
	3850 3600 3850 3950
Wire Wire Line
	3800 4750 3850 4750
Connection ~ 3850 4750
Wire Wire Line
	3850 4750 3850 4850
Wire Wire Line
	3800 4350 3850 4350
Connection ~ 3850 4350
Wire Wire Line
	3850 4350 3850 4750
Wire Wire Line
	3800 3950 3850 3950
Connection ~ 3850 3950
Wire Wire Line
	3850 3950 3850 4350
Wire Wire Line
	2950 3600 3100 3600
Wire Wire Line
	2950 3950 3100 3950
Wire Wire Line
	2950 4350 3100 4350
Wire Wire Line
	2950 4750 3100 4750
Wire Wire Line
	3400 3600 3500 3600
Wire Wire Line
	3400 3950 3500 3950
Wire Wire Line
	3500 4350 3400 4350
Wire Wire Line
	3500 4750 3400 4750
$Comp
L Device:LED D1
U 1 1 5ECB48C5
P 3650 3600
F 0 "D1" H 3643 3345 50  0000 C CNN
F 1 "HEARTBEAT" H 3643 3436 50  0000 C CNN
F 2 "LEDs:LED_D3.0mm" H 3650 3600 50  0001 C CNN
F 3 "~" H 3650 3600 50  0001 C CNN
	1    3650 3600
	-1   0    0    1   
$EndComp
$Comp
L Device:LED D2
U 1 1 5EC988AD
P 3650 3950
F 0 "D2" H 3643 3695 50  0000 C CNN
F 1 "STATUS1" H 3643 3786 50  0000 C CNN
F 2 "LEDs:LED_D3.0mm" H 3650 3950 50  0001 C CNN
F 3 "~" H 3650 3950 50  0001 C CNN
	1    3650 3950
	-1   0    0    1   
$EndComp
$Comp
L Device:LED D3
U 1 1 5ECA4EF4
P 3650 4350
F 0 "D3" H 3643 4095 50  0000 C CNN
F 1 "STATUS2" H 3643 4186 50  0000 C CNN
F 2 "LEDs:LED_D3.0mm" H 3650 4350 50  0001 C CNN
F 3 "~" H 3650 4350 50  0001 C CNN
	1    3650 4350
	-1   0    0    1   
$EndComp
$Comp
L Device:LED D4
U 1 1 5ECAC84E
P 3650 4750
F 0 "D4" H 3643 4495 50  0000 C CNN
F 1 "STATUS3" H 3643 4586 50  0000 C CNN
F 2 "LEDs:LED_D3.0mm" H 3650 4750 50  0001 C CNN
F 3 "~" H 3650 4750 50  0001 C CNN
	1    3650 4750
	-1   0    0    1   
$EndComp
$Comp
L canfd-usb-nucleo:Conn_02x19_Odd_Even CN10
U 5 1 5E62EF55
P 1050 4550
F 0 "CN10" H 1050 5550 50  0000 C CNN
F 1 "CONN" H 1132 5576 50  0001 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x19_Pitch2.54mm" H 2100 4850 50  0001 C CNN
F 3 "" H 2100 4850 50  0001 C CNN
	5    1050 4550
	1    0    0    -1  
$EndComp
Text GLabel 1350 3650 2    50   Input ~ 0
SW1
Text GLabel 1350 3750 2    50   Input ~ 0
SW2
Text GLabel 1350 3850 2    50   Input ~ 0
SW3
Text GLabel 1350 3950 2    50   Output ~ 0
LED1
Text GLabel 1350 4050 2    50   Output ~ 0
LED2
Text GLabel 1350 4150 2    50   Output ~ 0
LED3
Text GLabel 1350 4250 2    50   Output ~ 0
LED4
Text GLabel 2950 3600 0    50   Input ~ 0
LED1
Text GLabel 2950 3950 0    50   Input ~ 0
LED2
Text GLabel 2950 4350 0    50   Input ~ 0
LED3
Text GLabel 2950 4750 0    50   Input ~ 0
LED4
Wire Wire Line
	3100 1050 3100 1200
$Comp
L canfd-usb-nucleo:Conn_02x19_Odd_Even CN10
U 3 1 5E609071
P 1000 2700
F 0 "CN10" H 1000 3200 50  0000 C CNN
F 1 "CONN" H 1083 3224 50  0001 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x19_Pitch2.54mm" H 500 2250 50  0001 C CNN
F 3 "" H 500 2250 50  0001 C CNN
	3    1000 2700
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0103
U 1 1 5E6C9151
P 3100 1050
F 0 "#PWR0103" H 3100 900 50  0001 C CNN
F 1 "+3.3V" V 3091 1150 50  0000 L CNN
F 2 "" H 3100 1050 50  0001 C CNN
F 3 "" H 3100 1050 50  0001 C CNN
	1    3100 1050
	1    0    0    -1  
$EndComp
$Comp
L canfd-usb-nucleo:ATA6561-GAQW-N U2
U 1 1 5E5B11D6
P 5500 1800
F 0 "U2" H 6300 2187 60  0000 C CNN
F 1 "ATA6561-GAQW-N" H 6300 2081 60  0000 C CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 6300 2040 60  0001 C CNN
F 3 "" H 5500 1550 60  0000 C CNN
	1    5500 1800
	1    0    0    -1  
$EndComp
$Comp
L canfd-usb-nucleo:BH_Conn_01x10 J?
U 2 1 5E642078
P 10350 5400
F 0 "J?" H 10368 5075 50  0000 C CNN
F 1 "Conn_01x10" H 10368 5166 50  0000 C CNN
F 2 "" H 10350 5400 50  0001 C CNN
F 3 "~" H 10350 5400 50  0001 C CNN
	2    10350 5400
	1    0    0    1   
$EndComp
Wire Wire Line
	10150 5500 10100 5500
Wire Wire Line
	10100 5600 10150 5600
Wire Wire Line
	9750 5600 9750 5550
Wire Wire Line
	9750 5550 9650 5550
Wire Wire Line
	10000 5700 10000 5900
Wire Wire Line
	9000 5900 10000 5900
Wire Wire Line
	10000 5400 10000 5200
Wire Wire Line
	9000 5200 10000 5200
$Comp
L canfd-usb-nucleo:BH_Conn_01x10 J?
U 2 1 5E6372F3
P 10300 3700
F 0 "J?" H 10318 3375 50  0000 C CNN
F 1 "Conn_01x10" H 10318 3466 50  0000 C CNN
F 2 "" H 10300 3700 50  0001 C CNN
F 3 "~" H 10300 3700 50  0001 C CNN
	2    10300 3700
	1    0    0    1   
$EndComp
Wire Wire Line
	10200 1900 10150 1900
Wire Wire Line
	10150 1900 10150 1950
Wire Wire Line
	10150 2000 10200 2000
Connection ~ 10150 1950
Wire Wire Line
	10150 1950 10150 2000
Wire Wire Line
	10100 5500 10100 5550
Wire Wire Line
	10000 5400 10150 5400
Wire Wire Line
	10000 5700 10150 5700
Wire Wire Line
	9750 5550 10100 5550
Connection ~ 9750 5550
Connection ~ 10100 5550
Wire Wire Line
	10100 5550 10100 5600
Wire Wire Line
	9750 3850 9650 3850
Wire Wire Line
	9750 3850 10050 3850
Wire Wire Line
	10050 3850 10050 3800
Wire Wire Line
	10050 3800 10100 3800
Connection ~ 9750 3850
Wire Wire Line
	10100 3900 10050 3900
Wire Wire Line
	10050 3900 10050 3850
Connection ~ 10050 3850
Wire Wire Line
	10100 3700 10000 3700
Wire Wire Line
	10000 3700 10000 3500
Wire Wire Line
	9000 3500 10000 3500
Wire Wire Line
	10100 4000 10000 4000
Wire Wire Line
	10000 4000 10000 4200
Wire Wire Line
	10000 4200 9000 4200
Connection ~ 9000 4200
$Comp
L canfd-usb-nucleo:BH_Conn_01x10 J?
U 2 1 5E5E91C9
P 10400 1800
F 0 "J?" H 10418 1475 50  0000 C CNN
F 1 "Conn_01x10" H 10418 1566 50  0000 C CNN
F 2 "" H 10400 1800 50  0001 C CNN
F 3 "~" H 10400 1800 50  0001 C CNN
	2    10400 1800
	1    0    0    1   
$EndComp
Wire Wire Line
	10200 2100 10100 2100
Wire Wire Line
	10100 2300 9000 2300
Wire Wire Line
	10100 2100 10100 2300
Connection ~ 9000 2300
Wire Wire Line
	9000 1600 10100 1600
Wire Wire Line
	10100 1600 10100 1800
Wire Wire Line
	10100 1800 10200 1800
Connection ~ 9000 1600
Wire Wire Line
	9650 1950 9750 1950
Wire Wire Line
	9750 2000 9750 1950
Connection ~ 9750 1950
Wire Wire Line
	9750 1950 10150 1950
$Comp
L Device:D_Schottky D?
U 1 1 5E92324D
P 2000 5150
F 0 "D?" V 1954 5229 50  0000 L CNN
F 1 "D_Schottky" V 2045 5229 50  0000 L CNN
F 2 "" H 2000 5150 50  0001 C CNN
F 3 "~" H 2000 5150 50  0001 C CNN
	1    2000 5150
	0    1    1    0   
$EndComp
Wire Wire Line
	2000 5350 2000 5300
Wire Wire Line
	2000 4950 2000 5000
Wire Wire Line
	7800 3700 8650 3700
Wire Wire Line
	7800 4000 8650 4000
Wire Wire Line
	7800 1800 8650 1800
Wire Wire Line
	7800 2100 8650 2100
Wire Wire Line
	7800 5400 8650 5400
Wire Wire Line
	7800 5700 8650 5700
Wire Wire Line
	5250 4100 5500 4100
Wire Wire Line
	5250 5800 5500 5800
Wire Wire Line
	5250 2200 5500 2200
Text GLabel 1950 4550 2    50   Output ~ 0
STANDBY_3
Text GLabel 1950 4450 2    50   Output ~ 0
STANDBY_2
Text GLabel 1950 4350 2    50   Output ~ 0
STANDBY_1
Wire Wire Line
	7100 3650 7350 3650
$Comp
L Connector:USB_C_Receptacle_USB2.0 J?
U 1 1 5E770DAD
P 1100 5950
F 0 "J?" H 1207 6817 50  0000 C CNN
F 1 "USB_C_Receptacle_USB2.0" H 1207 6726 50  0000 C CNN
F 2 "" H 1250 5950 50  0001 C CNN
F 3 "https://www.usb.org/sites/default/files/documents/usb_type-c.zip" H 1250 5950 50  0001 C CNN
	1    1100 5950
	1    0    0    -1  
$EndComp
$Comp
L Power_Protection:NUP2202 U?
U 1 1 5E775B10
P 2100 6950
F 0 "U?" H 2344 6996 50  0000 L CNN
F 1 "USBLC6-2SC6" H 2344 6905 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-363_SC-70-6" H 2180 7025 50  0001 C CNN
F 3 "http://www.onsemi.ru.com/pub_link/Collateral/NUP2202W1-D.PDF" H 2180 7025 50  0001 C CNN
	1    2100 6950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5E775B16
P 2100 7150
F 0 "#PWR?" H 2100 6900 50  0001 C CNN
F 1 "GND" H 2105 6977 50  0000 C CNN
F 2 "" H 2100 7150 50  0001 C CNN
F 3 "" H 2100 7150 50  0001 C CNN
	1    2100 7150
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 5950 1800 5950
Wire Wire Line
	1800 5850 1700 5850
Wire Wire Line
	1700 6050 1800 6050
Wire Wire Line
	1700 6150 1800 6150
Wire Wire Line
	1900 6100 1900 6950
Wire Wire Line
	1700 5350 2000 5350
Wire Wire Line
	2100 5350 2100 6750
Text GLabel 2450 6100 2    50   BiDi ~ 0
D+
Text GLabel 2450 5900 2    50   BiDi ~ 0
D-
Wire Wire Line
	2450 5900 2300 5900
Connection ~ 2000 5350
Wire Wire Line
	2000 5350 2100 5350
Connection ~ 2100 7150
Wire Wire Line
	1800 5850 1800 5900
Wire Wire Line
	1800 6050 1800 6100
Wire Wire Line
	2300 5900 1800 5900
Connection ~ 2300 5900
Connection ~ 1800 5900
Wire Wire Line
	1800 5900 1800 5950
Connection ~ 1800 6100
Wire Wire Line
	1800 6100 1800 6150
$Comp
L Device:R R?
U 1 1 5E8F963A
P 800 7000
F 0 "R?" H 869 6954 50  0000 L CNN
F 1 "220k" H 869 7045 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 730 7000 50  0001 C CNN
F 3 "~" H 800 7000 50  0001 C CNN
	1    800  7000
	-1   0    0    1   
$EndComp
$Comp
L Device:C C?
U 1 1 5E90F92E
P 1000 7000
F 0 "C?" H 1115 7046 50  0000 L CNN
F 1 "10nf" H 1115 6955 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 1038 6850 50  0001 C CNN
F 3 "~" H 1000 7000 50  0001 C CNN
	1    1000 7000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1000 6850 800  6850
Connection ~ 800  6850
Wire Wire Line
	1000 7150 800  7150
Wire Wire Line
	1000 7150 1100 7150
Connection ~ 1000 7150
Wire Wire Line
	1100 6850 1100 7150
Connection ~ 1100 7150
Wire Wire Line
	1100 7150 2100 7150
Wire Wire Line
	1800 6100 1900 6100
Connection ~ 1900 6100
Wire Wire Line
	1900 6100 2450 6100
Wire Wire Line
	2300 5900 2300 6950
Wire Wire Line
	7350 3250 7350 3650
Wire Wire Line
	1300 900  1300 1050
Wire Wire Line
	1300 1700 1300 1900
Wire Wire Line
	1300 1500 1150 1500
Connection ~ 1450 1900
Wire Wire Line
	1450 1900 1300 1900
Connection ~ 1450 1600
$Comp
L Device:CP C?
U 1 1 5EAE958F
P 1450 1750
F 0 "C?" H 1565 1796 50  0000 L CNN
F 1 "22u" H 1565 1705 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 1488 1600 50  0001 C CNN
F 3 "~" H 1450 1750 50  0001 C CNN
	1    1450 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 1600 1450 1600
Wire Wire Line
	1450 1600 1450 1350
$Comp
L power:+5V #PWR05
U 1 1 5E6835E5
P 1450 1350
F 0 "#PWR05" H 1450 1200 50  0001 C CNN
F 1 "+5V" V 1441 1522 50  0000 C CNN
F 2 "" H 1450 1350 50  0001 C CNN
F 3 "" H 1450 1350 50  0001 C CNN
	1    1450 1350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR07
U 1 1 5E67A7A7
P 1450 1900
F 0 "#PWR07" H 1450 1650 50  0001 C CNN
F 1 "GND" H 1455 1727 50  0000 C CNN
F 2 "" H 1450 1900 50  0001 C CNN
F 3 "" H 1450 1900 50  0001 C CNN
	1    1450 1900
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 5EBC17FF
P 1700 4350
F 0 "R?" V 1907 4350 50  0000 C CNN
F 1 "1k" V 1816 4350 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 1630 4350 50  0001 C CNN
F 3 "~" H 1700 4350 50  0001 C CNN
	1    1700 4350
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R?
U 1 1 5EBCD74E
P 1700 4450
F 0 "R?" V 1750 4650 50  0000 C CNN
F 1 "1k" V 1750 4300 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 1630 4450 50  0001 C CNN
F 3 "~" H 1700 4450 50  0001 C CNN
	1    1700 4450
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R?
U 1 1 5EBD9490
P 1700 4550
F 0 "R?" V 1585 4550 50  0000 C CNN
F 1 "1k" V 1494 4550 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 1630 4550 50  0001 C CNN
F 3 "~" H 1700 4550 50  0001 C CNN
	1    1700 4550
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1850 4350 1950 4350
Wire Wire Line
	1850 4450 1950 4450
Wire Wire Line
	1850 4550 1950 4550
Wire Wire Line
	1550 4350 1350 4350
Wire Wire Line
	1550 4450 1350 4450
Wire Wire Line
	1350 4550 1550 4550
Text GLabel 2650 1950 2    50   Output ~ 0
SW1
$Comp
L Device:R R?
U 1 1 5EC9FF47
P 4050 1400
F 0 "R?" H 4120 1446 50  0000 L CNN
F 1 "100R" H 4120 1355 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3980 1400 50  0001 C CNN
F 3 "~" H 4050 1400 50  0001 C CNN
	1    4050 1400
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 5ECACACF
P 2600 1400
F 0 "R?" H 2670 1446 50  0000 L CNN
F 1 "100R" H 2670 1355 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2530 1400 50  0001 C CNN
F 3 "~" H 2600 1400 50  0001 C CNN
	1    2600 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4050 1250 4050 1200
Wire Wire Line
	4050 1200 3850 1200
Connection ~ 3850 1200
Wire Wire Line
	3300 1250 3300 1200
Connection ~ 3300 1200
Wire Wire Line
	3300 1200 3850 1200
Wire Wire Line
	2600 1250 2600 1200
Connection ~ 2600 1200
Wire Wire Line
	2600 1200 3100 1200
$Comp
L Device:R R?
U 1 1 5EC8E742
P 3300 1400
F 0 "R?" H 3370 1446 50  0000 L CNN
F 1 "100R" H 3370 1355 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3230 1400 50  0001 C CNN
F 3 "~" H 3300 1400 50  0001 C CNN
	1    3300 1400
	1    0    0    -1  
$EndComp
$Comp
L Jumper:Jumper_2_Open JP?
U 1 1 5EDB1A38
P 7550 3250
F 0 "JP?" H 7550 3400 50  0000 C CNN
F 1 "Jumper_2_Open" H 7550 3394 50  0001 C CNN
F 2 "" H 7550 3250 50  0001 C CNN
F 3 "~" H 7550 3250 50  0001 C CNN
	1    7550 3250
	1    0    0    -1  
$EndComp
Text Notes 7900 3200 0    50   ~ 0
Solder jumper or 0R resistor\nfor STANDBY mode
Text Notes 7950 4800 0    50   ~ 0
Solder jumper or 0R resistor\nfor STANDBY mode
$Comp
L Jumper:Jumper_2_Open JP?
U 1 1 5EE2007F
P 7150 3250
F 0 "JP?" H 7150 3393 50  0000 C CNN
F 1 "Jumper_2_Open" H 7150 3394 50  0001 C CNN
F 2 "" H 7150 3250 50  0001 C CNN
F 3 "~" H 7150 3250 50  0001 C CNN
	1    7150 3250
	1    0    0    -1  
$EndComp
Connection ~ 7350 3250
Wire Wire Line
	7100 1750 7350 1750
$Comp
L power:GND #PWR?
U 1 1 5EED5122
P 7750 1400
F 0 "#PWR?" H 7750 1150 50  0001 C CNN
F 1 "GND" H 7838 1363 50  0000 L CNN
F 2 "" H 7750 1400 50  0001 C CNN
F 3 "" H 7750 1400 50  0001 C CNN
	1    7750 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 1350 7750 1400
Wire Wire Line
	7350 1350 7350 1750
$Comp
L Jumper:Jumper_2_Open JP?
U 1 1 5EED512A
P 7550 1350
F 0 "JP?" H 7550 1500 50  0000 C CNN
F 1 "Jumper_2_Open" H 7550 1494 50  0001 C CNN
F 2 "" H 7550 1350 50  0001 C CNN
F 3 "~" H 7550 1350 50  0001 C CNN
	1    7550 1350
	1    0    0    -1  
$EndComp
$Comp
L Jumper:Jumper_2_Open JP?
U 1 1 5EED5130
P 7150 1350
F 0 "JP?" H 7150 1493 50  0000 C CNN
F 1 "Jumper_2_Open" H 7150 1494 50  0001 C CNN
F 2 "" H 7150 1350 50  0001 C CNN
F 3 "~" H 7150 1350 50  0001 C CNN
	1    7150 1350
	1    0    0    -1  
$EndComp
Connection ~ 7350 1350
Wire Wire Line
	7100 5350 7350 5350
$Comp
L power:GND #PWR?
U 1 1 5EEFA92F
P 7750 5000
F 0 "#PWR?" H 7750 4750 50  0001 C CNN
F 1 "GND" H 7838 4963 50  0000 L CNN
F 2 "" H 7750 5000 50  0001 C CNN
F 3 "" H 7750 5000 50  0001 C CNN
	1    7750 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 4950 7750 5000
Wire Wire Line
	7350 4950 7350 5350
$Comp
L Jumper:Jumper_2_Open JP?
U 1 1 5EEFA937
P 7550 4950
F 0 "JP?" H 7550 5100 50  0000 C CNN
F 1 "Jumper_2_Open" H 7550 5094 50  0001 C CNN
F 2 "" H 7550 4950 50  0001 C CNN
F 3 "~" H 7550 4950 50  0001 C CNN
	1    7550 4950
	1    0    0    -1  
$EndComp
$Comp
L Jumper:Jumper_2_Open JP?
U 1 1 5EEFA93D
P 7150 4950
F 0 "JP?" H 7150 5093 50  0000 C CNN
F 1 "Jumper_2_Open" H 7150 5094 50  0001 C CNN
F 2 "" H 7150 4950 50  0001 C CNN
F 3 "~" H 7150 4950 50  0001 C CNN
	1    7150 4950
	1    0    0    -1  
$EndComp
Connection ~ 7350 4950
$Comp
L Device:C C?
U 1 1 5E6151B0
P 1800 1750
F 0 "C?" H 1915 1796 50  0000 L CNN
F 1 "2.2u" H 1915 1705 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 1838 1600 50  0001 C CNN
F 3 "~" H 1800 1750 50  0001 C CNN
	1    1800 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 1900 1450 1900
Wire Wire Line
	1800 1600 1800 1050
Wire Wire Line
	1800 1050 1300 1050
Connection ~ 1300 1050
Wire Wire Line
	1300 1050 1300 1500
$EndSCHEMATC
