EESchema Schematic File Version 4
LIBS:RPi_Hat-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 7
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
L jfng:RPi_GPIO J1
U 1 1 5516AE26
P 4300 2700
F 0 "J1" H 5050 2950 60  0000 C CNN
F 1 "RPi_GPIO" H 5050 2850 60  0000 C CNN
F 2 "RPi_Hat:Pin_Header_Straight_2x20" H 4300 2700 60  0001 C CNN
F 3 "" H 4300 2700 60  0000 C CNN
	1    4300 2700
	1    0    0    -1  
$EndComp
Text Notes 4600 5000 0    60   Italic 0
Thru-Hole Connector
Wire Wire Line
	3500 2700 3925 2700
Wire Wire Line
	3925 2700 4100 2700
Wire Wire Line
	3925 2700 3925 2575
$Comp
L RPi_Hat-rescue:+3.3V-RESCUE-RPi_Hat #PWR05
U 1 1 5891FB75
P 3925 2575
F 0 "#PWR05" H -25 -150 50  0001 C CNN
F 1 "+3.3V" H 3940 2748 50  0000 C CNN
F 2 "" H -25 0   50  0001 C CNN
F 3 "" H -25 0   50  0001 C CNN
	1    3925 2575
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 2700 6250 2700
Wire Wire Line
	6250 2700 6700 2700
Wire Wire Line
	6250 2600 6250 2700
Wire Wire Line
	6250 2700 6250 2800
$Comp
L RPi_Hat-rescue:+5V-RESCUE-RPi_Hat #PWR06
U 1 1 5891FBCE
P 6250 2600
F 0 "#PWR06" H 0   -150 50  0001 C CNN
F 1 "+5V" H 6265 2773 50  0000 C CNN
F 2 "" H 0   0   50  0001 C CNN
F 3 "" H 0   0   50  0001 C CNN
	1    6250 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 2800 6000 2800
Connection ~ 6250 2700
$Comp
L RPi_Hat-rescue:+3.3V-RESCUE-RPi_Hat #PWR07
U 1 1 5891FC65
P 3925 3350
F 0 "#PWR07" H -175 -300 50  0001 C CNN
F 1 "+3.3V" H 3940 3523 50  0000 C CNN
F 2 "" H -175 -150 50  0001 C CNN
F 3 "" H -175 -150 50  0001 C CNN
	1    3925 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3925 3350 3925 3500
Wire Wire Line
	3925 3500 4100 3500
Wire Wire Line
	4100 3100 3750 3100
Wire Wire Line
	3750 3100 3750 3200
$Comp
L RPi_Hat-rescue:GND-RESCUE-RPi_Hat #PWR08
U 1 1 5891FD82
P 3750 3200
F 0 "#PWR08" H 0   -250 50  0001 C CNN
F 1 "GND" H 3755 3027 50  0000 C CNN
F 2 "" H 0   0   50  0001 C CNN
F 3 "" H 0   0   50  0001 C CNN
	1    3750 3200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 3900 3750 3900
Wire Wire Line
	3750 3900 3750 4000
$Comp
L RPi_Hat-rescue:GND-RESCUE-RPi_Hat #PWR09
U 1 1 5891FDD5
P 3750 4000
F 0 "#PWR09" H 0   -250 50  0001 C CNN
F 1 "GND" H 3755 3827 50  0000 C CNN
F 2 "" H 0   0   50  0001 C CNN
F 3 "" H 0   0   50  0001 C CNN
	1    3750 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 4600 3750 4600
Wire Wire Line
	3750 4600 3750 4675
$Comp
L RPi_Hat-rescue:GND-RESCUE-RPi_Hat #PWR010
U 1 1 5891FDFC
P 3750 4675
F 0 "#PWR010" H 0   -250 50  0001 C CNN
F 1 "GND" H 3755 4502 50  0000 C CNN
F 2 "" H 0   0   50  0001 C CNN
F 3 "" H 0   0   50  0001 C CNN
	1    3750 4675
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 4300 6350 4300
Wire Wire Line
	6350 4300 6350 4375
$Comp
L RPi_Hat-rescue:GND-RESCUE-RPi_Hat #PWR011
U 1 1 5891FE4D
P 6350 4375
F 0 "#PWR011" H 6400 4425 50  0001 C CNN
F 1 "GND" H 6450 4275 50  0000 C CNN
F 2 "" H 0   0   50  0001 C CNN
F 3 "" H 0   0   50  0001 C CNN
	1    6350 4375
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 4100 6350 4100
Wire Wire Line
	6350 4100 6350 4150
$Comp
L RPi_Hat-rescue:GND-RESCUE-RPi_Hat #PWR012
U 1 1 5891FE94
P 6350 4150
F 0 "#PWR012" H 6400 4200 50  0001 C CNN
F 1 "GND" H 6450 4050 50  0000 C CNN
F 2 "" H 0   0   50  0001 C CNN
F 3 "" H 0   0   50  0001 C CNN
	1    6350 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 3600 6900 3600
Wire Wire Line
	6900 3600 6900 3675
$Comp
L RPi_Hat-rescue:GND-RESCUE-RPi_Hat #PWR013
U 1 1 5891FF6C
P 6900 3675
F 0 "#PWR013" H 6950 3725 50  0001 C CNN
F 1 "GND" H 6975 3550 50  0000 C CNN
F 2 "" H 550 0   50  0001 C CNN
F 3 "" H 550 0   50  0001 C CNN
	1    6900 3675
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 3300 6750 3300
Wire Wire Line
	6750 3300 6750 3400
$Comp
L RPi_Hat-rescue:GND-RESCUE-RPi_Hat #PWR014
U 1 1 5891FF9A
P 6750 3400
F 0 "#PWR014" H 400 -250 50  0001 C CNN
F 1 "GND" H 6850 3275 50  0000 C CNN
F 2 "" H 400 0   50  0001 C CNN
F 3 "" H 400 0   50  0001 C CNN
	1    6750 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 2900 6775 2900
Wire Wire Line
	6775 2900 6925 2900
Wire Wire Line
	6775 2900 6775 3000
$Comp
L RPi_Hat-rescue:GND-RESCUE-RPi_Hat #PWR015
U 1 1 5892002F
P 6775 3000
F 0 "#PWR015" H 425 -250 50  0001 C CNN
F 1 "GND" H 6780 2827 50  0000 C CNN
F 2 "" H 425 0   50  0001 C CNN
F 3 "" H 425 0   50  0001 C CNN
	1    6775 3000
	1    0    0    -1  
$EndComp
NoConn ~ 6000 3900
NoConn ~ 6000 4000
NoConn ~ 6000 4200
NoConn ~ 6000 4400
NoConn ~ 6000 4500
NoConn ~ 6000 4600
NoConn ~ 4100 4300
NoConn ~ 4100 4200
NoConn ~ 4100 4100
NoConn ~ 4100 4000
NoConn ~ 4100 3400
NoConn ~ 4100 3300
NoConn ~ 4100 3200
NoConn ~ 4100 3000
$Comp
L RPi_Hat-rescue:PWR_FLAG-RESCUE-RPi_Hat #FLG016
U 1 1 58936B87
P 3500 2575
F 0 "#FLG016" H -625 150 50  0001 C CNN
F 1 "PWR_FLAG" H 3500 2749 50  0000 C CNN
F 2 "" H -625 75  50  0001 C CNN
F 3 "" H -625 75  50  0001 C CNN
	1    3500 2575
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 2700 3500 2575
Connection ~ 3925 2700
$Comp
L RPi_Hat-rescue:PWR_FLAG-RESCUE-RPi_Hat #FLG017
U 1 1 58936BD5
P 6700 2600
F 0 "#FLG017" H 2575 175 50  0001 C CNN
F 1 "PWR_FLAG" H 6700 2774 50  0000 C CNN
F 2 "" H 2575 100 50  0001 C CNN
F 3 "" H 2575 100 50  0001 C CNN
	1    6700 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 2700 6700 2600
$Comp
L RPi_Hat-rescue:PWR_FLAG-RESCUE-RPi_Hat #FLG018
U 1 1 58936C10
P 6925 2825
F 0 "#FLG018" H 2800 400 50  0001 C CNN
F 1 "PWR_FLAG" H 6925 2999 50  0000 C CNN
F 2 "" H 2800 325 50  0001 C CNN
F 3 "" H 2800 325 50  0001 C CNN
	1    6925 2825
	1    0    0    -1  
$EndComp
Wire Wire Line
	6925 2900 6925 2825
Connection ~ 6775 2900
Text HLabel 6000 3500 2    60   BiDi ~ 0
swdio
Text HLabel 6000 3400 2    60   BiDi ~ 0
swclk
Text HLabel 6000 3000 2    60   Output ~ 0
RPI_TXD
Text HLabel 6000 3100 2    60   Input ~ 0
RPI_RXD
Text HLabel 6000 3200 2    60   Output ~ 0
nRST
Text HLabel 6000 3700 2    60   Input ~ 0
MCP2515_INT
Text HLabel 6000 3800 2    60   Output ~ 0
MCP2515_CS_SPI
Text HLabel 4100 2800 0    60   BiDi ~ 0
RPi_SDA_I2C
Text HLabel 4100 2900 0    60   Output ~ 0
RPi_SCL_I2C
Text HLabel 4100 3600 0    60   Output ~ 0
RPi_MOSI_SPI
Text HLabel 4100 3700 0    60   Input ~ 0
RPi_MISO_SPI
Text HLabel 4100 3800 0    60   Output ~ 0
RPi_CLK_SPI
$Comp
L RPi_Hat-rescue:R-RESCUE-RPi_Hat R4
U 1 1 5901649B
P 2875 4750
F 0 "R4" H 2975 4750 50  0000 C CNN
F 1 "1k" V 2775 4750 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2805 4750 50  0001 C CNN
F 3 "" H 2875 4750 50  0000 C CNN
	1    2875 4750
	1    0    0    -1  
$EndComp
$Comp
L RPi_Hat-rescue:R-RESCUE-RPi_Hat R5
U 1 1 59016698
P 3375 4750
F 0 "R5" H 3475 4750 50  0000 C CNN
F 1 "1k" V 3275 4750 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3305 4750 50  0001 C CNN
F 3 "" H 3375 4750 50  0000 C CNN
	1    3375 4750
	1    0    0    -1  
$EndComp
$Comp
L RPi_Hat-rescue:LED-RESCUE-RPi_Hat D1
U 1 1 590167AA
P 2875 5150
F 0 "D1" V 2820 5228 50  0000 L CNN
F 1 "LED_RED" V 2900 5225 50  0000 L CNN
F 2 "LEDs:LED_0805" H 2875 5150 50  0001 C CNN
F 3 "" H 2875 5150 50  0001 C CNN
	1    2875 5150
	0    1    1    0   
$EndComp
$Comp
L RPi_Hat-rescue:LED-RESCUE-RPi_Hat D2
U 1 1 59016879
P 3375 5150
F 0 "D2" V 3320 5228 50  0000 L CNN
F 1 "LED_YELLOW" V 3411 5228 50  0000 L CNN
F 2 "LEDs:LED_0805" H 3375 5150 50  0001 C CNN
F 3 "" H 3375 5150 50  0001 C CNN
	1    3375 5150
	0    1    1    0   
$EndComp
Wire Wire Line
	4100 4400 2875 4400
Wire Wire Line
	2875 4400 2875 4600
Wire Wire Line
	4100 4500 3375 4500
Wire Wire Line
	3375 4500 3375 4600
Wire Wire Line
	2875 5000 2875 4900
Wire Wire Line
	3375 5000 3375 4900
$Comp
L RPi_Hat-rescue:GND-RESCUE-RPi_Hat #PWR019
U 1 1 59016BFF
P 2875 5400
F 0 "#PWR019" H -875 475 50  0001 C CNN
F 1 "GND" H 2880 5227 50  0000 C CNN
F 2 "" H -875 725 50  0001 C CNN
F 3 "" H -875 725 50  0001 C CNN
	1    2875 5400
	1    0    0    -1  
$EndComp
$Comp
L RPi_Hat-rescue:GND-RESCUE-RPi_Hat #PWR020
U 1 1 59016C14
P 3375 5400
F 0 "#PWR020" H -375 475 50  0001 C CNN
F 1 "GND" H 3380 5227 50  0000 C CNN
F 2 "" H -375 725 50  0001 C CNN
F 3 "" H -375 725 50  0001 C CNN
	1    3375 5400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2875 5400 2875 5300
Wire Wire Line
	3375 5400 3375 5300
$EndSCHEMATC
