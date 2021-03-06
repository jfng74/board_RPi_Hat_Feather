EESchema Schematic File Version 4
LIBS:RPi_Hat-cache
EELAYER 26 0
EELAYER END
$Descr USLetter 11000 8500
encoding utf-8
Sheet 7 7
Title ""
Date "2017-02-08"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L jfng:FEATHER_MO_LORA Adafruit_Feather-1
U 1 1 587BD5DB
P 4275 3725
F 0 "Adafruit_Feather-1" H 5000 2450 60  0000 C CNN
F 1 "FEATHER_MO_LORA" H 4225 4875 60  0000 C CNN
F 2 "caribou:FEATHER_MO_LORA" H 4275 4200 60  0001 C CNN
F 3 "" H 4275 4200 60  0001 C CNN
	1    4275 3725
	1    0    0    -1  
$EndComp
$Comp
L RPi_Hat-rescue:GND-RESCUE-RPi_Hat #PWR052
U 1 1 587BD966
P 2750 3250
F 0 "#PWR052" H 2750 3000 50  0001 C CNN
F 1 "GND" H 2750 3100 50  0000 C CNN
F 2 "" H 2750 3250 50  0000 C CNN
F 3 "" H 2750 3250 50  0000 C CNN
	1    2750 3250
	1    0    0    -1  
$EndComp
Text Label 3350 3950 0    60   ~ 0
A5
Text Label 3350 3825 0    60   ~ 0
A4
Text Label 3350 3700 0    60   ~ 0
A3
Text Label 3350 3575 0    60   ~ 0
A2
Text Label 3350 3450 0    60   ~ 0
A1
Text Label 3350 3325 0    60   ~ 0
A0
Text Label 3350 3075 0    60   ~ 0
ARF
Text Label 3350 2825 0    60   ~ 0
RST
Text Label 5000 3325 0    60   ~ 0
VBAT
Text Label 5000 3450 0    60   ~ 0
EN
Text Label 5000 3575 0    60   ~ 0
VUSB
Text Label 5000 3700 0    60   ~ 0
D13
Text Label 5000 3825 0    60   ~ 0
D12
Text Label 5000 3950 0    60   ~ 0
D11
Text Label 5000 4075 0    60   ~ 0
D10
Text Label 5000 4200 0    60   ~ 0
D9
Text Label 5000 4575 0    60   ~ 0
SCL
Text Label 5000 4700 0    60   ~ 0
SDA
Text Label 6500 4250 0    60   ~ 0
MISO
Text Label 6500 4125 0    60   ~ 0
MOSI
Text Label 6500 4000 0    60   ~ 0
SCK
Text Label 6500 3875 0    60   ~ 0
A5
Text Label 6500 3750 0    60   ~ 0
A4
Text Label 6500 3625 0    60   ~ 0
A3
Text Label 6500 3500 0    60   ~ 0
A2
Text Label 6500 3375 0    60   ~ 0
A1
Text Label 6500 3250 0    60   ~ 0
A0
Text Label 6500 3000 0    60   ~ 0
ARF
Text Label 6500 2750 0    60   ~ 0
RST
Text Label 8150 3250 0    60   ~ 0
VBAT
Text Label 8150 3375 0    60   ~ 0
EN
Text Label 8150 3500 0    60   ~ 0
VUSB
Text Label 8150 3625 0    60   ~ 0
D13
Text Label 8150 3750 0    60   ~ 0
D12
Text Label 8150 3875 0    60   ~ 0
D11
Text Label 8150 4000 0    60   ~ 0
D10
Text Label 8150 4125 0    60   ~ 0
D9
Text Label 8150 4250 0    60   ~ 0
D6
Text Label 8150 4375 0    60   ~ 0
D5
Text Label 8150 4500 0    60   ~ 0
SCL
Text Label 8150 4625 0    60   ~ 0
SDA
$Comp
L RPi_Hat-rescue:GND-RESCUE-RPi_Hat #PWR053
U 1 1 587C072D
P 5900 3225
F 0 "#PWR053" H 5900 2975 50  0001 C CNN
F 1 "GND" H 5900 3075 50  0000 C CNN
F 2 "" H 5900 3225 50  0000 C CNN
F 3 "" H 5900 3225 50  0000 C CNN
	1    5900 3225
	1    0    0    -1  
$EndComp
Wire Wire Line
	3575 3200 2750 3200
Wire Wire Line
	2750 3200 2750 3250
Wire Wire Line
	3575 2825 3300 2825
Wire Wire Line
	3575 3075 3300 3075
Wire Wire Line
	3575 3325 3300 3325
Wire Wire Line
	3575 3450 3300 3450
Wire Wire Line
	3575 3575 3300 3575
Wire Wire Line
	3575 3700 3300 3700
Wire Wire Line
	3575 3825 3300 3825
Wire Wire Line
	3575 3950 3300 3950
Wire Wire Line
	4950 3325 5275 3325
Wire Wire Line
	4950 3450 5275 3450
Wire Wire Line
	4950 3575 5475 3575
Wire Wire Line
	4950 3700 5275 3700
Wire Wire Line
	4950 3825 5275 3825
Wire Wire Line
	4950 3950 5275 3950
Wire Wire Line
	4950 4075 5275 4075
Wire Wire Line
	4950 4200 5275 4200
Wire Wire Line
	4950 4575 5275 4575
Wire Wire Line
	4950 4700 5275 4700
Wire Wire Line
	6725 3125 5900 3125
Wire Wire Line
	6725 2750 6450 2750
Wire Wire Line
	6725 3000 6450 3000
Wire Wire Line
	6725 3250 6450 3250
Wire Wire Line
	6725 3375 6450 3375
Wire Wire Line
	6725 3500 6450 3500
Wire Wire Line
	6725 3625 6450 3625
Wire Wire Line
	6725 3750 6450 3750
Wire Wire Line
	6725 3875 6450 3875
Wire Wire Line
	6725 4000 6450 4000
Wire Wire Line
	6725 4125 6450 4125
Wire Wire Line
	6725 4250 6450 4250
Wire Wire Line
	8100 3250 8425 3250
Wire Wire Line
	8100 3375 8425 3375
Wire Wire Line
	8100 3500 8650 3500
Wire Wire Line
	8100 3625 8425 3625
Wire Wire Line
	8100 3750 8425 3750
Wire Wire Line
	8100 3875 8425 3875
Wire Wire Line
	8100 4000 8425 4000
Wire Wire Line
	8100 4125 8425 4125
Wire Wire Line
	8100 4250 8425 4250
Wire Wire Line
	8100 4375 8425 4375
Wire Wire Line
	8100 4500 8425 4500
Wire Wire Line
	8100 4625 8425 4625
Wire Wire Line
	5900 3125 5900 3225
$Comp
L jfng:FEATHER_WING Adafruit_Feather_Wing-1
U 1 1 587990FA
P 7425 3650
F 0 "Adafruit_Feather_Wing-1" H 7875 2375 60  0000 C CNN
F 1 "FEATHER_WING" H 7375 4800 60  0000 C CNN
F 2 "caribou:FEATHER_WING" H 7425 4125 60  0001 C CNN
F 3 "" H 7425 4125 60  0001 C CNN
	1    7425 3650
	1    0    0    -1  
$EndComp
NoConn ~ 6725 4375
NoConn ~ 6725 4500
$Comp
L RPi_Hat-rescue:+5V-RESCUE-RPi_Hat #PWR054
U 1 1 589285EE
P 5475 3525
F 0 "#PWR054" H 150 -225 50  0001 C CNN
F 1 "+5V" H 5490 3698 50  0000 C CNN
F 2 "" H 150 -75 50  0001 C CNN
F 3 "" H 150 -75 50  0001 C CNN
	1    5475 3525
	1    0    0    -1  
$EndComp
Wire Wire Line
	5475 3575 5475 3525
Wire Wire Line
	8650 3500 8650 3350
$Comp
L RPi_Hat-rescue:+5V-RESCUE-RPi_Hat #PWR055
U 1 1 58928823
P 8650 3350
F 0 "#PWR055" H 8700 3400 50  0001 C CNN
F 1 "+5V" H 8665 3523 50  0000 C CNN
F 2 "" H 0   0   50  0001 C CNN
F 3 "" H 0   0   50  0001 C CNN
	1    8650 3350
	1    0    0    -1  
$EndComp
Text Label 3350 4325 0    60   ~ 0
MISO
Text Label 3350 4200 0    60   ~ 0
MOSI
Text Label 3350 4075 0    60   ~ 0
SCK
Wire Wire Line
	3575 4075 3300 4075
Wire Wire Line
	3575 4200 3300 4200
Wire Wire Line
	3575 4325 3300 4325
Text Label 5000 4325 0    60   ~ 0
D6
Text Label 5000 4450 0    60   ~ 0
D5
Wire Wire Line
	4950 4325 5275 4325
Wire Wire Line
	4950 4450 5275 4450
NoConn ~ 6725 4625
NoConn ~ 3575 4700
Text HLabel 4125 5075 3    60   BiDi ~ 0
swdio
Text HLabel 4425 5075 3    60   BiDi ~ 0
swclk
Text HLabel 3300 2825 0    60   Input ~ 0
nRST
Text HLabel 3575 4450 0    60   Input ~ 0
FEATHER_RXD
Text HLabel 3575 4575 0    60   Output ~ 0
FEATHER_TXD
Text HLabel 5275 4325 2    60   Output ~ 0
MCP2515_CS
Text HLabel 5275 4450 2    60   Input ~ 0
MCP2515_INT
Text HLabel 5275 4575 2    60   Output ~ 0
FEATHER_SCL_I2C
Text HLabel 5275 4700 2    60   BiDi ~ 0
FEATHER_SDA_I2C
Text HLabel 3300 4075 0    60   Output ~ 0
FEATHER_SCK_SPI
Text HLabel 3300 4200 0    60   Output ~ 0
FEATHER_MOSI_SPI
Text HLabel 3300 4325 0    60   Input ~ 0
FEATHER_MISO_SPI
Wire Wire Line
	6725 2875 6075 2875
Text Label 6075 2875 0    60   ~ 0
FEATHER_3.3V
Wire Wire Line
	3575 2950 2925 2950
Text Label 2925 2950 0    60   ~ 0
FEATHER_3.3V
Text HLabel 2925 2950 0    60   UnSpc ~ 0
FEATHER_3.3V
Text HLabel 5275 4200 2    60   Input ~ 0
INT_SQW
$EndSCHEMATC
