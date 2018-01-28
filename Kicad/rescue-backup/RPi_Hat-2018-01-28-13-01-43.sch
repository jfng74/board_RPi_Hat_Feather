EESchema Schematic File Version 2
LIBS:RPi_Hat-rescue
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:jfng
LIBS:RPi_Hat-cache
EELAYER 26 0
EELAYER END
$Descr USLetter 11000 8500
encoding utf-8
Sheet 1 7
Title ""
Date "2017-02-08"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Sheet
S 3375 1050 1700 2625
U 5515D395
F0 "RPi_GPIO" 60
F1 "RPi_GPIO.sch" 60
F2 "RPI_TXD" O R 5075 1375 60 
F3 "RPI_RXD" I R 5075 1275 60 
F4 "swdio" B R 5075 1700 60 
F5 "swclk" B R 5075 1800 60 
F6 "nRST" O R 5075 1900 60 
F7 "MCP2515_INT" I R 5075 3500 60 
F8 "MCP2515_CS_SPI" O R 5075 3375 60 
F9 "RPi_SDA_I2C" B L 3375 1375 60 
F10 "RPi_SCL_I2C" O L 3375 1475 60 
F11 "RPi_MOSI_SPI" O R 5075 3000 60 
F12 "RPi_MISO_SPI" I R 5075 3125 60 
F13 "RPi_CLK_SPI" O R 5075 3250 60 
$EndSheet
$Sheet
S 1800 1850 1275 625 
U 58965077
F0 "1-wire" 60
F1 "1-wire.sch" 60
F2 "SCL" I R 3075 2075 60 
F3 "SDA" B R 3075 1975 60 
F4 "1_WIRE_IO" B L 1800 2175 60 
F5 "1_WIRE_GND" U L 1800 2275 60 
$EndSheet
$Sheet
S 5325 2850 1700 825 
U 589B5416
F0 "RPi_CAN_BUS" 60
F1 "RPi_CAN_BUS.sch" 60
F2 "~CS" I L 5325 3375 60 
F3 "SCK" I L 5325 3250 60 
F4 "SI" I L 5325 3000 60 
F5 "SO" O L 5325 3125 60 
F6 "CANH" B R 7025 3075 60 
F7 "CANL" B R 7025 3175 60 
F8 "~INT" O L 5325 3500 60 
$EndSheet
$Sheet
S 7450 1625 1025 925 
U 589B5AEE
F0 "feather_CAN_BUS" 60
F1 "feather_CAN_BUS.sch" 60
F2 "~CS" I L 7450 2250 60 
F3 "SCK" I L 7450 2125 60 
F4 "SI" I L 7450 1875 60 
F5 "SO" O L 7450 2000 60 
F6 "~INT" O L 7450 2375 60 
F7 "CANH" B R 8475 2150 60 
F8 "CANL" B R 8475 2250 60 
F9 "V_IO" U L 7450 1750 60 
$EndSheet
$Comp
L CONN_01X02 CAN1
U 1 1 589BA6CF
P 9425 1750
F 0 "CAN1" H 9503 1791 50  0000 L CNN
F 1 "CONN_01X02" H 9503 1700 50  0000 L CNN
F 2 "caribou:0436500219" H 8125 -650 50  0001 C CNN
F 3 "" H 8125 -650 50  0001 C CNN
	1    9425 1750
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X02 CAN2
U 1 1 589BA6D6
P 9450 3925
F 0 "CAN2" H 9528 3966 50  0000 L CNN
F 1 "CONN_01X02" H 9528 3875 50  0000 L CNN
F 2 "caribou:0436500219" H 8150 1525 50  0001 C CNN
F 3 "" H 8150 1525 50  0001 C CNN
	1    9450 3925
	1    0    0    -1  
$EndComp
Text Label 9000 2675 2    60   ~ 0
CAN+
Text Label 9100 2675 0    60   ~ 0
CAN-
$Comp
L CONN_01X04 I2C-1
U 1 1 589E2857
P 9425 1325
F 0 "I2C-1" H 9503 1366 50  0000 L CNN
F 1 "CONN_01X04" H 9503 1275 50  0000 L CNN
F 2 "caribou:0436500417" H 175 -375 50  0001 C CNN
F 3 "" H 175 -375 50  0001 C CNN
	1    9425 1325
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR01
U 1 1 589E2862
P 9075 1100
F 0 "#PWR01" H 9075 850 50  0001 C CNN
F 1 "GND" H 9075 950 50  0000 C CNN
F 2 "" H 9075 1100 50  0000 C CNN
F 3 "" H 9075 1100 50  0000 C CNN
	1    9075 1100
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X03 1Wire-1
U 1 1 58A1C4EE
P 1125 2175
F 0 "1Wire-1" H 1203 2216 50  0000 L CNN
F 1 "CONN_01X03" H 1203 2125 50  0000 L CNN
F 2 "caribou:0436500317" H -4650 -2925 50  0001 C CNN
F 3 "" H -4650 -2925 50  0001 C CNN
	1    1125 2175
	-1   0    0    1   
$EndComp
$Comp
L +5V #PWR02
U 1 1 58A1C4F5
P 1675 2025
F 0 "#PWR02" H 75  -2075 50  0001 C CNN
F 1 "+5V" H 1690 2198 50  0000 C CNN
F 2 "" H 75  -1925 50  0001 C CNN
F 3 "" H 75  -1925 50  0001 C CNN
	1    1675 2025
	-1   0    0    -1  
$EndComp
$Comp
L CAN_NODE_JUNCTION J5
U 1 1 58AA1A82
P 8975 2200
F 0 "J5" H 9203 2246 50  0000 L CNN
F 1 "CAN_NODE_JUNCTION" H 9203 2155 50  0000 L CNN
F 2 "caribou:CAN_NODE_JONCTION" H -800 -1225 50  0001 C CNN
F 3 "" H -800 -1225 50  0001 C CNN
	1    8975 2200
	1    0    0    -1  
$EndComp
$Comp
L CAN_NODE_JUNCTION J4
U 1 1 58AB23E0
P 8975 3125
F 0 "J4" H 9203 3171 50  0000 L CNN
F 1 "CAN_NODE_JUNCTION" H 9203 3080 50  0000 L CNN
F 2 "caribou:CAN_NODE_JONCTION" H -800 -300 50  0001 C CNN
F 3 "" H -800 -300 50  0001 C CNN
	1    8975 3125
	1    0    0    -1  
$EndComp
Text Label 8775 2150 2    60   ~ 0
CAN+
Text Label 8525 2250 0    60   ~ 0
CAN-
Text Label 7975 3075 2    60   ~ 0
CAN+
Text Label 7725 3175 0    60   ~ 0
CAN-
Text Label 9100 3575 0    60   ~ 0
CAN-
Text Label 9000 3575 2    60   ~ 0
CAN+
Text Label 9100 1900 0    60   ~ 0
CAN-
Text Label 9000 1900 2    60   ~ 0
CAN+
$Comp
L C_Small C15
U 1 1 590057AA
P 8500 950
F 0 "C15" V 8271 950 50  0000 C CNN
F 1 "0.1u" V 8362 950 50  0000 C CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 3250 -1125 50  0001 C CNN
F 3 "" H 3250 -1125 50  0001 C CNN
	1    8500 950 
	0    1    1    0   
$EndComp
$Comp
L GND #PWR03
U 1 1 59008424
P 8700 1025
F 0 "#PWR03" H 8700 775 50  0001 C CNN
F 1 "GND" H 8705 852 50  0000 C CNN
F 2 "" H 8700 1025 50  0001 C CNN
F 3 "" H 8700 1025 50  0001 C CNN
	1    8700 1025
	1    0    0    -1  
$EndComp
$Sheet
S 1800 1050 1275 600 
U 5891EF58
F0 "RTC" 60
F1 "RTC.sch" 60
F2 "SCL" I R 3075 1475 60 
F3 "SDA" B R 3075 1375 60 
F4 "INT_SQW" O L 1800 1175 60 
$EndSheet
$Sheet
S 5325 1050 1700 1525
U 5892155B
F0 "feather_connectors" 60
F1 "feather_connector.sch" 60
F2 "swdio" B L 5325 1700 60 
F3 "swclk" B L 5325 1800 60 
F4 "nRST" I L 5325 1900 60 
F5 "FEATHER_RXD" I L 5325 1375 60 
F6 "FEATHER_TXD" O L 5325 1275 60 
F7 "MCP2515_CS" O R 7025 2250 60 
F8 "MCP2515_INT" I R 7025 2375 60 
F9 "FEATHER_SCL_I2C" O R 7025 1475 60 
F10 "FEATHER_SDA_I2C" B R 7025 1375 60 
F11 "FEATHER_SCK_SPI" O R 7025 2125 60 
F12 "FEATHER_MOSI_SPI" O R 7025 1875 60 
F13 "FEATHER_MISO_SPI" I R 7025 2000 60 
F14 "FEATHER_3.3V" U R 7025 1275 60 
F15 "INT_SQW" I R 7025 1175 60 
$EndSheet
$Comp
L C_Small C16
U 1 1 5936FEF7
P 1525 2500
F 0 "C16" V 1400 2500 50  0000 C CNN
F 1 "1uF" V 1650 2500 50  0000 C CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 1525 2500 50  0001 C CNN
F 3 "" H 1525 2500 50  0001 C CNN
	1    1525 2500
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5075 1700 5325 1700
Wire Wire Line
	5325 1275 5075 1275
Wire Wire Line
	5325 1375 5075 1375
Wire Wire Line
	5075 1800 5325 1800
Wire Wire Line
	5325 1900 5075 1900
Wire Wire Line
	9000 3425 9000 3875
Wire Wire Line
	9100 3425 9100 3975
Wire Wire Line
	9100 1800 9225 1800
Wire Wire Line
	9000 1700 9225 1700
Wire Wire Line
	9100 3975 9250 3975
Wire Wire Line
	9000 3875 9250 3875
Wire Wire Line
	5075 3000 5325 3000
Wire Wire Line
	5325 3125 5075 3125
Wire Wire Line
	5075 3250 5325 3250
Wire Wire Line
	5075 3375 5325 3375
Wire Wire Line
	5075 3500 5325 3500
Wire Wire Line
	9225 1175 9075 1175
Wire Wire Line
	9075 1175 9075 1100
Wire Wire Line
	7025 1875 7450 1875
Wire Wire Line
	7025 2000 7450 2000
Wire Wire Line
	7025 2125 7450 2125
Wire Wire Line
	7025 2250 7450 2250
Wire Wire Line
	7025 2375 7450 2375
Wire Wire Line
	3075 1375 3375 1375
Wire Wire Line
	3075 1475 3375 1475
Wire Wire Line
	3275 1475 3275 2075
Wire Wire Line
	3275 2075 3075 2075
Connection ~ 3275 1475
Wire Wire Line
	3175 1375 3175 1975
Wire Wire Line
	3175 1975 3075 1975
Connection ~ 3175 1375
Wire Wire Line
	7025 1275 9225 1275
Wire Wire Line
	7025 1375 9225 1375
Wire Wire Line
	7025 1475 9225 1475
Wire Wire Line
	7450 1750 7225 1750
Wire Wire Line
	7225 1750 7225 1275
Connection ~ 7225 1275
Wire Wire Line
	8475 2150 8775 2150
Wire Wire Line
	8775 2250 8475 2250
Wire Wire Line
	9000 1700 9000 1900
Wire Wire Line
	9100 1900 9100 1800
Wire Wire Line
	7025 3075 8775 3075
Wire Wire Line
	7025 3175 8775 3175
Wire Wire Line
	9000 2500 9000 2825
Wire Wire Line
	9100 2500 9100 2825
Wire Wire Line
	1325 2175 1800 2175
Wire Wire Line
	1325 2275 1800 2275
Wire Wire Line
	8600 950  8700 950 
Wire Wire Line
	8700 950  8700 1025
Wire Wire Line
	8400 950  8275 950 
Wire Wire Line
	8275 950  8275 1275
Connection ~ 8275 1275
Wire Wire Line
	1800 1175 1625 1175
Wire Wire Line
	1625 1175 1625 750 
Wire Wire Line
	1625 750  7325 750 
Wire Wire Line
	7325 750  7325 1175
Wire Wire Line
	7325 1175 7025 1175
Wire Wire Line
	1325 2075 1675 2075
Wire Wire Line
	1675 2025 1675 2500
Connection ~ 1675 2075
Wire Wire Line
	1675 2500 1625 2500
Wire Wire Line
	1425 2500 1375 2500
Wire Wire Line
	1375 2500 1375 2275
Connection ~ 1375 2275
$EndSCHEMATC
