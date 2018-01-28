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
Sheet 6 7
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
L DS3231 U1
U 1 1 587B5F3E
P 4675 2675
F 0 "U1" H 4575 3150 50  0000 R CNN
F 1 "DS3231" H 4575 3075 50  0000 R CNN
F 2 "caribou:SOIC-16W_7.5x10.3mm_Pitch1.27mm" H 4725 2275 50  0001 L CNN
F 3 "" H 4945 2925 50  0001 C CNN
	1    4675 2675
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR048
U 1 1 587B605B
P 4675 3275
F 0 "#PWR048" H 4675 3025 50  0001 C CNN
F 1 "GND" H 4675 3125 50  0000 C CNN
F 2 "" H 4675 3275 50  0000 C CNN
F 3 "" H 4675 3275 50  0000 C CNN
	1    4675 3275
	1    0    0    -1  
$EndComp
Wire Wire Line
	4675 3175 4675 3275
$Comp
L +3.3V #PWR049
U 1 1 587B639C
P 4675 1875
F 0 "#PWR049" H 4675 1725 50  0001 C CNN
F 1 "+3.3V" H 4675 2015 50  0000 C CNN
F 2 "" H 4675 1875 50  0000 C CNN
F 3 "" H 4675 1875 50  0000 C CNN
	1    4675 1875
	1    0    0    -1  
$EndComp
Wire Wire Line
	4675 1875 4675 2175
NoConn ~ 4175 2875
NoConn ~ 5175 2675
$Comp
L Battery_Cell BT1
U 1 1 587B6407
P 5450 3075
F 0 "BT1" H 5550 3175 50  0000 L CNN
F 1 "Battery_Cell" H 5550 3075 50  0000 L CNN
F 2 "caribou:BC501SM" V 5450 3135 50  0001 C CNN
F 3 "" V 5450 3135 50  0000 C CNN
	1    5450 3075
	1    0    0    -1  
$EndComp
Wire Wire Line
	5175 2875 6050 2875
Wire Wire Line
	5450 3175 5450 3225
Wire Wire Line
	5450 3225 4675 3225
Connection ~ 4675 3225
$Comp
L PWR_FLAG #FLG050
U 1 1 589372D2
P 6050 2800
F 0 "#FLG050" H 350 175 50  0001 C CNN
F 1 "PWR_FLAG" H 6050 2974 50  0000 C CNN
F 2 "" H 350 100 50  0001 C CNN
F 3 "" H 350 100 50  0001 C CNN
	1    6050 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	6050 2875 6050 2800
Connection ~ 5450 2875
Text HLabel 4175 2575 0    60   Input ~ 0
SCL
Text HLabel 4175 2675 0    60   BiDi ~ 0
SDA
$Comp
L C_Small C14
U 1 1 589C24D4
P 5625 2025
F 0 "C14" H 5717 2071 50  0000 L CNN
F 1 "0.1u" H 5717 1980 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 2075 150 50  0001 C CNN
F 3 "" H 2075 150 50  0001 C CNN
	1    5625 2025
	1    0    0    -1  
$EndComp
Wire Wire Line
	4675 1925 5625 1925
Connection ~ 4675 1925
$Comp
L GND #PWR051
U 1 1 589C27AC
P 5625 2275
F 0 "#PWR051" H 475 -275 50  0001 C CNN
F 1 "GND" H 5630 2102 50  0000 C CNN
F 2 "" H 475 -25 50  0001 C CNN
F 3 "" H 475 -25 50  0001 C CNN
	1    5625 2275
	1    0    0    -1  
$EndComp
Wire Wire Line
	5625 2125 5625 2275
Text Label 5200 2875 0    60   ~ 0
VBAT
Text HLabel 5175 2575 2    60   Output ~ 0
INT_SQW
$EndSCHEMATC