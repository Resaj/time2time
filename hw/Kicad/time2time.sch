EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 3
Title "time2time project"
Date "2021-02-28"
Rev "V1.1"
Comp "Puma Pride Robotics Team"
Comment1 "Designed by Rubén Espino San José"
Comment2 "CC BY-NC-SA license"
Comment3 ""
Comment4 ""
$EndDescr
$Sheet
S 2650 2800 1950 1600
U 5CD46B2A
F0 "supply & charger" 50
F1 "supply_and_charger.sch" 50
F2 "USB[1..4]" B R 4600 3650 50 
F3 "PW_STATE[1..3]" O R 4600 3300 50 
F4 "SLEEP_12V" I R 4600 4000 50 
$EndSheet
$Sheet
S 5500 2300 3100 2800
U 5CD46B96
F0 "microcontroller" 50
F1 "microcontroller.sch" 50
F2 "USB[1..4]" B L 5500 3650 50 
F3 "PW_STATE[1..3]" I L 5500 3300 50 
F4 "SLEEP_12V" O L 5500 4000 50 
$EndSheet
Wire Bus Line
	4600 3300 5500 3300
Wire Bus Line
	4600 3650 5500 3650
Wire Wire Line
	4600 4000 5500 4000
$EndSCHEMATC
