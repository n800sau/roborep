#!/usr/bin/python
# Example using a character LCD connected to a Raspberry Pi or BeagleBone Black.
import time
import pyping
from datetime import timedelta

import Adafruit_CharLCD as LCD

lcd_rs        = 'P8_15'
lcd_en        = 'P8_13'
lcd_d4        = 'P8_14'
lcd_d5        = 'P8_12'
lcd_d6        = 'P8_10'
lcd_d7        = 'P8_8'
lcd_backlight = 'P9_14'
# R/W connected to GND

# Define LCD column and row size for 16x2 LCD.
lcd_columns = 16
lcd_rows    = 2

# Alternatively specify a 20x4 LCD.
# lcd_columns = 20
# lcd_rows    = 4

# Initialize the LCD using the pins above.
lcd = LCD.Adafruit_CharLCD(lcd_rs, lcd_en, lcd_d4, lcd_d5, lcd_d6, lcd_d7,
                           lcd_columns, lcd_rows, lcd_backlight)

lcd.set_backlight(1)
while True:

	lcd.home()
	with open('/proc/uptime', 'r') as f:
		response = pyping.ping('192.168.1.1')
		uptime_seconds = float(f.readline().split()[0])
		uptime_string = 'Uptime: %s' % timedelta(seconds = uptime_seconds)
		lcd.message(time.strftime('%H:%M:%S') + (' p: %s' % (not response.ret_code)) + '\n' + uptime_string.split('.')[0])

#	lcd.message(time.strftime('%d/%m/%Y\n%H:%M:%S'))

	time.sleep(1)

