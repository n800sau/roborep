#!/usr/bin/python
# once in a time reads redis "lcdline" and output it
import time
import redis
import Adafruit_CharLCD as LCD

REDIS_KEY = 'LCD_lines'

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

r = redis.Redis()

# Initialize the LCD using the pins above.
lcd = LCD.Adafruit_CharLCD(lcd_rs, lcd_en, lcd_d4, lcd_d5, lcd_d6, lcd_d7,
                           lcd_columns, lcd_rows, lcd_backlight)
old_line = None

lcd.set_backlight(1)
while True:

	new_line = r.get(REDIS_KEY)
	if new_line != old_line:
		lcd.home()
		if new_line:
			out_lines = []
			for line in new_line.split('\n')[:lcd_rows]:
				out_lines.append((line + (' ' * lcd_columns))[:lcd_columns])
			lcd.message('\n'.join(out_lines))
		else:
			lcd.clear()
		old_line = new_line
	time.sleep(1)

