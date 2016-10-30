#!/usr/bin/python

import time
import redis
from orpi_lcd import LCD16x2

REDIS_KEY = 'LCD_lines'
REDIS_PRI_KEY = 'LCD_timed_lines'

r = redis.Redis()

# Initialize the LCD using the pins above.
lcd = LCD16x2()

old_line = None

while True:

	new_line = r.get(REDIS_PRI_KEY)
	if not new_line:
		new_line = r.get(REDIS_KEY)
	if new_line != old_line:
		if new_line:
			lcd.message(new_line)
		else:
			lcd.clear()
		old_line = new_line
	time.sleep(1)

