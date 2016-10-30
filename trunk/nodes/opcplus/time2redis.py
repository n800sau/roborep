#!/usr/bin/python
# Example using a character LCD connected to a Raspberry Pi or BeagleBone Black.
import time
from datetime import timedelta
import redis

REDIS_KEY = 'LCD_lines'

r = redis.Redis()

while True:

	with open('/proc/uptime', 'r') as f:
		uptime_seconds = float(f.readline().split()[0])
		uptime_string = 'Uptime: %s' % timedelta(seconds = uptime_seconds)
		r.set(REDIS_KEY, time.strftime('%H:%M:%S') + '\n' + uptime_string.split('.')[0])

	time.sleep(1)

