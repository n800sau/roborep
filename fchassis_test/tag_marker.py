#!/usr/bin/env python

import sys, os, time, json, redis, picamera, cv2

from lib.utils import dbprint
from lib.camera import update_img

from lib.frobo_ng import frobo_ng
from lib.marker import use_camera, release_camera, marker_offset, make_shot

TARGET = 618
#TARGET = 477
#TARGET = 1023

# make a marker to be in the view centre

if __name__ == '__main__':

	r = redis.Redis()

	c = frobo_ng()
	c.debug = True

	off = None
	try:
		use_camera(r)
		time.sleep(4)
		min_pwr = 20
		max_pwr = 100
		for k in range(10):
			found = c.search_marker(r, marker_id=TARGET)
			if found:
				for j in range(10):
					for i in range(10):
						off = marker_offset(r, TARGET)
						if off is None:
							continue
						else:
							aoff = abs(off)
							dbprint('H offset: %d degree%s'  % (aoff, ((' in %s direction' % ('left' if off < 0 else 'right')) if int(off) != 0 else '')))
							pwr = min_pwr + (max_pwr - min_pwr) * min(1, (aoff / 180.))
							if int(off) == 0:
								break
							if off < 0:
								c.tick_left(pwr=pwr)
							else:
								c.tick_right(pwr=pwr)
					if off is None:
						break
					elif int(off) == 0:
						dbprint("Move straingt")
						c.move_straight(fwd=True, max_steps=c.m2steps(0.2), max_secs=1)
					else:
						break
			else:
				dbprint('Not found')
				break
			if c.hit_warn:
				dbprint("Deadend reached")
				break
	finally:
		c.cmd_mstop()
		c.wait_until_stop()
		make_shot(r)
		release_camera(r)
