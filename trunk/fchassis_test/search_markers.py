#!/usr/bin/env python

import sys, os, time, json, redis

from lib.utils import dbprint
from lib.frobo_ng import frobo_ng
from lib.marker import collect_markers, use_camera, release_camera, make_shot

TARGET = 618
TARGET = None

if __name__ == '__main__':

	clockwise = int(sys.argv[1])

	r = redis.Redis()

	c = frobo_ng()
	c.debug = True

	try:
		use_camera(r)
		c.update_state()
		dbprint('BEFORE %s m to %s' % (c.state['sonar'], c.compass.heading()))
		found = c.search_marker(r, clockwise=clockwise, marker_id=TARGET)
		dbprint('MARKER: %s' % json.dumps(found, indent=2))
		dbprint('AFTER %s m to %s' % (c.state['sonar'], c.compass.heading()))
		json.dump(c.dots, file('dots.json', 'w'), indent=2)
	finally:
		c.cmd_mstop()
		c.wait_until_stop()
		make_shot(r)
		release_camera(r)

	dbprint('TARGET: %s' % found)

