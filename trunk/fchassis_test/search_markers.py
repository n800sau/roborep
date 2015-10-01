#!/usr/bin/env python

import sys, os, time, json, redis

from lib.utils import dbprint
from lib.frobo_ng import frobo_ng
from lib.marker import collect_markers, use_camera, release_camera, make_shot

servant = 'raspiCamServant'
queue = 'raspiCamServant.js.obj'

#TARGET = 618
target_loc = None

i = 0
def stop_cb(c, r):
	global i
	rs = False
	dbprint('h: %s' % c.compass.heading())
	markers = collect_markers(r, fpath = os.path.join(os.path.expanduser('~/public_html'), 'pic%d.jpg' % i))
	if markers:
		dbprint('FOUND %d markers' % len(markers))
		for m in markers:
			dbprint('\t%s' % m['id'])
		#	os.system('espeak "%s"' % ' '.join([c for c in str(m['id'])]))
		#	if m['id'] == TARGET:
		#		target_loc = m
		#		rs = True
		#		break
		i += 1
	return rs

if __name__ == '__main__':

	clockwise = int(sys.argv[1])

	r = redis.Redis()

	c = frobo_ng()
#	c.debug = True

	try:
		use_camera(r)
		c.update_state()
		dbprint('BEFORE %s m to %s' % (c.state['sonar'], c.compass.heading()))
		c.search_around(lambda c, r=r: stop_cb(c, r), clockwise=clockwise)
		dbprint('AFTER %s m to %s (%d steps)' % (c.state['sonar'], c.compass.heading(), i))
		json.dump(c.dots, file('dots.json', 'w'), indent=2)
	finally:
		c.cmd_mstop()
		c.wait_until_stop()
		make_shot(r)
		release_camera(r)

	dbprint('TARGET: %s' % target_loc)

