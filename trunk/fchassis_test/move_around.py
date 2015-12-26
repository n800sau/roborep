#!/usr/bin/env python

import sys, os, time, json, redis, random

from lib.utils import dbprint, html_path
from lib.marker import use_camera, release_camera, collect_markers, make_shot

from lib.frobo_ng import frobo_ng

if __name__ == '__main__':

	random.seed()

	clockwise = int(sys.argv[1])


	c = frobo_ng()
	c.debug = True

	r = redis.Redis()
	use_camera(r)
	time.sleep(4)
	try:
		dbprint('BEFORE %s cm to %s' % (c.state['sonar'], c.heading()))
		for i in range(30):
			c.update_state()
			c.move_straight(fwd=True, max_secs=2, max_steps=c.m2steps(random.randint(10, 100)/100.), power=100)
			c.find_distance(60, clockwise=random.choice((True, False)))
			markers = collect_markers(r, fpath = os.path.join(html_path('data'), 'pic%d.jpg' % i))
			if markers:
				dbprint('Step %d. Found %d markers' % (i, len(markers)))
		dbprint('AFTER %s cm to %s' % (c.state['sonar'], c.heading()))
		json.dump(c.dots, file('dots.json', 'w'), indent=2)
	finally:
		make_shot(r)
		release_camera(r)
