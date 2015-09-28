#!/usr/bin/env python

import sys, os, time, json, redis, picamera

from lib.utils import dbprint
from lib.camera import update_img

from lib.frobo_ng import frobo_ng

servant = 'raspiCamServant'
queue = 'raspiCamServant.js.obj'

#TARGET = 618
target_loc = None

i = 0
def stop_cb(c, r):
	global i
	rs = False
	dbprint('h: %s' % c.compass.heading())
	r.delete(queue)
	r.publish(servant, json.dumps({
		'cmd': 'find_markers',
		'path': os.path.join(os.path.expanduser('~/public_html'), 'pic%d.jpg' % i),
		'draw_markers': True
	}))
	v = r.blpop(queue, timeout=2)
	if v:
		v = json.loads(v[1])
#		dbprint('DATA=%s' % (v,))
		if v['markers']:
			dbprint('FOUND %d markers' % len(v['markers']))
			for m in v['markers']:
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
		r.publish(servant, json.dumps({
			'cmd': 'start_camera',
		}))
		c.update_state()
		dbprint('BEFORE %s m to %s' % (c.state['sonar'], c.compass.heading()))
		c.search_around(lambda c, r=r: stop_cb(c, r), clockwise=clockwise)
		dbprint('AFTER %s m to %s (%d steps)' % (c.state['sonar'], c.compass.heading(), i))
		json.dump(c.dots, file('dots.json', 'w'), indent=2)
	finally:
		r.publish(servant, json.dumps({
			'cmd': 'stop_camera',
		}))
		c.cmd_mstop()
		time.sleep(6)
		update_img(picamera.PiCamera())

	dbprint('TARGET: %s' % target_loc)

