#!/usr/bin/env python

import sys, os, time, json, redis, picamera

from lib.utils import dbprint
from lib.camera import update_img

from lib.frobo import frobo

servant = 'raspiCamServant'
queue = 'raspiCamServant.js.obj'

i = 0
def stop_cb(c, r):
	global i
	rs = False
	while True:
		v = r.lpop(queue)
		if v:
			v = json.loads(v)
			dbprint('FOUND %d markers' % len(v['markers']))
			for m in v['markers']:
				dbprint('%s' % m['id'])
				#os.system('espeak %s' % m['id'])
				#time.sleep(1)
		else:
			break
	r.publish(servant, json.dumps({
		'cmd': 'find_markers',
		'path': os.path.join(os.path.expanduser('~/public_html'), 'pic%d.jpg' % i),
		'draw_markers': True
	}))
	i += 1
	return rs

if __name__ == '__main__':

	clockwise = int(sys.argv[1])

	s_port = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH00ZTCM-if00-port0'

	r = redis.Redis()
	r.delete(queue)

	with frobo(s_port) as c:
		#c.debug = False

		try:
			r.publish(servant, json.dumps({
				'cmd': 'start_camera',
			}))
			c.update_dist()
			dbprint('BEFORE %s cm to %s' % (c.curr_dist, c.compass.heading()))
			c.search_around(lambda c, r=r: stop_cb(c, r), clockwise=clockwise)
			dbprint('AFTER %s cm to %s (%d steps)' % (c.curr_dist, c.compass.heading(), i))
			json.dump(c.dots, file('dots.json', 'w'), indent=2)
		finally:
			r.publish(servant, json.dumps({
				'cmd': 'stop_camera',
			}))
			c.stop()
			time.sleep(6)
			update_img(picamera.PiCamera())
