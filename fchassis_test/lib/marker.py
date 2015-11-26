#!/usr/bin/env python

import os, json, cv2
from utils import dbprint, html_data_path

SERVANT = 'raspiCamServant'
QUEUE = SERVANT + '.js.obj'

# http://www.truetex.com/raspberrypi

# for 2592 x 1944 still
#V_ANGLE = 40
# H_ANGLE = 53

# for 1920 x 1080 video
V_ANGLE = 22
H_ANGLE = 39

def make_fpath(fname=None):
	return html_data_path(fname or 'picam_0.jpg')

def use_camera(r, **params):
	params.update({
		'cmd': 'start_camera',
	})
	r.publish(SERVANT, json.dumps(params))

def release_camera(r):
	r.publish(SERVANT, json.dumps({
		'cmd': 'stop_camera',
	}))

def collect_markers(r, fpath=None):
	rs = []
	r.delete(QUEUE)
	params = {
		'cmd': 'find_markers',
		'draw_markers': True
	}
	if fpath:
		params['path'] = fpath
	r.publish(SERVANT, json.dumps(params))
	dbprint('Published %s' % json.dumps(params, indent=2))
	v = r.blpop(QUEUE, timeout=40)
	if v:
		v = json.loads(v[1])
#		dbprint('DATA=%s' % (json.dumps(v, indent=2),))
		dbprint('FOUND %d markers: %s' % (len(v['markers']), ','.join([str(m['id']) for m in v['markers']])))
		rs = v['markers']
	else:
		dbprint('No answer')
	return rs


def get_marker(r, marker_id, fpath=None):
	rs = None
	markers = collect_markers(r, fpath=fpath)
	for m in markers:
		dbprint('\t%s' % m['id'])
#		os.system('espeak "%s"' % ' '.join([c for c in str(m['id'])]))
		if m['id'] == marker_id:
			rs = {'coords': m['coords']}
			for c in rs['coords']:
				c['x'] /= m['width']
				c['y'] /= m['height']
			xmin = xmax = m['coords'][0]['x']
			ymin = ymax = m['coords'][0]['y']
			for c in m['coords'][1:]:
				xmin = min(xmin, c['x'])
				xmax = max(xmax, c['x'])
				ymin = min(ymin, c['y'])
				ymax = max(ymax, c['y'])
			rs['dot'] = {'x': (xmax+xmin)/2, 'y': (ymax+ymin)/2}
			rs['fpath'] = fpath
			break
	return rs

def marker_offset(r, marker_id, fpath=None):
	rs = None
	if fpath is None:
		fpath = html_data_path('pic0.jpg')
	marker = get_marker(r, marker_id, fpath=fpath)
	if marker:
#		dbprint('Marker: %s' % json.dumps(marker, indent=2))
		frame = cv2.imread(marker['fpath'])
		h,w = frame.shape[:2]
		x = int(w * marker['dot']['x'])
		y = int(h * marker['dot']['y'])
		cv2.circle(frame, (x, y), max(20, frame.shape[0] / 20), (0, 0, 255), -1)
		cv2.imwrite(html_data_path('pic1.jpg'), frame)
		h_off = (marker['dot']['x'] - 0.5) * H_ANGLE
		v_off = (marker['dot']['y'] - 0.5) * V_ANGLE
		rs = h_off
	return rs

def make_shot(r, fpath=None, **params):
	if fpath is None:
		fpath = make_fpath()
	r.delete(QUEUE)
	params.update({
		'cmd': 'make_shot',
		'path': fpath,
	})
	r.publish(SERVANT, json.dumps(params))
	v = r.blpop(QUEUE, timeout=5)
	if v:
		v = json.loads(v[1])
#		dbprint('DATA=%s' % (json.dumps(v, indent=2),))
	else:
		raise Exception('make_shot: answer queue timeout')
	