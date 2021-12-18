#!/usr/bin/env python3

import sys
import time
import cv2
import datetime

IFNAME = '/dev/video0'
IFNAME = 'test.mp4'
OSIZE = (640, 480)
PREVIEW_SIZE = (960, 720)
EXT = 'avi'
#FORMAT = 'XVID'
#FORMAT = 'VP90'
FORMAT = 'MJPG'

MAX_MODE = 4
mode_labels = [
	{
		'label': '0.1mm (x4)',
		'x': 'x4',
		'sz': 59
	},
	{
		'label': '0.1mm (x10)',
		'x': 'x10',
		'sz': 140
	},
	{
		'label': '10 mkm (x40)',
		'x': 'x40',
		'sz': 55
	},
	{
		'label': '10 mkm (x60)',
		'x': 'x60',
		'sz': 88
	},
]

def capture(fname):

	mode = 0
	do_write = False

	fps = 30
#	vs = cv2.VideoCapture(fname)
	vs = cv2.VideoCapture(0)
	vs.set(cv2.CAP_PROP_FPS, fps)
	vs.set(cv2.CAP_PROP_FOURCC , cv2.VideoWriter_fourcc(*'MJPG'));
#	fps = vs.get(cv2.CAP_PROP_FPS)

	time.sleep(1.0)

	fourcc = cv2.VideoWriter_fourcc(*FORMAT)
	out = None

	while True:
		(grabbed, frame) = vs.read()
		if grabbed:
			oframe = cv2.resize(frame, OSIZE)
			x1 = (OSIZE[0] - mode_labels[mode]['sz']) // 2
			x2 = x1 + mode_labels[mode]['sz']
			v = mode_labels[mode]['label']
			clr = (0, 0, 255)
			cv2.line(oframe, (x1, 40), (x2, 40), clr, 2)
			cv2.line(oframe, (x1, 30), (x1, 50), clr, 2)
			cv2.line(oframe, (x2, 30), (x2, 50), clr, 2)
			cv2.putText(oframe, v, (x2+10, 45) , cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255))
			previmg = cv2.resize(oframe, PREVIEW_SIZE)
			cv2.imshow('preview', previmg)
			k = cv2.waitKey(1)
			if k>= 0:
				print('key', k)
				if k == 27: # esc
					break
				elif k == 119 and not do_write: # w
					print('write on')
					do_write = True
				elif k == 115 and do_write: # s
					print('write off')
					do_write = False
				elif k == 49: # 1
					print('mode 1')
					mode = 0
				elif k == 50: # 2
					print('mode 2')
					mode = 1
				elif k == 51: # 3
					print('mode 3')
					mode = 2
				elif k == 52: # 4
					print('mode 4')
					mode = 3
				elif k == 105: # i
					ofname = 'shot_{}_{}.{}'.format(mode_labels[mode]['x'], datetime.datetime.now().strftime('%Y-%m-%d_%H_%M'), 'png')
					print('screenshort')
					cv2.imwrite(ofname, oframe)
			if do_write:
				if out is None:
					ofname = 'cap_{}.{}'.format(datetime.datetime.now().strftime('%Y-%m-%d_%H_%M'), EXT)
					out = cv2.VideoWriter(ofname, fourcc, fps, OSIZE)
					fcount = 0
					tstart = time.time()
				out.write(oframe)
				fcount += 1
				if fcount % 100 == 0:
					print('frame {:3d}\x0d'.format(fcount), end='')
					sys.stdout.flush()
			elif not out is None:
				out.release()
				out = None
				dt = time.time()-tstart
				print('time: {:d}s, fps: {:.2f}'.format(int(dt), fcount/dt))
		else:
			break

	vs.release()
	if not out is None:
		out.release()
	cv2.destroyAllWindows()

if __name__ == '__main__':
	capture(IFNAME)
