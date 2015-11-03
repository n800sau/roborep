#!/usr/bin/env python

import sys, os, time, json

import picamera, cv2
from lib.utils import dbprint, html_data_path
from lib.camera import update_img, ShapeSearch, capture_cvimage

if __name__ == '__main__':

	with picamera.PiCamera() as camera:

		ss = ShapeSearch(camera)
		try:
			i = 0
			t_start = 120
			t_end = 200
			cnt = 20
			for tt in [cv2.THRESH_BINARY, cv2.THRESH_BINARY_INV, cv2.THRESH_TRUNC, cv2.THRESH_TOZERO, cv2.THRESH_TOZERO_INV]:
				for t in range(t_start, 255, (t_end - t_start) / cnt):
					data = ss.find_shapes(threshold=t, threshold_type=tt)
					if data:
						if os.environ.get('RASPICAM_ROTATE', ''):
							angle = int(os.environ['RASPICAM_ROTATE'])
							rows,cols,depth = data['frame'].shape
							M = cv2.getRotationMatrix2D((cols/2,rows/2), 180, 1)
							data['frame'] = cv2.warpAffine(data['frame'], M, (cols,rows))
						cv2.imwrite(html_data_path('shapes_%03d.jpg' % i), data['frame'])
#						cv2.putText(data['thresh'], 'Thresh: %d' % t, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
						cv2.putText(data['thresh'], 'Thresh: %d, Type: %d' % (t, tt), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
						cv2.imwrite(html_data_path('thresh_%03d.jpg' % i), data['thresh'])
						i += 1
						dbprint(('*' * 10) + ' Image %d written' % i)
			json.dump({'imgcount': i}, file(html_data_path('shapes.json'), 'w'), indent=2)
		finally:
			update_img(camera)
