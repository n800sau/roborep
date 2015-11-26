#!/usr/bin/env python

import sys, os, time, json

import matplotlib as mpl
mpl.use('Agg')
import picamera, cv2
from lib.utils import dbprint, html_data_path
from lib.camera import update_img, ColorFix
from matplotlib import pyplot as plt

#hsvLower = [0, 10, 10]
#hsvUpper = [64, 255, 255]

# cat range
hsvLower = [0, 65, 115]
hsvUpper = [23, 145, 227]


if __name__ == '__main__':

	with picamera.PiCamera() as camera:

		ss = ColorFix(camera)
		try:
			if True:
#			data = ss.colorise()
#			data = ss.colorise1()
#			data = ss.locate_object(hsvLower, hsvUpper)
#			w = 10
#			i = 0
#			for l in range(0, 200, w):
#				hsvLower[0] = l
#				hsvUpper[0] = l + w
#				data = ss.mask_range(hsvLower, hsvUpper)
#				data = ss.gradients()
#				data = ss.blur()
#				data = ss.edges()
				data = ss.contours(brightness=100, contrast=100)
				if data:
#					cv2.imwrite(html_data_path('frame_%03d.jpg' % i), data['frame'])
#					cv2.imwrite(html_data_path('iframe_%03d.jpg' % i), data['iframe'])
#					cv2.imwrite(html_data_path('oframe_%03d.jpg' % i), data['oframe'])
					cv2.imwrite(html_data_path('frame.jpg'), data['frame'])
					cv2.imwrite(html_data_path('iframe.jpg'), data['iframe'])
					cv2.imwrite(html_data_path('oframe.jpg'), data['oframe'])
#					for l in data['hlist']:
#						fig = plt.figure()
#						ax = fig.add_subplot(111)
#						ax.hist(l['hist'])
#						plt.savefig(html_data_path('his_%d_%d.jpg' % (l['iy'], l['ix'])))
#						cv2.imwrite(html_data_path('img_%d_%d.jpg' % (l['iy'], l['ix'])), l['img'])
#					json.dump(data['hlist'], file('colorfix.json', 'w'))
				else:
					dbprint("NOT FOUND")
#			json.dump({'imgcount': i}, file(html_data_path('frames.json'), 'w'), indent=2)
		finally:
			update_img(camera)
