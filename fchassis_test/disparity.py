#!/usr/bin/env python

import sys, os, time, json

import picamera, cv2
from lib.utils import dbprint, html_data_path
from lib.camera import update_img, StereoDisparity

from lib.frobo_ng import frobo_ng

if __name__ == '__main__':

	c = frobo_ng()
#	c.debug = True

	with picamera.PiCamera() as camera:

		fp = StereoDisparity(camera)
		try:
			c.tick_left(min_angle=1)
			fp.left_frame()
			c.tick_right(min_angle=1)
			fp.right_frame()
			fp.write_ply(html_data_path('disparity.ply'))
			cv2.imwrite(html_data_path('pic0.jpg'), fp.lframe)
			cv2.imwrite(html_data_path('pic1.jpg'), fp.rframe)
			cv2.imwrite(html_data_path('pic2.jpg'), fp.disparity)
		finally:
			update_img(camera)
