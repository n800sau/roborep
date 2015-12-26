#!/usr/bin/env python

import sys, os, time, redis, json

import picamera
from lib.camera import update_img
from lib.utils import dbprint
from lib.marker import collect_markers, use_camera, release_camera, make_shot
from lib.utils import html_data_path

# 0.87 m - 3cm -> 55.4564987823 px

#F = (101px x 24in) / 4in = 606

#F = (55.4564987823 * 0.87) / 0.03 ~ 1608


# 1.86 m - 3 cm -> 38.4478816566 px

#F = (38.4478816566 * 1.86) / 0.03 ~ 2384
#D = 1608 * 0.03 / 38.4478816566 ~ 1.25 m

#D = F(3.60mm) * 0.03 * image_height(960px) / (obj_height(pixels) * sensor_height(2.74mm))

#1280x960
#D = 0.0036 * 0.03 * 960 / (38.45 * 0.0027) ~ 1m (sonar- 2m)

#640x480
#D = 0.0036 * 0.03 * 480 / (18.86 * 0.0027) ~ 1m (sonar- 2m)

#D = 0.0036 * 0.03 * 480 / (30 * 0.0027) ~ 0.64m (sonar- 0.89m)

#2592 x 1944
#2.74/0.0014=1957
#3.67/0.0014=2621
#pixel size = 1.4 x 1.4

#sz=math.sqrt((x1-x2)**2 + (y1-y2)**2)

# measured 1m 23.76px
#D = 0.0036 * 0.03 * 480 / (23.76 * 0.0027) ~ 0.808 (ruler- 1m)

#F = (23.76 * 1.) / 0.03 ~ 792

#F = (49.4687 * 0.5) / 0.03 ~ 824

#F = (34.6151 * 0.7) / 0.03 ~ 808

#Favg = (792+824+808)/3 = 808

#D = 808 * 0.03 / 50.68 ~ 0.478 (measured 0.49)
#D = 808 * 0.03 / 32.31 ~ 0.75 (measured 0.745)

if __name__ == '__main__':

	if 1:
		r = redis.Redis()
		use_camera(r)
#		use_camera(r, brightness=80, contrast=85)
#		use_camera(r, width=640, height=480, brightness=80, contrast=80)
#		use_camera(r, brightness=90, contrast=90)
		time.sleep(4)
		try:
			markers = collect_markers(r, fpath = html_data_path('markers.jpg'))
			print json.dumps(dict([(m['id'], m['distance']) for m in markers]), indent=2)
		finally:
			time.sleep(1)
			make_shot(r)
			release_camera(r)
	else:
		cam = picamera.PiCamera()
		time.sleep(2)
		update_img(cam)
