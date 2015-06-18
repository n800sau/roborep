#!/usr/bin/python

import subprocess, json, os, time
from flask import Flask, request
from fractions import Fraction
import picamera
from picamera.array import PiRGBArray
import cv2

app = Flask(__name__)

@app.route('/')
def hello_world():
	return 'Hello Cruel World!'


def img_fname():
	rs = os.path.expanduser('~n800s/public_html/image000.jpg')
	if os.path.exists(rs):
		os.unlink(rs)
	return rs

def the_camera():
	rs = picamera.PiCamera()
	rs.resolution = (80, 60)
	if 'brightness' in request.args:
		rs.brightness = int(request.args['brightness'])
	if 'contrast' in request.args:
		rs.contrast = int(request.args['contrast'])
	shutter = int(request.args.get('shutter', 0))
	if shutter > 0:
		rs.framerate = Fraction(1, shutter)
		rs.shutter_speed = shutter * 1000000
		rs.exposure_mode = 'off'
		rs.iso = 800
	return rs

@app.route('/threshold_image/<int:sti>')
def process_image(sti=3):
	style = (
		cv2.THRESH_BINARY,
		cv2.THRESH_BINARY_INV,
		cv2.THRESH_TRUNC,
		cv2.THRESH_TOZERO,
		cv2.THRESH_TOZERO_INV,
	)
	titles = ['BINARY','BINARY_INV','TRUNC','TOZERO','TOZERO_INV']
	# initialize the camera and grab a reference to the raw camera capture
	with the_camera() as camera:
		rawCapture = PiRGBArray(camera)
	# allow the camera to warmup
#		time.sleep(2)
	# grab an image from the camera
		camera.capture(rawCapture, format="bgr")
		img = rawCapture.array
		img = cv2.medianBlur(img,5)
		ret,img = cv2.threshold(img, 127, 255, style[sti])
		cv2.imwrite(img_fname(), img)
		rs = json.dumps({'result': '%s threshold an image at %s' % (titles[sti], time.strftime('%H:%M:%S'))})
	return rs

@app.route('/update_image/')
def update_image():
	with the_camera() as camera:
# led operation needs root access
# instead disable_camera_led=1 is added to /boot/config.txt
#		camera.led = False
#		camera.exposure_mode = 'auto'
#		time.sleep(1)
		shutter = int(request.args.get('shutter', 0))
		camera.capture(img_fname(), use_video_port=shutter == 0)
		rs = json.dumps({'result': 'Captured an image at %s with %d, %d, %d' % (
			time.strftime('%H:%M:%S'), camera.brightness, camera.contrast, shutter)})
	return rs

if __name__ == '__main__':
	app.debug = True
	app.run()
