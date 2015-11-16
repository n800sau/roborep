#!/usr/bin/env python

import sys
import os
import glob
from picamera.array import PiRGBArray
from picamera import PiCamera
import argparse
import warnings
import datetime
import imutils
from imutils.object_detection import non_max_suppression
import json
import time
import cv2
import numpy as np

def dbprint(line):
	print >>sys.stderr, line

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-c", "--conf", required=False, help="path to the JSON configuration file")
args = vars(ap.parse_args())

# filter warnings, load the configuration
warnings.filterwarnings("ignore")
conf = json.load(open(args["conf"] if args['conf'] else 'conf.json'))
client = None

dbprint("Config: %s" % json.dumps(conf, indent=2))

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = tuple(conf["resolution"])
camera.framerate = conf["fps"]
rawCapture = PiRGBArray(camera, size=tuple(conf["resolution"]))
imdir = conf["imdir"]

cascade = cv2.CascadeClassifier(os.path.join(os.path.dirname(__file__), 'cat_cascade.xml'))

if not os.path.exists(imdir):
	os.makedirs(imdir)

# allow the camera to warmup, then initialize the average frame, last
# uploaded timestamp, and frame motion counter
dbprint("[INFO] warming up...")
time.sleep(conf["camera_warmup_time"])
avg = None
lastUploaded = datetime.datetime.now()
motionCounter = 0
frame_counter = 0

# cat range
#lowertuple = (0, 65, 115)
#highertuple = (23, 145, 227)

lowertuple = (145, 50, 33) 
highertuple = (168, 128, 117)

#lowertuple = (0, 0, 0)
#highertuple = (200, 245, 327)

ft = time.time()
fps = 0
# capture frames from the camera
for f in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

	# grab the raw NumPy array representing the image and initialize
	# the timestamp and occupied/unoccupied text
	frame = f.array
	timestamp = datetime.datetime.now()
	text = "Unoccupied"

#	frame = imutils.rotate(frame, angle=180)

	# resize the frame, convert it to grayscale, and blur it
#	frame = imutils.resize(frame, width=500)

	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	gray = cv2.GaussianBlur(gray, (21, 21), 0)

	# if the average frame is None, initialize it
	if avg is None:
		dbprint("[INFO] starting background model...")
		avg = gray.copy().astype("float")
		rawCapture.truncate(0)
		continue

	# accumulate the weighted average between the current frame and
	# previous frames, then compute the difference between the current
	# frame and running average
	cv2.accumulateWeighted(gray, avg, 0.5)
	frameDelta = cv2.absdiff(gray, cv2.convertScaleAbs(avg))

	# threshold the delta image, dilate the thresholded image to fill
	# in holes, then find contours on thresholded image
	thresh = cv2.threshold(frameDelta, conf["delta_thresh"], 255,
		cv2.THRESH_BINARY)[1]

#	thresh = cv2.adaptiveThreshold(frameDelta, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
	thresh = cv2.dilate(thresh, None, iterations=2)
	(cnts, _) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)

	ts = timestamp.strftime("%A %d %B %Y %I:%M:%S%p")

	cat_found = False
	occupied = False

	good_cnts = []
	# loop over the contours
	for c in cnts:

		area = cv2.contourArea(c)
		# if the contour is too small, ignore it
		if area < conf["min_area"]:
			continue

		# compute the bounding box for the contour, draw it on the frame,
		# and update the text
		(x, y, w, h) = cv2.boundingRect(c)

		if w > conf['max_width'] or h > conf['max_height']:
			continue

		good_cnts.append({'c':c, 'x': x, 'y': y, 'w': w, 'h': h, 'area': area})

		fimg = frame[y:y+h, x:x+w]
		gray = cv2.cvtColor(fimg, cv2.COLOR_BGR2GRAY)

		rects = cascade.detectMultiScale(gray, scaleFactor=1.3, minNeighbors=4, minSize=(30, 30), flags = cv2.cv.CV_HAAR_SCALE_IMAGE)
		if len(rects) > 0:
			# apply non-maxima suppression to the bounding boxes using a
			# fairly large overlap threshold to try to maintain overlapping
			# boxes that are still people
			rects = np.array([[rx, ry, rx + rw, ry + rh] for (rx, ry, rw, rh) in rects])
			pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)

			# draw the bounding boxes
			for (xA, yA, xB, yB) in pick:
				cv2.rectangle(frame, (x+xA, y+yA), (x+xB, y+yB), (0, 0, 255), 2)
			dbprint('Cat found')
			cat_found = True


#		fblur = cv2.GaussianBlur(fimg, (11, 11), 0)
#		fblur = cv2.cvtColor(fblur, cv2.COLOR_BGR2HSV)
#		mask = cv2.inRange(fimg, np.array(lowertuple), np.array(highertuple))
#		mask = cv2.erode(mask, None, iterations=2)
#		mask = cv2.dilate(mask, None, iterations=2)
#		cv2.bitwise_and(fimg, fimg, frame[y:y+h, x:x+w], mask = mask)
#		dbprint('Cat pixels count %d in shape %s' % (np.count_nonzero(mask), fimg.shape))
#		if np.count_nonzero(mask) > 0:
#			dbprint('Cat pixels count %d in shape %s' % (np.count_nonzero(mask), fimg.shape))
#			cat_found = True

		cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
		text = "Occupied"
		occupied = True
		if cat_found:
			text += " with CAT"

	if good_cnts:
		dbprint('Found at %s %d cnts' % (ts, len(good_cnts)))

	for c in good_cnts:
		dbprint('\t%d,%d %dx%d area=%d' % (c['x'], c['y'], c['w'], c['h'], c['area']))

	frame_counter += 1
	t = time.time()

	if t - ft > conf['shot_timeout']:
		fps = frame_counter / (t -ft)
		frame_counter = 0

	# draw the text and timestamp on the frame
	cv2.putText(frame, "Room Status: {} fps: {:.2f}".format(text, fps), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
	cv2.putText(frame, ts, (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

	if frame_counter == 0:
		fname = 'current.jpg'
		fname_tmp = 'current_tmp.jpg'
		cv2.imwrite(os.path.join(imdir, fname_tmp), imutils.resize(frame, width=320))
		os.rename(os.path.join(imdir, fname_tmp), os.path.join(imdir, fname))
		fname = 'thresh.jpg'
		fname_tmp = 'thresh_tmp.jpg'
		cv2.imwrite(os.path.join(imdir, fname_tmp), imutils.resize(thresh, width=320))
		os.rename(os.path.join(imdir, fname_tmp), os.path.join(imdir, fname))
		ft = t

	# check to see if the room is occupied
	if occupied and cat_found:
		# check to see if enough time has passed between uploads
		if (timestamp - lastUploaded).seconds >= conf["min_upload_seconds"]:
			# increment the motion counter
			motionCounter += 1

			# check to see if the number of frames with consistent motion is
			# high enough
			if motionCounter >= conf["min_motion_frames"]:

				fname = 'motion@%s.jpg' % timestamp.strftime('%Y%m%d_%H%M%S')
				slname = os.path.join(imdir, 'motion.jpg')
				cv2.imwrite(os.path.join(imdir, fname), imutils.resize(frame, width=320))
				if os.path.exists(slname):
					os.unlink(slname)
				os.symlink(os.path.join(imdir, fname), slname)

				dbprint("Image %s written" % fname)

				existing = glob.glob(os.path.join(imdir, 'motion@*.jpg'))
				cnt = len(existing)
				if cnt > conf['max_files']:
					existing.sort()
					for fn in existing[:cnt-conf['max_files']]:
						try:
							os.unlink(fn)
							dbprint("Image %s deleted" % fn)
						except Exception, e:
							dbprint("Can not delete image %s: %s" % (fn, e))

				# update the last uploaded timestamp and reset the motion
				# counter
				lastUploaded = timestamp
				motionCounter = 0

	# otherwise, the room is not occupied
	else:
		motionCounter = 0

	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)
