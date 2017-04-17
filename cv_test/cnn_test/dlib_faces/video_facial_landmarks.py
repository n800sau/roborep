#!/usr/bin/env python

# import the necessary packages
from imutils.video import WebcamVideoStream
from imutils import face_utils
import datetime
import argparse
import imutils
import time
import dlib
import cv2

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-p", "--shape-predictor", required=True, help="path to facial landmark predictor")
ap.add_argument("-i", "--input", required=True, help="video file name")
ap.add_argument("-o", "--output", default=None, help="output video file name")
args = vars(ap.parse_args())

# initialize dlib's face detector (HOG-based) and then create
# the facial landmark predictor
print("[INFO] loading facial landmark predictor...")
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor(args["shape_predictor"])


fourcc = cv2.cv.CV_FOURCC('X', 'V', 'I', 'D')
out = None
fps = 30

# initialize the video stream and allow the cammera sensor to warmup
print("[INFO] camera sensor warming up...")
vs = WebcamVideoStream(args["input"]).start()
time.sleep(2.0)

# loop over the frames from the video stream
while True:
	# grab the frame from the threaded video stream, resize it to
	# have a maximum width of 400 pixels, and convert it to
	# grayscale
	frame = vs.read()
	frame = imutils.resize(frame, width=400)
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	# detect faces in the grayscale frame
	rects = detector(gray, 0)

	print 'Found {}'.format(len(rects))

	# loop over the face detections
	for rect in rects:
		# determine the facial landmarks for the face region, then
		# convert the facial landmark (x, y)-coordinates to a NumPy
		# array
		shape = predictor(gray, rect)
		shape = face_utils.shape_to_np(shape)

		# loop over the (x, y)-coordinates for the facial landmarks
		# and draw them on the image
		for (x, y) in shape:
			cv2.circle(frame, (x, y), 1, (0, 0, 255), -1)
	
	if len(rects) > 0 and args["output"]:
		if out is None:
			sz = list(reversed(frame.shape[:2]))
			out = cv2.VideoWriter(OVIDEOPATH, fourcc, fps, tuple(sz))
		out.write(frame)

	# show the frame
	cv2.imshow("Frame", frame)
#	key = cv2.waitKey(1) & 0xFF

	# if the `q` key was pressed, break from the loop
#	if key == ord("q"):
#		break

if not out is None:
	out.release()

# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()
