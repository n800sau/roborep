#!/usr/bin/env python

# import the necessary packages
from imutils import face_utils
from imutils.paths import list_images
import datetime
import argparse
import imutils
import time
import dlib
import cv2
import os, sys
import face_recognition

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-p", "--shape-predictor", required=True, help="path to facial landmark predictor")
ap.add_argument("-i", "--input", required=True, help="video file name")
ap.add_argument("-o", "--output", default=None, help="output video file name")
ap.add_argument("-s", "--search", default=None, help="directory of face images to search for")
args = vars(ap.parse_args())

ACTON_FRAME = 10
FACE_DIR = 'faces'

# initialize dlib's face detector (HOG-based) and then create
# the facial landmark predictor
print("[INFO] loading facial landmark predictor...")
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor(args["shape_predictor"])


fourcc = cv2.cv.CV_FOURCC('X', 'V', 'I', 'D')
out = None

if args['search']:
	search_face = {}
	search_face_encoding = {}
	for fname in list_images(args['search']):
		bname = os.path.basename(fname)
		search_face[bname] = face_recognition.load_image_file(fname)
		search_face_encoding[bname] = face_recognition.face_encodings(search_face[bname])[0]
else:
	search_face = None

# initialize the video stream
vs = cv2.VideoCapture(args["input"])

if not vs.isOpened(): 
	print >>sys.stderr, "could not open :", args["input"]
	sys.exit(1)

n_frames = int(vs.get(cv2.cv.CV_CAP_PROP_FRAME_COUNT))
width  = int(vs.get(cv2.cv.CV_CAP_PROP_FRAME_WIDTH))
height = int(vs.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT))
fps = vs.get(cv2.cv.CV_CAP_PROP_FPS)

tt = time.time()
i = 1
try:

	# loop over the frames from the video stream
	while True:
		# grab the frame from the threaded video stream, resize it to
		# have a maximum width of 400 pixels, and convert it to
		# grayscale
		(grabbed, frame) = vs.read()

		if not grabbed:
			break

		# every {ACTON_FRAME}th frame
		if i % ACTON_FRAME:

#			frame = imutils.resize(frame, width=400)
			gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

			# detect faces in the grayscale frame
			t = time.time()
			rects = detector(gray, 0)

			print '{: 4}/{}: Found {} in {:.2f} sec'.format(i, n_frames, len(rects), time.time()-t)

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

			if not search_face is None:
				# Find all the faces and face enqcodings in the frame of video
				face_locations = face_recognition.face_locations(frame)
				face_encodings = face_recognition.face_encodings(frame, face_locations)

				found = False
				# Loop through each face in this frame of video
				for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):

					for bname,sf_encoding in search_face_encoding.items():
						print('Searching for {} location {}...'.format(bname, (top, right, bottom, left)))
						# See if the face is a match for the known face(s)
						match = face_recognition.compare_faces([sf_encoding], face_encoding, tolerance=0.6)

						if match[0]:
							# Draw a box around the face
							cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)
							if not os.path.exists(FACE_DIR):
								os.makedirs(FACE_DIR)
							cv2.imwrite(os.path.join(FACE_DIR, '{:02d}_{}.jpg'.format(i, bname)), frame)
							found = True
							print('...Found searched in frame {}'.format(i))
						else:
							print('...Not found')

#			if len(rects) > 0 and args["output"]:
			if args["output"]:
				oframe = imutils.resize(frame, width=720)
				if out is None:
					sz = list(reversed(oframe.shape[:2]))
					out = cv2.VideoWriter(args["output"], fourcc, fps, tuple(sz))
				for j in range(ACTON_FRAME):
					out.write(oframe)

			# show the frame
#			cv2.imshow("Frame", frame)
#			key = cv2.waitKey(1) & 0xFF

			# if the `q` key was pressed, break from the loop
#			if key == ord("q"):
#				break
		i += 1

finally:
	tdiff = (time.time() - tt)
	print('Done in {:.2f} secs, ({:.2f} fps)'.format(tdiff, i/tdiff))

if not out is None:
	out.release()

# do a bit of cleanup
#cv2.destroyAllWindows()

print('Finished')
