import sys, os, time
import cv2

OFNAME = 'output.avi'
OSIZE = (640, 480)
MAX_SECONDS = 60

fps = 0.5

#vs = cv2.VideoCapture('/dev/video0')
#vs = cv2.VideoCapture('http://admin:9video9@192.168.1.94:38081/0/stream')
#vs = cv2.VideoCapture('http://192.168.1.132:3333/')
#vs = cv2.VideoCapture('http://192.168.1.132:5001/')
#vs = cv2.VideoCapture('http://192.168.1.132:8080/')
vs = cv2.VideoCapture('http://hubee.local:8080/stream')
time.sleep(1.0)

fourcc = cv2.VideoWriter_fourcc('X', 'V', 'I', 'D')
out = cv2.VideoWriter(OFNAME, fourcc, 30, OSIZE)

t = time.time()
t1 = time.time()
n = 0
while True:
	(grabbed, frame) = vs.read()
	t2 = time.time()
	dt = t2 - t1
	if grabbed and dt > 1/float(fps):
		t1 = t2
		print(n, ':', time.strftime('%H:%M:%S', time.localtime(t2)))
		n = n + 1
		oframe = cv2.resize(frame, OSIZE)
		out.write(oframe)
	if t2 - t > MAX_SECONDS:
		break

vs.release()
out.release()

