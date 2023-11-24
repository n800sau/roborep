import sys, os, time
import cv2

OFNAME = 'output.avi' if len(sys.argv) < 2 else sys.argv[1]
PREVIEW_SIZE = (640, 480)
#OSIZE = (640, 480)
OSIZE = (1280, 720)
# 5 hours should be enough
MAX_SECONDS = 60*60*5

#fps = 0.2
fps = 30

LAPSE_ID_FNAME = 'lapse_id.txt'
lapse_id = open(LAPSE_ID_FNAME).read().strip() if os.path.exists(LAPSE_ID_FNAME) else ''

#vs = cv2.VideoCapture('/dev/video0')
#vs = cv2.VideoCapture('http://admin:9video9@192.168.1.94:38081/0/stream')
#vs = cv2.VideoCapture('http://192.168.1.132:3333/')
#vs = cv2.VideoCapture('http://192.168.1.132:5001/')
#vs = cv2.VideoCapture('http://192.168.1.132:8080/')
#vs = cv2.VideoCapture('http://hubee.local:8090/?action=stream')
#vs = cv2.VideoCapture('http://hubee.local:9000/cam/stream.mjpg')
vs = cv2.VideoCapture('http://spectra.local:9000/cam/stream.mjpg')
#vs = cv2.VideoCapture('http://hubee.local:9001/')
#vs = cv2.VideoCapture('http://hubee.local:9000/video_feed')
#vs = cv2.VideoCapture('http://bbspeaker.local:82/cam/video_feed')
time.sleep(1.0)

fourcc = cv2.VideoWriter_fourcc('X', 'V', 'I', 'D')
out = cv2.VideoWriter(OFNAME, fourcc, 10, OSIZE)

t = time.time()
t1 = time.time()
n = 0
not_printed = True
while True:
	(grabbed, frame) = vs.read()
	t2 = time.time()
	dt = t2 - t1
	if grabbed:
		not_printed = True
		cv2.imshow('preview', cv2.resize(frame, PREVIEW_SIZE))
		cv2.waitKey(1)
		if dt > 1/float(fps):
			t1 = t2
			print(n, ':', time.strftime('%H:%M:%S', time.localtime(t2)))
			n = n + 1
			oframe = cv2.resize(frame, OSIZE)
			cv2.putText(oframe, lapse_id + time.strftime(' %d/%m/%Y %H:%M:%S'), (20, 40) , cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255))
			out.write(oframe)
			cv2.imshow('lapsed', cv2.resize(oframe, PREVIEW_SIZE))
			k = chr(cv2.waitKey(1))
			if k.upper() == 'S':
				cv2.imwrite('frame_{}.jpg'.format(n), frame)
	else:
		if not_printed:
			print('Not grabbed')
			not_printed = False
	if t2 - t > MAX_SECONDS:
		break

vs.release()
out.release()
cv2.destroyAllWindows()
