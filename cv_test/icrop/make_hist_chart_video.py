import os, cv2, sys
from imutils import paths, resize
import matplotlib
matplotlib.use('agg')
from matplotlib import pyplot as plt
import numpy as np

fourcc = cv2.cv.CV_FOURCC('X', 'V', 'I', 'D')

IVIDEOPATH = os.path.expanduser('~/work/cat_test_1920x2643000.avi')
OVIDEOPATH = 'cat_test_1920x2643000_hist.avi'

out = None
camera = cv2.VideoCapture(IVIDEOPATH)
fps = camera.get(cv2.cv.CV_CAP_PROP_FPS)

i = 0
while True:
	(grabbed, image) = camera.read()
	if image is None:
		break
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	h,s,v = cv2.split(image)
	hist = cv2.calcHist([h], [0], None, [256], [0, 256])
	hist /= hist.sum()
	fig = plt.figure(figsize=(4.5, 1.8), dpi=50)
	ax = fig.add_subplot(121)
	ax.set_title("Histogram")
	ax.set_xlabel("Bins")
	ax.set_ylabel("# of Pixels")
	ax.plot(hist)
	ax.set_xlim([0, 256])
	ax = fig.add_subplot(122)
	ax.imshow(cv2.cvtColor(resize(image, width=180), cv2.COLOR_BGR2RGB), cmap=plt.cm.gray_r, interpolation="nearest")
	fig.tight_layout(pad=0)
	fig.canvas.draw()
	data = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
	frame = data.reshape(fig.canvas.get_width_height()[::-1] + (3,))
	plt.close(fig)
	print i, frame.shape
	sys.stdout.flush()
	if i == 0:
		sz = list(reversed(frame.shape[:2]))
		out = cv2.VideoWriter(OVIDEOPATH, fourcc, fps, tuple(sz))
	out.write(frame)
	i += 1

if not out is None:
	out.release()
camera.release()
