import os, cv2
from imutils import paths, resize
from matplotlib import pyplot as plt

BASEPATH = 'data'
IMAGES = os.path.join(BASEPATH, 'cropped')
HISTOGRAMS = os.path.join(BASEPATH, 'histograms')

if not os.path.exists(HISTOGRAMS):
	os.makedirs(HISTOGRAMS)

for fname in paths.list_images(IMAGES):
	image = resize(cv2.imread(fname), width=100, height=100)
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	h,s,v = cv2.split(image)
	hist = cv2.calcHist([h], [0], None, [256], [0, 256])
#	hist /= hist.max()
	fig = plt.figure(figsize=(4.5, 1.8))
	ax = fig.add_subplot(121)
	ax.set_title("Histogram")
	ax.set_xlabel("Bins")
	ax.set_ylabel("# of Pixels")
	ax.set_ylim([0, 300])
	ax.plot(hist)
	ax.set_xlim([0, 256])
	ax = fig.add_subplot(122)
	ax.imshow(cv2.cvtColor(resize(image, width=180), cv2.COLOR_BGR2RGB), cmap=plt.cm.gray_r, interpolation="nearest")
#	ax.imshow(cv2.cvtColor(resize(image, width=180), cv2.COLOR_BGR2RGB), cmap=plt.cm.gray_r, interpolation="nearest")
	dfname = os.path.join(HISTOGRAMS, os.path.basename(fname))
	plt.savefig(dfname, dpi = (50))
	plt.close(fig)

print 'Finished'
