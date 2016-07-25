import cv2
from skimage import feature

IMG_SIDE = 100

def extract_image_h1c2d(image):
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	h,s,v = cv2.split(hsv)
	hist = cv2.calcHist([h], [0], None, [180], [0, 180])
#	print 'SHAPE', hist.shape
	hist /= hist.max()
	return hist.reshape(18, 10)

def extract_image_hs1c1d(image):
#	image = cv2.resize(image, (IMG_SIDE, IMG_SIDE))
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	h,s,v = cv2.split(hsv)
	hist = cv2.calcHist([h], [0], None, [180], [0, 180])
	hist /= hist.max()
	hist = hist.flatten()
#	print 'SHAPE', hist.shape
	return hist

def extract_image_hs2c2d(image):
#	image = cv2.resize(image, (IMG_SIDE, IMG_SIDE))
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	h,s,v = cv2.split(hsv)
	hist = cv2.calcHist([h, s], [0, 1], None, [180, 10], [0, 180, 0, 10])
#	print 'SHAPE', hist.shape
	hist /= hist.max()
	return hist
#	return hist.reshape(16, 16)

def extract_image_hog(image):
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	gray = cv2.resize(gray, (100, 100))
	rs = feature.hog(gray, orientations=9, pixels_per_cell=(8, 8), cells_per_block=(2, 2), transform_sqrt=True, visualise=False)
#	print rs.shape
	return rs


extract_image_data = extract_image_h1c2d
