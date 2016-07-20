import cv2

IMG_SIDE = 100

def extract_image_data(image):
#	image = cv2.resize(image, (IMG_SIDE, IMG_SIDE))
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	h,s,v = cv2.split(hsv)
	hist = cv2.calcHist([h, s], [0, 1], None, [180, 256], [0, 180, 0, 256])
#	print 'SHAPE', hist.shape
	hist /= hist.max()
	return hist
#	return hist.reshape(16, 16)
