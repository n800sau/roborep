import cv2

IMG_SIDE = 100

def extract_image_data(image):
#	image = cv2.resize(image, (IMG_SIDE, IMG_SIDE))
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	h,s,v = cv2.split(hsv)
	hist = cv2.calcHist([h], [0], None, [256], [0, 256])
	hist /= hist.max()
	return hist.reshape(16, 16)
