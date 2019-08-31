# USAGE
# python recognize.py --images ../testing_lp_dataset

# import the necessary packages
from __future__ import print_function
from pyimagesearch.license_plate import LicensePlateDetector
from imutils import paths
import argparse
import imutils
import cv2
import numpy as np
from resultsmontage import ResultsMontage
import os, random

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--images", required=True, help="path to the images to be classified")
args = vars(ap.parse_args())

#montage = ResultsMontage((240, 320), 5, 10)
montage = ResultsMontage((120, 160), 5, 10)

random.seed()

imagePaths = list(paths.list_images(args["images"]))
imagePaths = random.sample(imagePaths, min(100, len(imagePaths)))

#imagePaths = ['/home/n800s/sshfs/asus/root/rus_hard/scaremaster/frame@20160104_101910.jpg']

found = 0
# loop over the images
for imagePath in sorted(imagePaths):
	# load the image
	image = cv2.imread(imagePath)

	rect = ((image.shape[1] // 3, 400), (image.shape[1] - 40, image.shape[0]))
	# hide useless areas
#	mask = np.zeros(image.shape[:2], dtype="uint8")
	#x1,y1  x2,y2
#	cv2.rectangle(mask, rect[0], rect[1], 255, -1)
#	image = cv2.bitwise_and(image, image, mask=mask)

	# crop it instead
	image = image[rect[0][1]:rect[1][1], rect[0][0]:rect[1][0]]

	print(imagePath)

	# if the width is greater than 640 pixels, then resize the image
#	if image.shape[1] > 640:
#		image = imutils.resize(image, width=640)

	# initialize the license plate detector and detect the license plates and charactors
	lpd = LicensePlateDetector(image)
	data = lpd.detect()
	plates = data['regions']

	if plates:
		images = data['images']

#	image = cv2.cvtColor(images[-1], cv2.COLOR_GRAY2BGR)

#	montage.addResult(images[0])

		print('found', len(plates))
		# loop over the license plate regions and draw the bounding box surrounding the
		# license plate
		for lpBox in plates:
#			cv2.drawContours(image, [lpBox], -1, (0, 255, 0), 2)
			ymin = np.min(lpBox[:,1])
			ymax = np.max(lpBox[:,1])
			xmin = np.min(lpBox[:,0])
			xmax = np.max(lpBox[:,0])
			print(xmax-xmin, ymax-ymin)
			plateimage = image[ymin:ymax, xmin:xmax]

			montage.addResult(plateimage)

			found += 1

			if found >= 10:
				break

		if found >= 10:
			break

#	cv2.imwrite(os.path.expanduser('~n800s/public_html/output.png'), imutils.resize(image, height=680))
	# display the output image
#	cv2.imshow("image", image)
#	cv2.waitKey(0)

cv2.imwrite(os.path.expanduser('~n800s/public_html/output.png'), imutils.resize(montage.montage, height=680))
