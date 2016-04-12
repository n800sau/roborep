#!/usr/bin/env python

# to extract masked image

import os, glob, json, time, random
import cv2
import imutils
import numpy as np


ROOT_PATH = os.path.expanduser('~/sshfs/asus/root/rus_hard/garage')
BASE_DIR = '2016-03-12'
IMAGE_NAME = '00D6FB009223(n800sau)_1_20160312082657_80096.jpg'

OUT_PATH = os.path.join(os.path.dirname(__file__), 'output')


def write_image(image, fname):
	fn = os.path.join(OUT_PATH, fname)
	cv2.imwrite(fn, imutils.resize(image, width=160))

def apply_mask(image):
	mask = np.zeros(image.shape[:2], dtype="uint8")
#	cv2.rectangle(mask, (70, 270), (90, image.shape[0]), 255, -1)
	cv2.rectangle(mask, (int(image.shape[1]*.09), int(image.shape[0]*.35)), (int(image.shape[1]*.7), int(image.shape[0])), 255, -1)
	image = cv2.bitwise_and(image, image, mask=mask)
	return image
# to use car root area only
#	return image[90:, 70:270]

def process_image(image):
	image = apply_mask(image)
	image = imutils.resize(image, height=480)
	write_image(image, 'masked.png')

if __name__ == '__main__':

	time_mark = int(time.time())

	fn = os.path.join(ROOT_PATH, BASE_DIR, IMAGE_NAME)

	image = cv2.imread(fn)
	process_image(image)

