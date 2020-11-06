import numpy as np
import cv2
import torch

#  pip install imgaug
import imgaug as ia
from imgaug import augmenters as iaa

class ResizeImage(object):
	def __init__(self, new_size, interpolation=cv2.INTER_LINEAR):
		self.new_size = tuple(new_size) #  (w, h)
		self.interpolation = interpolation

	def __call__(self, sample):
		sample['image'] = cv2.resize(sample['image'], self.new_size, interpolation=self.interpolation)

class ImageBaseAug(object):
	def __init__(self):
		sometimes = lambda aug: iaa.Sometimes(0.5, aug)
		self.seq = iaa.Sequential(
			[
				# Blur each image with varying strength using
				# gaussian blur (sigma between 0 and 3.0),
				# average/uniform blur (kernel size between 2x2 and 7x7)
				# median blur (kernel size between 3x3 and 11x11).
				iaa.OneOf([
					iaa.GaussianBlur((0, 3.0)),
					iaa.AverageBlur(k=(2, 7)),
					iaa.MedianBlur(k=(3, 11)),
				]),
				# Sharpen each image, overlay the result with the original
				# image using an alpha between 0 (no sharpening) and 1
				# (full sharpening effect).
				sometimes(iaa.Sharpen(alpha=(0, 0.5), lightness=(0.75, 1.5))),
				# Add gaussian noise to some images.
				sometimes(iaa.AdditiveGaussianNoise(loc=0, scale=(0.0, 0.05*255), per_channel=0.5)),
				# Add a value of -5 to 5 to each pixel.
				sometimes(iaa.Add((-5, 5), per_channel=0.5)),
				# Change brightness of images (80-120% of original value).
				sometimes(iaa.Multiply((0.8, 1.2), per_channel=0.5)),
				# Improve or worsen the contrast of images.
				sometimes(iaa.LinearContrast((0.5, 2.0), per_channel=0.5)),
			],
			# do all of the above augmentations in random order
			random_order=True
		)

	def __call__(self, sample):
		seq_det = self.seq.to_deterministic()
		sample['image'] = seq_det.augment_images([sample['image']])[0]
