# USAGE
# python test.py --arch cifar10_deploy.prototxt \
#	--snapshot output/cifar10/snapshots/cifar10_iter_70000.caffemodel.h5 \
#	--mean output/cifar10/dataset_mean.binaryproto --val output/cifar10/val.txt \
#	--test-images test_images

# import the necessary packages
from __future__ import print_function
from imutils import paths
import numpy as np
import argparse
import imutils
import random
import caffe
import cv2
import sys
import os
import time

def arg_parser():
	# construct the argument parser and parse the command line arguments
	ap = argparse.ArgumentParser()
	ap.add_argument("-a", "--arch", required=True, help="path to network architecture definition")
	ap.add_argument("-c", "--classlist", required=True, help="path to class list file")
	ap.add_argument("-s", "--snapshot", required=True, help="path network snapsot")
	ap.add_argument("-m", "--mean", required=True, help="path to mean image")
	ap.add_argument("-g", "--gpu", type=int, default=-1, help="GPU device index")
	ap.add_argument("-o", "--outpath", required=False, help="path for classified links")
	ap.add_argument("-d", "--dim", required=True, help="image dimention WxH")
	return ap


class CNNClassificator:

	def __init__(self, classlist, snapshot, arch, mean, gpu=-1, output=None, dim=[]):
		self.gtLabels = [v for k,v in sorted([(int(k), v) for k,v in [l.split(' ') for l in file(classlist).read().split('\n') if l]])]

#		print('Labels:', self.gtLabels)

		# check to see if the GPU should be utilized
		if gpu > -1:
			print("[INFO] using GPU: {}".format(gpu))
			caffe.set_mode_gpu()
			caffe.set_device(gpu)

		# otherwise, the CPU is being used
		else:
			print("[INFO] using CPU")

		# load the mean image
		blob = caffe.proto.caffe_pb2.BlobProto()
		data = open(mean, "rb").read()
		blob.ParseFromString(data)
		self.mean = np.array(caffe.io.blobproto_to_array(blob))
		self.dim = dim

		# load the network
		self.net = caffe.Classifier(arch, snapshot, image_dims=self.dim, mean=self.mean[0], raw_scale=255)

	def detect_from_file(self, fd):
		return self.detect(caffe.io.load_image(fd))

	def detect(self, image):
		image = cv2.resize(image, self.dim)

		# make a prediction on the image
		pred = self.net.predict([image])
		print(','.join([('%d' % (p * 100)) for p in pred[0]]))
		i = pred[0].argmax()
		return self.gtLabels[i],pred[0][i]

if __name__ == '__main__':

	ap = arg_parser()
	ap.add_argument("-t", "--test-images", required=True, help="path to the directory of testing images")
	args = vars(ap.parse_args())
	outpath = args.get('outpath', None)

	co = CNNClassificator(args['classlist'], args['snapshot'], args['arch'], args['mean'], gpu=args['gpu'], dim=tuple([int(v) for v in args['dim'].split('x')]))
	# move on to testing images...
	print("[INFO] testing images from {} directory".format(args["test_images"]))

	# loop over the images not part of the CIFAR-10 dataset
	for imagePath in paths.list_images(args["test_images"]):
		# load the image and resize it to a fixed size
		bname = os.path.basename(imagePath)
		bdname = os.path.basename(os.path.dirname(imagePath))
		imagePath = os.path.abspath(imagePath)
		print("[INFO] classifying {}".format(bname))
		t = time.time()
		label,prob = co.detect_from_file(imagePath)
		if prob < 0.6:
			label = '_'
		print("{}: {} in {} secs".format(label, int(prob * 100), time.time() - t))
		if outpath:
			dpath = os.path.join(os.path.expanduser(outpath), bdname, label)
			if not os.path.exists(dpath):
				os.makedirs(dpath)
			lpath = os.path.join(dpath, bname)
			print("{} -> {}".format(lpath, imagePath))
			os.symlink(imagePath, lpath)
