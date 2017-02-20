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
import time

# construct the argument parser and parse the command line arguments
ap = argparse.ArgumentParser()
ap.add_argument("-a", "--arch", required=True,
	help="path to network architecture definition")
ap.add_argument("-s", "--snapshot", required=True, help="path network snapsot")
ap.add_argument("-m", "--mean", required=True, help="path to mean image")
ap.add_argument("-t", "--test-images", required=True,
	help="path to the directory of testing images")
ap.add_argument("-g", "--gpu", type=int, default=-1, help="GPU device index")
args = vars(ap.parse_args())

gtLabels = [v for k,v in sorted([(int(k), v) for k,v in [l.split(' ') for l in file('data/class.lst').read().split('\n') if l]])]

print(gtLabels)

#sys.exit()

# check to see if the GPU should be utilized
if args["gpu"] > -1:
	print("[INFO] using GPU: {}".format(args["gpu"]))
	caffe.set_mode_gpu()
	caffe.set_device(args["gpu"])

# otherwise, the CPU is being used
else:
	print("[INFO] using CPU")

# load the mean image
blob = caffe.proto.caffe_pb2.BlobProto()
data = open(args["mean"], "rb").read()
blob.ParseFromString(data)
mean = np.array(caffe.io.blobproto_to_array(blob))

# load the network
net = caffe.Classifier(args["arch"], args["snapshot"], image_dims=(160, 160), mean=mean[0], raw_scale=255)

# move on to testing images...
print("[INFO] testing images from {} directory".format(args["test_images"]))

# loop over the images not part of the CIFAR-10 dataset
for imagePath in paths.list_images(args["test_images"]):
	# load the image and resize it to a fixed size
	print("[INFO] classifying {}".format(imagePath[imagePath.rfind("/") + 1:]))
	t = time.time()
	image = caffe.io.load_image(imagePath)
	orig = image.copy()
	image = cv2.resize(image, (160, 160))

	# make a prediction on the image
	pred = net.predict([image])
	i = pred[0].argmax()
	print("{}: {} in {} secs".format(gtLabels[i], int(pred[0][i] * 100), time.time() - t))
