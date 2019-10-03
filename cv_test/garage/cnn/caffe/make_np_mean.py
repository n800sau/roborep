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
	ap.add_argument("-m", "--mean", required=True, help="path to mean image")
	return ap

ap = arg_parser()
args = vars(ap.parse_args())
data = open(args['mean'], "rb").read()
# load the mean image
blob = caffe.proto.caffe_pb2.BlobProto()
blob.ParseFromString(data)
mean = np.array(caffe.io.blobproto_to_array(blob))
np.save(args['mean'] + '.np', mean)
