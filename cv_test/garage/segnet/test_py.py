import os
import numpy as np
import matplotlib.pyplot as plt
import os.path
import scipy
import argparse
import math
import cv2
import sys
import time
from resultsmontage import ResultsMontage
from imutils.paths import list_images

INPUT='garage_tagged'
#INPUT='data/images'
OUTPUT='output'
OUTPUT_MONTAGE = 'output_montage'
OUTPUT_DATA = 'odata'

LAST_LAYER = 'argmax'
#LAST_LAYER = 'loss'
#LAST_LAYER = 'prob'
#LAST_LAYER = 'conv1_1_D'

sys.path.append('/usr/local/lib/python2.7/site-packages')
# Make sure that caffe is on the python path:
caffe_root = os.path.expanduser('~/install/caffe-segnet-cudnn5/')
sys.path.insert(0, os.path.join(caffe_root, 'python'))
import caffe

# Import arguments
parser = argparse.ArgumentParser()
parser.add_argument('--model', type=str, required=True)
parser.add_argument('--weights', type=str, required=True)
args = parser.parse_args()

net = caffe.Net(args.model,
                args.weights,
                caffe.TEST)

caffe.set_mode_gpu()

#transformer = caffe.io.Transformer({'data': net.blobs['data'].data.shape})
#transformer.set_transpose('data', (2,0,1))
#transformer.set_channel_swap('data', (2,1,0))

input_shape = net.blobs['data'].data.shape
output_shape = net.blobs[LAST_LAYER].data.shape

for fname in list_images(INPUT):

#	c_image = caffe.io.load_image(fname)
#	transformed_image = transformer.preprocess('data', c_image)

	frame = cv2.imread(fname)
	print 'input.shape', frame.shape

	frame = cv2.resize(frame, (input_shape[3],input_shape[2]))
	input_image = frame.transpose((2,0,1))
#	input_image = frame
	# input_image = input_image[(2,1,0),:,:] # May be required, if you do not open your data with opencv
	input_image = np.asarray([input_image])
	print 'shape', frame.shape, 'input image shape:', input_image.shape, 'network input shape:', input_shape

	start = time.time()
#	net.blobs['data'].data[...] = transformed_image
#	out = net.forward_all()
	out = net.forward_all(data=input_image)
	end = time.time()
	print '%30s' % 'Executed SegNet in ', str((end - start)*1000), 'ms'

	start = time.time()

	predicted = net.blobs[LAST_LAYER].data
	print 'predicted shape', predicted.shape
#	print 'Max=%d' % (predicted.max(),)
	output = np.squeeze(predicted[0,:,:,:])
#	print 'Car pixels count=%d, max=%d, shape=%s' % (np.count_nonzero(output[0,:,:] == 1), output.max(), output.shape)

#	print 'amax=', np.amax(output, axis=0)
#	mm = np.where(output > 0.6)[0]
#	print 'indeces=', mm.shape

	print 'output=', output.shape, output.min(), output.max()
#	print '000', output[0,:,:].min(), output[0,:,:].max()
#	print '111', output[1,:,:].min(), output[1,:,:].max()

#	ind = np.argmax(output, axis=0)

#	print '####', ind.min(), ind.max(), ind.shape, output.shape

	ind = output

	r = ind.copy()
	g = ind.copy()
	b = ind.copy()

	Unlabelled = [0,0,0]
	Car = [128,128,128]

	label_colours = np.array([Unlabelled, Car])
#	print 'labels=', label_colours.shape[0], label_colours.shape

	for l in range(0, label_colours.shape[0]):
		r[ind==l] = label_colours[l, 0]
		g[ind==l] = label_colours[l, 1]
		b[ind==l] = label_colours[l, 2]

	rgb = np.zeros((ind.shape[0], ind.shape[1], 3))
	rgb[:,:,0] = r
	rgb[:,:,1] = g
	rgb[:,:,2] = b

	print rgb.max(), r.max(), b.max(), g.max()
#	oframe = rgb.astype(float)/255

	end = time.time()
	print '%30s' % 'Processed results in ', str((end - start)*1000), 'ms\n'

#	output = output.transpose((1,2,0))
#	print 'output', output.shape, output.max()
#	output = (output[:,:,1]/output.max()*255).astype(np.uint8)

	montage = ResultsMontage(frame.shape[:2], 1, 2)
	montage.addResult(frame)
#	rgb = cv2.cvtColor(output,cv2.COLOR_GRAY2BGR)
	montage.addResult(rgb)

#	np.savetxt(os.path.join(OUTPUT_DATA, os.path.basename(fname) + '.csv'), output, fmt='%d')
	cv2.imwrite(os.path.join(OUTPUT, os.path.basename(fname)), rgb)
	cv2.imwrite(os.path.join(OUTPUT_MONTAGE, os.path.basename(fname)), montage.montage)

