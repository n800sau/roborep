import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib'))

import cv2
import numpy as np
from keras.preprocessing.image import ImageDataGenerator, array_to_img, img_to_array, load_img
import argparse
from pyimagesearch.utils import Conf
from paths import r_list_dirs, list_images

# construct the argument parser and parse the command line arguments
ap = argparse.ArgumentParser()
ap.add_argument("-c", "--conf", required=True, help="path to configuration file")
args = vars(ap.parse_args())

# load the configuration and grab all image paths in the dataset
conf = Conf(args["conf"])

datagen = ImageDataGenerator(
		rotation_range=40,
		width_shift_range=0.2,
		height_shift_range=0.2,
		shear_range=0.2,
		zoom_range=0.2,
		horizontal_flip=True,
		fill_mode='nearest')

for subdir in r_list_dirs(conf["dataset_path"]):

	for fname in list_images(conf["dataset_path"], subdir):

		fpath = os.path.join(conf["dataset_path"], subdir, fname)

		label = os.path.basename(os.path.dirname(fpath))
		out_dir = os.path.join('generated', subdir)
		if not os.path.exists(out_dir):
			os.makedirs(out_dir)

		img = cv2.imread(fpath) # this is a Numpy array with shape (150, 150, 3)
		img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
		x = np.swapaxes(img, 0, 2) # this is a Numpy array with shape (3, 150, 150)
		x = x.reshape((1,) + x.shape)  # this is a Numpy array with shape (1, 3, 150, 150)

		# the .flow() command below generates batches of randomly transformed images
		# and saves the results to the `preview/` directory
		i = 0
		for batch in datagen.flow(x, batch_size=1,
					save_to_dir=out_dir, save_prefix=os.path.splitext(fname)[0], save_format=os.path.splitext(fname)[1].lstrip('.')):
			i += 1
			if i > 20:
				break  # otherwise the generator would loop indefinitely
