import numpy as np
import os
import glob
import torch
from torch.utils.data import Dataset
from PIL import Image

class PredictImageDataset(Dataset):

	def __init__(self, input_dir, img_shape):
		self.img_shape = img_shape
		self.fnames = []
		for fname in glob.glob(os.path.join(input_dir, '*')):
			ext = os.path.splitext(fname)[-1].lower()
			if ext in ('.png', '.jpg'):
				# image
				self.fnames.append(fname)

	def __len__(self):
		return len(self.fnames)

	def __getitem__(self, index):
		fname = self.fnames[index]
		image = self.load_image(Image.open(fname))
		rs = {
				'fname': fname,
				'image': torch.FloatTensor(image),
				}
		return rs

	def load_image(self, raw_image):
		raw_image = raw_image.resize(self.img_shape)
#		print('1 max=', np.array(raw_image).max())
		raw_image = np.transpose(raw_image, (2,1,0))
#		print('2 max=', np.array(raw_image).max())
		imx_t = np.array(raw_image, dtype=np.float32)/255.0
#		print('3 max=', imx_t.max())
#		print('image shape:%s' % (imx_t.shape,))
		return imx_t
