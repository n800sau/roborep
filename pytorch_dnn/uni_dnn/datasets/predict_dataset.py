import numpy as np
import os
import glob
import torch
from torch.utils.data import Dataset
from PIL import Image
import cv2

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
			'loaded': True,
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

class PredictVideoDataset(Dataset):

	def __init__(self, vid_path, frame_shape):
		self.input_path = vid_path
		self.bname = os.path.basename(self.input_path)
		self.frame_shape = frame_shape
		self.video_reader = cv2.VideoCapture(vid_path)
		self.frame_h = int(self.video_reader.get(cv2.CAP_PROP_FRAME_HEIGHT))
		self.frame_w = int(self.video_reader.get(cv2.CAP_PROP_FRAME_WIDTH))
		self.fps = self.video_reader.get(cv2.CAP_PROP_FPS)
#		print('fps', self.fps)
		self.dataset_size = int(self.video_reader.get(cv2.CAP_PROP_FRAME_COUNT))

	def __len__(self):
#		print('frame count', self.dataset_size)
		return self.dataset_size

	def __getitem__(self, index):
		loaded, image = self.video_reader.read()
#		print('frame', index, loaded)
		image = self.load_image(Image.fromarray(image if loaded else np.zeros((100, 100, 3), dtype=np.uint8)))
		rs = {
			'loaded': loaded,
			'frame': index,
			'fname': self.bname,
			'image': image,
		}
		return rs

	def load_image(self, raw_image):
		raw_image = raw_image.resize(self.frame_shape)
#		print('1 max=', np.array(raw_image).max())
		raw_image = np.transpose(raw_image, (2,1,0))
#		print('2 max=', np.array(raw_image).max())
		imx_t = np.array(raw_image, dtype=np.float32)/255.0
#		print('3 max=', imx_t.max())
#		print('image shape:%s' % (imx_t.shape,))
		return imx_t
