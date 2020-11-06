import os
import glob
import cv2
import numpy as np
import torch
from torch.utils.data import Dataset
from utils import data_transforms

class YoloImageDataset(Dataset):
	def __init__(self, img_dir, img_size, is_debug=False):
		Dataset.__init__(self)
		self.max_objects = 50
		self.image_root_dir = img_dir
		self.img_size = img_size
		self.is_debug = is_debug
		self.bnames = [os.path.basename(fname) for fname in (glob.glob(os.path.join(img_dir, '*.png')) + glob.glob(os.path.join(img_dir, '*.jpg')))]
		print("Total images: {}".format(len(self.bnames)))

	def __len__(self):
		return len(self.bnames)

	def make_sample(self, index):
		bname = self.bnames[index % len(self.bnames)]
		img_path = os.path.join(self.image_root_dir, bname)
		img = cv2.imread(img_path, cv2.IMREAD_COLOR)
		if img is None:
			raise Exception("Read image error: {}".format(img_path))
		ori_h, ori_w = img.shape[:2]
		img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
		return {
			'image': img,
			'fname': img_path,
			'origin_size': [ori_w, ori_h],
		}

	def prepare_image(self, image):
		return torch.from_numpy(np.transpose((image.astype(np.float32) / 255), (2, 0, 1)))

	def recover_image(self, image_tensor):
		return (np.transpose(image_tensor.detach().cpu().numpy(), (1, 2, 0)) * 255).astype(np.uint8)

	def __getitem__(self, index):
		sample = self.make_sample(index)
		self.transform(sample)
		sample['image'] = self.prepare_image(sample['image'])
		return sample

	def transform(self, sample):
		data_transforms.ResizeImage(self.img_size)(sample)
