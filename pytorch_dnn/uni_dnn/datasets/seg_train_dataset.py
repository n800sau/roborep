import numpy as np
import os
import glob
import torch
from torch.utils.data import Dataset
from PIL import Image

class SegmentTrainDataset(Dataset):

	def __init__(self, img_dir, mask_dir, img_size, num_classes, label_map=None):
		self.label_map = label_map if label_map else {}
		self.img_extension = ".png"
		self.mask_extension = ".png"

		self.image_root_dir = img_dir
		self.mask_root_dir = mask_dir

		self.bnames = [os.path.splitext(os.path.basename(fname))[0] for fname in glob.glob(os.path.join(img_dir, '*' + self.img_extension))] if img_dir else []
		self.num_classes = num_classes
		self.img_size = img_size
		self.counts = self.__compute_class_probability()

	def __len__(self):
		return len(self.bnames)

	def __getitem__(self, index):
		name = self.bnames[index]
		image_path = os.path.join(self.image_root_dir, name + self.img_extension)
		mask_path = os.path.join(self.mask_root_dir, name + self.mask_extension)
		self.input_path = image_path
		image = self.load_image(Image.open(image_path))
		gt_mask = self.load_mask(Image.open(mask_path))
		rs = {
				'fname': image_path,
				'image': torch.FloatTensor(image),
				'mask' : torch.LongTensor(gt_mask)
				}
		return rs

	def load_image(self, raw_image):
		raw_image = raw_image.resize(self.img_size)
#		print('1 max=', np.array(raw_image).max())
		raw_image = np.transpose(raw_image, (2,1,0))
#		print('2 max=', np.array(raw_image).max())
		imx_t = np.array(raw_image, dtype=np.float32)/255.0
#		print('3 max=', imx_t.max())
#		print('image shape:%s' % (imx_t.shape,))
		return imx_t

	def load_mask(self, raw_image):
		raw_image = raw_image.resize(self.img_size, resample=Image.NEAREST)
		# take a single colour
		imx_t = np.array(raw_image)
		if len(imx_t.shape) > 2:
			imx_t = imx_t[:,:,0]
		vf = np.vectorize(lambda v: self.label_map.get(v, v))
		imx_t = vf(imx_t)
		# everything outside is background
		imx_t[imx_t>=self.num_classes] = 0
#		print('mask shape:%s, uniq: %s, count:%s' % (imx_t.shape, np.unique(imx_t[:,:]), dict([(i, np.sum(imx_t==i)) for i in np.unique(imx_t[:,:])])))
		return imx_t

	def get_class_probability(self):
		values = np.array(list(self.counts.values()))
		return torch.Tensor(values)
#		p_values = values/np.sum(values)
#		return torch.Tensor(p_values)

	def __compute_class_probability(self):
		counts = dict((i, 0) for i in range(self.num_classes))
		total = 0

		max_label = 0

		for name in self.bnames:
			mask_path = os.path.join(self.mask_root_dir, name + self.mask_extension)

			raw_image = Image.open(mask_path)
			imx_t = self.load_mask(raw_image)
#			print('uniq 0:', np.unique(imx_t[:,:,0]))
#			print('uniq 1:', np.unique(imx_t[:,:,1]))
#			print('uniq 2:', np.unique(imx_t[:,:,2]))

			# also find just to show max label
			l_max = imx_t.max()
			if max_label < l_max:
				max_label = l_max

			for i in range(self.num_classes):
				counts[i] += np.sum(imx_t == i)
			total += imx_t.shape[0] * imx_t.shape[1]

		print('total:%f, counts: %s' % (total, counts))
		freq = []
		for i in range(self.num_classes):
			freq.append(float(counts[i]) / total)
		mfreq = np.median(freq)
		print('mfreq: %f, freq: %s' % (mfreq, freq))
		weightings = dict((i, mfreq/f) for i,f in enumerate(freq))
		print('class_weightings: %s' % weightings)

		print('max label: %d' % max_label)
#		return counts
		return weightings
