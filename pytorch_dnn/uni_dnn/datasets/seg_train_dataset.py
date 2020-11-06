import numpy as np
import os
import glob
import torch
from .seg_image_dataset import SegImageDataset
from PIL import Image

class SegTrainDataset(SegImageDataset):

	def __init__(self, img_dir, mask_dir, img_size, num_classes, label_map=None):
		SegImageDataset.__init__(self, img_dir, img_size)
		self.mask_extension = '.png'
		self.label_map = label_map if label_map else {}
		self.mask_root_dir = mask_dir
		self.num_classes = num_classes
		clean_bnames = []
		for bname in self.bnames:
			mfname = os.path.join(self.mask_root_dir, os.path.splitext(bname)[0] + self.mask_extension)
			if os.path.exists(mfname):
				clean_bnames.append(bname)
			else:
				print("no mask file found. skip it: {}".format(mfname))
		self.bnames = clean_bnames
		print("Total labelled images: {}".format(len(self.bnames)))
		self.counts = self.__compute_class_probability()

	def __len__(self):
		return len(self.bnames)

	def __getitem__(self, index):
		sample = SegImageDataset.__getitem__(self, index)
		bname = os.path.splitext(self.bnames[index])[0]
		mask_path = os.path.join(self.mask_root_dir, bname + self.mask_extension)
		gt_mask = self.load_mask(Image.open(mask_path))
		sample['mask'] = torch.LongTensor(gt_mask)
		return sample

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

		for bname in self.bnames:
			mask_path = os.path.join(self.mask_root_dir, os.path.splitext(bname)[0] + self.mask_extension)

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
