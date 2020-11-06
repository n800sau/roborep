import os
import glob
import numpy as np
import cv2
import csv

import torch
from .yolo_image_dataset import YoloImageDataset

from utils import data_transforms


class YoloTrainDataset(YoloImageDataset):
	def __init__(self, img_dir, label_dir, img_size, is_debug=False):
		YoloImageDataset.__init__(self, img_dir, img_size, is_debug=is_debug)
		self._labels = []
		self.label_root_dir = label_dir
		self.label_data = []
		clean_bnames = []
		for bname in self.bnames:
			lbname = os.path.join(self.label_root_dir, bname + '.csv')
			if os.path.exists(lbname):
				clean_bnames.append(bname)
				labels = []
				for line in csv.reader(open(lbname, 'rt')):
					if line:
						if line[0] in self._labels:
							lindex = self._labels.index(line[0])
						else:
							lindex = len(self._labels)
							self._labels.append(line[0])
						line[0] = lindex
						labels.append(line)
				self.label_data.append(np.array(labels))
			else:
				print("no label found. skip it: {}".format(lbname))
		self.bnames = clean_bnames
		print("Total labelled images: {}".format(len(self.bnames)))

	def __len__(self):
		return len(self.bnames)

	def make_sample(self, index):
		sample = YoloImageDataset.make_sample(self, index)
		bname = self.bnames[index % len(self.bnames)]
		label_data = self.label_data[index % len(self.bnames)]
		filled_labels = np.zeros((self.max_objects, 5), np.float32)
		filled_labels[range(len(label_data))[:self.max_objects]] = label_data[:self.max_objects]
		sample['label'] = torch.from_numpy(filled_labels)
		return sample

	def transform(self, sample):
#		data_transforms.ImageBaseAug()(sample)
		YoloImageDataset.transform(self, sample)

	def labels(self):
		return self._labels

#  use for test dataloader
if __name__ == "__main__":
	dataloader = torch.utils.data.DataLoader(YoloTrainDataset('data/yolo_images', 'data/yolo_labels',
				(416, 416), is_debug=True), batch_size=2, shuffle=False, num_workers=1, pin_memory=False)
	for step, sample in enumerate(dataloader):
		for i, (image, label) in enumerate(zip(sample['image'], sample['label'])):
			image = image.numpy()
			h, w = image.shape[:2]
			for l in label:
				if l.sum() == 0:
					continue
				x1 = int((l[1] - l[3] / 2) * w)
				y1 = int((l[2] - l[4] / 2) * h)
				x2 = int((l[1] + l[3] / 2) * w)
				y2 = int((l[2] + l[4] / 2) * h)
				cv2.rectangle(image, (x1, y1), (x2, y2), (0, 0, 255))
			image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
			cv2.imwrite("step{}_{}.jpg".format(step, i), image)
		# only one batch
		break
