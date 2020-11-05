import os
import glob
import numpy as np
import cv2
import csv

import torch
from torch.utils.data import Dataset

from utils import data_transforms


class YoloTrainDataset(Dataset):
	def __init__(self, img_dir, label_dir, img_size, is_debug=False):
		self._labels = []
		self.image_root_dir = img_dir
		self.label_root_dir = label_dir
		self.img_size = img_size
		self.max_objects = 50
		self.is_debug = is_debug
		self.bnames = []
		self.label_data = []
		for bname in [os.path.basename(fname) for fname in (glob.glob(os.path.join(img_dir, '*.png')) + glob.glob(os.path.join(img_dir, '*.jpg')))]:
			lbname = os.path.join(self.label_root_dir, bname + '.csv')
			if os.path.exists(lbname):
				self.bnames.append(bname)
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
		print("Total images: {}".format(len(self.bnames)))

		#  transforms and augmentation
		self.transforms = data_transforms.Compose()
		self.transforms.add(data_transforms.ImageBaseAug())
		# self.transforms.add(data_transforms.KeepAspect())
		self.transforms.add(data_transforms.ResizeImage(self.img_size))
		self.transforms.add(data_transforms.ToTensor(self.max_objects, self.is_debug))

	def __len__(self):
		return len(self.bnames)

	def __getitem__(self, index):
		bname = self.bnames[index % len(self.bnames)]
		img_path = os.path.join(self.image_root_dir, bname)
		img = cv2.imread(img_path, cv2.IMREAD_COLOR)
		if img is None:
			raise Exception("Read image error: {}".format(img_path))
		ori_h, ori_w = img.shape[:2]
		img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
		sample = {'image': img, 'label': self.label_data[index % len(self.bnames)]}
		if self.transforms is not None:
			sample = self.transforms(sample)
		sample["image_path"] = img_path
		sample["origin_size"] = [ori_w, ori_h]
		return sample

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
