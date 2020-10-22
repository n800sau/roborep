import os
import glob
import torch
import time
import cv2
import numpy as np
from .colors import label_preview_colors
from .resultsmontage import ResultsMontage

def save_model_state(model, dname, wbname_tpl, symlink_bname):
	slink_name = os.path.join(dname, symlink_bname + '.pth')
	wname = time.strftime(wbname_tpl) + '.pth'
	torch.save(model.state_dict(), os.path.join(dname, wname))
	if os.path.exists(slink_name) or os.path.islink(slink_name):
#		print('file or link exists, unlink')
		os.unlink(slink_name)
	os.symlink(wname, slink_name)

def make_test_masked_images(dest_dir, batch):
	for i in range(len(batch['fname'])):
		test_fname = os.path.join(dest_dir, os.path.basename(batch['fname'][i]))
		if not os.path.exists(test_fname):
			montage = ResultsMontage(batch['mask'][i].shape, 2, 2)
			montage.addResult(np.transpose(batch['image'][i], (2,1,0))*255)
			m = np.resize(batch['mask'][i], list(batch['mask'][i].shape) + [3])
			num_classes = batch['mask'].max()
			for j in range(min(num_classes, len(label_preview_colors))):
				m[batch['mask'][i]==j] = label_preview_colors[j][:3]
				montage.addResult(m)
				cv2.imwrite(test_fname, montage.montage)

def get_criterion(train_dataset, gpu_id=-1):
	class_weights = train_dataset.get_class_probability()
	if gpu_id >= 0:
		class_weights = class_weights.cuda(gpu_id)

#	class_weights = 1 / class_weights
	print('class_weights', class_weights)

	criterion = torch.nn.CrossEntropyLoss(weight=class_weights)
	if gpu_id >= 0:
		criterion = criterion.cuda(gpu_id)
	return criterion

def create_or_clean_dirs(dirlist):
	for dname in dirlist:
		if dname:
			if os.path.exists(dname):
				for fname in glob.glob(os.path.join(dname, '*.png')):
					os.unlink(fname)
			else:
				os.makedirs(dname)
