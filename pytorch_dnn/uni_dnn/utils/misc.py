import os
import glob
import torch
import time
import cv2
from PIL import Image
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
	m2colour = np.vectorize(lambda v: np.array(label_preview_colors[v]))
	for i in range(len(batch['fname'])):
		test_fname = os.path.join(dest_dir, os.path.basename(batch['fname'][i]))
		if not os.path.exists(test_fname):
			montage = ResultsMontage(batch['mask'][i].shape, 2, 2)
			img = (np.transpose(batch['image'][i], (2,1,0))*255).data.numpy().astype(np.uint8)
			mask = batch['mask'][i].data.numpy().astype(np.uint8)
			montage.addResult(img)
			m = np.zeros(img.shape, img.dtype)
#			print('mask shape', mask.shape)
#			print('m shape', m.shape)
			num_classes = mask.max()
			with np.nditer(mask, op_flags=['readwrite'], flags=['multi_index']) as it:
				for item in it:
					clr = label_preview_colors[item][:3]
					m[it.multi_index[0], it.multi_index[1]] = clr
#					print('clr , mindex', clr, it.multi_index)
			montage.addResult(m)
			im = Image.fromarray(montage.montage)
			im.save(test_fname, 'PNG')

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
