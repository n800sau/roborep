#!/usr/bin/env python3

import os
import glob
import time
import torch
from torch.utils.data import DataLoader
from torch.utils.tensorboard import SummaryWriter
from datasets import SegmentTrainDataset
from models import UNet
import cv2
from utils.resultsmontage import ResultsMontage
from utils.colors import label_preview_colors
import numpy as np

CUDA = True
GPU_ID = 0
NUM_CLASSES = 6
IMG_DIR = 'data/images'
MASK_DIR = 'data/masks'
MONTAGE_DIR = None
SAVE_DIR = '.'
BATCH_SIZE = 6
NUM_INPUT_CHANNELS = 3
NUM_OUTPUT_CHANNELS = NUM_CLASSES
INITIAL_WNAME = 'unet_model_best.pth'
CHECKPOINT_WNAME = INITIAL_WNAME
LEARNING_RATE = 0.002
NUM_EPOCHS = 40


def get_criterion(train_dataset):
	class_weights = train_dataset.get_class_probability()
	if CUDA:
		class_weights = class_weights.cuda(GPU_ID)
	class_weights = 1 / class_weights
	criterion = torch.nn.CrossEntropyLoss(weight=class_weights)
	if CUDA:
		criterion = criterion.cuda(GPU_ID)
	return criterion


def train(train_dataset):

	train_dataloader = DataLoader(train_dataset, batch_size=BATCH_SIZE, shuffle=True, num_workers=2)

	model = UNet(input_channels=NUM_INPUT_CHANNELS, output_channels=NUM_OUTPUT_CHANNELS)
	if os.path.exists(INITIAL_WNAME):
		model.load_state_dict(torch.load(INITIAL_WNAME))
	if CUDA:
		model = model.cuda(GPU_ID)

	criterion = get_criterion(train_dataset)

	optimizer = torch.optim.Adam(model.parameters(), lr=LEARNING_RATE)

	with SummaryWriter('logs', comment=f'LR_{LEARNING_RATE}_BS_{BATCH_SIZE}') as writer:

		is_better = True
		prev_loss = float('inf')

		model.train()

		global_step = 0
		for epoch in range(NUM_EPOCHS):
			loss_f = 0
			t_start = time.time()

			for batch in train_dataloader:

				# make sample and save
#				print('!!!!!!!!!!!', batch['fname'])
				if MONTAGE_DIR:
					for i in range(BATCH_SIZE):
						test_fname = os.path.join(MONTAGE_DIR, os.path.basename(batch['fname'][i]))
						if not os.path.exists(test_fname):
							montage = ResultsMontage(batch['mask'][i].shape, 2, 2)
							montage.addResult(np.transpose(batch['image'][i], (2,1,0))*255)
							m = np.resize(batch['mask'][i], list(batch['mask'][i].shape) + [3])
							for j in range(min(NUM_CLASSES, len(label_preview_colors))):
								m[batch['mask'][i]==j] = label_preview_colors[j][:3]
							montage.addResult(m)
							cv2.imwrite(test_fname, montage.montage)

#				print(batch['fname'])
#				print('input tensor', input_tensor.shape)
#				print('target tensor min/max', target_tensor.shape, target_tensor.min(), target_tensor.max())

				input_tensor = torch.autograd.Variable(batch['image'])
				target_tensor = torch.autograd.Variable(batch['mask'])
				if CUDA:
					input_tensor = input_tensor.cuda(GPU_ID)
					target_tensor = target_tensor.cuda(GPU_ID)

				predicted_tensor, softmaxed_tensor = model(input_tensor)

				optimizer.zero_grad()

				loss = criterion(predicted_tensor, target_tensor)

				writer.add_scalar('Loss/train', loss.item(), global_step)

				loss.backward()
				optimizer.step()

			loss_f = loss.float()
#				prediction_f = softmaxed_tensor.float()

			delta = time.time() - t_start
			is_better = loss_f < prev_loss

			print("Epoch #{}\tLr: {:g}\tLoss: {:.8f}\t Time: {:2f}s".format(epoch+1, optimizer.param_groups[0]['lr'], loss_f, delta), end='')
			if is_better:
				print(' {:.3f} better'.format(prev_loss-loss_f))
				prev_loss = loss_f
				torch.save(model.state_dict(), os.path.join(SAVE_DIR, INITIAL_WNAME))

				for param_group in optimizer.param_groups:
					param_group['lr'] *= 0.9
			else:
				print(' no better')

			global_step += 1
			if global_step % (len(train_dataset) // (10 * BATCH_SIZE)) == 0:
				for tag, value in model.named_parameters():
					tag = tag.replace('.', '/')
					writer.add_histogram('weights/' + tag, value.data.cpu().numpy(), global_step)
					writer.add_histogram('grads/' + tag, value.grad.data.cpu().numpy(), global_step)
				writer.add_scalar('learning_rate', optimizer.param_groups[0]['lr'], global_step)

if __name__ == "__main__":

	for dname in (MONTAGE_DIR,):
		if dname:
			if os.path.exists(dname):
				for fname in glob.glob(os.path.join(MONTAGE_DIR, '*.png')):
					os.unlink(fname)
			else:
				os.makedirs(MONTAGE_DIR)

	train(SegmentTrainDataset(img_dir=IMG_DIR, mask_dir=MASK_DIR, num_classes=NUM_CLASSES, img_shape=(224, 224)))
	print('Finished')
