#!/usr/bin/env python3

import os
import glob
import time
import json
import torch
from torch.utils.data import DataLoader
from torch.utils.tensorboard import SummaryWriter
from datasets import SegmentTrainDataset
from models import SegNet
from utils.misc import save_model_state, make_test_masked_images, get_criterion, create_or_clean_dirs
import numpy as np

GPU_ID = 0
NUM_CLASSES = 6
DS_BASE_DIR = 'data'
IMG_DIR = 'images'
MASK_DIR = 'masks'
BATCH_SIZE = 8
NUM_INPUT_CHANNELS = 3
INITIAL_WNAME = 'segnet_model_best'
SAVE_WNAME = 'segnet_model_best_%Y-%m-%d_%H:%M:%S'
LEARNING_RATE = 0.02
NUM_EPOCHS = 500

MONTAGE_DIR = os.path.join(DS_BASE_DIR, 'test_images')
LOG_DIR = os.path.join(DS_BASE_DIR, 'segnet_logs')

def train(train_dataset):

	train_dataloader = DataLoader(train_dataset, batch_size=BATCH_SIZE, shuffle=True, num_workers=2)

	model = SegNet(input_channels=NUM_INPUT_CHANNELS, output_channels=NUM_CLASSES)
	if GPU_ID >= 0:
		model = model.cuda(GPU_ID)
	criterion = get_criterion(train_dataset, gpu_id=GPU_ID)

	if os.path.exists(INITIAL_WNAME):
		model.load_state_dict(torch.load(INITIAL_WNAME))

	optimizer = torch.optim.Adam(model.parameters(), lr=LEARNING_RATE)

	with SummaryWriter(LOG_DIR, comment=f'LR_{LEARNING_RATE}_BS_{BATCH_SIZE}') as writer:
		is_better = True
		prev_loss = float('inf')

		model.train()

		global_step = 0
		for epoch in range(NUM_EPOCHS):
			loss_f = 0
			t_start = time.time()

			for batch in train_dataloader:
				input_tensor = torch.autograd.Variable(batch['image'])
				target_tensor = torch.autograd.Variable(batch['mask'])

				# make sample and save
#				print('!!!!!!!!!!!', batch['fname'])
				if MONTAGE_DIR:
					make_test_masked_images(MONTAGE_DIR, batch)

#				print(batch['fname'])
#				print('input tensor', input_tensor.shape)
#				print('target tensor min/max', target_tensor.shape, target_tensor.min(), target_tensor.max())

				if GPU_ID >= 0:
					input_tensor = input_tensor.cuda(GPU_ID)
					target_tensor = target_tensor.cuda(GPU_ID)


				predicted_tensor, softmaxed_tensor = model(input_tensor)

				optimizer.zero_grad()
				loss = criterion(predicted_tensor, target_tensor)
				writer.add_scalar('Loss/train', loss.item(), global_step)
				loss.backward()
				optimizer.step()


			loss_f += loss.float()
#			prediction_f = softmaxed_tensor.float()

			delta = time.time() - t_start
			is_better = loss_f < prev_loss

			print("Epoch #{}\tLr: {:g}\tLoss: {:.8f}\t Time: {:2f}s".format(epoch+1, optimizer.param_groups[0]['lr'], loss_f, delta), end='')
			if is_better:
				print(' {:.3f} better'.format(prev_loss-loss_f))
				prev_loss = loss_f
				save_model_state(model, DS_BASE_DIR, '%s_%.4f' % (SAVE_WNAME, loss_f), INITIAL_WNAME)

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

	create_or_clean_dirs((MONTAGE_DIR, LOG_DIR))
	label_map = {}
	lname = os.path.join(DS_BASE_DIR, 'labels.json')
	if os.path.exists(lname):
		labels = json.load(open(lname))
		for l in labels:
			if 'segnet_val' in l:
				label_map[l['val']] = l['segnet_val']
		NUM_CLASSES = max(label_map.values()) + 1
	print('label_map=', label_map, NUM_CLASSES)
	ds = SegmentTrainDataset(img_dir=os.path.join(DS_BASE_DIR, IMG_DIR), mask_dir=os.path.join(DS_BASE_DIR, MASK_DIR),
		num_classes=NUM_CLASSES, img_shape=(224, 224), label_map=label_map)
	train(ds)
	print('Finished')
