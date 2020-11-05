#!/usr/bin/env python3

import os
import glob
import time
import torch
from torch.utils.data import DataLoader
from torch.utils.tensorboard import SummaryWriter
from datasets import SegmentTrainDataset
from models import UNet
from utils.misc import save_model_state, make_test_masked_images, get_criterion, create_or_clean_dirs
import numpy as np
from config import Config

C = Config('unet')
C.NUM_EPOCHS = 40
C.MONTAGE_DIR = None

def train(train_dataset):

	train_dataloader = DataLoader(train_dataset, batch_size=C.BATCH_SIZE, shuffle=True, num_workers=2)

	model = UNet(input_channels=C.NUM_INPUT_CHANNELS, output_channels=C.NUM_CLASSES)
	if C.is_cuda():
		model = model.cuda(C.GPU_ID)
	initial_wname = os.path.join(C.DS_BASE_DIR, C.INITIAL_WNAME) + '.pth'
	if os.path.exists(initial_wname):
		model.load_state_dict(torch.load(initial_wname))
	if C.is_cuda():
		model = model.cuda(C.GPU_ID)

	criterion = get_criterion(train_dataset, gpu_id=C.GPU_ID)

	optimizer = torch.optim.Adam(model.parameters(), lr=C.LEARNING_RATE)

	with SummaryWriter(C.LOG_DIR, comment=f'LR_{C.LEARNING_RATE}_BS_{C.BATCH_SIZE}') as tf_writer:

		is_better = True
		prev_loss = float('inf')

		model.train()

		global_step = 0
		for epoch in range(C.NUM_EPOCHS):
			loss_f = 0
			t_start = time.time()

			for batch in train_dataloader:

				# make sample and save
#				print('!!!!!!!!!!!', batch['fname'])
				if C.MONTAGE_DIR:
					make_test_masked_images(C.MONTAGE_DIR, batch)

#				print(batch['fname'])
#				print('input tensor', input_tensor.shape)
#				print('target tensor min/max', target_tensor.shape, target_tensor.min(), target_tensor.max())

				input_tensor = torch.autograd.Variable(batch['image'])
				target_tensor = torch.autograd.Variable(batch['mask'])
				if C.is_cuda():
					input_tensor = input_tensor.cuda(C.GPU_ID)
					target_tensor = target_tensor.cuda(C.GPU_ID)

				predicted_tensor, softmaxed_tensor = model(input_tensor)

				optimizer.zero_grad()

				loss = criterion(predicted_tensor, target_tensor)

				tf_writer.add_scalar('Loss/train', loss.item(), global_step)

				loss.backward()
				optimizer.step()

				loss_f += loss.float()
#				prediction_f = softmaxed_tensor.float()

			delta = time.time() - t_start
			is_better = loss_f < prev_loss

			print("Epoch #{}\tLr: {:g}\tLoss: {:.8f}\t Time: {:2f}s".format(epoch+1, optimizer.param_groups[0]['lr'], loss_f, delta), end='')
			if is_better:
				print(' {:.3f} better'.format(prev_loss-loss_f))
				prev_loss = loss_f
				save_model_state(model, C.DS_BASE_DIR, '%s_%.4f' % (C.SAVE_WNAME, loss_f), C.INITIAL_WNAME)

				for param_group in optimizer.param_groups:
					param_group['lr'] *= 0.9
			else:
				print(' no better')

			global_step += 1
			if global_step % (len(train_dataset) // (10 * C.BATCH_SIZE)) == 0:
				for tag, value in model.named_parameters():
					tag = tag.replace('.', '/')
					tf_writer.add_histogram('weights/' + tag, value.data.cpu().numpy(), global_step)
					tf_writer.add_histogram('grads/' + tag, value.grad.data.cpu().numpy(), global_step)
				tf_writer.add_scalar('learning_rate', optimizer.param_groups[0]['lr'], global_step)

if __name__ == "__main__":

	create_or_clean_dirs((C.MONTAGE_DIR, C.LOG_DIR))

	train(SegmentTrainDataset(img_dir=os.path.join(C.DS_BASE_DIR, C.IMG_DIR), mask_dir=os.path.join(C.DS_BASE_DIR, C.MASK_DIR), num_classes=C.NUM_CLASSES, img_size=(224, 224)))
	print('Finished')
