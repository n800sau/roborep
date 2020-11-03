#!/usr/bin/env python3

import os
import glob
import time
import json
import torch
from torch.utils.data import DataLoader
from torch.utils.tensorboard import SummaryWriter
import torch.optim as optim
import torch.nn as nn
from torchsummary import summary
from datasets import YoloTrainDataset
from models import Yolo3
from models.yolo3.yolo_loss import YOLOLoss
from utils.misc import save_model_state, make_test_masked_images, get_criterion, create_or_clean_dirs
import numpy as np
from config import Config
from data.yolo3_params import TRAINING_PARAMS

C = Config('yolo3')
C.BACKBONE_WNAME = 'weights/darknet53_weights_pytorch.pth'

def train(train_dataset):

	train_dataloader = DataLoader(train_dataset, batch_size=C.BATCH_SIZE, shuffle=True, num_workers=2)

	model = SegNet(input_channels=C.NUM_INPUT_CHANNELS, output_channels=C.NUM_CLASSES)
	if C.is_cuda():
		model = model.cuda(C.GPU_ID)

	criterion = get_criterion(train_dataset, gpu_id=C.GPU_ID)

	if os.path.exists(C.INITIAL_WNAME):
		model.load_state_dict(torch.load(C.INITIAL_WNAME))

	optimizer = torch.optim.Adam(model.parameters(), lr=C.LEARNING_RATE)

	with SummaryWriter(C.LOG_DIR, comment=f'LR_{C.LEARNING_RATE}_BS_{C.BATCH_SIZE}') as writer:
		is_better = True
		prev_loss = float('inf')

		model.train()

		global_step = 0
		for epoch in range(C.NUM_EPOCHS):
			loss_f = 0
			t_start = time.time()

			for batch in train_dataloader:
				input_tensor = torch.autograd.Variable(batch['image'])
				target_tensor = torch.autograd.Variable(batch['mask'])

				# make sample and save
#				print('!!!!!!!!!!!', batch['fname'])
				if C.MONTAGE_DIR:
					make_test_masked_images(C.MONTAGE_DIR, batch)

#				print(batch['fname'])
#				print('input tensor', input_tensor.shape)
#				print('target tensor min/max', target_tensor.shape, target_tensor.min(), target_tensor.max())

				if C.is_cuda():
					input_tensor = input_tensor.cuda(C.GPU_ID)
					target_tensor = target_tensor.cuda(C.GPU_ID)


				predicted_tensor, softmaxed_tensor = model(input_tensor)

				optimizer.zero_grad()
				loss = criterion(predicted_tensor, target_tensor)
				writer.add_scalar('Loss/train', loss.item(), global_step)
#				writer.add_scalar('prediction/train', softmaxed_tensor, global_step)
				loss.backward()
				optimizer.step()


				loss_f += loss.float()

			prediction_f = softmaxed_tensor.float()
#			print('prediction:', prediction_f)

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
					writer.add_histogram('weights/' + tag, value.data.cpu().numpy(), global_step)
					writer.add_histogram('grads/' + tag, value.grad.data.cpu().numpy(), global_step)
				add_image('prediction', prediction_f, global_step)
				writer.add_scalar('learning_rate', optimizer.param_groups[0]['lr'], global_step)

def _get_optimizer(net):
	optimizer = None

	# Assign different lr for each layer
	params = None
	base_params = list(
		map(id, net.backbone.parameters())
	)
	logits_params = filter(lambda p: id(p) not in base_params, net.parameters())

	if not TRAINING_PARAMS['lr']['freeze_backbone']:
		params = [
			{"params": logits_params, "lr": TRAINING_PARAMS['lr']["other_lr"]},
			{"params": net.backbone.parameters(), "lr": TRAINING_PARAMS['lr']["backbone_lr"]},
		]
	else:
		print("freeze backbone's parameters.")
		for p in net.backbone.parameters():
			p.requires_grad = False
		params = [
			{"params": logits_params, "lr": TRAINING_PARAMS['lr']["other_lr"]},
		]

	# Initialize optimizer class
	if TRAINING_PARAMS['optimizer']["type"] == "adam":
		optimizer = optim.Adam(params, weight_decay=TRAINING_PARAMS['optimizer']["weight_decay"])
	elif TRAINING_PARAMS['optimizer']["type"] == "amsgrad":
		optimizer = optim.Adam(params, weight_decay=TRAINING_PARAMS['optimizer']["weight_decay"],
							   amsgrad=True)
	elif TRAINING_PARAMS['optimizer']["type"] == "rmsprop":
		optimizer = optim.RMSprop(params, weight_decay=TRAINING_PARAMS['optimizer']["weight_decay"])
	else:
		# Default to sgd
		print("Using SGD optimizer.")
		optimizer = optim.SGD(params, momentum=0.9,
							  weight_decay=TRAINING_PARAMS['optimizer']["weight_decay"],
							  nesterov=(TRAINING_PARAMS['optimizer']["type"] == "nesterov"))

	return optimizer

if __name__ == "__main__":

	create_or_clean_dirs((C.MONTAGE_DIR, C.LOG_DIR))
#	print(TRAINING_PARAMS)
	TRAINING_PARAMS['classes'] = C.NUM_CLASSES
	load_weights = os.path.exists(C.INITIAL_WNAME)
	m = Yolo3(TRAINING_PARAMS, backbone_weights_path=None if load_weights else C.BACKBONE_WNAME)
	if C.is_cuda():
		m = m.cuda(C.GPU_ID)
	m.train(True)

	# Optimizer and learning rate
	optimizer = _get_optimizer(m)
	lr_scheduler = optim.lr_scheduler.StepLR(
		optimizer,
		step_size=TRAINING_PARAMS['lr']["decay_step"],
		gamma=TRAINING_PARAMS['lr']["decay_gamma"])

	# Set data parallel
	m = nn.DataParallel(m)

	if load_weights:
		print("Load pretrained weights from {}".format(C.INITIAL_WNAME))
		m.load_state_dict(torch.load(C.INITIAL_WNAME))

	# YOLO loss with 3 scales
	yolo_losses = []
	for i in range(3):
		yolo_losses.append(YOLOLoss(TRAINING_PARAMS["anchors"][i], C.NUM_CLASSES, (TRAINING_PARAMS["img_w"], TRAINING_PARAMS["img_h"])))

#	print(summary(m, (3, 416, 416), device='cpu'))

	dataloader = torch.utils.data.DataLoader(YoloTrainDataset(os.path.join(C.DS_BASE_DIR, C.YOLO_IMG_DIR), os.path.join(C.DS_BASE_DIR, C.YOLO_LABEL_DIR), img_size=(224, 224)),
					batch_size=C.BATCH_SIZE, shuffle=True, num_workers=32, pin_memory=True)

	# Start the training loop
	print("Start training.")
	global_step = 0
	for epoch in range(C.NUM_EPOCHS):
		for step, samples in enumerate(dataloader):
			images, labels = samples["image"], samples["label"]
			start_time = time.time()
			global_step += 1

			# Forward and backward
			optimizer.zero_grad()
			outputs = m(images)
			losses_name = ["total_loss", "x", "y", "w", "h", "conf", "cls"]
			losses = []
			for _ in range(len(losses_name)):
				losses.append([])
			for i in range(3):
				_loss_item = yolo_losses[i](outputs[i], labels)
				for j, l in enumerate(_loss_item):
					losses[j].append(l)
			losses = [sum(l) for l in losses]
			loss = losses[0]
			loss.backward()
			optimizer.step()

			if step > 0 and step % 10 == 0:
				_loss = loss.item()
				duration = float(time.time() - start_time)
				example_per_second = C.BATCH_SIZE / duration
				lr = optimizer.param_groups[0]['lr']
				print(
					"epoch [%.3d] iter = %d loss = %.2f example/sec = %.3f lr = %.5f "%
					(epoch, step, _loss, example_per_second, lr)
				)
				TRAINING_PARAMS["tensorboard_writer"].add_scalar("lr",
														lr,
														config["global_step"])
				TRAINING_PARAMS["tensorboard_writer"].add_scalar("example/sec",
														example_per_second,
														config["global_step"])
				for i, name in enumerate(losses_name):
					value = _loss if i == 0 else losses[i]
					TRAINING_PARAMS["tensorboard_writer"].add_scalar(name, value, global_step)

			if step > 0 and step % 1000 == 0:
				# m.train(False)
				_save_checkpoint(m.state_dict())
				# m.train(True)

		lr_scheduler.step()

	# m.train(False)
	_save_checkpoint(m.state_dict())
	# m.train(True)
	print('Finished')
