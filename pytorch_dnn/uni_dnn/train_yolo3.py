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
from models.yolo3.utils import non_max_suppression, bbox_iou
from utils.misc import save_model_state, make_test_masked_images, get_criterion, create_or_clean_dirs
import numpy as np
from config import Config
from data.yolo3_params import TRAINING_PARAMS

C = Config('yolo3')
C.BACKBONE_WNAME = 'weights/darknet53_weights_pytorch.pth'
C.BATCH_SIZE = 16

def train(train_dataset):
	pass

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

def _save_checkpoint(model, loss_f):
	save_model_state(model, C.DS_BASE_DIR, '%s_%.4f' % (C.SAVE_WNAME, loss_f), C.INITIAL_WNAME)

if __name__ == "__main__":

	create_or_clean_dirs((C.MONTAGE_DIR, C.LOG_DIR))
#	print(TRAINING_PARAMS)

	dataset = YoloTrainDataset(os.path.join(C.DS_BASE_DIR, C.YOLO_IMG_DIR), os.path.join(C.DS_BASE_DIR, C.YOLO_LABEL_DIR), img_size=(224, 224))
	json.dump(dataset.labels(), open(os.path.join(C.DS_BASE_DIR, 'yolo_labels.json'), 'wt'), indent=2)

	C.NUM_CLASSES = len(dataset.labels())
	TRAINING_PARAMS['classes'] = C.NUM_CLASSES
	initial_wname = os.path.join(C.DS_BASE_DIR, C.INITIAL_WNAME) + '.pth'
	load_weights = os.path.exists(initial_wname)
	m = Yolo3(TRAINING_PARAMS, backbone_weights_path=None if load_weights else C.BACKBONE_WNAME)
	if C.is_cuda():
		m = m.cuda(C.GPU_ID)


	prev_loss = float('inf')

	m.train(True)

	# Optimizer and learning rate
	optimizer = _get_optimizer(m)
	lr_scheduler = optim.lr_scheduler.StepLR(optimizer,
		step_size=TRAINING_PARAMS['lr']["decay_step"], gamma=TRAINING_PARAMS['lr']["decay_gamma"])

	# Set data parallel
	m = nn.DataParallel(m)

	if load_weights:
		print("Load pretrained weights from {}".format(initial_wname))
		m.load_state_dict(torch.load(initial_wname))

	# YOLO loss with 3 scales
	yolo_losses = []
	for i in range(3):
		yolo_losses.append(YOLOLoss(TRAINING_PARAMS["anchors"][i], C.NUM_CLASSES, (TRAINING_PARAMS["img_w"], TRAINING_PARAMS["img_h"])))

#	print(summary(m, (3, 416, 416), device='cpu'))

	dataloader = torch.utils.data.DataLoader(dataset, batch_size=C.BATCH_SIZE, shuffle=True, num_workers=32, pin_memory=True)

	with SummaryWriter(C.LOG_DIR, comment=f'LR_{C.LEARNING_RATE}_BS_{C.BATCH_SIZE}') as tf_writer:

		# Start the training loop
		print("Start training.")
		global_step = 0
		loss_f = 0
		for epoch in range(C.NUM_EPOCHS):
			loss_f = 0
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
				loss_f += loss.float()

				if step > 0 and step % 10 == 0:
					_loss = loss.item()
					duration = float(time.time() - start_time)
					example_per_second = C.BATCH_SIZE / duration
					lr = optimizer.param_groups[0]['lr']
					print(
						"epoch [%.3d] iter = %d loss = %.2f example/sec = %.3f lr = %.5f "%
						(epoch, step, _loss, example_per_second, lr)
					)
					tf_writer.add_scalar("lr", lr, global_step)
					tf_writer.add_scalar("example/sec", example_per_second, global_step)
					for i, name in enumerate(losses_name):
						value = _loss if i == 0 else losses[i]
						tf_writer.add_scalar(name, value, global_step)

			if loss_f < prev_loss:
				prev_loss = loss_f
				m.train(False)
				_save_checkpoint(m, loss_f)
				m.train(True)

			lr_scheduler.step()

		m.train(False)
		_save_checkpoint(m, loss_f)
		m.train(True)
	print('Finished')
