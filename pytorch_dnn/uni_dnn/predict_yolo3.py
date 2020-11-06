import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.switch_backend('agg')
import matplotlib.patches as patches
from matplotlib.ticker import NullLocator

from datasets import YoloImageDataset
from models import Yolo3
from config import Config
from data.yolo3_params import TRAINING_PARAMS

C = Config('yolo3')
C.BATCH_SIZE = 4

import os
import json
import torch
from PIL import Image
from utils.misc import create_or_clean_dirs
from models.yolo3.yolo_loss import YOLOLoss
from models.yolo3.utils import non_max_suppression, bbox_iou
from torch.utils.data import DataLoader
import torch.nn as nn
import numpy as np
from utils.colors import label_preview_colors_0_1

def predict_location(m, yolo_losses, input_dataset, classes, batch_size=1, gpu_id=-1):


	# Start inference
	input_dataloader = DataLoader(input_dataset, batch_size=batch_size, shuffle=False, num_workers=4)

	m.train(False)

	# Set data parallel
	m = nn.DataParallel(m)

	for batch_idx,batch in enumerate(input_dataloader):
		# inference
		with torch.no_grad():
			outputs = m(batch['image'])
			print('initial outputs shape', outputs[0].shape, outputs[1].shape, outputs[2].shape)
			output_list = []
			for i in range(3):
				output_list.append(yolo_losses[i](outputs[i]))
			print('precat output_list shapes', output_list[0].shape, output_list[1].shape, output_list[2].shape)
			output = torch.cat(output_list, 1)
			print('aftercat output shape', output.shape)
			batch_detections = non_max_suppression(output, C.NUM_CLASSES, conf_thres=TRAINING_PARAMS["confidence_threshold"], nms_thres=0.45)
		for idx, detections in enumerate(batch_detections):
			plt.figure()
			fig, ax = plt.subplots(1)
			input_image = input_dataset.recover_image(batch['image'][idx])
			ax.imshow(input_image)
			if detections is not None:
				print('%s - %d detections' % (batch['fname'][idx], len(detections)))
				unique_labels = detections[:, -1].cpu().unique()
				n_cls_preds = len(unique_labels)
				for x1, y1, x2, y2, conf, cls_conf, cls_pred in detections:
					color = label_preview_colors_0_1()[int(np.where(unique_labels == int(cls_pred))[0])]
					# Rescale coordinates to original dimensions
					ori_h, ori_w = input_image.shape[:2]
					pre_h, pre_w = TRAINING_PARAMS["img_h"], TRAINING_PARAMS["img_w"]
					box_h = ((y2 - y1) / pre_h) * ori_h
					box_w = ((x2 - x1) / pre_w) * ori_w
					y1 = (y1 / pre_h) * ori_h
					x1 = (x1 / pre_w) * ori_w
					# Create a Rectangle patch
					bbox = patches.Rectangle((x1, y1), box_w, box_h, linewidth=2, edgecolor=color, facecolor='none')
					# Add the bbox to the plot
					ax.add_patch(bbox)
					# Add label
					clabel = classes[int(cls_pred)]
					plt.text(x1, y1, s=clabel, color='white', verticalalignment='top', bbox={'color': color, 'pad': 0})
					print('Found %s conf:%.2f cls_conf:%.2f (x:%d, y:%d %dx%d)' % (clabel, conf, cls_conf, x1, y1, box_w, box_h))
			# Save generated image with detections
			plt.axis('off')
			plt.gca().xaxis.set_major_locator(NullLocator())
			plt.gca().yaxis.set_major_locator(NullLocator())
			fig.canvas.draw()
			ncols, nrows = fig.canvas.get_width_height()
			img_arr = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep='').reshape(nrows, ncols, 3)
			plt.close(fig)
			yield os.path.basename(batch['fname'][idx]), idx, img_arr

def load_model():
	m = Yolo3(TRAINING_PARAMS, backbone_weights_path=C.BACKBONE_WNAME)
	print("Load weights from {}".format(C.INITIAL_WNAME))
	m.load_state_dict(torch.load(os.path.join(C.DS_BASE_DIR, C.INITIAL_WNAME + '.pth')), strict=False)
	if C.is_cuda():
		m = m.cuda(C.GPU_ID)
	return m

if __name__ == "__main__":
	create_or_clean_dirs((C.PREDICT_OUTPUT_DIR, ))
	classes = json.load(open(os.path.join(C.DS_BASE_DIR, 'yolo_labels.json')))
	C.NUM_CLASSES = len(classes)
	TRAINING_PARAMS['classes'] = C.NUM_CLASSES
	input_dataset = YoloImageDataset(os.path.join(C.DS_BASE_DIR, C.YOLO_IMG_DIR), (TRAINING_PARAMS["img_w"], TRAINING_PARAMS["img_h"]))
	# YOLO loss with 3 scales
	yolo_losses = []
	for i in range(3):
		yolo_losses.append(YOLOLoss(TRAINING_PARAMS["anchors"][i], C.NUM_CLASSES, (TRAINING_PARAMS["img_w"], TRAINING_PARAMS["img_h"])))
	for bname,idx,img_arr in predict_location(load_model(), yolo_losses, input_dataset, classes, batch_size=C.BATCH_SIZE, gpu_id=C.GPU_ID):
		fname = os.path.join(C.PREDICT_OUTPUT_DIR, "{}_{}.png".format(bname, idx))
		im = Image.fromarray(img_arr)
		im.save(fname)

	print('Finished')
