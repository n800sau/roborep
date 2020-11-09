import os
import glob
from datasets import SegVideoDataset
from models import UNet
from data.labels import LABELS
from config import Config

C = Config('unet')
C.BATCH_SIZE = 4

import cv2
import torch
from utils.misc import create_or_clean_dirs
from utils.inference import predict_segmentation

def load_model():
	model = UNet(input_channels=C.NUM_INPUT_CHANNELS, output_channels=C.NUM_CLASSES).cuda(C.GPU_ID)
	model.load_state_dict(torch.load(os.path.join(C.DS_BASE_DIR, C.INITIAL_WNAME) + '.pth'))
	return model

if __name__ == "__main__":

	create_or_clean_dirs((C.PREDICT_VIDS_OUTPUT_DIR, ))
	label_map = {}
	for l in LABELS:
		if 'segnet_val' in l:
			label_map[l['val']] = l['segnet_val']
		# +1 for background
		C.NUM_CLASSES = max(label_map.values()) + 1
	print('label_map=', label_map, C.NUM_CLASSES)
	for fname in glob.glob(os.path.join(C.PREDICT_VIDS_INPUT_DIR, '*.mp4')):
		input_dataset = SegVideoDataset(vid_path=fname, frame_size=(224, 224))
		ofname = os.path.join(C.PREDICT_VIDS_OUTPUT_DIR, os.path.basename(fname))
		out = None
		for bname,idx,img_arr in predict_segmentation(load_model(), input_dataset, batch_size=C.BATCH_SIZE, gpu_id=C.GPU_ID, video_mode=True):
			if out is None:
				out = cv2.VideoWriter(ofname, cv2.CAP_FFMPEG, fourcc=cv2.VideoWriter_fourcc(*'mp4v'), fps=input_dataset.fps, frameSize=tuple(reversed(img_arr.shape[:2])))
#			print('write', idx)
			out.write(img_arr)
		if not out is None:
			out.release()

	print('Finished')
