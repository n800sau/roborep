from datasets import PredictImageDataset
from models import SegNet
from config import Config

C = Config('segnet')
C.NUM_CLASSES = 4
C.BATCH_SIZE = 4

import os
import torch
from PIL import Image
from utils.misc import create_or_clean_dirs
from utils.inference import save_segmentation

def load_model():
	model = SegNet(input_channels=C.NUM_INPUT_CHANNELS, output_channels=C.NUM_CLASSES).cuda(C.GPU_ID)
	model.load_state_dict(torch.load(os.path.join(C.DS_BASE_DIR, C.INITIAL_WNAME) + '.pth'))
	return model

if __name__ == "__main__":

	create_or_clean_dirs((C.PREDICT_OUTPUT_DIR, ))
	input_dataset = PredictImageDataset(input_dir=C.PREDICT_INPUT_DIR, img_shape=(224, 224))
	for bname,idx,img_arr in save_segmentation(load_model(), input_dataset, batch_size=C.BATCH_SIZE, gpu_id=C.GPU_ID):
		fname = os.path.join(C.PREDICT_OUTPUT_DIR, "{}_{}.png".format(bname, idx))
		im = Image.fromarray(img_arr)
		im.save(fname)

	print('Finished')