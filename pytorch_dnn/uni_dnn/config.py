import os

class Config:

	def __init__(self, net_name):
		self.GPU_ID = 0
		self.NUM_CLASSES = 6
		self.DS_BASE_DIR = 'data'
		self.IMG_DIR = 'images'
		self.MASK_DIR = 'masks'
		self.BATCH_SIZE = 8
		self.NUM_INPUT_CHANNELS = 3
		self.BACKBONE_WNAME = None
		self.INITIAL_WNAME = '%s_model_best' % net_name
		self.SAVE_WNAME = net_name + '_model_best_%Y-%m-%d_%H:%M:%S'
		self.LEARNING_RATE = 0.002
		self.NUM_EPOCHS = 40

		self.MONTAGE_DIR = os.path.join(self.DS_BASE_DIR, 'test_images')
		self.LOG_DIR = os.path.join(self.DS_BASE_DIR, '%s_logs' % net_name)
		# dir with images to predict
		self.PREDICT_INPUT_DIR = os.path.join(self.DS_BASE_DIR, 'input')
		# dir to output predicted images
		self.PREDICT_OUTPUT_DIR = os.path.join(self.DS_BASE_DIR, 'output')
		# dir with video to predict
		self.PREDICT_VIDS_INPUT_DIR = os.path.join(self.DS_BASE_DIR, 'input_vids')
		# dir to output predicted videos
		self.PREDICT_VIDS_OUTPUT_DIR = os.path.join(self.DS_BASE_DIR, 'output_vids')

	def is_cuda(self):
		return self.GPU_ID >= 0
