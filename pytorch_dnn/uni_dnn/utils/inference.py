import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.switch_backend('agg')
plt.axis('off')
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas

import os
import torch
from torch.utils.data import DataLoader
import torchvision.transforms as transforms

def predict_normalised_batch(model, batch, gpu_id=-1):
	input_tensor = torch.autograd.Variable(batch['image'])

	if gpu_id >= 0:
		input_tensor = input_tensor.cuda(gpu_id)

	print('is_cuda:', input_tensor.is_cuda)

	predicted_tensor, softmaxed_tensor = model(input_tensor)

	return input_tensor, predicted_tensor, softmaxed_tensor

def save_segmentation(model, input_dataset, output_dir, batch_size, gpu_id=-1):

	input_dataloader = DataLoader(input_dataset, batch_size=batch_size, shuffle=False, num_workers=2)

	model.eval()

	for batch_idx,batch in enumerate(input_dataloader):

		input_tensor, predicted_tensor, softmaxed_tensor = predict_normalised_batch(model, batch, gpu_id)

		for idx, predicted_mask in enumerate(softmaxed_tensor):
			fig = plt.figure(dpi=100)

			a = fig.add_subplot(1,2,1)
			input_image = input_tensor[idx].cpu().transpose(0, 2)
			plt.imshow(input_image)
			a.set_title('Input Image')

			a = fig.add_subplot(1,3,2)
			predicted_mx = predicted_mask.detach().cpu().numpy()
			predicted_mx = predicted_mx.argmax(axis=0)
			plt.imshow(predicted_mx)
			a.set_title('Predicted Mask')

			bname = os.path.splitext(os.path.basename(batch['fname'][idx]))[0]
			fig.savefig(os.path.join(output_dir, "{}_{}.png".format(bname, idx)))

			plt.close(fig)
