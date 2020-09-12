#!/usr/bin/env python3

import time
import shutil
import numpy as np
import os
import glob
import torch
from torch.utils.data import Dataset, DataLoader
import torchvision.models as models
from PIL import Image
from sklearn.datasets import make_blobs
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from pykeops.torch import LazyTensor

import lib

OUTPUT_DIR = 'output'
INPUT_DIR = 'images'

use_cuda = torch.cuda.is_available()
dtype = 'float32' if use_cuda else 'float64'
torchtype = {'float32': torch.float32, 'float64': torch.float64}

def KMeans(x, K=10, Niter=10, verbose=True):
	N, D = x.shape  # Number of samples, dimension of the ambient space

	# K-means loop:
	# - x  is the point cloud,
	# - cl is the vector of class labels
	# - c  is the cloud of cluster centroids
	start = time.time()
	c = x[:K, :].clone()  # Simplistic random initialization
	x_i = LazyTensor(x[:, None, :])  # (Npoints, 1, D)

	for i in range(Niter):

		c_j = LazyTensor(c[None, :, :])  # (1, Nclusters, D)
		D_ij = ((x_i - c_j) ** 2).sum(-1)  # (Npoints, Nclusters) symbolic matrix of squared distances
		cl = D_ij.argmin(dim=1).long().view(-1)  # Points -> Nearest cluster

		Ncl = torch.bincount(cl).type(torchtype[dtype])  # Class weights
		for d in range(D):  # Compute the cluster centroids with torch.bincount:
			c[:, d] = torch.bincount(cl, weights=x[:, d]) / Ncl

	end = time.time()

	if verbose:
		print("K-means example with {:,} points in dimension {:,}, K = {:,}:".format(N, D, K))
		print('Timing for {} iterations: {:.5f}s = {} x {:.5f}s\n'.format(
				Niter, end - start, Niter, (end-start) / Niter))

	return cl, c


class TheDataset(Dataset):

	def __init__(self, img_dir):
		self.image_root_dir = img_dir
		self.file_list = list(lib.dir_images_generator(self.image_root_dir))
		print('Found %d in %s' % (len(self.file_list), self.image_root_dir))

	def __len__(self):
		return len(self.file_list)

	def __getitem__(self, index):
		fname = self.file_list[index]
		image = self.load_image(Image.open(fname))
		return {
			'fname': fname,
			'image': torch.FloatTensor(image),
		}

	def load_image(self, raw_image):
		raw_image = np.transpose(raw_image.resize((224, 224)), (2,1,0))
		return np.array(raw_image, dtype=np.float32)/255.0

if __name__ == "__main__":

	batch_size = 20

	input_dataset = TheDataset(img_dir=INPUT_DIR)

	print('size=', len(input_dataset))

	input_dataloader = DataLoader(input_dataset,
								batch_size=batch_size,
								shuffle=False,
								num_workers=0)

	img_data = np.zeros([len(input_dataset), 1000], dtype=np.float32)
#	img_data = torch.zeros([len(input_dataset), 1000], dtype=torch.float32)
	vgg16 = models.vgg16(pretrained=True).cuda()
	for batch_idx,batch in enumerate(input_dataloader):
		input_tensor = torch.autograd.Variable(batch['image'])
		input_tensor = input_tensor.cuda()
		output_tensor = vgg16(input_tensor).cpu()
#		print(output_tensor.shape)
#		del input_tensor
		for i in range(output_tensor.shape[0]):
			img_data[batch_idx*batch_size+i,:] = output_tensor.detach().numpy()[i,:]
			print(batch_idx*batch_size+i, torch.cuda.memory_allocated()//1024//1024, 'M')
		torch.cuda.empty_cache()
		torch.cuda.ipc_collect()
	print('np max', img_data.max())
	img_data_t = torch.from_numpy(img_data)
	print('torch max', img_data.max())
	cl, c = KMeans(img_data_t, 20)
	cl = cl.cpu()
	c = c.cpu()
	print('kmeans:', cl.shape, cl[:2], c.shape)
#	centers = [[1, 1, 2, 3], [-1, -1, 4,10], [1, -1, -2, -4]]
#	X, labels_true = make_blobs(n_samples=130, centers=centers, cluster_std=0.4, random_state=0)
	img_data_n = StandardScaler().fit_transform(img_data)
	print('img_data shape:', img_data_n.shape)

	# eps - The maximum distance between two samples for one to be
	# considered as in the neighborhood of the other. This is not a maximum
	# bound on the distances of points within a cluster. This is the most
	# important DBSCAN parameter to choose appropriately for your data set
	# and distance function.

	clustering = DBSCAN(eps=30, min_samples=2).fit(img_data_n)
	print('dbscan:')
	# Number of clusters in labels, ignoring noise if present.
	labels = clustering.labels_
	n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
	print('Estimated number of clusters: %d' % n_clusters_)
	print('Estimated number of noise points: %d' % list(labels).count(-1))
	cl = labels
	fmap = {}
	for i,fname in enumerate(input_dataset.file_list):
		k = int(cl[i])
		fmap[cl[i]] = fmap.get(cl[i], []) + [fname]
	for dname in [OUTPUT_DIR]:
		if os.path.exists(OUTPUT_DIR):
			shutil.rmtree(OUTPUT_DIR)
	for k,flist in fmap.items():
		dname = os.path.join(OUTPUT_DIR, '%02d' % int(k))
		if not os.path.exists(dname):
			os.makedirs(dname)
		for fname in flist:
			bname = os.path.basename(fname)
			os.symlink(os.path.abspath(fname), os.path.join(dname, bname))
#	plt.figure(figsize=(8,8))
#	plt.scatter(img_data[:, 0], img_data[:, 1], c=cl, s= 30000 / len(img_data), cmap="tab10")
#	plt.scatter(c[:, 0], c[:, 1], c='black', s=50, alpha=.8)
#	plt.axis([0, 1, 0, 1])
#	plt.tight_layout()
#	plt.savefig('plot.png', bbox_inches='tight')

print('Finished')
