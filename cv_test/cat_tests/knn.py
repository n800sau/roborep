import os, sys, shutil
import numpy as np
from sklearn.cluster import KMeans

SRCDIR = 'output'
OUTDIR = 'sorted'
N_CLUSTERS = 4

bnames = []
ifnames = []
data = []

i = 0
for root, dirs, files in os.walk(SRCDIR, followlinks=True):
	for fname in files:
		fname = os.path.join(os.path.join(root, fname))
		try:
			if os.path.splitext(fname)[-1] == '.npy':
				features = np.load(fname)
#				print features.shape
				data.append(features)
				bnames.append(os.path.basename(fname))
				ifnames.append(os.path.splitext(fname)[0])
		except Exception, e:
			print e

clt = KMeans(n_clusters=N_CLUSTERS)
clt.fit(data)
q = [0 for i in range(len(np.unique(clt.labels_)))]
print 'Size', len(q)
for i in range(len(clt.labels_)):
	label = clt.labels_[i]
	q[label] += 1
	dpath = os.path.join(OUTDIR, str(label))
	if not os.path.exists(dpath):
		os.makedirs(dpath)
	np.save(os.path.join(dpath, bnames[i]), data[i])
	shutil.copyfile(ifnames[i], os.path.join(dpath, os.path.basename(ifnames[i])))

print q

print 'Finished'
