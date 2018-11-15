#!/usr/bin/env python3

import glob, os
from sklearn.cluster import DBSCAN, KMeans
import numpy as np

DIRPATH = os.path.expanduser('~/sshfs/asus/root/rus_hard/garage')
NPDIR = 'output/npdata'
DSTDIR = 'output/clustered'

data = []
for fname in glob.glob(os.path.join(NPDIR, '*.npz')):
	print('Processing %s...' % fname)
	loaded = np.load(fname)
	for k,v in loaded.items():
		print('Adding %s...' % k)
		v = v.flatten()
		print('shape:', v.shape)
		print('non-zero:', np.sum(v!=0))
		print('min:', np.min(v))
		print('max:', np.max(v))
		data.append({
			'name': k,
			'features': v,
		})

if not data:
	raise Exception('No data found')

features = np.array([d['features'] for d in data])
#print('features.shape:', features.shape)
clt = KMeans(n_jobs=-1)
#clt = DBSCAN(algorithm='auto', eps=3, leaf_size=30, metric="euclidean", n_jobs=-1)
print('Fitting...')
clt.fit(features)

# determine the total number of unique faces found in the dataset
labelIDs = np.unique(clt.labels_)
numUniques = len(np.where(labelIDs > -1)[0])
print("[INFO] # uniques: {}".format(numUniques))

# loop over the unique face integers
for labelID in labelIDs:
	# find all indexes into the `data` array that belong to the
	# current label ID, then randomly sample a maximum of 25 indexes
	# from the set
	print("[INFO] encoding for ID: {}".format(labelID))
	idxs = np.where(clt.labels_ == labelID)[0]

	print('%d of %s' % (len(idxs), labelID))

	for i in idxs:
		data[i]['label'] = labelID
#		print(data[i]['name'], '-', data[i]['label'])
		slname = os.path.splitext(data[i]['name'])
		slname = os.path.join(DSTDIR, slname[0] + '_' + ('%02d' % labelID) + slname[-1])
		if not os.path.exists(os.path.dirname(slname)):
			os.makedirs(os.path.dirname(slname))
		os.symlink(os.path.join(DIRPATH, data[i]['name']), slname)
