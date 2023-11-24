#!/usr/bin/env python3

import glob, os, datetime, pickle
from sklearn.cluster import DBSCAN, KMeans
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

DIRPATH = os.path.expanduser('~/sshfs/asus/root/rus_hard/garage')
NPDIR = 'output/npdata'
DSTDIR = 'output/clustered'
PLOTDIR = 'output/plots'

if not os.path.exists(PLOTDIR):
	os.makedirs(PLOTDIR)

data = []
for fname in glob.glob(os.path.join(NPDIR, '*.npz')):
	print('Processing %s...' % fname)
	loaded = np.load(fname)
	for k,v in loaded.items():
		print('Adding %s...' % k)
		v = v.flatten()
#		print('shape:', v.shape)
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
#clt = KMeans(n_jobs=-1)
clt = KMeans(n_clusters=2, random_state=0, n_jobs=-1)
#clt = DBSCAN(algorithm='auto', eps=3, leaf_size=30, metric="euclidean", n_jobs=-1)
print('Fitting...')
clt.fit(features)

# determine the total number of unique faces found in the dataset
labelIDs = np.unique(clt.labels_)
numUniques = len(np.where(labelIDs > -1)[0])
print("[INFO] # uniques: {}".format(numUniques))

plotdata = {}
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
		f_id = os.path.basename(data[i]['name']).split('-')[0]
		# 20180523165526-09-00...
		ts = datetime.datetime.strptime(f_id, '%Y%m%d%H%M%S')
		dt = ts.date().strftime('%Y%m%d')
		plotdata[dt] = plotdata.get(dt, [])
		plotdata[dt].append({
			'ts': ts,
			'v': labelID,
		})
		slname = os.path.splitext(data[i]['name'])
		slname = os.path.join(DSTDIR, slname[0] + '_' + ('%02d' % labelID) + slname[-1])
		if not os.path.exists(os.path.dirname(slname)):
			os.makedirs(os.path.dirname(slname))
		os.symlink(os.path.join(DIRPATH, data[i]['name']), slname)

pickle.dump(clt, open(os.path.join(DSTDIR, 'kmeans.pickle'), 'wb'), pickle.HIGHEST_PROTOCOL)
pickle.dump(plotdata, open(os.path.join(PLOTDIR, 'plotdata.pickle'), 'wb'), pickle.HIGHEST_PROTOCOL)

