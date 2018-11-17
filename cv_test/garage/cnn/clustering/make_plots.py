#!/usr/bin/env python3

import os, datetime, pickle, time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import matplotlib.dates as mdates
from sklearn.cluster import DBSCAN

PLOTDIR = 'output/plots'
TESTDIR = 'test_images'

if not os.path.exists(PLOTDIR):
	os.makedirs(PLOTDIR)

#plotdata = pickle.load(open(os.path.join(PLOTDIR, 'plotdata.pickle'), 'rb'))
plotdata = pickle.load(open(os.path.join(TESTDIR, 'plotdata.pickle'), 'rb'))

def get_median_filtered(signal, threshold=3):
	signal = signal.copy()
	difference = np.abs(signal - np.median(signal))
	median_difference = np.median(difference)
	if median_difference == 0:
		s = 0
	else:
		s = difference / float(median_difference)
	mask = s > threshold
	signal[mask] = np.median(signal)
	return signal

clt = DBSCAN(metric="euclidean", n_jobs=-1)

for k,v in plotdata.items():

	v.sort(key=lambda x: x['ts'])

	ts = [t['ts'] for t in v]

	# cluster by close time
	clt.fit([[time.mktime(t['ts'].timetuple())] for t in v])
	labelIDs = np.unique(clt.labels_)
	print('Uniq count', len(labelIDs))


	labels = np.array([l['v'] for l in v])
	filtered_labels = get_median_filtered(labels, threshold=2)
	print('filtered_labels', filtered_labels)
	outlier_idx = np.where(labels != filtered_labels)
	print('outlier_idx', outlier_idx)

	cdate = ts[0].date()

	kw = dict(marker='o', linestyle='none', color='r', alpha=0.3)
	fig, ax = plt.subplots(ncols=1, figsize=(10, 5))
	ax.xaxis.set_major_formatter(mdates.DateFormatter('%H'))
	ax.xaxis.set_major_locator(mdates.HourLocator())
	ax.set_xlim(cdate, cdate + datetime.timedelta(days=1))
	ax.set_ylim(0, 4)
	ax.plot(ts, labels, 'o-')
	ax.set_title(ts[0].strftime('%d/%m/%Y %a'))
	fig.autofmt_xdate()
	pbname = ts[0].strftime('%Y-%m-%d_%a') + '.png'
	pfname = os.path.join(PLOTDIR, pbname)
	plt.savefig(pfname, dpi=96)
