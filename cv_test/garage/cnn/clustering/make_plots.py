#!/usr/bin/env python3

import os, datetime, pickle
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import matplotlib.dates as mdates

PLOTDIR = 'output/plots'

if not os.path.exists(PLOTDIR):
	os.makedirs(PLOTDIR)

plotdata = pickle.load(open(os.path.join(PLOTDIR, 'plotdata.pickle'), 'rb'))

for k,v in plotdata.items():

	v.sort(key=lambda x: x['ts'])

	ts = [t['ts'] for t in v]
	labels = [l['v'] for l in v]

	cdate = ts[0].date()

	fig, ax = plt.subplots(ncols=1, figsize=(10, 5))
	ax.xaxis.set_major_formatter(mdates.DateFormatter('%H'))
	ax.xaxis.set_major_locator(mdates.HourLocator())
	ax.set_xlim(cdate, cdate + datetime.timedelta(days=1))
	ax.plot(ts, labels, 'o-')
	ax.set_title(ts[0].strftime('%d/%m/%Y'))
	fig.autofmt_xdate()
	pbname = ts[0].strftime('%Y-%m-%d') + '.png'
	pfname = os.path.join(PLOTDIR, pbname)
	plt.savefig(pfname, dpi=96)
