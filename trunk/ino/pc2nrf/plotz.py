#!/usr/bin/env python

import csv, time
import matplotlib as mpl
mpl.use('Agg')
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime, timedelta
import smtplib
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.image import MIMEImage

TIMEOUT = 120 #secs

def send_email(msgtxt, fnamelist):
	msg = MIMEMultipart()
	msg['Subject'] = 'door plots %s' % time.strftime('%Y/%m/%d %H:%M')
	msg['From'] = 'itmousecage@gmail.com'
	msg['To'] = 'n800sau@gmail.com'
	msg.attach(MIMEText(msgtxt, 'plain', 'utf-8'))
	for fname in fnamelist:
		msg.attach(MIMEImage(file(fname).read()))

	smtpserver = smtplib.SMTP("smtp.gmail.com",587)
	smtpserver.ehlo()
	smtpserver.starttls()
	smtpserver.ehlo
	smtpserver.login('itmousecage@gmail.com', 'rktnrf000')
	smtpserver.sendmail(msg['From'], msg['To'], msg.as_string())
	smtpserver.close()


timeout = timedelta(seconds=TIMEOUT)
f = file('q.R.csv')
vf = csv.reader(f)
data_time = []
data_x = []
last_t = None
fnamelist = []
for row in vf:
	ldict = dict(zip(row[::2], row[1::2]))
	t = datetime.fromtimestamp(float(ldict['secs']) + float(ldict['moffset'])/1000)
	if last_t is None or last_t + timeout < t:
		# draw plot and start next plot
		if data_x:
			# draw plot
			fig = plt.figure()
			ax1 = fig.add_subplot(111)

			ax1.set_title("Numbers")
			ax1.set_xlabel('time')
			ax1.set_ylabel('Z-z-z')

			ax1.plot(data_time, data_x, color='r', label='the data')
			plt.gcf().autofmt_xdate()

			leg = ax1.legend()

			#plt.show()
			fname = 'plot_%s.png' % last_t.strftime('%Y.%m.%d-%H:%M')
			plt.savefig(fname, bbox_inches='tight')
			fnamelist.append(fname)
			print fname, 'saved'
		data_x = []
		data_time = []
	data_time.append(t)
	data_x.append(float(ldict['acc_x']))
	last_t = t
send_email('dorr plots', fnamelist)

#data = np.genfromtxt('q.R.csv', delimiter=',', usecols = (1, 7), names=['x', 'y'])



