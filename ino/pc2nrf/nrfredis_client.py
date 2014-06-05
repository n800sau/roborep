#!/usr/bin/env python

import sys, os, redis, traceback, time, csv, json, subprocess
from subprocess import check_call
from datetime import datetime
import smtplib
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'include', 'swig'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib_py'))

from conf_ino import configure
import libcommon_py as common

MAX2READ = 100
MESSAGE_CHAN = 'nrf'

# secs
REPEAT_TIMEOUT = 300

def send_email(msgtxt, subject):
	msg = MIMEMultipart()
	msg['Subject'] = subject
	msg['From'] = 'itmousecage@gmail.com'
	msg['To'] = 'n800sau@gmail.com'
	msg.attach(MIMEText(msgtxt, 'plain', 'utf-8'))

	smtpserver = smtplib.SMTP("smtp.gmail.com",587)
	smtpserver.ehlo()
	smtpserver.starttls()
	smtpserver.ehlo
	smtpserver.login('itmousecage@gmail.com', 'rktnrf000')
	smtpserver.sendmail(msg['From'], msg['To'], msg.as_string())
	smtpserver.close()

def extract_time(ldict):
	return float(ldict['secs']) + float(ldict['moffset']) / 1000

def start_plot_process():
	subprocess.Popen([sys.executable, os.path.join(os.path.dirname(__file__), 'plotz.py')], close_fds=True)

if __name__ == '__main__':

	last_time = time.time()

	cfgobj = configure(os.path.dirname(__file__))
	cfg = cfgobj.as_dict('serial2redis')
	params = {'host': cfg.get('redis_host', 'localhost')}
	redis_port = cfg.get('redis_port', None)
	if redis_port:
		params['port'] = redis_port

	r = redis.Redis(**params)
	p = r.pubsub()
	p.subscribe(MESSAGE_CHAN)

	queues = {}
	for item in p.listen():
#		print item
		try:
			for marker in r.smembers('s.queues'):
				lasttimestamp = None
				if marker not in queues:
					queues[marker] = {'lastel' : None}
					queues[marker]['file'] = file(os.path.join(os.path.dirname(__file__), marker + '.csv'), 'a+')
					queues[marker]['csv'] = csv.writer(queues[marker]['file'])
					# read last csv file line
					for row in csv.reader(queues[marker]['file']):
						ldict = dict(zip(row[::2], row[1::2]))
						lasttimestamp = extract_time(ldict)
				qdata = queues[marker]
				els = r.lrange(marker, 0, MAX2READ)
				if qdata['lastel']:
					# remove old elements
					try:
						lastndx = els.index(qdata['lastel'])
					except ValueError:
						lastndx = -1
					if lastndx >= 0:
						els = els[:lastndx]
				if els:
					# save the last element for the future reference
					qdata['lastel'] = els[0]
				# write remains to csv
				for line in reversed(els):
					line = [s.strip() for s in line.split(common.DATA_SEPARATOR)[1:]]
					ldict = dict(zip(line[::2], line[1::2]))
					if ldict:
						if lasttimestamp is None or lasttimestamp < extract_time(ldict):
#						print time.strftime('%d/%m/%Y %H:%M:%S', time.localtime(float(ldict['msecs']) / 1000))
							if marker == 'q.R':
								if time.time() > last_time + REPEAT_TIMEOUT:
									r_collection = []
									last_time = time.time()
									send_email(
										'Triggered at %s : %s' % (datetime.fromtimestamp(extract_time(ldict)).strftime('%d/%m/%Y %H:%M:%S.%f'),
											'\n'.join(line)),
										'door notification %s, v: %s' % (time.strftime('%Y/%m/%d %H:%M'), ldict['v'])
									)
									start_plot_process()
								print ldict.keys()
								r.zadd('acc_x', extract_time(ldict), ldict['acc_x'])
								r.zadd('acc_y', extract_time(ldict), ldict['acc_y'])
								r.zadd('acc_z', extract_time(ldict), ldict['acc_z'])
							qdata['csv'].writerow(line + [
									datetime.fromtimestamp(extract_time(ldict)).strftime('%d/%m/%Y %H:%M:%S.%f'),
									datetime.now().strftime('%d/%m/%Y %H:%M:%S.%f')
								]
							)
						qdata['file'].flush()
		except KeyboardInterrupt:
			break
		except:
			traceback.print_exc()
		time.sleep(0.001)  # be nice to the system :)

	for qdata in queues.values():
		qdata['file'].close()

