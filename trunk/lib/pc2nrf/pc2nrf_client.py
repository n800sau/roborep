#!/usr/bin/env python

import sys, os, redis, traceback, time, csv, json, subprocess, json
from subprocess import check_call
from datetime import datetime
import smtplib
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText

inopath = os.path.join(os.path.dirname(__file__), '..', '..', 'ino')
sys.path.append(os.path.join(inopath, 'include', 'swig'))
sys.path.append(os.path.join(inopath, 'lib_py'))

from conf_ino import configure
import libcommon_py as common

MESSAGE_CHAN = 'pc2nrf_ready'
ABYSS = 'pc2nrf.js.obj'
LAST_ELEMENT = 'nrfredis_client_last_element'

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

def process_loop():
	global last_time, queues, r
	elements = r.lrange(ABYSS, 0, -1)
	last_element = r.get(LAST_ELEMENT)
	try:
		last_index = elements.index(last_element)
		elements = elements[last_index+1:]
	except ValueError:
		pass
	for elm in elements:
		try:
			ldict = json.loads(elm)['data']
		except ValueError:
			ldict = {}
		etype = ldict.get('type')
		if not etype is None:
			if etype not in queues:
				queues[etype] = {'file': file(os.path.join(os.path.dirname(__file__), 'data_%s.csv' % etype), 'a+')}
				queues[etype]['csv'] = csv.writer(queues[etype]['file'])
			qdata = queues[etype]
			# write data to csv
			if etype == common.PL_ACC:
				if time.time() > last_time + REPEAT_TIMEOUT:
					r_collection = []
					last_time = time.time()
					send_email(
						'Triggered at %s' % (datetime.fromtimestamp(extract_time(ldict)).strftime('%d/%m/%Y %H:%M:%S.%f')),
						'door notification %s, mv: %s' % (time.strftime('%Y/%m/%d %H:%M'), ldict['mv'])
					)
					start_plot_process()
				r.zadd('acc_x', extract_time(ldict), ldict['acc_x'])
				r.zadd('acc_y', extract_time(ldict), ldict['acc_y'])
				r.zadd('acc_z', extract_time(ldict), ldict['acc_z'])
			qdata['csv'].writerow([
					datetime.fromtimestamp(extract_time(ldict)).strftime('%d/%m/%Y %H:%M:%S.%f'),
					datetime.now().strftime('%d/%m/%Y %H:%M:%S.%f')
				] + reduce(lambda l, v: l + list(v), sorted(ldict.items()), [])
			)
			qdata['file'].flush()
		r.set(LAST_ELEMENT, elm)

if __name__ == '__main__':

	last_time = time.time()

	cfgobj = configure(os.path.dirname(__file__))
	cfg = cfgobj.as_dict('redis')
	params = {'host': cfg.get('redis_host', 'localhost')}
	redis_port = cfg.get('redis_port', None)
	if redis_port:
		params['port'] = redis_port

	r = redis.Redis(**params)
	p = r.pubsub()
	p.subscribe(MESSAGE_CHAN)

	queues = {}
	process_loop()
	for item in p.listen():
		try:
			process_loop()
		except KeyboardInterrupt:
			break
		except:
			traceback.print_exc()
		time.sleep(0.001)  # be nice to the system :)

	for qdata in queues.values():
		qdata['file'].close()

