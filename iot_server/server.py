#!/usr/bin/env python3

import json
import time
from datetime import datetime, timedelta
from flask import Flask, escape, request, render_template
from flask_socketio import SocketIO, send, emit
import gevent
from gevent import monkey
import redis
from dateutil.relativedelta import relativedelta
from calendar import monthrange

monkey.patch_socket()

REDIS_KEY = 'mq2_list'

async_mode = 'gevent'

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app, message_queue='redis://', async_mode=async_mode)

@app.route('/')
def hello():
	name = 'Hello'
	return render_template('index.html', name=name)

@socketio.on('connect')
def test_connect():
	gevent.spawn(pull_redis)
	emit('my response', {'data': 'Connected'})

@socketio.on('disconnect')
def test_disconnect():
	print('Client disconnected')

@socketio.on('full_data_load')
def handle_my_custom_event(json):
	print('received json: ' + str(json))

def collect_data_for_period(r, w_name, start_ts, end_ts):
	ts_format = '%Y-%m-%d %H:%M:%S'
	start_ts = start_ts.strftime(ts_format)
	end_ts = end_ts.strftime(ts_format)
	rs = {}
	for k in r.keys('avg.*.' + w_name + '.*'):
#		print('@@@', k.decode())
		_,sensor_id,_,param = k.decode().split('.')
		if sensor_id not in rs:
			rs[sensor_id] = {}
		if param not in rs[sensor_id]:
			rs[sensor_id][param] = []
		for ts,vobj in [(ts.decode(), json.loads(vstr)) for ts,vstr in r.hgetall(k).items()]:
			if ts == start_ts:
				rs[sensor_id][param].append(vobj['v'])
#		rs[sensor_id][param].sort(key=lambda v: float(v[0]))
		val = sum(rs[sensor_id][param])/len(rs[sensor_id][param]) if rs[sensor_id][param] else 0
		if val:
			rs[sensor_id][param] = val
		else:
			del rs[sensor_id][param]
	for sensor_id in list(rs.keys()):
		if not rs[sensor_id]:
			del rs[sensor_id]
	return rs

def collect_data(r):
	ts_format = '%Y-%m-%d %H:%M:%S'
	dt = datetime.now()
	data = {'hourly': [], 'daily': [], 'monthly': []}
	# hourly
	w_name = 'hourly'
	for i in range(24):
		ts_start = dt.replace(hour=0, microsecond=0, second=0, minute=0) - relativedelta(hours=i)
		ts_end = ts_start + relativedelta(hours=1)
		vals = collect_data_for_period(r, w_name, ts_start, ts_end)
		if vals:
			data[w_name].append({
				'ts_start': ts_start.strftime(ts_format),
				'vals': vals,
			})
	# daily
	w_name = 'daily'
	for i in range(1, monthrange(dt.year, dt.month)[1]+1):
		ts_start = dt.replace(day=i, hour=0, microsecond=0, second=0, minute=0)
		ts_end = ts_start + relativedelta(days=1)
		vals = collect_data_for_period(r, w_name, ts_start, ts_end)
		if vals:
			data[w_name].append({
				'ts_start': ts_start.strftime(ts_format),
				'vals': vals,
			})
	# monthly
	w_name = 'monthly'
	for i in range(1, 13):
		ts_start = dt.replace(month=i, day=1, hour=0, microsecond=0, second=0, minute=0)
		ts_end = ts_start + relativedelta(months=1)
		vals = collect_data_for_period(r, w_name, ts_start, ts_end)
		if vals:
			data[w_name].append({
				'ts_start': ts_start.strftime(ts_format),
				'vals': vals,
			})
	return data

def pull_redis():
	r = redis.Redis()
	with app.test_request_context('/'):
		while True:
			data = collect_data(r)
			if data:
				socketio.emit('current_data', data)
			gevent.sleep(5)


@socketio.on("my error event")
def on_my_event(data):
	raise RuntimeError()

@socketio.on_error_default
def default_error_handler(e):
	print(request.event["message"]) # "my error event"
	print(request.event["args"])	# (data,)

def ack():
	print('message was received!')

if __name__ == '__main__':
	socketio.run(app, debug=True)
