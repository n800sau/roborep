#!/usr/bin/env python3

import json
import time
from datetime import datetime, timedelta
from flask import Flask, escape, request, render_template
from flask_socketio import SocketIO, send, emit
import gevent
from gevent import monkey
import redis

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

@socketio.on('my event')
def handle_my_custom_event(json):
	print('received json: ' + str(json))

def collect_data(r, w_name, start_ts, end_ts):
	rs = {}
	for k in r.keys('avg.*.' + w_name + '.*'):
		_,sensor_id,_,param = str(k).split('.')
		if sensor_id not in rs:
			rs[sensor_id] = {}
		if param not in rs[sensor_id]:
			rs[sensor_id][param] = []
		for ts,vobj in [(ts, json.loads(vstr)) for ts,vstr in r.hgetall(k).items()]:
			ts = float(ts)
			if ts >= start_ts and ts < end_ts:
				rs[sensor_id][param].append((ts, vobj['v']))
		rs[sensor_id][param].sort(key=lambda v: float(v[0]))
	return rs

def pull_redis():
	r = redis.Redis()
	with app.test_request_context('/'):
		while True:
			dt = datetime.now()
			data = {}
			for w_name,dt,td in (
						['monthly', dt.replace(day=1, hour=0, microsecond=0, second=0, minute=0), timedelta(days=31)],
						['daily', dt.replace(hour=0, microsecond=0, second=0, minute=0), timedelta(days=1)],
						['hourly', dt.replace(microsecond=0, second=0, minute=0), timedelta(hours=1)],
					):
				ts_start = time.mktime(dt.timetuple())
				ts_end = time.mktime((dt + td).timetuple())
				data[w_name] = {
					'ts_start': ts_start,
					'ts_end': ts_end,
					'vals': collect_data(r, w_name, ts_start, ts_end),
				}
				if not data[w_name]:
					del data[w_name]
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
