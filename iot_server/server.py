#!/usr/bin/env python3

import json
import time
from datetime import datetime, timedelta
from flask import Flask, escape, request, render_template, jsonify, session
from flask_socketio import SocketIO, send, emit
import gevent
from gevent import monkey, local as glocal
import redis
from dateutil.relativedelta import relativedelta
from calendar import monthrange

monkey.patch_socket()

REDIS_KEY = 'mq_list'

async_mode = 'gevent'

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app, message_queue='redis://', async_mode=async_mode)

def dprint(*args, **kwds):
	print(time.strftime('[%Y-%m-%d %H:%M:%S] '), end='')
	print(*args, **kwds)

@app.route('/')
def hello():
	name = 'Hello'
	return render_template('index.html', name=name)

@app.route('/refresh')
def refresh():
	print('Refresh')
	gevent.kill(session['redis_process'], exception=GreenletExit)
	session['redis_process'] = gevent.spawn(pull_redis)
	full_data = True
	return jsonify(ok=True)

@socketio.on('connect')
def test_connect():
	session['redis_process'] = gevent.spawn(pull_redis)
	emit('my response', {'data': 'Connected'})

@socketio.on('disconnect')
def test_disconnect():
	dprint('Client disconnected')

@socketio.on('full_data_load')
def handle_my_custom_event(json):
	dprint('received json from',  request.environ.get('HTTP_X_FORWARDED_FOR', request.remote_addr), ':', str(json))

def collect_data_for_period(r, w_name, start_ts, end_ts):
	ts_format = '%Y-%m-%d %H:%M:%S'
	start_ts = start_ts.strftime(ts_format)
	end_ts = end_ts.strftime(ts_format)
	rs = {}
	for k in r.keys('avg.*.' + w_name + '.*'):
		_,sensor_id,_,param = k.decode().split('.')
		if sensor_id not in rs:
			rs[sensor_id] = {}
		if param not in rs[sensor_id]:
			rs[sensor_id][param] = []
		for ts,vobj in [(ts.decode(), json.loads(vstr)) for ts,vstr in r.hgetall(k).items()]:
#			if w_name == 'hourly':
#				dprint(ts, start_ts)
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

def collect_data(r, full_data):
	ts_format = '%Y-%m-%d %H:%M:%S'
	dt = datetime.now()
	data = {'hourly': [], 'daily': [], 'monthly': []}
	# hourly
	w_name = 'hourly'
	i_range = range(max(1, dt.hour+1)) if full_data else range(max(0, dt.hour-1), max(1, dt.hour+1))
#	print('i_range', list(i_range))
	for i in i_range:
		start_ts = dt.replace(hour=0, microsecond=0, second=0, minute=0) + relativedelta(hours=i)
		end_ts = start_ts + relativedelta(hours=1)
		vals = collect_data_for_period(r, w_name, start_ts, end_ts)
#		print(i, len(vals))
		if vals:
			data[w_name].append({
				'start_ts': start_ts.strftime(ts_format),
				'vals': vals,
			})
	print('hourly count', len(data[w_name]), ', range', len(i_range))
	# daily
	w_name = 'daily'
	i_range = range(1, monthrange(dt.year, dt.month)[1]+1) if full_data else range(max(1, dt.day-1), monthrange(dt.year, dt.month)[1]+1)
	for i in i_range:
		start_ts = dt.replace(day=i, hour=0, microsecond=0, second=0, minute=0)
		end_ts = start_ts + relativedelta(days=1)
		vals = collect_data_for_period(r, w_name, start_ts, end_ts)
		if vals:
			data[w_name].append({
				'start_ts': start_ts.strftime(ts_format),
				'vals': vals,
			})
	# monthly
	w_name = 'monthly'
	i_range = range(1, dt.month+1)  if full_data else range(max(1, dt.month-1), dt.month+1)
	for i in i_range:
		start_ts = dt.replace(month=i, day=1, hour=0, microsecond=0, second=0, minute=0)
		end_ts = start_ts + relativedelta(months=1)
		vals = collect_data_for_period(r, w_name, start_ts, end_ts)
		if vals:
			data[w_name].append({
				'start_ts': start_ts.strftime(ts_format),
				'vals': vals,
			})
	data['server_ts'] =  time.strftime(ts_format)
	data['last'] = {}
	for last_rec in reversed(r.lrange(REDIS_KEY, 0, -1)):
		last_rec = json.loads(last_rec)
		if last_rec['sensor_id'] not in data['last']:
			data['last'][last_rec['sensor_id']] = {}
			data['last'][last_rec['sensor_id']] = last_rec
			data['last'][last_rec['sensor_id']]['timestamp'] = datetime.fromtimestamp(last_rec['ts']).strftime(ts_format)
	return data

def pull_redis():
	dprint('redis puller started')
	full_data = True
	r = redis.Redis()
	with app.test_request_context('/'):
		while True:
			data = collect_data(r, full_data=full_data)
			if data:
				if full_data:
					dprint('Sending full data')
				dprint('Sent hourly len:', len(data['hourly']))
				full_data = False
				socketio.emit('current_data', data)
			gevent.sleep(5)


@socketio.on("my error event")
def on_my_event(data):
	raise RuntimeError()

@socketio.on_error_default
def default_error_handler(e):
	dprint(request.event["message"]) # "my error event"
	dprint(request.event["args"])	# (data,)

def ack():
	dprint('message was received!')

if __name__ == '__main__':
	socketio.run(app, port=5000, debug=True)
