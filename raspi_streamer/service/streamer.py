from __future__ import print_function
import re, random, time, sys, os, urlparse, json, zipfile, tempfile, urllib2, imp, datetime, traceback
import redis

REDIS_KEY = 'raspicam_settings'

def load_values(env):
	rs = {}
	return json.dumps({'settings': json.loads(redis.Redis().get(REDIS_KEY))})

def apply_values(env):
	rs = None
	data = json.load(env['wsgi.input'])
	print('INPUT DATA', data)
	r = redis.Redis()
	r.set(REDIS_KEY, json.dumps(data['settings']))
	return load_values(env)

def read_frame(env):
	with output.condition:
		output.condition.wait()
		frame = output.frame
		output.camera.brightness = random.randint(0, 100)
	return 'Hello'

def router(env, start_response):
	try:
		path_info = env.get('PATH_INFO', '/')
		print('ENV(' + path_info + '):', env)
		rs = None
		print( 'Query(' + path_info + '): %s' % env['QUERY_STRING'])
		headers = []
		if re.match(r'.*/stream.mjpg', path_info, re.I):
			headers += [
				('Age', '0'),
				('Pragma', 'no-cache'),
				('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME'),
			]
			start_response('200 OK', headers)
			while True:
				frame = read_frame(env)
				if frame is None:
					break
				yield b'--FRAME\r\n'
				yield b'Content-Type:image/jpeg\r\n'
				yield b'Content-Length:%s\r\n' % len(frame)
				yield frame
				yield b'\r\n'
		else:
			content_type = 'text/html'
			if re.match(r'.*/apply_values', path_info, re.I):
				content_type = 'text/json'
				rs = apply_values(env)
			elif re.match(r'.*/load_values', path_info, re.I):
				content_type = 'text/json'
				rs = load_values(env)
			if rs is None:
				start_response('404 File Not Found', [])
			else:
				start_response('200 OK', [('Content-Type', content_type)] + headers)
	except Exception, e:
		print(traceback.format_exc())
		start_response('500 Internal Server Error', [('Content-Type', 'text/json')])
		rs = json.dumps({'error': str(e)})
		yield rs
