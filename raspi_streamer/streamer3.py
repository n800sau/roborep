import io
import random
import picamera
import logging
import socketserver
from threading import Condition
from http import server
import redis
import json

PAGE="""\
<html>
<head>
<title>Angular image demo</title>
</head>
<body>
<div>Angular Image Demo</div>
<img src="stream.mjpg" width="160" height="120" />
</body>
</html>
"""

REDIS_KEY = 'raspicam_settings'

settings_map = {
	'one': 'brightness',
	'two': 'contrast',
}

class StreamingOutput(object):
	def __init__(self, camera):
		self.frame = None
		self.camera = camera
		self.buffer = io.BytesIO()
		self.condition = Condition()
		self.old_settings = {}

	def update_settings(self):
		r = redis.Redis()
		rval = r.get(REDIS_KEY)
		settings = json.loads(rval.decode('utf-8')) if rval else None
		if settings:
			for k,v in settings.items():
				km = settings_map.get(k, None)
				if km:
					if v != self.old_settings.get(k, None):
						setattr(self.camera, km, v)
						print('set %s to %s' % (km, v))
			self.old_settings = settings

	def write(self, buf):
		if buf.startswith(b'\xff\xd8'):
			# New frame, copy the existing buffer's content and notify all
			# clients it's available
			self.buffer.truncate()
			with self.condition:
				self.frame = self.buffer.getvalue()
				self.condition.notify_all()
			self.buffer.seek(0)
		return self.buffer.write(buf)

class StreamingHandler(server.BaseHTTPRequestHandler):

	def do_GET(self):
		if self.path == '/favicon.ico':
			self.send_response(404)
			self.end_headers()
		elif self.path == '/cam/':
			self.send_response(301)
			self.send_header('Location', '/cam/index.html')
			self.end_headers()
		elif self.path == '/cam/index.html':
			content = open('static/index.html').read()
			self.send_response(200)
			self.send_header('Content-Type', 'text/html')
			self.send_header('Content-Length', len(content))
			self.end_headers()
			self.wfile.write(content.encode('utf-8'))
		elif self.path == '/cam/app.js':
			content = open('static/app.js').read()
			self.send_response(200)
			self.send_header('Content-Type', 'application/js')
			self.send_header('Content-Length', len(content))
			self.end_headers()
			self.wfile.write(content.encode('utf-8'))
		elif self.path == '/cam/load_values':
			data = redis.Redis().get(REDIS_KEY)
			if data:
				content = json.dumps({'settings': json.loads(data.decode('utf-8'))})
			else:
				content = '{}'
			self.send_response(200)
			self.send_header('Content-Type', 'application/js')
			self.send_header('Content-Length', len(content))
			self.end_headers()
			self.wfile.write(content.encode('utf-8'))
		elif self.path == '/cam/stream.mjpg':
			self.send_response(200)
			self.send_header('Age', 0)
			self.send_header('Cache-Control', 'no-cache, private')
			self.send_header('Pragma', 'no-cache')
			self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
			self.end_headers()
			try:
				while True:
					with output.condition:
						output.condition.wait()
						frame = output.frame
					output.update_settings()
					self.wfile.write(b'--FRAME\r\n')
					self.send_header('Content-Type', 'image/jpeg')
					self.send_header('Content-Length', len(frame))
					self.end_headers()
					self.wfile.write(frame)
					self.wfile.write(b'\r\n')
			except Exception as e:
				logging.warning(
					'Removed streaming client %s: %s',
					self.client_address, str(e))
		else:
			self.send_error(404)
			self.end_headers()

	def do_POST(self):
		if self.path == '/cam/apply_values':
			data = json.loads(self.rfile.read(int(self.headers.get('Content-Length'))).decode('utf-8'))
			rs = json.dumps(data).encode('utf-8')
			self.send_response(200)
			self.send_header('Content-Type', 'text/json')
			self.send_header('Content-Length', len(rs))
			self.end_headers()
			r = redis.Redis()
			r.set(REDIS_KEY, json.dumps(data['settings']).encode('utf-8'))
			self.wfile.write(rs)
		else:
			self.send_error(404)
			self.end_headers()

class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
	allow_reuse_address = True
	daemon_threads = True

with picamera.PiCamera(resolution='640x480', framerate=24) as camera:
	output = StreamingOutput(camera)
	camera.start_recording(output, format='mjpeg')
	try:
		address = ('', 9000)
		server = StreamingServer(address, StreamingHandler)
		server.serve_forever()
	finally:
		camera.stop_recording()
