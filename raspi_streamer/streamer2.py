import io
import socket
import struct
import time
import json
import threading
import picamera
import redis

REDIS_KEY = 'raspicam_settings'

settings_map = {
	'one': 'brightness',
	'two': 'contrast',
}


OSIZE = (640, 480)

server_socket = socket.socket()
server_socket.bind(('0.0.0.0', 9001))
server_socket.listen(0)

# Accept a single connection and make a file-like object out of it
connection = server_socket.accept()[0].makefile('wb')

try:
	connection_lock = threading.Lock()
	pool_lock = threading.Lock()
	pool = []

	class ImageStreamer(threading.Thread):
		def __init__(self):
			super(ImageStreamer, self).__init__()
			self.stream = io.BytesIO()
			self.event = threading.Event()
			self.terminated = False
			self.start()

		def run(self):
			# This method runs in a background thread
			while not self.terminated:
				# Wait for the image to be written to the stream
				if self.event.wait(1):
					try:
						with connection_lock:
							connection.write(struct.pack('<L', self.stream.tell()))
							connection.flush()
							self.stream.seek(0)
							connection.write(self.stream.read())
					finally:
						self.stream.seek(0)
						self.stream.truncate()
						self.event.clear()
						with pool_lock:
							pool.append(self)

	count = 0
	start = time.time()
	finish = time.time()

	def streams(camera):
		global count, finish, settings_map
		while finish - start < 30:
			r = redis.Redis()
			rval = r.get(REDIS_KEY)
			settings = json.loads(rval) if rval else None
			r.delete(REDIS_KEY)
			if settings:
				for k,v in settings.items():
					k = settings_map.get(k, None)
					if k:
						setattr(camera, k, v)
						print('set %s to %s' % (k, v))
			with pool_lock:
				if pool:
					streamer = pool.pop()
				else:
					streamer = None
			if streamer:
				yield streamer.stream
				streamer.event.set()
				count += 1
			else:
				# When the pool is starved, wait a while for it to refill
				time.sleep(0.1)
			finish = time.time()

	with picamera.PiCamera() as camera:
		pool = [ImageStreamer() for i in range(4)]
		camera.resolution = OSIZE
		camera.framerate = 30
		time.sleep(2)
		start = time.time()
		camera.capture_sequence(streams(camera), 'jpeg', use_video_port=True)

	# Shut down the streamers in an orderly fashion
	while pool:
		streamer = pool.pop()
		streamer.terminated = True
		streamer.join()

	# Write the terminating 0-length to the connection to let the server
	# know we're done
	with connection_lock:
		connection.write(struct.pack('<L', 0))

finally:
	connection.close()
	server_socket.close()

print('Sent %d images in %d seconds at %.2ffps' % (
	count, finish-start, count / (finish-start)))

