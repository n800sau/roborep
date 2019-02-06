import io
import socket
import struct
import time
import threading
import picamera

OSIZE = (640, 480)

server_socket = socket.socket()
server_socket.bind(('0.0.0.0', 9000))
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

	br_inc = 1

	def streams(camera):
		global count, finish, br_inc
		while finish - start < 30:
			if camera.brightness >= 100:
				br_inc = -1
			if camera.brightness <= 0:
				br_inc = 1
			camera.brightness += br_inc
			print('brightness', camera.brightness)
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

