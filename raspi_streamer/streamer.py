import socket
import time
import picamera

OSIZE = (1280, 720)

with picamera.PiCamera() as camera:
	camera.resolution = OSIZE
	camera.framerate = 24

	server_socket = socket.socket()
	server_socket.bind(('0.0.0.0', 9000))
	server_socket.listen(0)

	# Accept a single connection and make a file-like object out of it
	connection = server_socket.accept()[0].makefile('wb')
	try:
		i = 1
		while True:
			camera.annotate_background = picamera.Color('black')
			camera.annotate_text = 'Hello world %d' % i
			camera.start_recording(connection, format='h264')
			camera.wait_recording(60)
			camera.stop_recording()
			i += 1
			print('continue...', i)
	finally:
		connection.close()
		server_socket.close()
