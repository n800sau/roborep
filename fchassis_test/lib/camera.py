import os

def update_img(camera, fname=None):
	fname = os.path.join(os.path.expanduser('~/public_html'), fname or 'picam_0.jpg')
	camera.resolution = (320, 240)
	camera.exposure_mode = 'auto'
	camera.brightness = 70
	camera.contrast = 70
	camera.capture(fname, use_video_port=False)
