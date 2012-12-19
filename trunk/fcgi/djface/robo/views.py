from django.shortcuts import render_to_response
from django.template.context import RequestContext
from django.http import HttpResponse
import redis, json, base64
import Image
from StringIO import StringIO

QCMD = 'cam_cmd'
STEP = 100

def home(request):
	return render_to_response('home.html', {}, context_instance=RequestContext(request))

def histogram(request):
	return render_to_response('histogram.html', {})

def move_left(request):
	redis.Redis().lpush(QCMD, json.dumps({'cmd': 'move_left', 'value': STEP}))
	return HttpResponse('OK')

def move_right(request):
	redis.Redis().lpush(QCMD, json.dumps({'cmd': 'move_right', 'value': STEP}))
	return HttpResponse('OK')

def move_up(request):
	redis.Redis().lpush(QCMD, json.dumps({'cmd': 'move_up', 'value': STEP}))
	return HttpResponse('OK')

def move_down(request):
	redis.Redis().lpush(QCMD, json.dumps({'cmd': 'move_down', 'value': STEP}))
	return HttpResponse('OK')

def make_image(request):
	redis.Redis().lpush(QCMD, json.dumps({'cmd': 'make_image'}))
	return HttpResponse('OK')

def get_image(request):
	img = StringIO(redis.Redis().get("cur_image"))
	img = Image.open(img)
	img = img.rotate(90)
#	img.save("/tmp/out.png")
	df = StringIO()
	img.save(df, "PNG")
	return HttpResponse( df.getvalue(), mimetype='image/png')
