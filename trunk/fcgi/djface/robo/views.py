from django.shortcuts import render_to_response
from django.template.context import RequestContext
from django.http import HttpResponse
import redis, json

QCMD = 'cmd'
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
