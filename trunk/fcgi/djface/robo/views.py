from django.shortcuts import render_to_response
from django.template.context import RequestContext
from django.http import HttpResponse

def home(request):
	return render_to_response('home.html', {}, context_instance=RequestContext(request))

def histogram(request):
	return render_to_response('histogram.html', {})

def move_left(request):
	print 'move_left'
	return HttpResponse('OK')

def move_right(request):
	print 'move_right'
	return HttpResponse('OK')

def move_up(request):
	print 'move_up'
	return HttpResponse('OK')

def move_down(request):
	print 'move_down'
	return HttpResponse('OK')
