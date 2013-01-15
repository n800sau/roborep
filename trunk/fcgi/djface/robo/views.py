from django.shortcuts import render_to_response
from django.template.context import RequestContext
from django.http import HttpResponse
import redis, json, base64, datetime, time
import Image
from StringIO import StringIO

QCMD = 'cam_cmd'
STEP = 100

def home(request):
	return render_to_response('home.html', {}, context_instance=RequestContext(request))

def histogram(request):
	return render_to_response('histogram.html', {}, context_instance=RequestContext(request))

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

def set_histogram_mode(request):
	redis.Redis().lpush(QCMD, json.dumps({'cmd': 'set_histogram_mode', 'bins': request.GET['bins'], 'chan': request.GET['chan']}))
	return HttpResponse('OK')

def get_histogram(request):
	cur_hist = redis.Redis().get("cur_histogram")
	return HttpResponse(cur_hist, mimetype='text/json')

def get_servos(request):
	return HttpResponse(json.dumps({'cur_pan': redis.Redis().get("cur_pan"), 'cur_tilt': redis.Redis().get("cur_tilt")}), mimetype='text/json')

def monitor(request):
	return render_to_response('monitor.html', {}, context_instance=RequestContext(request))

def cam_values(request):
	cur_time = time.time()
	values = {'timestamp': time.strftime('%d.%m.%Y %H:%M:%S', time.localtime(cur_time))}
	r = redis.Redis()
	values['cur_pan'] = r.get('cur_pan')
	values['cur_tilt'] = r.get('cur_tilt')
	values['cur_heading_degrees'] = r.get('heading.degrees')
	values['acc_x'] = r.get('adxl345.x')
	values['acc_y'] = r.get('adxl345.y')
	values['acc_z'] = r.get('adxl345.z')
	try:
		act_time = float(r.get('timestamp'))
		atimestamp = time.strftime('%d.%m.%Y %H:%M:%S', time.localtime(act_time))
		time_diff = int(cur_time - act_time)
		time_diff = '%2.2dh %2.2dm %2.2ds' % (time_diff // 3600, time_diff // 60 % 60, time_diff % 60)
	except:
		atimestamp = 'unknown'
		time_diff =  'unknown'
	values['time_diff'] = time_diff
	values['last_activity_timestamp'] = atimestamp
	return render_to_response('cam_values.html', values, context_instance=RequestContext(request))

