from __future__ import print_function
import re, random, time, sys, os, urlparse, json, zipfile, tempfile, urllib2, imp, datetime, traceback
from optparse import OptionParser
from cgi import parse_qs

def load_values(env):
	rs = {}
	return json.dumps(rs)

def apply_values(env):
	rs = None
	data = json.load(env['wsgi.input'])
	print('INPUT DATA', data)
	return load_values(env)
#	rs = {'current_values': data['config']}
#	return json.dumps(rs)

def router(env, start_response):
	try:
		path_info = env.get('PATH_INFO', '/')
		print('ENV(' + path_info + '):', env)
		rs = None
		print( 'Query(' + path_info + '): %s' % env['QUERY_STRING'])
		headers = []
		env['GET'] = {}
		content_type = 'text/html'
		if re.match(r'.*/apply_values', path_info, re.I):
			content_type = 'text/json'
			rs = apply_values(env)
		elif re.match(r'.*/load_values', path_info, re.I):
			content_type = 'text/json'
			rs = load_values(env)
		if rs is None:
			start_response('404 File Not Found')
		else:
			start_response('200 OK', [('Content-Type', content_type)] + headers)
	except Exception, e:
		print(traceback.format_exc())
		start_response('500 Internal Server Error', [('Content-Type', 'text/json')])
		rs = json.dumps({'error': str(e)})
	return rs
