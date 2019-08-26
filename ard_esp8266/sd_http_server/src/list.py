#!/usr/bin/env python

from __future__ import absolute_import, division, print_function
import os, sys, json
import requests


HOSTNAME = 'esp8266sd.local'
LS_URL = 'http://' + HOSTNAME + '/ls?dir='
RM_URL= 'http://' + HOSTNAME + '/rmdir?dir='

def sort_key(v):
	return (0, v['dir']['name']) if 'dir' in v else (1, v['file']['name'])

def list_dir(dname, alist):
#	print('dname', dname)
	r = requests.get(LS_URL+dname)
#	print(r.status_code)
	if r.status_code == 200:
		data = r.json()

		for d in data:
			if d['type'] == 'dir':
				dlist = []
				list_dir(dname + d['name'], dlist)
				dlist.sort(key=sort_key)
				alist.append({'dir': d, 'children': dlist})
			else:
				d['dname'] = dname
				alist.append({'file': d})
	else:
		print('failed', dname)

def print_dir(dlist, indent):
	for d in dlist:
		if 'dir' in d:
			print('  ' * indent, 'dir', d['dir']['name'])
			print_dir(d['children'], indent+1)
		else:
			print('  ' * indent, 'file', d['file']['name'], d['file']['size'])


alist = []
list_dir('/', alist)

alist.sort(key=sort_key)
print_dir(alist, 0)
