#!/usr/bin/env python3

import sys, json

data = json.load(sys.stdin)
idlist = data['IdList']
with open('list.txt', 'w') as f:
	for item in idlist:
		f.write('%s\n' % item)
