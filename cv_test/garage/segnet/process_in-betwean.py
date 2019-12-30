import os, sys, imghdr, re, shutil, datetime

SRC = 'output'
TAGGED = 'tagged'
OUTPUT = 'in-betwean'

r = re.compile('^[0-9A-F]+\(.*\)_\d_(\d+)_\d+\.jpg')
dt_min = datetime.timedelta(seconds=60)

def fname2ts(fname):
	rs = None
	bname = os.path.basename(fname)
	m = r.match(bname)
	if m:
		rs = datetime.datetime.strptime(m.groups()[0], '%Y%m%d%H%M%S')
	else:
		print 'No match for %s' % bname
	return rs

all_files = []
for root, dirs, files in os.walk(TAGGED, followlinks=True):
	for fname in files:
		all_files.append(os.path.join(root, fname))

all_files.sort(key=fname2ts)

fcount = len(all_files)
print 'File count', fcount

prev_tag = None
in_betwean = set()
for i in range(fcount):
	fname = all_files[i]
	tag = os.path.basename(os.path.dirname(fname))
	if prev_tag is None:
		prev_tag = tag
	if prev_tag != tag:
		# tag change
		# get change timestamp
		#00D6FB009223(n800sau)_1_20160516142921_30928.jpg
		tag_ts = fname2ts(fname)
		if not tag_ts is None:
			j = i
			# if prev tag is car then following 1 min goes to in-betwean
			if prev_tag == 'car':
				j += 1
				while j < fcount:
					tfname = all_files[j]
					ts = fname2ts(tfname)
					if ts - tag_ts > dt_min:
						break
					in_betwean.add(tfname)
					j += 1
			# if prev tag is nocar then previous 1 min goes to in-betwean
			elif prev_tag == 'nocar':
				j -= 1
				while j >= 0:
					tfname = all_files[j]
					ts = fname2ts(tfname)
					if tag_ts - ts > dt_min:
						break
					in_betwean.add(tfname)
					j -= 1
			prev_tag = tag

if not os.path.exists(OUTPUT):
	os.makedirs(OUTPUT)

for fname in list(in_betwean):
	dfname = os.path.join(OUTPUT, os.path.basename(fname))
	shutil.move(fname, dfname)
