#!/usr/bin/env python3

import os, sys, datetime, re, json

#pip3 install lat-lon-parser
from lat_lon_parser import parse
# pip3 install ffmpeg-python
import ffmpeg

from geopy.geocoders import Nominatim

#apt install python3-magic
import magic

ROOT_DIR = 'recover_mov'
#ROOT_DIR = 'images_tokyo-may-june-2024'
BAD_DIR = ROOT_DIR + '_bad'
RECOVERED_DIR = ROOT_DIR + '_good'

COORD_CACHE_FNAME = 'coord_cache.json'

def fileHasVideoStream(file_path):
	global coord_cache
	probe = None
	try:
		probe = ffmpeg.probe(file_path)
	except ffmpeg.Error as e:
		print(os.path.basename(file_path), '\n'.join(e.stderr.decode().strip().split('\n')[-1:]), file=sys.stderr)

	if probe:
#		print(probe['format']['tags'], file=sys.stderr)
#		print([stream for stream in probe['streams'] if stream['codec_type'] in ('video', )], file=sys.stderr)
#		print([stream['codec_type'] for stream in probe['streams']])
		rs = [stream for stream in probe['streams'] if stream['codec_type'] == 'video']
		if rs:
			rs = {k:v for k,v in rs[0].items() if k in ('codec_name', 'width', 'height', 'duration', 'bit_rate', 'nb_frames')}
			for k in ('major_brand', 'creation_time', 'location'):
				rs[k] = probe['format'].get('tags', {}).get(k, None)
#			2022-12-04T09:21:59.000000Z
			if rs['creation_time']:
				rs['creation_time'] = datetime.datetime.strptime(rs['creation_time'], '%Y-%m-%dT%H:%M:%S.%f%z')
			if rs['location']:
				loc = rs['location'].split('/')[0]
#				print(loc)
				loc = re.match(r"^([-+][\d.]+)([-+][\d.]+)", loc)
				if loc:
#					print('LOC', loc)
					coord = '{}, {}'.format(*[v for v in loc.groups()])
#					print('COORDS	', coord)
					if coord in coord_cache:
						loc_ext = coord_cache[coord]
					else:
						loc_ext = dict(geolocator.reverse(coord, exactly_one=True, language='en').raw)
						coord_cache[coord] = loc_ext
					loc_ext = loc_ext['address']
					loc_ext_s = []
					for kl in (('locality', 'suburb', 'city', 'municipality', 'village', 'town', 'hamlet'), ('province', ), ('ISO3166-2-lvl4',)):
						for k in kl:
							if k in loc_ext:
								loc_ext_s.append(loc_ext[k].replace(' ', '_'))
								break
					print('LOC', loc_ext_s)
					rs['location'] = '-'.join(loc_ext_s)
		else:
			rs = None
	else:
		print('{} has no video stream'.format(os.path.basename(file_path)), file=sys.stderr)
		rs = None
	return rs

if __name__ == '__main__':

	geolocator = Nominatim(user_agent="renamer")

	coord_cache = json.load(open(COORD_CACHE_FNAME, 'r')) if os.path.exists(COORD_CACHE_FNAME) else {}

	try:
		for (root,dirs,files) in os.walk(ROOT_DIR,topdown=True):
			for bname in files:
				fname = os.path.join(root, bname)
				ftype = magic.from_file(fname, mime=True).split('/')[0]
				if ftype == 'video':
					vstream = fileHasVideoStream(fname)
#					continue
					if vstream:
						odname = RECOVERED_DIR
						if not os.path.exists(odname):
							os.makedirs(odname)
						ofname = os.path.splitext(os.path.basename(fname))
						ofname = '{}{}_{}_{}{}'.format(
							vstream['creation_time'].astimezone().strftime('%Y-%m-%d_%H-%M-%S') if vstream['creation_time'] else 'no_time',
							'_{}'.format(vstream['location']) if vstream['location'] else '',
							vstream['codec_name'],
							ofname[0],
							ofname[1])
						os.symlink(os.path.relpath(fname, start=odname), os.path.join(odname, ofname))
#	#					break
					else:
						odname = BAD_DIR
						if not os.path.exists(odname):
							os.makedirs(odname)
						print(os.path.basename(fname), 'is BAD')
						os.symlink(os.path.relpath(fname, start=odname), os.path.join(odname, os.path.basename(fname)))
	finally:
		json.dump(coord_cache, open(COORD_CACHE_FNAME, 'w'), indent=2)
