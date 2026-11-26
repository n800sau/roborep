#!/usr/bin/env python3

import os, sys, datetime, re, json, time, traceback, argparse

#pip3 install lat-lon-parser
from lat_lon_parser import parse
# pip3 install ffmpeg-python
import ffmpeg

from geopy.geocoders import Nominatim
from geopy.exc import GeopyError

#apt install python3-magic
import magic

COORD_CACHE_FNAME = 'coord_cache.json'

def fileHasVideoStream(file_path):
	print(file_path)
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
				loc = rs['location'].split('/', 1)[0]
#				print(loc)
				loc = re.match(r"^([-+][\d.]+)([-+][\d.]+)", loc)
				if loc:
#					print('LOC', loc)
					coord = '{}, {}'.format(*[v for v in loc.groups()])
					print('COORDS	', coord)
					if coord in coord_cache:
						loc_ext = coord_cache[coord]
					else:
						try:
							time.sleep(1.1)
							loc_ext = geolocator.reverse(coord, exactly_one=True, language='en')
							if loc_ext:
								coord_cache[coord] = loc_ext = dict(loc_ext.raw)
						except GeopyError as e:
							print('Geocoding {} failed: {}: {}'.format(coord, type(e).__name__, str(e)), file=sys.stderr)
							loc_ext = None
					if loc_ext:
						print('LOC_EXT', loc_ext)
						loc_ext = loc_ext['address']
						loc_ext_s = []
						for kl in (('locality', 'suburb', 'city', 'municipality', 'village', 'town', 'hamlet'), ('province', ), ('ISO3166-2-lvl4',)):
							for k in kl:
								if k in loc_ext:
									loc_ext_s.append(loc_ext[k].replace(' ', '_'))
									break
					else:
						loc_ext_s = ''
					print('LOC', loc_ext_s)
					rs['location'] = '-'.join(loc_ext_s)
		else:
			rs = None
	else:
		print('{} has no video stream'.format(os.path.basename(file_path)), file=sys.stderr)
		rs = None
	return rs

if __name__ == '__main__':

	parser = argparse.ArgumentParser(description='Rename videos based on metadata creation_time and location')
	parser.add_argument('--root_dir', required=True, help='Source directory to process')
	args = parser.parse_args()

	global ROOT_DIR, BAD_DIR, RECOVERED_DIR
	ROOT_DIR = args.root_dir
	BAD_DIR = ROOT_DIR + '_bad'
	RECOVERED_DIR = ROOT_DIR + '_good'

	geolocator = Nominatim(user_agent="renamer")

	coord_cache = json.load(open(COORD_CACHE_FNAME, 'r')) if os.path.exists(COORD_CACHE_FNAME) else {}

	try:
		for (root,dirs,files) in os.walk(ROOT_DIR,topdown=True):
			for bname in files:
				fname = os.path.join(root, bname)
				try:
					ftype = magic.from_file(fname, mime=True).split('/')[0]
					if ftype in ('video', 'audio'):
						vstream = fileHasVideoStream(fname)
						if vstream:
							odname = RECOVERED_DIR
							if not os.path.exists(odname):
								os.makedirs(odname)
							ofname = os.path.splitext(os.path.basename(fname))
							print('LOCATION', vstream['location'])
							ofname = '{}{}_{}_{}{}'.format(
								vstream['creation_time'].astimezone().strftime('%Y-%m-%d_%H-%M-%S') if vstream['creation_time'] else 'no_time',
								'_{}'.format(vstream['location']) if vstream['location'] else '',
								vstream['codec_name'],
								ofname[0],
								ofname[1])
							os.symlink(os.path.relpath(fname, start=odname), os.path.join(odname, ofname))
						else:
							odname = BAD_DIR
							if not os.path.exists(odname):
								os.makedirs(odname)
							print(os.path.basename(fname), 'is BAD')
							os.symlink(os.path.relpath(fname, start=odname), os.path.join(odname, os.path.basename(fname)))
				except Exception as e:
					print('Processing {} error: {}: {}'.format(fname, type(e).__name__, str(e)), file=sys.stderr)
					traceback.print_exc(file=sys.stderr)
					odname = BAD_DIR
					if not os.path.exists(odname):
						os.makedirs(odname)
					bad_fname = os.path.join(odname, bname)
					if os.path.lexists(bad_fname):
						bn = os.path.splitext(bad_fname)
						ndx = 1
						while os.path.lexists('{bn[0]}_{ndx}_{bn[1]}'.format(bn=bn, ndx=ndx)):
							ndx += 1
						bad_fname = '{bn[0]}_{ndx}_{bn[1]}'.format(bn=bn, ndx=ndx)
					os.symlink(os.path.relpath(fname, start=odname), bad_fname)
	finally:
		json.dump(coord_cache, open(COORD_CACHE_FNAME, 'w'), indent=2)
