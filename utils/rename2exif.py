#!/usr/bin/env python3


import os, sys, shutil, json, time, traceback, argparse
import piexif
from pathlib import Path
# from python3-exif
#import exifread
from PIL import Image, UnidentifiedImageError
# pip3 install exif
#from exif import Image as ExifImage
from PIL.ExifTags import TAGS, GPSTAGS
from geopy.geocoders import Nominatim
from geopy.exc import GeopyError

parser = argparse.ArgumentParser(description='Rename images based on EXIF DateTime and GPS location')
parser.add_argument('--root_dir', required=True, help='Source directory to process')
args = parser.parse_args()

ROOT_DIR = args.root_dir
#ROOT_DIR = 'recovered2sort'
#ROOT_DIR = 'motorola2sortMX'
#ROOT_DIR = os.path.expanduser('~/venieer/DCIM')
#ROOT_DIR = 'images_osaka_june-2025'
GOOD_DIR = ROOT_DIR + '_processed'
BAD_DIR = ROOT_DIR + '_bad'

COORD_CACHE_FNAME = 'coord_cache.json'

IMAGE_EXTS = {'.jpg', '.jpeg', '.png', '.webp', '.heic', '.heif', '.tif', '.tiff', '.bmp', '.gif', '.dng', '.raw', '.cr2', '.nef', '.arw'}

#print(json.dumps(list(sorted(TAGS.values())), indent=2))
#sys.exit()

def _convert_to_degress(value):
	return value[0] + (value[1] / 60.0) + (value[2] / 3600.0)

def get_lat_lon(gps_info):
	"""Returns the latitude and longitude, if available, from the provided exif_data (obtained through get_exif_data above)"""
	lat = None
	lon = None

	try:
		gps_latitude = gps_info["GPSLatitude"]
		gps_latitude_ref = gps_info['GPSLatitudeRef']
		gps_longitude = gps_info['GPSLongitude']
		gps_longitude_ref = gps_info['GPSLongitudeRef']

#	print('GENERAL', gps_latitude_ref, gps_longitude_ref)

		if gps_latitude and gps_latitude_ref and gps_longitude and gps_longitude_ref:
			lat = _convert_to_degress(gps_latitude)
			if gps_latitude_ref != "N":
				lat = 0 - lat

			lon = _convert_to_degress(gps_longitude)
			if gps_longitude_ref != "E":
				lon = 0 - lon
	except KeyError:
		print('Bad GPS Info')

	return lat, lon

def get_lat_lon_pie(gps_info):
	"""Returns the latitude and longitude, if available, from the provided exif_data (obtained through get_exif_data above)"""
	lat = None
	lon = None

	int2float = lambda vl: [v[0]/v[1] for v in vl]


	gps_latitude = int2float(gps_info[piexif.GPSIFD.GPSLatitude])
	gps_latitude_ref = gps_info[piexif.GPSIFD.GPSLatitudeRef].decode()
	gps_longitude = int2float(gps_info[piexif.GPSIFD.GPSLongitude])
	gps_longitude_ref = gps_info[piexif.GPSIFD.GPSLongitudeRef].decode()

#	print('PIE', gps_latitude_ref, gps_longitude_ref)

	if gps_latitude and gps_latitude_ref and gps_longitude and gps_longitude_ref:
		lat = _convert_to_degress(gps_latitude)
		if gps_latitude_ref != "N":
			lat = 0 - lat

		lon = _convert_to_degress(gps_longitude)
		if gps_longitude_ref != "E":
			lon = 0 - lon

	return lat, lon

geolocator = Nominatim(user_agent="renamer")

def mark_bad(fname, used_fnames):
	print('{} is not good jpg'.format(fname))
	if not os.path.exists(BAD_DIR):
		os.makedirs(BAD_DIR)
	bad_fname = os.path.join(BAD_DIR, os.path.basename(fname))
	ndx = used_fnames.get(bad_fname, 0)
	while os.path.lexists(bad_fname) and not Path(bad_fname).is_symlink():
		ndx += 1
		bad_fname = '{bn[0]}_{ndx}_{bn[1]}'.format(bn=os.path.splitext(os.path.join(BAD_DIR, os.path.basename(fname))), ndx=ndx)
	if Path(bad_fname).is_symlink():
		os.unlink(bad_fname)
	os.symlink(os.path.relpath(fname, start=os.path.dirname(bad_fname)), bad_fname)
	used_fnames[bad_fname] = ndx

coord_cache = json.load(open(COORD_CACHE_FNAME, 'r')) if os.path.exists(COORD_CACHE_FNAME) else {}

try:
	used_fnames = {}
	for (root,dirs,files) in os.walk(ROOT_DIR,topdown=True):
		for bname in files:
			if os.path.splitext(bname)[1].lower() not in IMAGE_EXTS:
				continue
			fname = os.path.join(root, bname)
			try:
				img = Image.open(fname)
			except UnidentifiedImageError as e:
				print(str(e), file=sys.stderr)
				mark_bad(fname, used_fnames)
				continue
			except Exception as e:
				print(str(e), file=sys.stderr)
				mark_bad(fname, used_fnames)
				continue
			failed = False
#			print('TAGS', tags)
			try:
				tags = img.getexif()
				exif_dict = piexif.load(img.info['exif']) if tags else {}
				loc_ext = None
				loc_ext_s = []
				subd_ext = ''
				htags = {}
				for k,tname in TAGS.items():
					if k in tags:
						if tname == 'GPSInfo':
							gps_info = tags.get_ifd(k)
							htags[tname] = {}
							for gk,gtname in GPSTAGS.items():
								if gk in gps_info:
									htags[tname][gtname] = gps_info[gk].decode().replace('\0', '') if isinstance(gps_info[gk], bytes) else gps_info[gk]
							lat,lon = get_lat_lon(htags[tname])
#							print('LATLON', lat, lon)
#							print('DIR', get_lat_lon_pie(exif_dict['GPS']))
							if lat and lon:
								coord = f"{lat}, {lon}"
								if coord in coord_cache:
									loc_ext = coord_cache[coord]
								else:
									try:
										time.sleep(1.1)
										loc_ext = dict(geolocator.reverse(coord, exactly_one=True, language='en').raw)
										coord_cache[coord] = loc_ext
									except GeopyError as e:
										print('Geocoding {} failed: {}: {}'.format(coord, type(e).__name__, str(e)), file=sys.stderr)
										loc_ext = None
								if loc_ext is not None:
									exif_dict['GPS'][piexif.GPSIFD.GPSAreaInformation] = loc_ext['display_name'].encode()
									exif_dict['0th'][piexif.ImageIFD.ImageDescription] = loc_ext['display_name'].encode()
#									print(json.dumps(loc_ext, indent=2, default=str))
									loc_ext = loc_ext['address']
									htags[tname]['named_location'] = loc_ext
									for kl in (('locality', 'suburb', 'city', 'municipality', 'village', 'town', 'hamlet'), ('province', ), ('ISO3166-2-lvl4',)):
										for k in kl:
											if k in loc_ext:
												if not subd_ext:
													subd_ext = loc_ext[k].replace(' ', '_')
												loc_ext_s.append(loc_ext[k].replace(' ', '_'))
												break
						else:
							htags[tname] = tags[k].replace('\0', '') if isinstance(tags[k], str) else tags[k]
				if set(('DateTime',)).difference(htags.keys()):
					print('No DateTime in {}, copying as _nometa'.format(fname), file=sys.stderr)
					bn = os.path.splitext(os.path.basename(fname))
					dfname = os.path.join(GOOD_DIR, 'pics', 'nometa', '{}_nometa{}'.format(bn[0], bn[1]))
					if dfname in used_fnames:
						b2 = os.path.splitext(dfname)
						ndx = used_fnames[dfname]
						while True:
							ndx += 1
							cand = '{bn[0]}_{ndx}_{bn[1]}'.format(bn=b2, ndx=ndx)
							if cand not in used_fnames:
								dfname = cand
								break
						used_fnames[b2[0] + b2[1]] = ndx
					if not os.path.exists(os.path.dirname(dfname)):
						os.makedirs(os.path.dirname(dfname))
					print('Copying {} as {} ...'.format(fname, dfname))
					shutil.copy2(fname, dfname)
					used_fnames[dfname] = used_fnames.get(dfname, 0) + 1
				else:
#					print('TAGS', exif_dict.keys(), piexif.TAGS.keys(), piexif.TAGS['Image'][piexif.ImageIFD.ImageDescription])
					data_d = {'named_location': loc_ext}
					for ifd in exif_dict:
						if ifd in piexif.TAGS:
							data_d[ifd] = {piexif.TAGS[ifd][tag]["name"]:(exif_dict[ifd][tag].decode(errors='replace').replace('\0', '') if isinstance(exif_dict[ifd][tag], bytes) else exif_dict[ifd][tag]) for tag in exif_dict[ifd]}
					t = {k:str(htags[k]) for k in ('DateTime', )}
					d_s,t_s = t['DateTime'].split(' ')
					d_s = d_s.replace(':', '-')
					t_s = t_s.replace(':', '-')
					t['DateTime'] = d_s + ' ' + t_s
					bname = '-'.join(t.values()).replace(' ', '_')
					if loc_ext:
						bname += '_' + '-'.join(loc_ext_s)
					bname += '.jpg'
					subdname = d_s
					if subd_ext:
						subdname += '_' + subd_ext
					dfname = os.path.join(GOOD_DIR, 'pics', subdname, bname)
					if dfname in used_fnames:
						print('{} is already used'.format(dfname))
						bn = os.path.splitext(dfname)
						ndx = used_fnames[dfname]
						while True:
							ndx += 1
							cand = '{bn[0]}_{ndx}_{bn[1]}'.format(bn=bn, ndx=ndx)
							if cand not in used_fnames:
								dfname = cand
								break
						used_fnames[bn[0] + bn[1]] = ndx
					dfname_exif = os.path.join(GOOD_DIR, 'exif', subdname, os.path.basename(dfname) + '.exif')
					for dname in (dfname, dfname_exif):
						dname = os.path.dirname(dname)
						if not os.path.exists(dname):
							os.makedirs(dname)
#					open(dfname_exif, 'wt').write(json.dumps(dict(htags.items()), indent=2, default=str, ensure_ascii=False))
					open(dfname_exif, 'wt').write(json.dumps(data_d, indent=2, ensure_ascii=False))
					if Path(dfname).is_symlink():
						os.unlink(dfname)
					print('Saving {} as {} ...'.format(fname, dfname))
					try:
						if not exif_dict.get('thumbnail'):
							exif_dict.pop('thumbnail', None)
						try:
							exif_bytes = piexif.dump(exif_dict)
						except piexif.InvalidImageDataError:
							print('Dropping thumbnail for {}: {}'.format(fname, exif_dict.get('thumbnail', b'')[:20]), file=sys.stderr)
							exif_dict.pop('thumbnail', None)
							exif_bytes = piexif.dump(exif_dict)
						img.save(dfname, exif=exif_bytes)
#						img.save('{bn[0]}_exifed_{bn[1]}'.format(bn=os.path.splitext(dfname)), exif=piexif.dump(exif_dict))
						used_fnames[dfname] = used_fnames.get(dfname, 0) + 1
#						os.symlink(os.path.relpath(fname, start=os.path.dirname(dfname)), dfname)
					except Exception as e:
						print('Save {} made from {} error: {}: {}'.format(dfname, fname, type(e).__name__, str(e)), file=sys.stderr)
						traceback.print_exc(file=sys.stderr)
						for p in (dfname, dfname_exif):
							if os.path.lexists(p):
								os.unlink(p)
						failed = True
			except Exception as e:
				failed = True
				print('Exif processing {} error: {}: {}'.format(fname, type(e).__name__, str(e)), file=sys.stderr)
				traceback.print_exc(file=sys.stderr)
			if failed:
				mark_bad(fname, used_fnames)
finally:
	json.dump(coord_cache, open(COORD_CACHE_FNAME, 'w'), indent=2)
