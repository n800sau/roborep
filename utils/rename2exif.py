#!/usr/bin/env python3


import os, sys, shutil, json
import piexif
from pathlib import Path
# from python3-exif
#import exifread
from PIL import Image, UnidentifiedImageError
# pip3 install exif
#from exif import Image as ExifImage
from PIL.ExifTags import TAGS, GPSTAGS
from geopy.geocoders import Nominatim

ROOT_DIR = 'recover1'
GOOD_DIR = ROOT_DIR + '_processed'
BAD_DIR = ROOT_DIR + '_bad'

#print(json.dumps(list(sorted(TAGS.values())), indent=2))
#sys.exit()

def _convert_to_degress(value):
	return value[0] + (value[1] / 60.0) + (value[2] / 3600.0)

def get_lat_lon(gps_info):
	"""Returns the latitude and longitude, if available, from the provided exif_data (obtained through get_exif_data above)"""
	lat = None
	lon = None

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

used_fnames = {}
for (root,dirs,files) in os.walk(ROOT_DIR,topdown=True):
	for bname in files:
		fname = os.path.join(root, bname)
		try:
			img = Image.open(fname)
		except UnidentifiedImageError as e:
			print(str(e), file=sys.stderr)
			continue
		except Exception as e:
			print(str(e), file=sys.stderr)
			continue
		tags = img.getexif()
#		print('TAGS', tags)
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
#					print('LATLON', lat, lon)
#					print('DIR', get_lat_lon_pie(exif_dict['GPS']))
					coord = f"{lat}, {lon}"
					loc_ext = geolocator.reverse(coord, exactly_one=True, language='en').raw
					exif_dict['GPS'][piexif.GPSIFD.GPSAreaInformation] = loc_ext['display_name'].encode()
					exif_dict['0th'][piexif.ImageIFD.ImageDescription] = loc_ext['display_name'].encode()
#					print(json.dumps(loc_ext, indent=2, default=str))
					loc_ext = loc_ext['address']
					htags[tname]['named_location'] = loc_ext
					for kl in (('village', 'town', 'hamlet', 'city', 'suburb'), ('province', ), ('ISO3166-2-lvl4',)):
						for k in kl:
							if k in loc_ext:
								if not subd_ext:
									subd_ext = loc_ext[k].replace(' ', '_')
								loc_ext_s.append(loc_ext[k].replace(' ', '_'))
								break
				else:
					htags[tname] = tags[k].replace('\0', '') if isinstance(tags[k], str) else tags[k]
		failed = False
		if not set(('DateTime',)).difference(htags.keys()):
#			print('TAGS', exif_dict.keys(), piexif.TAGS.keys(), piexif.TAGS['Image'][piexif.ImageIFD.ImageDescription])
			data_d = {'named_location': loc_ext}
			for ifd in exif_dict:
				if ifd in piexif.TAGS:
					data_d[ifd] = {piexif.TAGS[ifd][tag]["name"]:(exif_dict[ifd][tag].decode().replace('\0', '') if isinstance(exif_dict[ifd][tag], bytes) else exif_dict[ifd][tag]) for tag in exif_dict[ifd]}
			t = {k:str(htags[k]) for k in ('DateTime', )}
			d_s,t_s = t['DateTime'].split(' ')
			d_s = d_s.replace(':', '-')
			t['DateTime'] = d_s + ' ' + t_s
			bname = '-'.join(t.values()).replace(' ', '_')
			if loc_ext:
				bname += '_' + '-'.join(loc_ext_s)
			bname += '.jpg'
			subdname = d_s
			if subd_ext:
				subdname += '_' + subd_ext
			dfname = os.path.join(GOOD_DIR, subdname, bname)
			if dfname in used_fnames:
				print('{} is already used'.format(dfname))
				dfname = '{bn[0]}_{ndx}_{bn[1]}'.format(bn=os.path.splitext(dfname), ndx=used_fnames[dfname]+1)
			dfname_exif = os.path.join(GOOD_DIR, 'exif', bname + '.exif')
			for dname in (dfname, dfname_exif):
				dname = os.path.dirname(dname)
				if not os.path.exists(dname):
					os.makedirs(dname)
#			open(dfname_exif, 'wt').write(json.dumps(dict(htags.items()), indent=2, default=str, ensure_ascii=False))
			open(dfname_exif, 'wt').write(json.dumps(data_d, indent=2, ensure_ascii=False))
			if Path(dfname).is_symlink():
				os.unlink(dfname)
			print('Saving {} ...'.format(dfname))
			try:
				img.save('{bn[0]}_exifed_{bn[1]}'.format(bn=os.path.splitext(dfname)), exif=piexif.dump(exif_dict))
				used_fnames[dfname] = used_fnames.get(dfname, 0) + 1
				os.symlink(os.path.relpath(fname, start=os.path.dirname(dfname)), dfname)
			except Exception as e:
				print(str(e), file=sys.stderr)
				failed = True
		else:
			failed = True
		if failed:
			print('{} is not good jpg'.format(fname))
			if not os.path.exists(BAD_DIR):
				os.makedirs(BAD_DIR)
			bad_fname = os.path.join(BAD_DIR, os.path.basename(fname))
			if bad_fname in used_fnames:
				print('{} is already used'.format(bad_fname))
				bad_fname = '{bn[0]}_{ndx}_{bn[1]}'.format(bn=os.path.splitext(bad_fname), ndx=used_fnames[bad_fname]+1)
			if Path(bad_fname).is_symlink():
				os.unlink(bad_fname)
			os.symlink(os.path.relpath(fname, start=os.path.dirname(bad_fname)), bad_fname)
			used_fnames[bad_fname] = used_fnames.get(bad_fname, 0) + 1
