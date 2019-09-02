import os
import imghdr

def r_list_dirs(root_dir, subdir='.', real_paths=None):
	rs = []
	bpath = os.path.join(root_dir, subdir)
	if real_paths is None:
		real_paths = []
	real_path = os.path.realpath(bpath)
	if real_path not in real_paths and os.path.isdir(real_path):
		real_paths.append(real_path)
#		print real_path
		rs.append(subdir)
		for d in os.listdir(real_path):
			rs += r_list_dirs(root_dir, os.path.join(subdir, d), real_paths)
	return rs

def list_images(root_dir, subdir='.'):
	rs = []
	for fname in os.listdir(os.path.join(root_dir, subdir)):
		fpath = os.path.join(root_dir, subdir, fname)
		if os.path.isfile(fpath) and imghdr.what(fpath):
			rs.append(fname)
	return rs

if __name__ == '__main__':

	import json

	print json.dumps(list_dirs('/usr'), indent=2)
