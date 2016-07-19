import os

def list_dirs(root_dir, subdir='.', real_paths=None):
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
			rs += list_dirs(root_dir, os.path.join(subdir, d), real_paths)
	return rs

if __name__ == '__main__':

	import json

	print json.dumps(list_dirs('/usr'), indent=2)
