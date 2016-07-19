import os

def list_dirs(root_dir, subdir='.', real_paths=None):
	rs = []
	bpath = os.path.join(root_dir, subdir)
	if real_paths is None:
		real_paths = []
	real_path = os.path.realpath(bpath)
	if real_path not in real_paths:
		real_paths.append(real_path)
		for d in os.listdir(real_path):
			if os.path.isdir(os.path.join(real_path, d)):
				rs += [os.path.join(subdir, dr) for dr in list_dirs(root_dir, os.path.join(subdir, d), real_paths)]
	return rs

if __name__ == '__main__':

	import json

	print json.dumps(list_dirs('/usr'), indent=2)
