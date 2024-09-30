#!/usr/bin/env python3

import os, sys

# pip3 install ffmpeg-python
import ffmpeg

ROOT_DIR = 'recover_mov'
BAD_DIR = ROOT_DIR + '_bad'
RECOVERED_DIR_PREFIX = ROOT_DIR + '_'

def fileHasVideoStream(file_path):
	probe = None
	try:
		probe = ffmpeg.probe(file_path)
	except ffmpeg.Error as e:
		print(os.path.basename(file_path), '\n'.join(e.stderr.decode().strip().split('\n')[-1:]), file=sys.stderr)

	if probe:
		print([stream['codec_type'] for stream in probe['streams'] if stream['codec_type'] not in ('audio', 'video')], file=sys.stderr)
#		print([stream['codec_type'] for stream in probe['streams']])
		return [stream for stream in probe['streams'] if stream['codec_type'] == 'video']
	else:
		print('No video stream found', file=sys.stderr)
		return []

if __name__ == '__main__':

	for (root,dirs,files) in os.walk(ROOT_DIR,topdown=True):
		for bname in files:
			fname = os.path.join(root, bname)
			if fileHasVideoStream(fname):
				odname = RECOVERED_DIR_PREFIX
				if not os.path.exists(odname):
					os.makedirs(odname)
				os.symlink(os.path.relpath(fname, start=odname), os.path.join(odname, os.path.basename(fname)))
			else:
				odname = BAD_DIR
				if not os.path.exists(odname):
					os.makedirs(odname)
				print(os.path.basename(fname), 'is BAD')
				os.symlink(os.path.relpath(fname, start=odname), os.path.join(odname, os.path.basename(fname)))
