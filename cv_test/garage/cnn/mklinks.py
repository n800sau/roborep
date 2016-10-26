#!/usr/bin/env python

import sys, glob, random, os, shutil

OUTPATH = 'data'
if os.path.exists(OUTPATH):
	shutil.rmtree(OUTPATH)

label_counter = {}
label_flist = {}

#BPATHLIST = ['~/work/roborep/cv_test/icrop/data_inside_empty/labels', '~/work/roborep/cv_test/icrop/data/labels']
#BPATHLIST = ['~/work/roborep/cv_test/garage/traindata',]
BPATHLIST = ['~/work/roborep/cv_test/garage/cnn/input']

for bpath in BPATHLIST:

	bpath = os.path.expanduser(bpath)

	labels = os.listdir(bpath)

	for label in labels:
		if label not in label_counter:
			label_counter[label] = 1
		if label not in label_flist:
			label_flist[label] = []
		label_flist[label] += glob.glob(os.path.join(bpath, label, '*.jpg'))

for label,flist in label_flist.items():
	random.shuffle(flist)
	totnum = len(flist)
	spos = 0
	for act,num in (('train', 75), ('validation', 25)):
		dpath = os.path.join(OUTPATH, act, label)
		if not os.path.exists(dpath):
			os.makedirs(dpath)
		for fpath in flist[spos:spos+(totnum * num // 100)]:
			os.symlink(fpath, os.path.join(dpath, label + ('%03d.jpg' % label_counter[label])))
			label_counter[label] += 1
			spos += 1
