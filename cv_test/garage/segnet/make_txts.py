#!/usr/bin/env python

import os, sys, glob, random

ANNDIR = 'data/annotations'
IMGDIR = 'data/images'

anns = dict([(os.path.basename(fname), fname) for fname in glob.glob(os.path.join(ANNDIR, '*.png'))])

print 'Expected:', len(anns.keys())

allpairs = []

for root, dirs, files in os.walk(IMGDIR, followlinks=True):
	for name in files:
		fname = os.path.join(root, name)
		bname = os.path.basename(fname)
		if bname in anns.keys():
			allpairs.append((fname, anns[bname]))

tcount = len(allpairs)

print 'Found:', tcount

# split to train, val, test
random.shuffle(allpairs)

n_train = int(tcount)
#n_train = int(tcount * 0.5)

#n_val = int((tcount - n_train) * 0.5)

#n_test = tcount - n_train - n_val

#for fname,num in (('train.txt', n_train), ('val.txt', n_val), ('test.txt', n_test)):
for fname,num in (('train.txt', n_train),):

	df = file(fname, 'w')
	for i in range(num):
		p = allpairs.pop()
		df.write('{} {}\n'.format(p[0], p[1]))
	df.close()
