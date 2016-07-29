#!/usr/bin/env python

import glob, random, os, shutil

fcats = glob.glob('dogs_and_cats/cat/*.jpg')
fdogs = glob.glob('dogs_and_cats/dog/*.jpg')


random.shuffle(fcats)
random.shuffle(fdogs)

shutil.rmtree('data')

for p in ('data/train/cats', 'data/train/dogs', 'data/validation/cats', 'data/validation/dogs'):
	if not os.path.exists(p):
		os.makedirs(p)

i = 1
for fcat in fcats[:1000]:
	os.symlink(os.path.join('../../..', fcat), 'data/train/cats/cat%03d.jpg' % i)
	i += 1

i = 1
for fdog in fdogs[:1000]:
	os.symlink(os.path.join('../../..', fdog), 'data/train/dogs/dog%03d.jpg' % i)
	i += 1

i = 1
for fcat in fcats[-400:]:
	os.symlink(os.path.join('../../..', fcat), 'data/validation/cats/cat%03d.jpg' % i)
	i += 1

i = 1
for fdog in fdogs[-400:]:
	os.symlink(os.path.join('../../..', fdog), 'data/validation/dogs/dog%03d.jpg' % i)
	i += 1
