#!/usr/bin/env python2

import os
from Bio.PDB import *

fname = 'list.txt'

pdbl = PDBList()
#pdbl.retrieve_pdb_file('1FAT')

if os.path.exists(fname):

	id_list = [v.strip() for v in open(fname).read().split()]

	print id_list
	pdbl.download_pdb_files(id_list, pdir=".")

	#parser = PDBParser()

print('Finished')
