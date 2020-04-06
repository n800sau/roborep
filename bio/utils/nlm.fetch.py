import sys, os
from entrez_head import *

handle = Entrez.esearch(db="pubmed", term=sys.argv[1])
record = Entrez.read(handle)
print 'Count:', len(record["IdList"])
for rid in record["IdList"]:
	handle = Entrez.efetch(db="pubmed", id=rid, rettype='medline')
	r = handle.read()
	print r
