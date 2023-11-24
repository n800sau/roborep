import sys, os
from entrez_head import *

handle = Entrez.esearch(db="pubmed", term=sys.argv[1])
record = Entrez.read(handle)
print(json.dumps(record, indent=2))
