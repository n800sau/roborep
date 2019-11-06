import sys, os
from entrez_head import *

handle = Entrez.efetch(db="pubmed", id=sys.argv[0])
print handle.read()
