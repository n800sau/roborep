#!/usr/bin/env python

import sys, os
from entrez_head import *

handle = Entrez.einfo()
record = Entrez.read(handle)
print json.dumps(record, indent=2)

