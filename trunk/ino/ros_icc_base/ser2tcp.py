#!/usr/bin/env python

import os, sys
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib_py'))

from ser2tcp_twisted import run

run(os.path.dirname(__file__))

