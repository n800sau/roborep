#!/usr/bin/env python3

from heater_lib import *

send_cmd(init_serial(reset=False), b'G')
