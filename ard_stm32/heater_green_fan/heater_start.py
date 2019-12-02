#!/usr/bin/env python3

import time
from heater_lib import *

ser = init_serial(reset=True)
time.sleep(1)
send_cmd(ser, b'G')
read_print_all(ser)
