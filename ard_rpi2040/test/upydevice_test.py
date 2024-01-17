#!/usr/bin/env python3

import os
from upydevice import Device


exec(open('vars.sh').read())

pico = Device(os.path.realpath(DEV), init=False)

print(pico)

pico.wr_cmd('from machine import ADC, Pin')
pico.wr_cmd('adc = ADC(Pin(26))')
pico.wr_cmd('print("Hello", adc.read_u16())')

pico.disconnect()
