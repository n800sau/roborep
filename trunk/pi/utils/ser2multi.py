#!/usr/bin/env python

import os
from subprocess import check_call

B0 = '18'
B1 = '23'

def is_busy():
    rs = False
    for b in (B0, B1):
        if os.path.exists('/sys/devices/virtual/gpio/gpio%s/direction' % b):
	    rs = True
	    break
    return rs


def m_on():
    for b in (B0, B1):
        check_call('gpio-admin export %s' % b, shell=True)

def m_off():
    for b in (B0, B1):
        check_call('gpio-admin unexport %s' % b, shell=True)

def switch(v0, v1):
    for b,v in ((B0, v0), (B1, v1)):
        file('/sys/devices/virtual/gpio/gpio%s/value' % b,'w').write(str(v))

def arduino_on():
    switch(0, 0)

def picaxe_on():
    switch(1, 0)

def camera_on():
    switch(0, 1)

if __name__ == '__main__':
    m_on()
    print is_busy()
    arduino_on()
    camera_on()
    picaxe_on()
    m_off()

