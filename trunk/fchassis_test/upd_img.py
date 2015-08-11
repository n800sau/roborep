#!/usr/bin/env python

import sys, os
import time

import picamera
from lib.camera import update_img
from lib.utils import dbprint

if __name__ == '__main__':

	with picamera.PiCamera() as camera:
		time.sleep(1)
		update_img(camera)
