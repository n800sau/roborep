#!/usr/bin/python

from pyfann import libfann

ann = libfann.neural_net()
ann.create_from_file("net")

print ann.run([2, 70])
