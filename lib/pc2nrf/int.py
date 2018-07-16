#!/usr/bin/env python

print 'Waiting...'
print file('/sys/class/gpio/gpio73/value').read(2)
