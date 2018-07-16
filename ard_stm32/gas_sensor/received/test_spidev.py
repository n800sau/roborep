#!/usr/bin/env python

import spidev
spi = spidev.SpiDev()
spi.open(2, 0)
to_send = [0x01, 0x02, 0x03]
print spi.xfer(to_send)
