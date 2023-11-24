#!/usr/bin/env python3

import time
from tcs3472 import TCS3472
from smbus2 import SMBus
import Adafruit_BBIO.PWM as PWM

bus = SMBus(2)
s = TCS3472(i2c_dev=bus)
print('val=', s.get_rgbc())

# PWM:
# BB-PWM0-00A0
# P9_21
# P9_22
#BB-PWM1-00A0
# P9_14
# P9_16
#BB-PWM2-00A0
# P8_13
# P8_19

pwm_pin_1 = "P9_16"
pwm_pin_2 = "P9_14"
pwm_pin_3 = "P8_19"

pinlist = (pwm_pin_1, pwm_pin_2, pwm_pin_3)

for pin in pinlist:
    PWM.stop(pin)

for pin in pinlist:
#PWM.start(channel, duty, freq=2000, polarity=0)
    PWM.start(pin, 50)
#PWM.stop(pin)

for pin in pinlist:
    time.sleep(0.01)
    PWM.set_duty_cycle(pin, 25.5)
    time.sleep(0.01)
    print('PIN', pin)
    PWM.set_frequency(pin, 100)

PWM.cleanup()

