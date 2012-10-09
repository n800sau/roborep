import time, os

def reset_m():
    os.system('gpio-admin export 4')
    file('/sys/devices/virtual/gpio/gpio4/direction', 'w').write('out')
    file('/sys/devices/virtual/gpio/gpio4/value', 'w').write('1')
    time.sleep(0.1)
    file('/sys/devices/virtual/gpio/gpio4/value', 'w').write('0')
    time.sleep(0.1)
    file('/sys/devices/virtual/gpio/gpio4/value', 'w').write('1')
    os.system('gpio-admin unexport 4')
