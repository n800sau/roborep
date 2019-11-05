#!/usr/bin/env python

import mecode
#g = mecode.G(
#    direct_write=True, 
#   direct_write_mode="serial", 
#    printer_port="/dev/ttyUSB0",
#    baudrate=115200
#)

g = mecode.G()
#g.write("M302 S0")  # send g-Code. Here: allow cold extrusion. Danger: Make sure extruder is clean without filament inserted 
g.absolute()  # Absolute positioning mode
g.move(z=10, F=500)
g.move(x=0, y=0, z=2, F=500)  # feedrate of 500 mm/min


#g.retract(10)  # Move extruder motor
#g.write("M400")  # IMPORTANT! wait until execution of all commands is finished
#g.teardown()  # Disconnect (close serial connection)g.write("M302 S0")  # send g-Code. Here: allow cold extrusion. Danger: Make sure extruder is clean without filament inserted 
#g.absolute()  # Absolute positioning mode
#g.move(x=20, y=20, F=500)  # move 10mm in x and 10mm in y and 10mm in z at a feedrate of 500 mm/min
#g.retract(10)  # Move extruder motor
g.write("M400")  # IMPORTANT! wait until execution of all commands is finished
g.teardown()  # Disconnect (close serial connection)
