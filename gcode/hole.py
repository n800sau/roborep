import mecode

g = mecode.G()
#g.write("M302 S0")  # send g-Code. Here: allow cold extrusion. Danger: Make sure extruder is clean without filament inserted 
g.absolute()  # Absolute positioning mode
g.move(z=10, F=500)
g.move(x=0, y=0, F=500)  # feedrate of 500 mm/min
g.move(z=0, F=500)
g.write('M4')
for r in range(3000, 1, -10):
	r /= 1000.
	zpos = r - 3
	g.move(z=zpos, F=500)
	g.arc(x=0, y=0, radius=r)
g.move(z=10, F=500)
g.write('M5')
