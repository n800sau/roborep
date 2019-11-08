import mecode

# from bottom of drilled 2.5 mm center up
bit_d = 2
depth = 5
min_d = 2.5
max_d = 6
d_step = 0.1
h_step = depth/((max_d - bit_d)/d_step)

d_step_1000 = int(d_step * 1000)
min_d_1000 = int(min_d * 1000)
max_d_1000 = int(max_d * 1000)

g = mecode.G()
#g.write("M302 S0")  # send g-Code. Here: allow cold extrusion. Danger: Make sure extruder is clean without filament inserted 
g.absolute()  # Absolute positioning mode
g.move(z=10, F=500)
g.move(x=0, y=0, F=500)  # feedrate of 500 mm/min
g.move(z=-depth, F=500)
g.write('M4')
print 'h_step', h_step
zpos = -depth
for d in range(min_d_1000, max_d_1000, d_step_1000):
	d /= 1000.
	g.move(z=zpos, F=500)
	g.arc(x=0, y=0, radius=d/2)
	zpos += h_step
g.move(z=10, F=500)
g.write('M5')
