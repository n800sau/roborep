
print 'G91 ;relative'
print 'G90 ;absolute'
print 'G1 Z10 F500'
print 'G1 X0 Y0 F500'
print 'G1 Z0 F500'
print 'M4'

for r in range(3000, 1, -10):
	r /= 1000.
	zpos = r - 3
	print 'G1 Z%f' % zpos
	print 'G2 R%f' % r
print 'G1 Z10 F500'
print 'M5'
