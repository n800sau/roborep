' 2.4V pack 20M2

'left top
symbol seg1 = C.0
'middle
symbol seg2 = C.1
'left bottom
symbol seg3 = C.2
'bottom
symbol seg4 = C.4
'right bottom
symbol seg5 = C.5
'dot
symbol seg6 = C.7
'right top
symbol seg7 = B.3
'top
symbol seg8 = B.2

symbol touch_btn = B.6
symbol wstate = w0

main:
	touch16 touch_btn, wstate
	debug
	if wstate > 0x0b00 then
		low seg1
		pause 1000
		low seg2
		pause 1000
		low seg3
		pause 1000
		low seg4
		pause 1000
		low seg5
		pause 1000
		low seg6
		pause 1000
		low seg7
		pause 1000
		low seg8
		pause 1000
		high seg1
		high seg2
		high seg3
		high seg4
		high seg5
		high seg6
		high seg7
		high seg8
	endif
	pause 100
	goto main

