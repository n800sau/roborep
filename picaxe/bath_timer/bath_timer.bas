' 2.4V pack 20M2

symbol seg1 = C.0
symbol seg2 = C.1
symbol seg3 = C.2
symbol seg4 = C.4
symbol seg5 = C.5
symbol seg6 = C.7
symbol seg7 = B.3
symbol seg8 = B.2

symbol touch_btn = B.6
symbol bstate = b0

main:
	touch touch_btn, bstate
	if bstate > 100 then
		high seg1
		high seg2
		high seg3
		high seg4
		high seg5
		high seg6
		high seg7
		high seg8
		pause 1000
		low seg1
		low seg2
		low seg3
		low seg4
		low seg5
		low seg6
		low seg7
		low seg8
	endif
	pause 100
	goto main

