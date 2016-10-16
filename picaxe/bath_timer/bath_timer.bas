' 2.4V pack 20M2

'left top
symbol seg_lt = C.0
'middle
symbol seg_m = C.1
'left bottom
symbol seg_lb = C.2
'bottom
symbol seg_b = C.4
'right bottom
symbol seg_rb = C.5
'dot
symbol seg_dot = C.7
'right top
symbol seg_rt = B.3
'top
symbol seg_t = B.2


' array of pins
symbol a_pins_addr = 70
EEPROM a_pins_addr,(seg_t, seg_lt, seg_rt, seg_m, seg_lb, seg_rb, seg_b)
' 10 digits 7 bytes each

' zero
EEPROM 0,  (1, 1, 1, 0, 1, 1, 1)
' one
EEPROM 7,  (0, 0, 1, 0, 0, 1, 0)
' two
EEPROM 14, (1, 0, 1, 1, 1, 0, 1)
' three
EEPROM 21, (1, 0, 1, 1, 0, 1, 1)
' four
EEPROM 28, (0, 1, 1, 1, 0, 1, 0)
' five
EEPROM 35, (1, 1, 0, 1, 0, 1, 1)
' six
EEPROM 42, (1, 1, 0, 1, 1, 1, 1)
' seven
EEPROM 49, (1, 0, 1, 0, 0, 1, 0)
' eight
EEPROM 56, (1, 1, 1, 1, 1, 1, 1)
' none
EEPROM 63, (1, 1, 1, 1, 0, 1, 1)


symbol touch_btn = B.6
symbol wstate = w0

symbol curr_digit = b2

main:
	touch16 touch_btn, wstate
	debug
	if wstate > 0x0b00 then
		curr_digit = 0
		gosub show_digit
		pause 1000
		curr_digit = 0
		gosub show_digit
		pause 1000
		curr_digit = 1
		gosub show_digit
		pause 1000
		curr_digit = 2
		gosub show_digit
		pause 1000
		curr_digit = 3
		gosub show_digit
		pause 1000
		curr_digit = 4
		gosub show_digit
		pause 1000
		curr_digit = 6
		gosub show_digit
		pause 1000
		curr_digit = 7
		gosub show_digit
		pause 1000
		curr_digit = 8
		gosub show_digit
		pause 1000
		curr_digit = 9
		gosub show_digit
		pause 1000
		high seg_lt
		high seg_m
		high seg_lb
		high seg_b
		high seg_rb
		high seg_dot
		high seg_rt
		high seg_t
	endif
	pause 100
	goto main

show_digit:
	for b10 = 0 to 7
		b11 = curr_digit * 7 + b10
		read b11, b12
		b12 = a_pins_addr + b10
		read b12, b13
		if b12 = 0 then
			high b13
		else
			low b13
		end if
	next
	return
