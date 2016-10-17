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
symbol last_curr_digit = b3
symbol in_action = b4
symbol dot_on = b5
symbol digit_on = b6

symbol blink_period = 200

start0:
	touch16 touch_btn, wstate
	if wstate > 0x0b00 then
		time = 0
		in_action = 1
		dot_on = 0
		digit_on = 0
		restart 1
		restart 2
	end if
	if in_action = 0 then
'		disablebod
'		sleep 1
'		enablebod
		nap 3
	else
		pause 100
	end if
	goto start0

start1:
	if in_action = 1 then
		curr_digit = time / 60
		if curr_digit >= 10 then
			gosub hide_digit
			in_action = 0
		else
			if last_curr_digit = curr_digit then
				if digit_on = 1 then
					gosub show_digit
					digit_on = 0
				else
					gosub hide_digit
					digit_on = 1
				end if
			else
				gosub show_digit
				last_curr_digit = curr_digit
			end if
		end if
	else
		gosub hide_digit
		suspend 1
	endif
	pause blink_period
	goto start1

start2:
	if in_action = 1 then
		if dot_on = 1 then
			high seg_dot
			dot_on = 0
		else
			low seg_dot
			dot_on = 1
		end if
	else
		high seg_dot
		suspend 2
	end if
	b20 = time % 60
	b20 = 120 - b20
	b20 = blink_period  * b20 / 120
	pause b20
	goto start2

show_digit:
	for b10 = 0 to 7
		b11 = curr_digit * 7 + b10
		read b11, b12
		b13 = a_pins_addr + b10
		read b13, b14
		if b12 = 0 then
			high b14
		else
			low b14
		end if
	next
	return

hide_digit:
	for b10 = 0 to 7
		b12 = a_pins_addr + b10
		read b12, b13
		high b13
	next
	return
