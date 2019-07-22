symbol btn = pinC.6
symbol no_led = 10
symbol led_count = 10
symbol cur_led = b10

' 10 led pins
table 0,(B.0, B.1, B.2, B.3, B.4, B.5, B.6, B.7, C.0, C.1)
cur_led = no_led;

main:
	pullup %0100000000000000
	if btn = 0 then
		inc cur_led
		if cur_led >= led_count then
			cur_led = 0
		endif
		gosub update_leds
	endif
'	debug
'	pause 10
	goto main

update_leds:
	for b0=0 to 10
		readtable b0,b1
		if b0 = cur_led then
			high b1
		else
			low b1
		endif
	next b0
	pause 1000
	debug
	return

