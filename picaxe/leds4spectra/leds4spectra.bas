symbol btn = pinC.6
symbol no_led = 10
symbol led_count = 10
symbol led_count_minus_one = 9
symbol cur_led = b10

' 10 led pins
table 0,(B.0, B.1, B.2, B.3, B.4, B.5, B.6, B.7, C.0, C.1)
cur_led = no_led;

for b0=0 to led_count_minus_one
	readtable b0,b1
	high b1
	pause 300
	low b1
next

main:
	pullup %0100000000000000
	if btn = 0 then
		inc cur_led
		if cur_led > led_count then
			cur_led = 0
		endif
		gosub update_leds
	endif
'	debug
'	pause 10
	goto main

update_leds:
	for b0=0 to led_count_minus_one
		readtable b0,b1
		if b0 = cur_led then
'SERTXD ("high", #b1, 13, 10)
			high b1
		else
'SERTXD ("low", #b1, 13, 10)
			low b1
		endif
	next b0
'SERTXD (13, 10)
	pause 200
'	debug
	return

