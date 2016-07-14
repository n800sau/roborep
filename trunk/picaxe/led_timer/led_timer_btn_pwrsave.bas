' 4.5V pack 08M2

symbol btn = pinC.3
symbol led1 = C.0
symbol led2 = C.1
symbol led3 = C.2
symbol led4 = C.4
symbol light_on = b0
symbol bstate = b1
symbol timeval = w1

' 2 min
symbol period = 500
symbol blink_period = 200
symbol tick_pause = period - blink_period
symbol secticks = 1000 / period
symbol minticks = secticks * 60
symbol time_end = minticks * 2
symbol stagelen = time_end / 4

symbol stage1end = stagelen
symbol stage2end = stagelen * 2
symbol stage3end = stagelen * 3

main:
	gosub reset_leds
	if btn = 0 then
		let timeval = 0
		gosub action
		pause 1000
	endif
	pause 100
	goto main

action:
	if timeval >= time_end then
		high led1
		high led2
		high led3
		high led4
		return
	endif
	gosub blink
	pause tick_pause
	let timeval = timeval + period
	goto action

blink:
	if timeval >= stage3end then
		high led1
		high led2
		high led3
		high led4
		pause blink_period
		low led4
	elseif timeval >= stage2end then
		high led1
		high led2
		high led3
		pause blink_period
		low led3
	elseif timeval >= stage1end then
		high led1
		high led2
		pause blink_period
		low led2
	else
		high led1
		pause blink_period
		low led1
	endif
	return

reset_leds:
	let light_on = 0
	low led1
	low led2
	low led3
	low led4
	return
