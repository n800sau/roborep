' 4.5V pack 08M2

symbol btn = pinC.3
symbol led1 = C.0
symbol led2 = C.1
symbol led3 = C.2
symbol timeval = w1

' 2 min
symbol period = 500
symbol blink_period = 200
symbol tick_pause = period - blink_period
symbol secticks = 1000 / period
symbol minticks = secticks * 60
symbol time_end = minticks * 2
symbol stagelen = time_end / 3

symbol stage1end = stagelen
symbol stage2end = stagelen * 2

main:
	if btn = 0 then
		let timeval = 0
		gosub action
		gosub reset_leds
	endif
' sleep consumption 0.07
	disablebod
	sleep 1
	enablebod
	goto main

action:
	if timeval >= time_end then
		high led1
		high led2
		high led3
		return
	endif
	gosub blink
	pause tick_pause
	inc timeval
	goto action

blink:
	if timeval >= stage2end then
		low led1
		low led2
		high led3
		pause blink_period
		low led3
	elseif timeval >= stage1end then
		low led1
		high led2
		low led3
		pause blink_period
		low led2
	else
		high led1
		low led2
		low led3
		pause blink_period
		low led1
	endif
	return

reset_leds:
	low led1
	low led2
	low led3
	return
