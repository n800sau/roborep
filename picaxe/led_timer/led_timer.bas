' 4.5V pack 08M2

symbol touch_btn = C.4
symbol led1 = C.0
symbol led2 = C.1
symbol led3 = C.2
symbol light_on = b0
symbol bstate = b1
symbol timeval = w1

' 2 min
symbol period = 500
symbol secticks = 1000 / period
symbol minticks = secticks * 60
symbol twominticks = minticks * 2
symbol time_end = twominticks * 1000
symbol stagelen = time_end / 3

symbol stage1end = stagelen
symbol stage2end = stagelen * 2

main:
	gosub reset_leds
	touch touch_btn, bstate
	if bstate > 100 then
		let timeval = 0
'sertxd("The value of timeval is ",#timeval,13,10)
		gosub action
	endif
' sleep consumption 0.07
	disablebod
	sleep 1
	enablebod
	goto main

action:
	if timeval >= time_end then
		return
	endif
	gosub blink
	pause period
	let timeval = timeval + period
	debug
	goto action

blink:
	if timeval >= stage2end then
		high led1
		high led2
		if light_on = 0 then
			low led3
			let light_on = 0
		else
			high led3
			let light_on = 1
		endif
	elseif timeval >= stage1end then
		high led1
		if light_on = 1 then
			low led2
			let light_on = 0
		else
			high led2
			let light_on = 1
		endif
	else
		if light_on = 1 then
			low led1
			let light_on = 0
		else
			high led1
			let light_on = 1
		endif
	endif
	return

reset_leds:
	let light_on = 0
	low led1
	low led2
	low led3
	return
