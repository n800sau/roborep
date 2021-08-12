#picaxe 20m2

setfreq m16
'setfreq m4

' 19 steps only
SYMBOL LED_COUNT = 20
SYMBOL LED_BLOCK_SIZE = 5
SYMBOL LED_BLOCK_COUNT = LED_COUNT / LED_BLOCK_SIZE
SYMBOL LED_PIN_COUNT = 5
SYMBOL PWM_PIN_COUNT = 4
SYMBOL TICK_PAUSE = 400

SYMBOL LED_STATE = b1
SYMBOL ADDR = b2
SYMBOL IO_REF = b4
SYMBOL PWM_REF = b5
SYMBOL LED_INDEX = b6
SYMBOL PWM_PIN_INDEX = b7
SYMBOL LED_PIN_INDEX = b8
SYMBOL ACC = b9
SYMBOL ACC_ITEM = b10
SYMBOL I = b11
SYMBOL J = b12
SYMBOL K = b13
SYMBOL PWM_DUTY = w7
SYMBOL TIMEVAL = w8

SYMBOL LED_PIN_BASE = 0
SYMBOL LED_PIN_END = LED_PIN_BASE + LED_PIN_COUNT
SYMBOL PWM_PIN_BASE = 10
SYMBOL PWM_PIN_END = PWM_PIN_BASE + PWM_PIN_COUNT
SYMBOL LED_ARRAY_BASE = 15

SYMBOL BTN = pinC.6


' led pins
eeprom LED_PIN_BASE,(C.1, C.0, B.7, B.6, B.5)
' pwm pins
eeprom PWM_PIN_BASE,(C.5, B.1, C.2, C.3)
' led_states cleaned on start
gosub reset_leds
	pullup %0100000000000000 ' C.6 - pullup

main:
	if BTN = 0 then
sertxd("BTN == 0",CR, LF)
		let TIMEVAL = 0
		gosub run_timer
		gosub play_tune
		gosub reset_leds
	else
sertxd("BTN == 1",CR, LF)
	endif
' sleep consumption 0.07
	disablebod
	sleep 1
	enablebod
	goto main

reset_leds:
' turn off pins
	let ADDR = LED_PIN_BASE
	do while ADDR < LED_PIN_END
		read ADDR, IO_REF
		low IO_REF
		inc ADDR
	loop
' turn off pwm
	let ADDR = PWM_PIN_BASE
	do while ADDR < PWM_PIN_END
		read ADDR, PWM_REF
		pwmout PWM_REF, off
		high PWM_REF
		inc ADDR
	loop
	return

run_timer:
	let LED_INDEX = 0
	do while LED_INDEX < LED_COUNT
		for J=1 to LED_BLOCK_SIZE
			let LED_STATE = 1
			let K = TICK_PAUSE / J
			for I=1 to J
				gosub set_led
				pause K
				inc LED_INDEX
			next I
			let LED_INDEX = LED_INDEX - J
			let LED_STATE = 0
			for I=1 to J
				gosub set_led
				pause K
				inc LED_INDEX
			next I
			let LED_INDEX = LED_INDEX - 1
			let LED_STATE = 1
			for I=1 to J
				gosub set_led
				pause K
				dec LED_INDEX
			next I
			let LED_INDEX = LED_INDEX + J
			let LED_STATE = 0
			for I=1 to J
				gosub set_led
				pause K
				dec LED_INDEX
			next I
			let LED_INDEX = LED_INDEX + 1
		next J
		let LED_INDEX = LED_INDEX + LED_BLOCK_SIZE
	loop
	return
	let LED_INDEX = 1
	do while LED_INDEX < LED_COUNT
		for I=0 to 3
			let LED_STATE = 1
			gosub set_led
			pause TICK_PAUSE
			let LED_STATE = 0
			gosub set_led
			dec LED_INDEX
			let LED_STATE = 1
			gosub set_led
			pause TICK_PAUSE
			let LED_STATE = 0
			gosub set_led
			inc LED_INDEX
		next I
		inc LED_INDEX
	loop
	return

set_led:
	let ADDR = LED_INDEX % LED_BLOCK_SIZE
	let ADDR = ADDR + LED_ARRAY_BASE
	let PWM_PIN_INDEX = LED_INDEX / LED_BLOCK_SIZE
'sertxd("PWM PIN INDEX:",#PWM_PIN_INDEX, CR, LF)
'sertxd ("SET ADDR:",#ADDR, CR, LF)
	write ADDR, LED_STATE
	gosub update_leds
	return

update_leds:
	let ACC = 0
	let LED_PIN_INDEX = 0
	do while LED_PIN_INDEX < LED_PIN_COUNT
'sertxd("LED PIN INDEX:",#LED_PIN_INDEX, CR, LF)
		let ADDR = LED_ARRAY_BASE + LED_PIN_INDEX
		read ADDR, ACC_ITEM
		let ACC = ACC + ACC_ITEM
		let ADDR = LED_PIN_INDEX + LED_PIN_BASE
		read ADDR, IO_REF
		if ACC_ITEM = 0 then
			low IO_REF
		else
			high IO_REF
		end if
		inc LED_PIN_INDEX
	loop
	let ADDR = PWM_PIN_INDEX + PWM_PIN_BASE
	read ADDR, PWM_REF
	if ACC = 0 then
		pwmout PWM_REF, off
		high PWM_REF
	else
		let PWM_DUTY = ACC * 40 + 50
		let PWM_DUTY = 512 - PWM_DUTY
'sertxd ("pwm:",#ADDR,", ACC:", #ACC, ", set:", #PWM_DUTY, CR, LF)
		pwmout PWM_REF, 128, PWM_DUTY
	end if
	return

play_tune:
	tune C.7, 7,($4C,$42,$43)
	return 
