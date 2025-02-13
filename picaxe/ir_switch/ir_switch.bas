symbol ir_pin = B.5
symbol esp32_pin = B.7
symbol esp_sleep_request_pin = B.6
symbol tft_led_pin = C.5 ' pwm, hpwm pin
' only inputs C.1 to C.5 may be used for interrupts
symbol led_pin = C.7
symbol k1_inp = C.1
symbol k2_inp = C.2
symbol k3_inp = C.3
symbol k4_inp = C.4
symbol k1_outp = B.1
symbol k2_outp = B.2
symbol k3_outp = B.3
symbol k4_outp = B.4
symbol esp_sleep_request_state = b7
symbol kstate  = b8
symbol irstate = b9
symbol esp32state = b10
symbol start_time = b11
symbol diff_time = b12
symbol sleep_again_timeout = 5


	enabletime
	pullup on
' pullup C.1-C.4 on 20M2
	pullup %0001111000000000
	gosub led_on
	high esp32_pin
	high tft_led_pin
' current state of esp32 output
	esp32state = 0
' no ir state
	irstate = 0
' set output low to prevent power leaking to esp32
	low k1_outp
	low k2_outp
	low k3_outp
	low k4_outp
	pause 500
	gosub led_off
main:
	irin [200, no_ir], ir_pin, b0
	if b0 >= 0 and b0 < 4 then
		irstate = b0 + 1
		gosub led_on
		sertxd("ir=", #irstate, cr,lf)
		gosub read_keys
	endif
no_ir:
	gosub read_keys
	goto main
process_keys:
	if kstate <> 0 and esp32state = 0 then
		gosub led_on
		low esp32_pin
		esp32state = 1
		start_time = time
		sertxd("esp32 set on", cr,lf)
	endif
	if esp32state = 1 then
'		sertxd("esp32 is on", cr,lf)
		b0 = kstate & 1
		if b0 <> 0 then
			high k1_outp
		else
			low k1_outp
		endif
		b0 = kstate & 2
		if b0 <> 0 then
			high k2_outp
		else
			low k2_outp
		endif
		b0 = kstate & 4
		if b0 <> 0 then
			high k3_outp
		else
			low k3_outp
		endif
		b0 = kstate & 8
		if b0 <> 0  then
			high k4_outp
		else
			low k4_outp
		endif
	else
'		sertxd("esp32 is off", cr,lf)
		low k1_outp
		low k2_outp
		low k3_outp
		low k4_outp
	endif
	return
read_keys:
	b0 = 0
	if pinC.1 = 0 then
		let b0 = b0 | 1
	endif
	if pinC.2 = 0 then
		let b0 = b0 | 2
	endif
	if pinC.3 = 0 then
		let b0 = b0 | 4
	endif
	if pinC.4 = 0 then
		let b0 = b0 | 8
	endif
	if b0 = 0 then
		select irstate
			case 1
				kstate = 2
			case 2
				kstate = 1
			case 3
				kstate = 8
			case 4
				kstate = 4
		endselect
	else
		kstate = b0
		gosub led_on
		irstate = 0
	endif
	sertxd("kstate=", #kstate, cr,lf)
	gosub process_keys
	gosub is_sleeptime
	return
is_sleeptime:
	diff_time = time - start_time
	let esp_sleep_request_state = pinB.6
'	sertxd("diff_time=", #diff_time, ",", #start_time, ",", #time, ",", #esp_sleep_request_state, cr,lf)
	if esp32state = 1 and diff_time > sleep_again_timeout then
		if esp_sleep_request_state = 1 then
			sertxd("turn esp32 off", cr,lf)
			kstate = 0
			irstate = 0
			low k1_outp
			low k2_outp
			low k3_outp
			low k4_outp
			high esp32_pin
			pwmout tft_led_pin, OFF
			high tft_led_pin
			esp32state = 0
		else
			gosub led_off
			pwmout tft_led_pin, 50, 150
		endif
	endif
	return
led_on:
	high led_pin
	return
led_off:
	low led_pin
	return
