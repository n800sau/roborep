symbol ir_pin = B.5
symbol mosfet_pin = B.7
symbol esp_sleep_request_pin = B.6
' only inputs C.1 to C.5 may be used for interrupts
symbol k1_inp = C.1
symbol k2_inp = C.2
symbol k3_inp = C.3
symbol k4_inp = C.4
symbol k1_outp = B.1
symbol k2_outp = B.2
symbol k3_outp = B.3
symbol k4_outp = B.4
symbol kstate  = b8
symbol irstate = b9
symbol mosfetstate = b10
symbol start_time = b11
symbol diff_time = b12
symbol sleep_again_timeout = 5


	enabletime
	pullup on
' pullup C.1-C.4 on 20M2
	pullup %0001111000000000
	low mosfet_pin
' current state of mosfet output
	mosfetstate = 0
' set output low to prevent power leaking to esp32
	low k1_outp
	low k2_outp
	low k3_outp
	low k4_outp
main:
'	irin [200, no_ir], ir_pin, irstate
'	sertxd("ir=", irstate, cr,lf)
	pause 200
	if irstate > 0 and irstate < 16 then
		kstate = irstate
		gosub process_keys
	endif
no_ir:
	gosub read_keys
	goto main
process_keys:
	if kstate <> 0 and mosfetstate = 0 then
		high mosfet_pin
		mosfetstate = 1
		start_time = time
		sertxd("mosfet set on", cr,lf)
	endif
	if mosfetstate = 1 then
		sertxd("mosfet is on", cr,lf)
		if pinC.1 = 0 then
			high k1_outp
		else
			low k1_outp
		endif
		if pinC.2 = 0 then
			high k2_outp
		else
			low k2_outp
		endif
		if pinC.3 = 0 then
			high k3_outp
		else
			low k3_outp
		endif
		if pinC.4 = 0 then
			high k4_outp
		else
			low k4_outp
		endif
	else
		sertxd("mosfet is off", cr,lf)
		low k1_outp
		low k2_outp
		low k3_outp
		low k4_outp
	endif
	return
read_keys:
	kstate = pinsC
	kstate = not kstate
	kstate = kstate & %00011110 / 2
	sertxd("pinsC=", #kstate, cr,lf)
	gosub process_keys
	gosub is_sleeptime
	return
is_sleeptime:
	diff_time = time - start_time
	sertxd("diff_time=", #diff_time, ",", #start_time, ",", #time, cr,lf)
	if pinB.6 = 1 and kstate = 0 and diff_time > sleep_again_timeout then
		sertxd("turn mosfet off", cr,lf)
		low k1_outp
		low k2_outp
		low k3_outp
		low k4_outp
		low mosfet_pin
		mosfetstate = 0
	endif
	return
