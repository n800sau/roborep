symbol ir_pin = C.0
symbol mosfet_pin = C.1
symbol esp_sleep_request_pin = C.2
' only inputs C.1 to C.5 may be used for interrupts
symbol k1_inp = C.1
symbol k2_inp = C.2
symbol k3_inp = C.3
symbol k4_inp = C.4
symbol k1_outp = B.4
symbol k2_outp = B.5
symbol k3_outp = B.6
symbol k4_outp = B.7
symbol kstate  = b8
symbol tmp_b   = b1

	pullup on
	gosub reset_int
	low b.7
' set output low to prevent power leaking to esp32
	low k1_outp
	low k2_outp
	low k3_outp
	low k4_outp
main:
' test
	irin [100, sleeptime], ir_pin, tmp_b
	sertxd("ir=", tmp_b, cr,lf)
	gosub read_keys
'	debug
	if kstate > 0 then
		high mosfet_pin
	endif
	if pinC.2 = 1 then
		low mosfet_pin
	endif
	goto main
read_keys:
	kstate = pinsC & %000011110 / 2
	sertxd("pinsC=", #kstate, cr,lf)
	return
sleeptime:
	gosub read_keys
'	sertxd("goto sleep", cr,lf)
'	nap 8
'	sertxd("woke up", cr,lf)
	goto main
interrupt:
	sertxd("interrupt", cr,lf)
	gosub read_keys
	return
reset_int:
' pullup C.1-C.4 on 20M2?
	pullup %0001111000000000
' interrupt on C.1-C.4 low (default high)
	setint OR %10000000,%00011110
	return
