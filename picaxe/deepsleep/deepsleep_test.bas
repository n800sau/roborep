symbol led_pin = C.2

	setfreq k31
	high led_pin
main:
	setfreq m4
	sertxd("sleep with on", cr,lf)
	setfreq k31
	disablebod
	sleep 10		; sleep for 23 seconds (2.3x10) (1-65535)
	enablebod
	low led_pin
	setfreq m4
	sertxd("sleep with off", cr,lf)
	setfreq k31
	disablebod
	sleep 20
	enablebod
	high led_pin
	goto main
