
	setfreq k31
	high B.1
main:
	disablebod
	sleep 10		; sleep for 23 seconds (2.3x10) (1-65535)
	enablebod
	low B.1
	disablebod
	sleep 10
	enablebod
	high B.1
	goto main
