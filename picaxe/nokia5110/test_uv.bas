symbol in_min = 990
symbol in_max = 2800
symbol out_min = 0
symbol out_max = 15000

symbol uv_in = C.2
symbol vref = C.3

symbol dIn = 2800 - 990
symbol coef = 15000 / dIn

main:
	readadc10 uv_in,w1
	sertxd("0:", #w1, 13, 10)
	readadc10 vref,w2
	sertxd("1:", #w2, 13, 10)
	w3 = 3300 * w1 
	sertxd("2:", #w3, 13, 10)
	w3 = w3 / w2
	sertxd("3:", #w3, 13, 10)
	sertxd("4:", #coef, 13, 10)

	w4 = w3 - 990
	sertxd("5:", #w4, 13, 10)
	w5 = w4 * coef
	sertxd("UV Intensity:", #w5, " (mW/cm^2)", 13, 10)

	' scale then
	' (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	
'	debug			; transmit to computer
	pause 1000		; short delay
	goto main		; loop back to start
