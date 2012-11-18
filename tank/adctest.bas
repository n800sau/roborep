symbol v1 = b.0
symbol v2 = b.1

	fvrsetup FVR4096
	adcconfig %011
main:
	readadc10 v1,w1
	readadc10 v2,w2
	let w1 = 2 * 4 * w1 
	let w2 = 2  * 4 * w2
	let w3 = w1-w2
	readadc v1,b10
	readadc v2,b11
	let b13 = 100 + b11-b10
	debug
	goto main
	