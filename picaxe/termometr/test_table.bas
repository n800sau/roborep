TABLE 0, (0x1122, 0x2233)

main:
	readtable 0,w1
	serout 8,N2400,(w1)
	goto main
