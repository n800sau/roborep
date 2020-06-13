	low b.7
main:
	irin [1000, sleeptime], B.2, b1
	debug
	if b1 = 1 then
		high b.7
	endif
	if b1 = 2 then
		low b.7
	endif
sleeptime:
'	sertxd("goto sleep", cr,lf)
'	nap 8
'	sertxd("woke up", cr,lf)
	goto main
