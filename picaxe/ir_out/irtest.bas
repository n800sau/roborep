symbol b_marker = b2
symbol b_data = b1

		hsersetup B9600_4, %1000  ' no hserout
'		hsersetup B9600_4, 0      ' with hserout

		b_marker = 0
main:	b_data = $ff
		hserin b_data
		if b_data <> $ff then
			if b_marker = 0 then
'				sertxd("Sending ",#b_data,13,10)
				irout C.2,1,b_data
				hserout 1, (#b_data, 13, 10)
			end if
			b_marker = 1
		else
			if b_marker = 1 then
				sertxd(13,10)
				b_marker = 0
			end if
		end if
		goto main

'main:
'	for b_data = 1 to 100
'	sertxd("Sending ",#b_data,13,10)
'	irout C.2,5,b_data
'	pause 100
'	next b_data
'	goto main