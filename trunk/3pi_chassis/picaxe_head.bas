symbol raw_distance = w0
symbol distance = w1
symbol cmd = b4
symbol beacon = b5
symbol wait_ir_cmd = b6
symbol ir_cmd = b7
symbol max_lvl = b8
symbol USONIC_TRIG = c.1
symbol USONIC_ECHO = c.3
symbol BEACON_IR = c.4

main:
	setfreq m8
'	disconnect
mainloop:
	'get distance
	low USONIC_TRIG
	pulsout USONIC_TRIG, 1          ' Send a pulse to start the ranging
	pulsin USONIC_ECHO, 1, raw_distance ' Recieve timed pulse from SRF05/04
	distance = raw_distance * 10                ' Calculate distance
	distance = distance/58
'	debug
	'get maximum ir command from beacon
	max_lvl = 0
	wait_ir_cmd = 1
	gosub get_beacon
	if wait_ir_cmd = "!" then serial
	wait_ir_cmd = 2
	gosub get_beacon
	if wait_ir_cmd = "!" then serial
	wait_ir_cmd = 3
	gosub get_beacon
	if wait_ir_cmd = "!" then serial
	wait_ir_cmd = 4
	gosub get_beacon
	if wait_ir_cmd = "!" then serial
	wait_ir_cmd = 5
	gosub get_beacon
serial:
	'fixed baud rate 4800 (?38400 on 32Mh?), 8 data, no parity, 1 stop
'	disconnect
'	sertxd ("distance:", #distance, 13, 10)

'	sertxd ("beacon:", #beacon, 13, 10)
'	sertxd ("\n")
	disconnect
	serrxd [10000, timeout], b15
	sertxd ("echo:", #b15, 13, 10)
	goto notimeout
timeout:
	sertxd ("timeout", 13, 10)
notimeout:
	reconnect
	
'	select case cmd
'		case "d"
'			sertxd ("distance:", #distance)
'		case "b"
'			sertxd ("beacon:", #beacon)
'	endselect
'	reconnect
	goto mainloop

get_beacon:
	irin [150, not_found], BEACON_IR, ir_cmd
	if ir_cmd <> wait_ir_cmd then 
		max_lvl = ir_cmd
		return
	endif
not_found:
	beacon = max_lvl
	wait_ir_cmd = "!"
	return


'or just sending distance and beackon visibility once in a while?



'beacon sends 'i am here'
'3pi comes and sends 'lock me'
'beacon locks and stop sending i am here'
'3pi comes and sends 'unlock me'
'beacon unlocks and resume sending i am here'

'beacon sends i1, lowes power then i2, then lowers and i3 etc
'when 3pi receives all of them then it is close and in direction
'power is chaged by additional picaxe pin
