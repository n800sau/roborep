symbol raw_distance = w0
symbol distance = w1
symbol cmd = b4
symbol beacon = b5
symbol wait_ir_cmd = b6
symbol ir_cmd = b7
symbol USONIC_TRIG = c.1
symbol USONIC_ECHO = c.3
symbol BEACON_IR = c.4

start1:
	disconnect
	'fixed baud rate 4800 (38400 on 32Mh), 8 data, no parity, 1 stop
	serrxd [1000, start1], ("cmd:"), cmd
	select case cmd
		case "d"
			sertxd ("distance:", #distance)
		case "b"
			sertxd ("beacon:", #beacon)
	endselect
	reconnect

start2:
	'get distance
	low 1                   ' Set output pin low
	pulsout USONIC_TRIG, 1           ' Send a pulse to start the ranging
	pulsin USONIC_ECHO, 1, raw_distance ' Recieve timed pulse from SRF05/04
	distance = raw_distance * 10                ' Calculate distance
	distance = distance/58
	debug
	pause 10                ' Wait before next pulse
	goto start2

start3:
	'get maximum ir command from beacon
	beacon = ir_cmd
	wait_ir_cmd = 1
	gosub get_beacon
	if wait_ir_cmd = '!' then start3
	wait_ir_cmd = 2
	gosub get_beacon
	if wait_ir_cmd = '!' then start3
	wait_ir_cmd = 3
	gosub get_beacon
	if wait_ir_cmd = '!' then start3
	wait_ir_cmd = 4
	gosub get_beacon
	if wait_ir_cmd = '!' then start3
	wait_ir_cmd = 5
	gosub get_beacon
	goto start3

get_beacon:
	irin [1000, not_found], BEACON_IR, ir_cmd
	if ir_cmd = wait_ir_cmd then
		wait_ir_cmd = '!'
not_found:
	return


'or just sending distance and beackon visibility once in a while?


SendCommand2Charger:
	'send ir command
return

'beacon sends 'i am here'
'3pi comes and sends 'lock me'
'beacon locks and stop sending i am here'
'3pi comes and sends 'unlock me'
'beacon unlocks and resume sending i am here'

'beacon sends i1, lowes power then i2, then lowers and i3 etc
'when 3pi receives all of them then it is close and in direction
'power is chaged by additional picaxe pin
