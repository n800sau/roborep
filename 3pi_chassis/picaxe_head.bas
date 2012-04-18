symbol raw_distance = w0
symbol distance = w1
'what comes via ir
symbol ir_cmd = b7

symbol USONIC_TRIG = c.1
symbol USONIC_ECHO = c.3
symbol BEACON_IR = c.4

'fixed baud rate 4800 (?38400 on 32Mh?), 8 data, no parity, 1 stop
'just sending distance and beackon visibility once in a while

main:
	setint %00010000,%00010000
	setfreq m4
mainloop:
	'get distance
	low USONIC_TRIG
	pulsout USONIC_TRIG, 1          ' Send a pulse to start the ranging
	pulsin USONIC_ECHO, 1, raw_distance ' Recieve timed pulse from SRF05/04
	debug
	distance = raw_distance * 10                ' Calculate distance
	distance = distance/58
	if distance > 0 then
		sertxd ("distance:", #distance, 13, 10)
	endif
	pause 10
	goto mainloop

interrupt:
	'get ir command from beacon
	irin [20, not_found], BEACON_IR, ir_cmd
	goto send_beacon
not_found:
	ir_cmd = 0
send_beacon:
	sertxd ("beacon:", #ir_cmd, 13, 10)
	setint %00010000,%00010000 	; re-activate interrupt
	return




'beacon sends sequentially 1 to 5

'3pi comes and sends 'lock me'
'beacon locks and stop sending from 1 to 5
'3pi comes and sends 'unlock me'
'beacon unlocks and resume sending from 1 to 5

'beacon sends i1, lowes power then i2, then lowers and i3 etc
'when 3pi receives maximum from a direction that direction is considered right direction
'power is changed by additional picaxe pin on charger
