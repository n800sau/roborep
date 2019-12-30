start1:
	disconnect
	'fixed baud rate 4800 (9600 on X2 parts), 8 data, no parity, 1 stop
	serrxd [1000, timeout], "cmd", @ptrinc,@ptrinc,@ptr
	reconnect
