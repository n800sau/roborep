'#picaxe 14m2  '  4Bit Using Alternative pins marks
'#picaxe 18m2
#picaxe 20m2
'#picaxe 20x2

SYMBOL DB4  = outpinB.2
SYMBOL DB5  = outpinB.3
SYMBOL DB6  = outpinB.4
SYMBOL DB7  = outpinB.5
SYMBOL E    = C.0
SYMBOL RS   = C.1

SYMBOL INP_V = C.2
SYMBOL OUT_LED = C.3
SYMBOL CALIB_BTN = pinC.4

SYMBOL senddata  = b0
SYMBOL index     = b1
SYMBOL REF_V = w1

Symbol Exponent = 4
Symbol Result = w9

Symbol Numerator = w10
Symbol Divisor = w11

Symbol Holder = w12
Symbol N = b0
Symbol Factor = w13


	'value of count    = W1    ' b3,b2
	'display  count    = b4,b5,b6,b7,b8


	EEPROM 25,("Picaxe20M2")
' with freq it does not work
'	SETFREQ M32
	dirsB = %11111111
	dirsC = %11001111

Initialise:
	REF_V = 0
	FOR  index = 0 to 6
	LOOKUP index, ($33,$32,$28,$0C,$01,$02,$06),senddata : GOSUB Send    ' Initialise LCD/OLED
		'(WakeUp)*3(Set4Bit)(4Bit/2Line)(DisplayOn)(Clear Display)(Return Home)(Entry Mode Set)
	NEXT index : PAUSE 10
	pullup %0000100000000000

DisplayHello:
	LOW  RS                                ' commandmode
	senddata = 132 :           GOSUB Send  ' (128-147) Line 1 Cursor Position
	HIGH RS                                ' charactermode
	senddata = "H" :           GOSUB Send
	senddata = "e" :           GOSUB Send
	senddata = "l" :           GOSUB Send
	senddata = "l" :           GOSUB Send
	senddata = "o" :           GOSUB Send
	LOW  RS                                            ' commandmode
	senddata = $01 :           GOSUB Send : PAUSE 10   ' Clear Display
	senddata = 192 :           GOSUB Send              ' (192-211) Line 2 Cursor Position
	HIGH RS                                             ' charactermode
'	FOR  index = 25 to 34
'	READ index, senddata :      GOSUB Send              ' sending character
'	NEXT index

measurement:
	HIGH OUT_LED
	PAUSE 200                                                                    ' 1 second (M32)
	readadc10 INP_V,Divisor
	LOW OUT_LED
	if CALIB_BTN = 1 or REF_V = 0 then
		REF_V = Divisor
		sertxd("calibration:", #REF_V, 13, 10)
		BinToAscii Divisor,b8,b7,b6,b5,b4
		LOW  RS                ' commandmode
		senddata = 192 :       GOSUB Send              ' (192-211) Line 2 Cursor Position
		HIGH RS                ' charactermode
		FOR  index = 0 TO 5                                                           ' sending characters
			LOOKUP index,(" ",b8,b7,b6,b5,b4),senddata :          GOSUB Send
		NEXT index
	endif
	Numerator = REF_V
	gosub Divide
	sertxd("ref_v:", #REF_V, ",v:", #Divisor, ",Result:", #Result, 13, 10)
	BinToAscii Result,b8,b7,b6,b5,b4
	LOW  RS                ' commandmode
	senddata = 200 :       GOSUB Send              ' (192-211) Line 2 Cursor Position
	HIGH RS                ' charactermode
	FOR  index = 0 TO 5                                                           ' sending characters
		LOOKUP index,(" ",b8,".",b7,b6,b5,b4),senddata :          GOSUB Send
	NEXT index
	PAUSE 1000                                                                    ' 1 second (M32)
	goto measurement

Send:
'	sertxd("Send:", senddata, 13, 10)
	DB7 =   bit7
	DB6 =   bit6
	DB5 =   bit5
	DB4 =   bit4
	PULSOUT E,1
	DB7 =   bit3
	DB6 =   bit2
	DB5 =   bit1
	DB4 =   bit0
	PULSOUT E,1
	RETURN


Divide:
	Result = 0
	Holder = 0
	For N = 0 to Exponent
		Result = Result * 10
		Holder = Numerator / Divisor
		Result = Holder + Result
	 	Factor = Divisor * Holder
		Holder = Numerator - Factor   
		if Holder > 6553 then
			Holder = Holder / 10
			Divisor = Divisor / 10
		endif
		Numerator = Holder *10
	Next
	'I'm using 5 as a cut off for rounding because it's a simple method.
	'There are other rounding techniques.
	Holder = Numerator / Divisor
	if Holder >=5 then
		Result = Result + 1
	endif
	return