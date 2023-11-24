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

SYMBOL REF_V = w8
Symbol Value = w9
Symbol Divisor = w10
Symbol Holder = w11
Symbol ZeroValue = 50

	'value of count    = W1    ' b3,b2
	'display  count    = b4,b5,b6,b7,b8


' with freq it does not work
'	SETFREQ M32
	dirsB = %11111111
	dirsC = %11001111
	REF_V = 0
	pullup %0000100000000000 ' C.3 - pullup
	LOW  RS                                ' commandmode
	FOR  index = 0 to 6
	LOOKUP index, ($33,$32,$28,$0C,$01,$02,$06),senddata : GOSUB Send    ' Initialise LCD/OLED
		'(WakeUp)*3(Set4Bit)(4Bit/2Line)(DisplayOn)(Clear Display)(Return Home)(Entry Mode Set)
	NEXT index : PAUSE 10
	senddata = 128 :           GOSUB Send  ' (128-147) Line 1 Cursor Position
	HIGH RS                                ' charactermode
	EEPROM 25,("OD600")
	FOR index = 25 to 29
		READ index, senddata : GOSUB Send
'		sertxd("char:", #senddata, 13, 10)
	NEXT index
	LOW  RS                                            ' commandmode
	PAUSE 1000

measurement:
	gosub collect_measurement
	sertxd("Value:", #Value)
	LOW  RS                ' commandmode
	senddata = 200 :       GOSUB Send              ' (192-211) Line 2 Cursor Position
	HIGH RS                ' charactermode
	BinToAscii Value,b8,b7,b6,b5,b4
	FOR  index = 0 TO 5                                                           ' sending characters
		LOOKUP index,(" ",b8,b7,b6,b5,b4),senddata :          GOSUB Send
	NEXT index
	IF Value < ZeroValue THEN
		Value = 0
	ELSE
		Value = Value - ZeroValue
	ENDIF
	if CALIB_BTN = 1 or REF_V = 0 then
		REF_V = Value
		sertxd(", Calibration:", #REF_V)
		BinToAscii REF_V,b8,b7,b6,b5,b4
		LOW  RS                ' commandmode
		senddata = 192 :       GOSUB Send              ' (192-211) Line 2 Cursor Position
		HIGH RS                ' charactermode
		FOR  index = 0 TO 5                                                           ' sending characters
			LOOKUP index,(" ",b8,b7,b6,b5,b4),senddata :          GOSUB Send
		NEXT index
	else
		Divisor = REF_V / 10
		sertxd(", ref_v:", #REF_V, ", Divisor:", #Divisor)
		Value = Value * 10 / Divisor
		BinToAscii Value,b8,b7,b6,b5,b4
		sertxd(", ODD:", b8, b7, b6, ".", b5, b4)
		LOW  RS                ' commandmode
		senddata = 137 :       GOSUB Send              ' (192-211) Line 2 Cursor Position
		HIGH RS                ' charactermode
		FOR  index = 0 TO 4                                                           ' sending characters
			LOOKUP index,(" ",b6,".",b5,b4),senddata :          GOSUB Send
		NEXT index
	endif
	sertxd(13, 10)
	PAUSE 1000
	goto measurement

collect_measurement:
	Value = 0
	HIGH OUT_LED
	PAUSE 200
	FOR index = 0 TO 7                  ' 8 times
		readadc10 INP_V, Holder
		PAUSE 10
		Value = Value + Holder
'		sertxd("collection value:", #Value, 13, 10)
	NEXT index
	LOW OUT_LED
	Value = Value / 8
	sertxd("final value:", #Value, 13, 10)
	return

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
