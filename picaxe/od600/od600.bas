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

DisplayHello:
	LOW  RS                                ' commandmode
	senddata = 132 :           GOSUB Send  ' (128-147) Line 1 Cursor Position
	HIGH RS                                ' charactermode
	senddata = "H" :           GOSUB Send
	senddata = "e" :           GOSUB Send
	senddata = "l" :           GOSUB Send
	senddata = "l" :           GOSUB Send
	senddata = "o" :           GOSUB Send
	pause 10000
	LOW  RS                                            ' commandmode
	senddata = $01 :           GOSUB Send : PAUSE 10   ' Clear Display
	senddata = 192 :           GOSUB Send              ' (192-211) Line 2 Cursor Position
	HIGH RS                                             ' charactermode
	FOR  index = 25 to 34
	READ index, senddata :      GOSUB Send              ' sending character
	NEXT index
	pause 4000

measurement:
	HIGH OUT_LED
	PAUSE 200                                                                    ' 1 second (M32)
	readadc10 INP_V,w8
	LOW OUT_LED
	sertxd("v:", #w8, 13, 10)
	if CALIB_BTN = 1 or REF_V = 0 then
		REF_V = w8
		BinToAscii w8,b8,b7,b6,b5,b4
		LOW  RS                ' commandmode
		senddata = 192 :       GOSUB Send              ' (192-211) Line 2 Cursor Position
		HIGH RS                ' charactermode
		FOR  index = 0 TO 5                                                           ' sending characters
			LOOKUP index,(" ",b8,b7,b6,b5,b4),senddata :          GOSUB Send
		NEXT index
	else
		w9 = 100 * w8 / REF_V
		BinToAscii w9,b8,b7,b6,b5,b4
		LOW  RS                ' commandmode
		senddata = 202 :       GOSUB Send              ' (192-211) Line 2 Cursor Position
		HIGH RS                ' charactermode
		FOR  index = 0 TO 5                                                           ' sending characters
			LOOKUP index,(" ",b8,b7,".",b6,b5,b4),senddata :          GOSUB Send
		NEXT index
	endif
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
