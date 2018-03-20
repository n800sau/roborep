
; *****************************************************************************
; *                                                                           *
; * Nokia LCD Driver for the PICAXE-20X2 (or bitbanged 14M2)   Version 007    *
; * most code from matherP: http://www.picaxeforum.co.uk/showthread.php?25360 *
; * following  hippy and srnet; modified by lbenson for 14M2 bitbanging       *
; * see modified I/O Definitions below
; *                                                                           *
; *****************************************************************************

#Picaxe   20M2
#Terminal 38400

; .---------------------------------------------------------------------------.
; |     Configuration                                                         |
; `---------------------------------------------------------------------------'
' lb pins 8-1 gnd, led, vcc, clk, din, dc, ce, rst


;                         PICAXE-20X2
;               +3V -.-  .-----_-----.                              Nokia
;                    `---| +3V    0V |--------------.                LCD
;         .------------->| SI     SO |---.          |            .----------.
;  POT >--|------------->| C.7   B.0 |---|----------|----- VDD --|  _____   |
;         |             -| C.6   B.1 |-  |          }----- GND --| |     |  |
;         |   .----------| C.5   B.2 |---|----------|----- SCE --| |     |  |
;         |   |         -| C.4   B.3 |---|----------|----- RST --| |     |  |
;         |   |         -| C.3   B.4 |---|----------|----- D/C --| |     |  |
;         |   |         -| C.2   B.5 |-  |       .--|----- DAT --| |     |  |
;         |   |      .---| C.1   B.6 |<--|---. .-|--|----- CLK --| |_____|  |
;         |   |   .--|---| C.0   B.7 |---|---|-' |  |  .-- LED --|          |
;         |   |   |  |   `-----------'   |   |   |  |  |         `----------'
;         |   |   |  `-------------------|---|---'  |  |
;         |   |   |             ____     |   |      |  |           PCD8544
;   TX <--|---|---^----|<|-----|____|----'   |      |  |
;         |   |       1N4148    180R         |      |  |      Note that the top
;   RX >--^---|------------------------------'      |  |       line is to the
;             |              ____                   |  |       right as shown
;   0V >--.   `-------------|____|------------------|--'      where the larger
;        _|_                 180R                  _|_         metal area is
;
;
; Notes : Power supply is 3V ( 2V7 to 3V3 ) - See PCD8544 datasheet and below
;         Do not use SERTXD commands in the code
;         Serial Download 10K/22K input interface not shown
;         Decoupling capacitors not shown
;         Requires Hard-Reset ( power cycling ) to download new programs
;         In-circuit download may not work with all PC's / download cables
;         Connect a potentiometer to POT for manual control of contrast etc

  ; ***************************************************************************
  ; *                                                                         *
  ; *   5V Operation                                                          *
  ; *                                                                         *
  ; ***************************************************************************

; Despite claims of 5V operation that is outside the PCD8544 specification of
; 2V7 to 3V3. At best 5V is the absolute maximum rating so operation at that
; voltage is not guaranted and straying above it may damage the controller and
; display.
;
; The LED's on most modules appear to be unbuffered, wired in parallel with no
; current limiting resistors. They will almost certainly release the magic
; smoke if connected to 5V directly and it is recommended to use a current
; limiting resistor when connected to 3V or driven from a PICAXE output pin
; such as here. The 180R resistor will limit current into a 1V Vforward LED
; to below 25mA which is below the PICAXE pin limit.
;
; If you want really bright backlight LED's then wire via a resistor directly
; to the power supply and reduce the resistor at your own risk. In the author's
; opinion, dimmer backlight levels look better on these displays as there can
; be considerable bleed from the side LED lighting points. It's unfortunate
; that it's not a true backlight fitted.

; ***************************************************************************
; *                                                                         *
; *   I/O Definitions                                                       *
; *                                                                         *
; ***************************************************************************

symbol VDD     =    B.1         ; Power for the LCD ( less than a mA )
symbol SCE     =    B.2         ; Module Select
symbol RST     =    B.3         ; Module Reset
symbol D_C     =    B.4         ; Module Data / Command Indicator
symbol CLK     =    B.5         ; SPI Clock to module
symbol DAT     =    C.1         ; SPI Data Out ' In

  ; ***************************************************************************
  ; *                                                                         *
  ; *   Graphic Display Definition                                            *
  ; *                                                                         *
  ; ***************************************************************************

symbol NUM_ROWS      = 6
symbol NUM_COLS      = 84

symbol MAX_ROWS      = NUM_ROWS - 1
symbol MAX_COLS      = NUM_COLS - 1
symbol EXTENDED_COMMAND = 0x21
symbol BASIC_COMMAND  = 0x20
' display modes
symbol BLANK = 0x08
symbol ALL_ON = 0x09
symbol NORMAL = 0x0C
symbol INVERSE = 0x0D
'
symbol CONTRAST = 0xB8 'adjust for individual display 0xB0-0xBF
symbol positive=0
symbol negative=1
symbol yes=1
symbol no=0
'
symbol f_scale100=0x02
symbol f_scale10=0x01
symbol f_err= f_scale100 | f_scale10
symbol f_signed=0x04
symbol f_rightjustify=0x08
symbol f_byteout=0x10
symbol f_default=0x00
symbol f_termination=0x80
symbol n_array = 0x30 'space for using bintoascii, reserve 7 bytes 
symbol s_array=0x40 'space for string arrays
symbol bitmap_buffer = 0x50 'start of buffer used for bitmap output
symbol t_array = 0x60

  ; ***************************************************************************
  ; *                                                                         *
  ; *   Variable Definitions                                                  *
  ; *                                                                         *
  ; ***************************************************************************
symbol sign =bit0
symbol signed = bit1
symbol rightjustify = bit2
symbol scale100= bit3
symbol scale10 = bit4
symbol byteout = bit5
symbol overflow=bit6
symbol termination=bit7
symbol spiData=b1 ' data to be bitbanged out, msb first, bit15-bit8
symbol cmd = b2
symbol width = b3
symbol format=b4 
symbol i=b5
symbol j=b6
symbol char=b7
symbol xpos=b8
symbol ypos=b9
symbol OutDat = w5 '-32768 to 32767
symbol ODl = b10 '-128 to 127
symbol ODh = b11
symbol word2 = w6 
symbol W2l = b12 
symbol W2h = b13
symbol wordsave=w7
symbol ochar = b16
symbol loopCtr = b17
symbol pointersave=b18
'
' *   Initialisation                                                        *
'
Start:
  dirsB = %11111111             
  low VDD
' lb  high BKL				;make sure power to display is off
'  SetFreq M64                 ; Run as fast as possible
  SetFreq M32                 ; Run as fast as possible
' lb  HSpiSetup SPIMODE00, SPIMedium  ; Setup SPI so we can send commands
  pause 4000			  ;let things settle
  high RST				   ;start with reset high as per datasheet
  high VDD                       ;turn on power to device
  low RST				   ;do the reset
  high RST                       ; and release
  cmd=EXTENDED_COMMAND:Gosub SendCommand
  cmd=0x05:Gosub SendCommand ; temp coefficient
  cmd=0x14:Gosub SendCommand ; bias 1:48
  cmd=BASIC_COMMAND: Gosub SendCommand
  cmd=NORMAL : Gosub SendCommand
  gosub ClearScreen
  gosub setcontrast
' lb  pwmout BKL, 199, 320
  OutDat=-1110
  width=5
'
Main:
 sertxd("Main",cr,lf)
  poke s_array,"F","o","r","m","a","t"," ","t","e","s","t",0
  xpos=0:  ypos=0:  gosub cursor
  bptr=s_array
'  width=12:format=f_rightjustify:gosub outstring
  width=12: gosub outstring
  pause 64000
  xpos=0:  ypos=1:  gosub cursor
  format= f_default: width=3:  gosub FormNum
  xpos=48:  ypos=1:  gosub cursor
  format= f_signed | f_rightjustify: width=3:  gosub FormNum
  xpos=0:  ypos=2:  gosub cursor
  format = f_scale10:  width=6:  gosub FormNum
  xpos=42:  ypos=2:  gosub cursor
  format=f_signed | f_scale10 | f_rightjustify: width=7:  gosub FormNum
  xpos=0:  ypos=3:  gosub cursor
  format=f_scale100 :  width=7:  gosub FormNum
  xpos=36:  ypos=3:  gosub cursor
  format=f_scale100 | f_rightjustify | f_signed:  width=8:  gosub FormNum
  xpos=0:  ypos=4:  gosub cursor
  format= f_scale10 | f_scale100: width=5:  gosub FormNum
  xpos=42:  ypos=4:  gosub cursor
  format=f_byteout |F_rightjustify:  width=7:  gosub FormNum
  xpos=0:  ypos=5:  gosub cursor
  width=84:gosub baroutput
  OutDat=OutDat+3
  goto main
end

outchar:
 sertxd(@bptr)
  pointersave=bptr
  gosub readtabledata
  bptr=pointersave
return
; .---------------------------------------------------------------------------.
; |     SPI Transmit Routines                                                 |
; `---------------------------------------------------------------------------'

SendCommand:
  low D_C                   ; Send this byte as a command
  low SCE
'lb  HSpiout(cmd)
  bptr = 2                ; lb point to b2 for byte to be sent (cmd)
  gosub sendSPI           ; lb
  high SCE
Return
'
SendData:
  high D_C                   ; Send this byte as a command
  low SCE
'lb  HSpiout(@bptrinc)
  gosub sendSPI           ; lb
  high SCE
  inc xpos
Return
'
SetContrast:
  cmd=EXTENDED_COMMAND:Gosub SendCommand
  cmd=CONTRAST | 0x80:Gosub SendCommand 
  cmd=BASIC_COMMAND: Gosub SendCommand
return
'
sendchar:
  bptr=bitmap_buffer
  high D_C                   ; Send this byte as a command
  low SCE
'lb  HSpiout(@bptrinc)
'lb  HSpiout(@bptrinc)
'lb  HSpiout(@bptrinc)
'lb  HSpiout(@bptrinc)
'lb  HSpiout(@bptrinc)
'lb  HSpiout(@bptr)
  for loopCtr = 1 to 6
    gosub sendSPI           ; lb
  next loopCtr
  high SCE
  xpos=xpos+6
return
'
Cursor:
  cmd=xpos | 0x80:Gosub SendCommand
  cmd=ypos | 0x40:Gosub SendCommand
return
'
ClearScreen:
  gosub Home
  b0=0
  for i=0 to max_rows
    for j=0 to max_cols
      bptr=0
      Gosub SendData
    next j
  next i
'
Home:
  cmd= 0x80:Gosub SendCommand
  cmd= 0x40:Gosub SendCommand
  xpos=0:ypos=0
return
'
readtabledata:
   bptr=bitmap_buffer
   if char<0x20 or char>0x7F then
       @bptrinc=0:@bptrinc=0:@bptrinc=0:@bptrinc=0:@bptrinc=0:@bptr=0
   else
       ochar=char-0x20
       if ochar<=0x2F then
         ochar=ochar*5
         readtable ochar,@bptrinc
         inc ochar
         readtable ochar,@bptrinc
         inc ochar
         readtable ochar,@bptrinc
         inc ochar
         readtable ochar,@bptrinc
         inc ochar
         readtable ochar,@bptrinc
         @bptr=0
       else
         ochar=ochar-0x30
         ochar=ochar*5
         read ochar,@bptrinc
         inc ochar
         read ochar,@bptrinc
         inc ochar
         read ochar,@bptrinc
         inc ochar
         read ochar,@bptrinc
         inc ochar
         read ochar,@bptrinc
         @bptr=0
       endif
   endif
   gosub sendchar
return

'
FormNum:
  bptr = n_array
  b0=0 ' clear the status bits
  wordsave=OutDat
  sign=positive
  i= format & f_signed: if i<>0 then:signed=yes:endif
  i= format & f_byteout: if i<>0 then:byteout=yes:endif
  if byteout=yes then 'convert a byte input to a word copying sign bit if required
    OutDat=OutDat & 0xFF
    if signed = yes and OutDat>=$80 then
      OutDat =OutDat | 0xFF00
    endif
  endif
  If OutDat >= $8000 and signed=yes Then
    sign= negative
    OutDat = -OutDat
  endif
  i=format & f_err
  if i=f_err then  'check for conflicting formats
    @bptrinc="F":@bptrinc="_":@bptrinc="E":@bptrinc="r":@bptrinc="r":@bptr=0
  else 'process if formatting commands make sense
    if i=f_scale100 then :scale100=yes:endif
    if i=f_scale10 then :scale10=yes:endif
'    bintoascii OutDat,@bptrinc,@bptrinc,@bptrinc,@bptrinc,@bptr 'do the basic conversion
      @bptrinc = OutDat / 10000       + "0"
      @bptrinc = OutDat / 1000  // 10 + "0"
      @bptrinc = OutDat / 100   // 10 + "0"
      @bptrinc = OutDat / 10    // 10 + "0"
      @bptrinc = OutDat         // 10 + "0"

    W2l=@bptr
    if scale10 = yes then : @bptrinc="." :endif 'insert decimal point if scale10
    if scale100 = yes then 'insert decimal point if scale100
      W2l=@bptrdec
      W2h=@bptr
      @bptrinc="."
      @bptrinc=W2h
      @bptr=W2l
    endif
    @bptrinc=W2l
    @bptr=0 'denotes end of string with a null
    W2l=0
    bptr = n_array
    if @bptrinc="0" then ' at least one leading zero
	W2l=1
	if @bptrinc="0" then ' at least two leading zeros
   	  W2l=2
	  if @bptrinc="0" then ' at least three leading zeros
	    W2l=3
	  endif
	endif
    endif
    ODl=n_array+W2l 'move array left by the number of leading zeroes
    ODh=n_array+6
    W2h=n_array
    for i= ODl to ODh 
      bptr=i
	W2l=@bptr
	bptr=W2h
	@bptr=W2l
	W2h=W2h+1
    next i
    if signed = yes then 'move array right 1 space to allow for sign character if required
      ODl=n_array
      ODh=n_array+6
      for i= ODh to ODl step -1 
        bptr=i
        W2l=@bptrinc
        @bptr=W2l
      next i
      bptr=n_array 'insert the sign bit at the beginning of the string
      if sign=negative then
        @bptr="-"		
      else
        @bptr=" "		
      endif
    endif
    OutDat=wordsave
    bptr = n_array
    if scale10=no and @bptr="0" then 'special case of unsigned positive numbers less than 10
	inc bptr
	W2l=@bptr
	@bptrdec=0
      @bptr=W2l
    endif
    if signed=yes and scale10=no then 'special case of signed positive numbers between -10 and 10
      bptr=n_array+1
      if @bptr="0" then
	  W2l=bptr+1
	  peek W2l,W2h
        @bptrinc=W2h
        @bptr=0
      endif
    endif
    if scale100=yes and OutDat<100 and signed=no then  'special case of unsigned 2-decimal positive numbers less than 100
      bptr=n_array+1
      W2l=@bptrinc
      W2h=@bptr
      bptr=n_array
      @bptrinc="0":@bptrinc=".":@bptrinc=W2l:@bptrinc=W2h:@bptr=0
    endif
    if scale100=yes and signed=yes then 'special case of signed 2-decimal positive numbers between -100 and 100
      if OutDat>65436 or OutDat<100 then
        bptr=n_array+2
        W2l=@bptrinc
        W2h=@bptr
        bptr=n_array
        if sign=negative then
          @bptrinc="-"		
        else
          @bptrinc=" "		
        endif
        @bptrinc="0":@bptrinc=".":@bptrinc=W2l:@bptrinc=W2h:@bptr=0
      endif
    endif
  endif
  bptr = n_array  'reset bptr ready for output

'outputs a zero terminated string pointed to by bptr   
outstring: 
 sertxd("outstring ",@bptr,cr,lf)
  i= format & f_rightjustify: if i<>0 then:rightjustify=yes:endif
  i= format & f_termination: if i<>0 then:termination=yes:endif
  W2l=0
  do while @bptrinc<>0 'count the valid characters
    inc W2l
  loop 
  if W2l>width then: overflow=yes: W2l=width: endif 'too big so replace with * characters 
  bptr=bptr-W2l-1
  if rightjustify=yes and W2l<>width then 'if right justified pad out beginning of the string with spaces
    char=" "
    W2l=W2l+1
    for W2h=width to W2l step -1
   	gosub outchar
    next W2h
    W2l=W2l-1
  endif
  for W2h=1 to W2l
    if overflow=no then: char=@bptrinc :else :char="*":endif
   	gosub outchar
  next W2h
  if rightjustify=no and W2l<>width then 'if left justified pad out end of the string with spaces
    char=" "
    W2l=W2l+1
    for W2h=width to W2l step -1
   	gosub outchar
    next W2h
  endif
  if termination=yes then
    bptr=t_array
    do 
      char=@bptrinc
      gosub outchar
    loop while @bptr<>0
  endif 
return
'
' outputs a barchart from the cursor position of size "width" and filled as per ODl (0 to 255)
'
baroutput:
  i=max_cols-xpos ' this is the length of the barchart
  if i>width then:i=width:endif
  word2=i * ODl
  bptr=bitmap_buffer
  @bptr=0xFF
  for i=0 to W2h 'high byte of word2 implies divide by 256
    gosub senddata
    dec bptr
    j=i+2:if j=width then:@bptr=0xFF: else: @bptr=%10111101:endif 'set the full end if all columns full
  next i 
  W2h=W2h+1
  if W2h<>width then
    W2l=width-1
    for i=W2h to W2l
      j=i+1:if j=width then:@bptr=0xFF: else: @bptr=%10000001:endif 'set the full end if all columns full
      gosub senddata
      dec bptr
    next i 
  endif 
return

sendSPI:
' ***** Shiftout MSB first *****
shiftout_MSBFirst:
'  if cmd < BASIC_COMMAND then : sertxd(@bptr) : endif
  low clk
  spiData = @bptrinc
  if bit15 = 1 then high DAT : else low DAT : endif
  pulsout clk,8 ' ? pulse clock for 10us @ 32mHz
  if bit14 = 1 then high DAT : else low DAT : endif
  pulsout clk,8 ' pulse clock for 10us
  if bit13 = 1 then high DAT : else low DAT : endif
  pulsout clk,8 ' pulse clock for 10us
  if bit12 = 1 then high DAT : else low DAT : endif
  pulsout clk,8 ' pulse clock for 10us
  if bit11 = 1 then high DAT : else low DAT : endif
  pulsout clk,8 ' pulse clock for 10us
  if bit10 = 1 then high DAT : else low DAT : endif
  pulsout clk,8 ' pulse clock for 10us
  if bit9 = 1 then high DAT : else low DAT : endif
  pulsout clk,8  ' pulse clock for 10us
  if bit8 = 1 then high DAT : else low DAT : endif
  pulsout clk,8 ' pulse clock for 10us
	  
  return

  ; ***************************************************************************
  ; *                                                                         *
  ; *   Font Tables                                                           *
  ; *                                                                         *
  ; ***************************************************************************

; We accept full 8-bit data but use the msb to indicate if the character is
; shown normally or double-width so we only have a 7-bit character code and of
; that only $20-$7F represents printable ASCII characters. We split those into
; two sets of $20-$4F and $50-$7F which gives two font tables each holding 3x16
; characters; 48 characters per table.
;
; We use a 5x7 font so there are 5 bytes per character so we require 5x48
; bytes per font table; 240 bytes which can be held in Eeprom and Table
; respectively. We don't have any need to use external I2C Eeprom of LookUp
; tables unless we extend the font set beyond this and this gives us the
; highest access speed to font data.
;
; We place the lowercase letters within Eeprom so we could provide a means of
; replacing them over serial if required ( to change descenders etc ).
;
; We have 16 bytes of Eeprom left at the end, conveniently at $F0-$FF which
; can be used however we wish. This area is used to hold configuration data
; which means we can change that data via serial without downloading a new
; program if required. These Eeprom entries are placed in sections of the
; source code which use them.

; .---------------------------------------------------------------------------.
; |     Font Table for Characters $20 - $4F                                   |
; `---------------------------------------------------------------------------'

  Table  $00, ( $00,$00,$00,$00,$00 )   ; $20 - Space
  Table  $05, ( $00,$00,$2F,$00,$00 )   ; $21 - Exclamation mark
  Table  $0A, ( $00,$07,$00,$07,$00 )   ; $22 - Double quote
  Table  $0F, ( $14,$7F,$14,$7F,$14 )   ; $23 - Hash
  Table  $14, ( $24,$2A,$7F,$2A,$12 )   ; $24 - Dollar sign
  Table  $19, ( $C4,$C8,$10,$26,$46 )   ; $25 - Percent
  Table  $1E, ( $36,$49,$55,$22,$50 )   ; $26 - Ampersand
  Table  $23, ( $00,$05,$03,$00,$00 )   ; $27 - Single quote
  Table  $28, ( $00,$1C,$22,$41,$00 )   ; $28 - (
  Table  $2D, ( $00,$41,$22,$1C,$00 )   ; $29 - )
  Table  $32, ( $14,$08,$3E,$08,$14 )   ; $2A - Star
  Table  $37, ( $08,$08,$3E,$08,$08 )   ; $2B - Plus
  Table  $3C, ( $00,$00,$50,$30,$00 )   ; $2C - Comma
  Table  $41, ( $10,$10,$10,$10,$10 )   ; $2D - Minus
  Table  $46, ( $00,$60,$60,$00,$00 )   ; $2E - Period / Full Stop
  Table  $4B, ( $20,$10,$08,$04,$02 )   ; $2F - /
  Table  $50, ( $3E,$51,$49,$45,$3E )   ; $30 - 0
  Table  $55, ( $00,$42,$7F,$40,$00 )   ; $31 - 1
  Table  $5A, ( $42,$61,$51,$49,$46 )   ; $32 - 2
  Table  $5F, ( $21,$41,$45,$4B,$31 )   ; $33 - 3
  Table  $64, ( $18,$14,$12,$7F,$10 )   ; $34 - 4
  Table  $69, ( $27,$45,$45,$45,$39 )   ; $35 - 5
  Table  $6E, ( $3C,$4A,$49,$49,$30 )   ; $36 - 6
  Table  $73, ( $01,$71,$09,$05,$03 )   ; $37 - 7
  Table  $78, ( $36,$49,$49,$49,$36 )   ; $38 - 8
  Table  $7D, ( $06,$49,$49,$29,$1E )   ; $39 - 9
  Table  $82, ( $00,$36,$36,$00,$00 )   ; $3A - Colon
  Table  $87, ( $00,$56,$36,$00,$00 )   ; $3B - Semicolon
  Table  $8C, ( $08,$14,$22,$41,$00 )   ; $3C - <
  Table  $91, ( $14,$14,$14,$14,$14 )   ; $3D - =
  Table  $96, ( $00,$41,$22,$14,$08 )   ; $3E - >
  Table  $9B, ( $02,$01,$51,$09,$06 )   ; $3F - ?
  Table  $A0, ( $32,$49,$59,$51,$3E )   ; $40 - @
  Table  $A5, ( $7E,$11,$11,$11,$7E )   ; $41 - A
  Table  $AA, ( $7F,$49,$49,$49,$36 )   ; $42 - B
  Table  $AF, ( $3E,$41,$41,$41,$22 )   ; $43 - C
  Table  $B4, ( $7F,$41,$41,$22,$1C )   ; $44 - D
  Table  $B9, ( $7F,$49,$49,$49,$41 )   ; $45 - E
  Table  $BE, ( $7F,$09,$09,$09,$01 )   ; $46 - F
  Table  $C3, ( $3E,$41,$49,$49,$7A )   ; $47 - G
  Table  $C8, ( $7F,$08,$08,$08,$7F )   ; $48 - H
  Table  $CD, ( $00,$41,$7F,$41,$00 )   ; $49 - I
  Table  $D2, ( $20,$40,$41,$3F,$01 )   ; $4A - J
  Table  $D7, ( $7F,$08,$14,$22,$41 )   ; $4B - K
  Table  $DC, ( $7F,$40,$40,$40,$40 )   ; $4C - L
  Table  $E1, ( $7F,$02,$0C,$02,$7F )   ; $4D - M
  Table  $E6, ( $7F,$04,$08,$10,$7F )   ; $4E - N
  Table  $EB, ( $3E,$41,$41,$41,$3E )   ; $4F - O

; .---------------------------------------------------------------------------.
; |     Font Table for Vertical Bar Graphs                                    |
; `---------------------------------------------------------------------------'

  ' Bits set :   0    1    2    3    4    5    6    7

  Table  $F0, ( $00, $80, $C0, $E0, $F0, $F8, $FC, $FE )
  table   $F8, ("Hello")

; .---------------------------------------------------------------------------.
; |     Font Table for Charcters $50 - $7F                                    |
; `---------------------------------------------------------------------------'

  Eeprom $00, ( $7F,$09,$09,$09,$06 )   ; $50 - P
  Eeprom $05, ( $3E,$41,$51,$21,$5E )   ; $51 - Q
  Eeprom $0A, ( $7F,$09,$19,$29,$46 )   ; $52 - R
  Eeprom $0F, ( $46,$49,$49,$49,$31 )   ; $53 - S
  Eeprom $14, ( $01,$01,$7F,$01,$01 )   ; $54 - T
  Eeprom $19, ( $3F,$40,$40,$40,$3F )   ; $55 - U
  Eeprom $1E, ( $1F,$20,$40,$20,$1F )   ; $56 - V
  Eeprom $23, ( $3F,$40,$38,$40,$3F )   ; $57 - W
  Eeprom $28, ( $63,$14,$08,$14,$63 )   ; $58 - X
  Eeprom $2D, ( $07,$08,$70,$08,$07 )   ; $59 - Y
  Eeprom $32, ( $61,$51,$49,$45,$43 )   ; $5A - Z
  Eeprom $37, ( $00,$7F,$41,$41,$00 )   ; $5B - [
  Eeprom $3C, ( $02,$04,$08,$10,$20 )   ; $5C - \
  Eeprom $41, ( $00,$41,$41,$7F,$00 )   ; $5D - ]
  Eeprom $46, ( $04,$02,$01,$02,$04 )   ; $5E - Up arrow
  Eeprom $4B, ( $40,$40,$40,$40,$40 )   ; $5F - Underscore
  Eeprom $50, ( $00,$01,$02,$04,$00 )   ; $60 - Back tick
  Eeprom $55, ( $20,$54,$54,$54,$78 )   ; $61 - a
  Eeprom $5A, ( $7F,$48,$44,$44,$38 )   ; $62 - b
  Eeprom $5F, ( $38,$44,$44,$44,$20 )   ; $63 - c
  Eeprom $64, ( $38,$44,$44,$48,$7F )   ; $64 - d
  Eeprom $69, ( $38,$54,$54,$54,$18 )   ; $65 - e
  Eeprom $6E, ( $08,$7E,$09,$01,$02 )   ; $66 - f
  Eeprom $73, ( $0C,$52,$52,$52,$3E )   ; $67 - g
  Eeprom $78, ( $7F,$08,$04,$04,$78 )   ; $68 - h
  Eeprom $7D, ( $00,$44,$7D,$40,$00 )   ; $69 - i
  Eeprom $82, ( $20,$40,$44,$3D,$00 )   ; $6A - j
  Eeprom $87, ( $7F,$10,$28,$44,$00 )   ; $6B - k
  Eeprom $8C, ( $00,$41,$7F,$40,$00 )   ; $6C - l
  Eeprom $91, ( $7C,$04,$18,$04,$78 )   ; $6D - m
  Eeprom $96, ( $7C,$08,$04,$04,$78 )   ; $6E - n
  Eeprom $9B, ( $38,$44,$44,$44,$38 )   ; $6F - o
  Eeprom $A0, ( $7C,$14,$14,$14,$08 )   ; $70 - p
  Eeprom $A5, ( $08,$14,$14,$18,$7C )   ; $71 - q
  Eeprom $AA, ( $7C,$08,$04,$04,$08 )   ; $72 - r
  Eeprom $AF, ( $48,$54,$54,$54,$20 )   ; $73 - s
  Eeprom $B4, ( $04,$3F,$44,$40,$20 )   ; $74 - t
  Eeprom $B9, ( $3C,$40,$40,$20,$7C )   ; $75 - u
  Eeprom $BE, ( $1C,$20,$40,$20,$1C )   ; $76 - v
  Eeprom $C3, ( $3C,$40,$30,$40,$3C )   ; $77 - w
  Eeprom $C8, ( $44,$28,$10,$28,$44 )   ; $78 - x
  Eeprom $CD, ( $0C,$50,$50,$50,$3C )   ; $79 - y
  Eeprom $D2, ( $44,$64,$54,$4C,$44 )   ; $7A - z
  Eeprom $D7, ( $00,$08,$36,$41,$00 )   ; $7B - {
  Eeprom $DC, ( $00,$00,$7F,$00,$00 )   ; $7C - Vertical bar - See note below
  Eeprom $E1, ( $00,$41,$36,$08,$00 )   ; $7D - }
  Eeprom $E6, ( $04,$02,$04,$08,$04 )   ; $7E - Tilde
  Eeprom $EB, ( $1C,$12,$11,$12,$1C )   ; $7F - Triangle

