; I2C Address Scanner for Picaxe 14M2
#no_data
#terminal 4800
;
; I2C register
;
symbol ssp1buf = $91                            ;14M2

pause 5000                                ;chance to start logging.
sertxd (cr,lf,"START PROGRAM")

for b2 = %00000010 to %11111110 step 2    ;addresses are shifted left one bit, as bit 0 is R/W flag.

	hi2csetup i2cmaster,b2,i2cslow,i2cbyte ; Set up I2c. Enable SDA/SCL, set BRG divisor.

	hi2cout (0)                         ;Just send a 0 as data (this can't be confused with a valid i2c address)
	;
	; If the address setup sequence is NAK'd, then hi2cout will abort and not send the data byte(0)...
	; ...SSP1BUF always contains the last data byte sent. If no data was sent, SSP1BUF will still contain the address.
	;
	peeksfr ssp1buf,b0                  ;so, what was that last byte?
	if b0 = 0 then                      ;if it's zero, it must be data - so the address must have been ACK'd.
		b0 = b2 / 2                   ;convert address used to 'normal' i2c address

		sertxd (cr,lf,"** i2c response @ [",#bit7,#bit6,#bit5,#bit4,"],[",#bit3,#bit2,#bit1,#bit0,"] 0x")
		;
		; Print in hex too
		;
		b1 = b0 / 16 + "0"            ;get top  nybble, convert to ASCII
		if b1 > "9" then
		      b1 = b1 + 7             ;convert 9-15 into A-F
		endif
		sertxd(b1)                    ;print it

		b1 = b0 & %00001111 + "0"     ;get bottom nybble, convert to ASCII
		if b1 > "9" then
		      b1 = b1 + 7             ;convert 9-15 into A-F
		endif
		sertxd(b1)                    ;print that too.
	else
		if b0 <> b2 then
		      sertxd (cr,lf,"Um...something unexpected happened!",#b2) ; this never appears ;-)
		endif
	endif
next
sertxd (cr,lf,"END PROGRAM")
end
