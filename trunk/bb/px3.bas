'using interrupts to transfer data


;Read a byte from PC kbd via a serial port.
;Raise line to high briefly before sending byte in TTL
;level to master PICAXE.

MainLoop:
  serin 2,N2400,b0      ;recv 1 byte from kbd
  high 4                ;set line high
  pause 20              ;allow master to respond to hi
  serout 4,T2400,(b0)   ;transmit char (T=True Output)
  low 4                 ;set line low
  goto MainLoop
