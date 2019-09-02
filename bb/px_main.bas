;Continuously send a loop count to PC and flash an LED. When a byte is
;sent from slave PICAXE, an interrupt is generated. The byte is
;read and then sent to PC before master resumes its main loop.

setint %00000100,%00000100    ;generate interrupt on pin 2 high
b1=0                          ;init counter

MainLoop:
  serout 4,N2400,("Loop ",#b1,$0d,$0a)   ;continuously output loop count
  b1=b1+1                       ;inc counter
  high 1                        ;turn LED on
  pause 100                     ;sleep
  low 1                         ;turn LED off
  pause 100                     ;sleep
  goto MainLoop

Interrupt:                      ;high detected on pin 2
  serin 2,T2400,b0              ;receive byte (T=True input)
  serout 4,N2400,("Received ",b0,13,10)	   ;output byte
  setint %00000100,%00000100    ;setup interrupt again
  return
