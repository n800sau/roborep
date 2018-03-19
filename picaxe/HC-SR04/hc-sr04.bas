symbol trig = 4 ' Define output pin for Trigger pulse C.4
symbol echo = 3 ' Define input pin for Echo pulse C.3
symbol range = w1 ' 16 bit word variable for range
symbol serialpin = c.1


hsersetup B4800_4, %00

main:

pulsout trig,2 ' produce 20uS trigger pulse (must be minimum of 10uS)

pulsin echo,1,range ' measures the range in 10uS steps

pause 10 ' recharge period after ranging completes

' now convert range to cm (divide by 5.8) or inches (divide by 14.8)
'as picaxe cannot use 5.8, multiply by 10 then divide by 58 instead

let range = range * 10 / 58 ' multiply by 10 then divide by 58

hserout 0, ("Range:",#range,13,10)

'debug range 'display range via debug command
'sertxd("Range:",#range,13,10)


' output for standard serial
serout serialpin, T4800_4, ("Range:",#range,13,10)

goto main 'and around for
