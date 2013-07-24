#!/usr/bin/env gnuplot

wc=system("wc -l data.txt|cut -f1 -d ' '")+0
fl=wc-10
if (fl < 1) fl=1
set terminal dumb
set autoscale
set datafile separator "\t"
set xdata time
set timefmt "%s" 
#set xrange[-100:*]
set timefmt "%d/%m/%Y %H:%M:%S"
set format x "%H:%M:%S"
#set yrange[0:10]
plot "data.txt" every ::fl::wc using 1:2  with lines title "Example"
#replot
#pause -5
#reread

