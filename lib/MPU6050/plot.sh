#!/usr/bin/env gnuplot

set terminal dumb
set datafile separator "\t"
set xdata time
set timefmt "%d/%m/%Y %H:%M:%S"
set format x "%H:%M:%S"
#set xrange[2000:3000]
#set yrange[0:10]
plot "data.txt" every ::1 using 1:2  with lines title "Example"

