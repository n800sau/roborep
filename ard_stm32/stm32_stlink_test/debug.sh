#pio debug -e stm32 --interface=gdb -- -x gdbinit

pio debug -e stm32debug --interface=gdb -- -x .pioinit -x .gdbinit
