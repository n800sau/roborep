CC=g++
CCOPTS=-L. -g
STRIP=-strip
SRCS=BeagleBone_gpio.c BeagleBone_hd44780.c
OBJECTS=BeagleBone_gpio.o BeagleBone_hd44780.o
NAME=BeagleBone_IO_lib.a

.cpp.o:
		$(CC) $(CCOPTS) -c $<

all: $(NAME)

clean:
	rm -f $(OBJECTS) $(NAME)

$(NAME): $(OBJECTS) $(HEADERS)
	$(AR) r $(NAME) $(OBJECTS)
	ranlib $@

# what are the source dependencies
depend: $(SRCS)
	makedepend $(CCOPTS) $(SRCS)
# DO NOT DELETE

BeagleBone_gpio.o: BeagleBone_gpio.h /usr/include/stdio.h
BeagleBone_gpio.o: /usr/include/features.h /usr/include/libio.h
BeagleBone_gpio.o: /usr/include/_G_config.h /usr/include/wchar.h
BeagleBone_gpio.o: /usr/include/stdlib.h /usr/include/alloca.h
BeagleBone_gpio.o: /usr/include/unistd.h /usr/include/getopt.h
BeagleBone_gpio.o: /usr/include/string.h /usr/include/xlocale.h
BeagleBone_gpio.o: /usr/include/assert.h /usr/include/time.h
BeagleBone_hd44780.o: BeagleBone_hd44780.h /usr/include/stdio.h
BeagleBone_hd44780.o: /usr/include/features.h /usr/include/libio.h
BeagleBone_hd44780.o: /usr/include/_G_config.h /usr/include/wchar.h
BeagleBone_hd44780.o: /usr/include/stdlib.h /usr/include/alloca.h
BeagleBone_hd44780.o: /usr/include/unistd.h /usr/include/getopt.h
BeagleBone_hd44780.o: /usr/include/string.h /usr/include/xlocale.h
BeagleBone_hd44780.o: /usr/include/assert.h /usr/include/time.h
BeagleBone_hd44780.o: BeagleBone_gpio.h
