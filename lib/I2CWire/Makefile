I2CDEV=/dev/i2c-1


CC=g++
CCOPTS=-L. -DI2CDEV=\"$(I2CDEV)\"
STRIP=-strip
SRCS=I2CWire.cpp
OBJECTS=I2CWire.o
HEADERS=I2CWire.h
NAME=libi2cwire.a


.cpp.o: $(HEADERS)
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

I2CWire.o: /usr/include/stdio.h /usr/include/features.h /usr/include/libio.h
I2CWire.o: /usr/include/_G_config.h /usr/include/wchar.h
I2CWire.o: /usr/include/stdlib.h /usr/include/alloca.h /usr/include/fcntl.h
I2CWire.o: /usr/include/time.h /usr/include/math.h /usr/include/unistd.h
I2CWire.o: /usr/include/getopt.h /usr/include/string.h /usr/include/xlocale.h
I2CWire.o: /usr/include/linux/i2c-dev.h /usr/include/linux/types.h
I2CWire.o: /usr/include/linux/posix_types.h /usr/include/linux/stddef.h
I2CWire.o: I2CWire.h /usr/include/stdint.h
