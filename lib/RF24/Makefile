CC=g++
CCOPTS=-L.
STRIP=-strip
SRCS=RF24.cpp
OBJECTS=RF24.o
HEADERS=RF24.h RF24_config.h nRF24L01.h
NAME=libRF24.a

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

RF24.o: nRF24L01.h RF24_config.h ../spi/spi.h /usr/include/stdint.h
RF24.o: /usr/include/features.h /usr/include/unistd.h /usr/include/getopt.h
RF24.o: /usr/include/stdio.h /usr/include/libio.h /usr/include/_G_config.h
RF24.o: /usr/include/wchar.h /usr/include/stdlib.h /usr/include/alloca.h
RF24.o: /usr/include/fcntl.h /usr/include/time.h /usr/include/inttypes.h
RF24.o: /usr/include/linux/types.h /usr/include/linux/posix_types.h
RF24.o: /usr/include/linux/stddef.h /usr/include/linux/spi/spidev.h
RF24.o: ../gpio/gpio.h ../compatibility/compatibility.h /usr/include/string.h
RF24.o: /usr/include/xlocale.h RF24.h
