CC=g++
CCOPTS=-L.
STRIP=-strip
SRCS=RF24Network.cpp Sync.cpp
OBJECTS=RF24Network.o Sync.o
HEADERS=RF24Network.h RF24Network_config.h Sync.h
NAME=libRF24Network.a

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

RF24Network.o: RF24Network_config.h /usr/include/stdint.h
RF24Network.o: /usr/include/features.h /usr/include/stdio.h
RF24Network.o: /usr/include/libio.h /usr/include/_G_config.h
RF24Network.o: /usr/include/wchar.h /usr/include/string.h
RF24Network.o: /usr/include/xlocale.h ../RF24/RF24.h ../RF24/RF24_config.h
RF24Network.o: ../spi/spi.h /usr/include/unistd.h /usr/include/getopt.h
RF24Network.o: /usr/include/stdlib.h /usr/include/alloca.h
RF24Network.o: /usr/include/fcntl.h /usr/include/time.h
RF24Network.o: /usr/include/inttypes.h /usr/include/linux/types.h
RF24Network.o: /usr/include/linux/posix_types.h /usr/include/linux/stddef.h
RF24Network.o: /usr/include/linux/spi/spidev.h RF24Network.h
Sync.o: /usr/include/stdlib.h /usr/include/features.h /usr/include/alloca.h
Sync.o: RF24Network.h /usr/include/stdint.h Sync.h /usr/include/string.h
Sync.o: /usr/include/xlocale.h RF24Network_config.h /usr/include/stdio.h
Sync.o: /usr/include/libio.h /usr/include/_G_config.h /usr/include/wchar.h
