CC=g++
CCOPTS=-L.
STRIP=-strip
SRCS=spi.cpp
OBJECTS=spi.o
HEADERS=spi.h
NAME=libspi.a

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

spi.o: spi.h /usr/include/stdint.h /usr/include/features.h
spi.o: /usr/include/unistd.h /usr/include/getopt.h /usr/include/stdio.h
spi.o: /usr/include/libio.h /usr/include/_G_config.h /usr/include/wchar.h
spi.o: /usr/include/stdlib.h /usr/include/alloca.h /usr/include/fcntl.h
spi.o: /usr/include/time.h /usr/include/inttypes.h /usr/include/linux/types.h
spi.o: /usr/include/linux/posix_types.h /usr/include/linux/stddef.h
spi.o: /usr/include/linux/spi/spidev.h
