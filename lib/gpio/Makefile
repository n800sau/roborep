CC=g++
CCOPTS=-L.
STRIP=-strip
SRCS=gpio.cpp
OBJECTS=gpio.o
HEADERS=gpio.h
NAME=libgpio.a

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

gpio.o: gpio.h /usr/include/stdio.h /usr/include/features.h
gpio.o: /usr/include/libio.h /usr/include/_G_config.h /usr/include/wchar.h
