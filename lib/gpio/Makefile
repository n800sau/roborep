CC=g++
CCOPTS=-L.
STRIP=-strip
SRCS=gpio.cpp event_gpio.c
OBJECTS=gpio.o event_gpio.o
HEADERS=gpio.h event_gpio.h
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

gpio.o: gpio.h /usr/include/errno.h /usr/include/features.h
gpio.o: /usr/include/string.h /usr/include/xlocale.h /usr/include/poll.h
gpio.o: /usr/include/fcntl.h /usr/include/time.h /usr/include/syslog.h
event_gpio.o: /usr/include/pthread.h /usr/include/features.h
event_gpio.o: /usr/include/endian.h /usr/include/sched.h /usr/include/time.h
event_gpio.o: /usr/include/stdio.h /usr/include/libio.h
event_gpio.o: /usr/include/_G_config.h /usr/include/wchar.h
event_gpio.o: /usr/include/stdlib.h /usr/include/alloca.h
event_gpio.o: /usr/include/fcntl.h /usr/include/unistd.h
event_gpio.o: /usr/include/getopt.h /usr/include/string.h
event_gpio.o: /usr/include/xlocale.h event_gpio.h
