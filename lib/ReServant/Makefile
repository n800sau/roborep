CC=g++ -I/usr/include/hiredis
CCOPTS=-L.
STRIP=-strip
SRCS=reservant.cpp
OBJECTS=reservant.o
HEADERS=reservant.h
NAME=libreservant.a

.cpp.o: $(HEADERS) %.cpp %.h
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

reservant.o: reservant.h /usr/include/jansson.h /usr/include/stdio.h
reservant.o: /usr/include/features.h /usr/include/libio.h
reservant.o: /usr/include/_G_config.h /usr/include/wchar.h
reservant.o: /usr/include/stdlib.h /usr/include/alloca.h
reservant.o: /usr/include/jansson_config.h /usr/include/math.h
reservant.o: /usr/include/syslog.h /usr/include/signal.h /usr/include/time.h
reservant.o: /usr/include/malloc.h /usr/include/string.h
reservant.o: /usr/include/xlocale.h
