CC=g++ -I/usr/include/hiredis
CCOPTS=-L. -g
STRIP=-strip
SRCS=reservant.cpp hiredis_ext.cpp
OBJECTS=reservant.o hiredis_ext.o
NAME=libreservant.a

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

reservant.o: reservant.h /usr/include/jansson.h /usr/include/stdio.h
reservant.o: /usr/include/features.h /usr/include/libio.h
reservant.o: /usr/include/_G_config.h /usr/include/wchar.h
reservant.o: /usr/include/stdlib.h /usr/include/alloca.h
reservant.o: /usr/include/jansson_config.h /usr/include/event2/http.h
reservant.o: /usr/include/event2/util.h /usr/include/event2/event-config.h
reservant.o: /usr/include/stdint.h /usr/include/netdb.h
reservant.o: /usr/include/netinet/in.h /usr/include/endian.h
reservant.o: /usr/include/rpc/netdb.h /usr/include/time.h
reservant.o: /usr/include/xlocale.h /usr/include/math.h /usr/include/syslog.h
reservant.o: /usr/include/signal.h /usr/include/malloc.h
reservant.o: /usr/include/string.h /usr/include/unistd.h
reservant.o: /usr/include/getopt.h /usr/include/libgen.h
hiredis_ext.o: /usr/include/stdlib.h /usr/include/features.h
hiredis_ext.o: /usr/include/alloca.h /usr/include/string.h
hiredis_ext.o: /usr/include/xlocale.h /usr/include/syslog.h hiredis_ext.h
