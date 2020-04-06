CC=g++
CCOPTS=-L. -I../ReServant -I../wiringPi -I../CMUcam4 -I/usr/include/hiredis
STRIP=-strip
SRCS=camServant.cpp
OBJECTS=camServant.o
LIBS=../CMUcam4/libcmucam4.a ../wiringPi/libwiringPi.a ../ReServant/libreservant.a
NAME=camServant

.cpp.o:
		$(CC) $(CCOPTS) -c $<

all: $(NAME)

clean:
	rm -f $(OBJECTS) $(NAME)

$(NAME):	$(OBJECTS) $(LIBS)
	$(CC) -o $(NAME) $(CCOPTS) $(OBJECTS) $(LIBS) -lhiredis -levent -ljansson
#	$(STRIP) -s $(NAME)

# what are the source dependencies
depend: $(SRCS)
	makedepend $(CCOPTS) $(SRCS)
# DO NOT DELETE

camServant.o: ../ReServant/reservant.h /usr/include/jansson.h
camServant.o: /usr/include/stdio.h /usr/include/features.h
camServant.o: /usr/include/libio.h /usr/include/_G_config.h
camServant.o: /usr/include/wchar.h /usr/include/stdlib.h
camServant.o: /usr/include/alloca.h /usr/include/jansson_config.h
camServant.o: /usr/include/hiredis/hiredis.h /usr/include/hiredis/async.h
camServant.o: /usr/include/event2/http.h /usr/include/event2/util.h
camServant.o: /usr/include/event2/event-config.h /usr/include/stdint.h
camServant.o: /usr/include/netdb.h /usr/include/netinet/in.h
camServant.o: /usr/include/endian.h /usr/include/rpc/netdb.h
camServant.o: /usr/include/time.h /usr/include/xlocale.h
camServant.o: /usr/include/syslog.h /usr/include/unistd.h
camServant.o: /usr/include/getopt.h ../CMUcam4/CMUcam4.h ../CMUcam4/CMUcom4.h
camServant.o: /usr/include/setjmp.h /usr/include/string.h
