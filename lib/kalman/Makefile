CC=g++
CCOPTS=-L. -I/usr/include/hiredis -I../ReServant
STRIP=-strip
SRCS=kalmand.cpp kalman.cpp KalmanFilter.cpp
OBJECTS=kalmand.o kalman.o KalmanFilter.o
LIBS=../ReServant/libreservant.a
NAME=kalmand

.cpp.o:
		$(CC) $(CCOPTS) -c $<

all: $(NAME)

clean:
	rm -f $(OBJECTS) $(NAME)

$(NAME):	$(OBJECTS) $(LIBS)
	$(CC) -o $(NAME) $(OBJECTS) $(LIBS) $(CCOPTS) -lhiredis -levent -ljansson
#	$(STRIP) -s $(NAME)

# what are the source dependencies
depend: $(SRCS)
	makedepend $(CCOPTS) $(SRCS)
# DO NOT DELETE

kalmand.o: kalman.h ../ReServant/reservant.h /usr/include/jansson.h
kalmand.o: /usr/include/stdio.h /usr/include/features.h /usr/include/libio.h
kalmand.o: /usr/include/_G_config.h /usr/include/wchar.h
kalmand.o: /usr/include/stdlib.h /usr/include/alloca.h
kalmand.o: /usr/include/jansson_config.h /usr/include/hiredis/hiredis.h
kalmand.o: /usr/include/hiredis/async.h /usr/include/event2/http.h
kalmand.o: /usr/include/event2/util.h /usr/include/event2/event-config.h
kalmand.o: /usr/include/stdint.h /usr/include/netdb.h
kalmand.o: /usr/include/netinet/in.h /usr/include/endian.h
kalmand.o: /usr/include/rpc/netdb.h /usr/include/time.h
kalmand.o: /usr/include/xlocale.h KalmanFilter.h
kalman.o: kalman.h ../ReServant/reservant.h /usr/include/jansson.h
kalman.o: /usr/include/stdio.h /usr/include/features.h /usr/include/libio.h
kalman.o: /usr/include/_G_config.h /usr/include/wchar.h /usr/include/stdlib.h
kalman.o: /usr/include/alloca.h /usr/include/jansson_config.h
kalman.o: /usr/include/hiredis/hiredis.h /usr/include/hiredis/async.h
kalman.o: /usr/include/event2/http.h /usr/include/event2/util.h
kalman.o: /usr/include/event2/event-config.h /usr/include/stdint.h
kalman.o: /usr/include/netdb.h /usr/include/netinet/in.h
kalman.o: /usr/include/endian.h /usr/include/rpc/netdb.h /usr/include/time.h
kalman.o: /usr/include/xlocale.h KalmanFilter.h /usr/include/math.h
kalman.o: /usr/include/syslog.h /usr/include/string.h /usr/include/unistd.h
kalman.o: /usr/include/getopt.h ../ReServant/hiredis_ext.h
KalmanFilter.o: KalmanFilter.h
