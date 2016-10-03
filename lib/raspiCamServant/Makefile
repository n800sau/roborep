CC=g++
CCOPTS=-std=c++11 -I/usr/local/include -I/usr/include/hiredis -I../ReServant -L/opt/vc/lib -L/usr/local/lib
STRIP=-strip
SRCS=raspiCamServantd.cpp raspiCamServant.cpp
OBJECTS=raspiCamServantd.o raspiCamServant.o
LIBS=../ReServant/libreservant.a
NAME=raspiCamServantd

.cpp.o:
		$(CC) $(CCOPTS) -c $<

all: $(NAME)

clean:
	rm -f $(OBJECTS) $(NAME)

$(NAME):	$(OBJECTS) $(LIBS)
	$(CC) -o $(NAME) $(OBJECTS) $(LIBS) $(CCOPTS) -lhiredis -levent -ljansson -lraspicam -lraspicam_cv -lmmal -lmmal_core -lmmal_util -lopencv_core -lopencv_imgproc -lopencv_highgui -laruco
#	$(STRIP) -s $(NAME)

# what are the source dependencies
depend: $(SRCS)
	makedepend $(CCOPTS) $(SRCS)
# DO NOT DELETE

raspiCamServantd.o: raspiCamServant.h ../ReServant/reservant.h
raspiCamServantd.o: /usr/include/jansson.h /usr/include/stdio.h
raspiCamServantd.o: /usr/include/features.h /usr/include/libio.h
raspiCamServantd.o: /usr/include/_G_config.h /usr/include/wchar.h
raspiCamServantd.o: /usr/include/stdlib.h /usr/include/alloca.h
raspiCamServantd.o: /usr/include/jansson_config.h
raspiCamServantd.o: /usr/include/hiredis/hiredis.h
raspiCamServantd.o: /usr/include/hiredis/async.h /usr/include/event2/http.h
raspiCamServantd.o: /usr/include/event2/util.h
raspiCamServantd.o: /usr/include/event2/event-config.h /usr/include/stdint.h
raspiCamServantd.o: /usr/include/netdb.h /usr/include/netinet/in.h
raspiCamServantd.o: /usr/include/endian.h /usr/include/rpc/netdb.h
raspiCamServantd.o: /usr/include/time.h /usr/include/xlocale.h
raspiCamServantd.o: /usr/include/event2/bufferevent.h
raspiCamServant.o: raspiCamServant.h ../ReServant/reservant.h
raspiCamServant.o: /usr/include/jansson.h /usr/include/stdio.h
raspiCamServant.o: /usr/include/features.h /usr/include/libio.h
raspiCamServant.o: /usr/include/_G_config.h /usr/include/wchar.h
raspiCamServant.o: /usr/include/stdlib.h /usr/include/alloca.h
raspiCamServant.o: /usr/include/jansson_config.h
raspiCamServant.o: /usr/include/hiredis/hiredis.h
raspiCamServant.o: /usr/include/hiredis/async.h /usr/include/event2/http.h
raspiCamServant.o: /usr/include/event2/util.h
raspiCamServant.o: /usr/include/event2/event-config.h /usr/include/stdint.h
raspiCamServant.o: /usr/include/netdb.h /usr/include/netinet/in.h
raspiCamServant.o: /usr/include/endian.h /usr/include/rpc/netdb.h
raspiCamServant.o: /usr/include/time.h /usr/include/xlocale.h
raspiCamServant.o: /usr/include/event2/bufferevent.h /usr/include/unistd.h
raspiCamServant.o: /usr/include/getopt.h /usr/include/math.h
raspiCamServant.o: /usr/include/syslog.h
