CC=g++
CCOPTS=-L. -I../ReServant -I../wiringPi -I../CMUcam4 -I/usr/include/hiredis
STRIP=-strip
SRCS=camTr.cpp
OBJECTS=camTr.o
LIBS=../CMUcam4/libcmucam4.a ../wiringPi/libwiringPi.a ../ReServant/libreservant.a
NAME=camTr

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

camTr.o: ../ReServant/reservant.h /usr/include/jansson.h /usr/include/stdio.h
camTr.o: /usr/include/features.h /usr/include/libio.h
camTr.o: /usr/include/_G_config.h /usr/include/wchar.h /usr/include/stdlib.h
camTr.o: /usr/include/alloca.h /usr/include/jansson_config.h
camTr.o: /usr/include/hiredis/hiredis.h /usr/include/hiredis/async.h
camTr.o: /usr/include/syslog.h /usr/include/time.h /usr/include/xlocale.h
camTr.o: /usr/include/unistd.h /usr/include/getopt.h ../CMUcam4/CMUcam4.h
camTr.o: ../CMUcam4/CMUcom4.h /usr/include/stdint.h /usr/include/setjmp.h
camTr.o: /usr/include/string.h
