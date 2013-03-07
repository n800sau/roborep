CC=g++
CCOPTS=-I/usr/include/hiredis -I../ReServant -I../I2CWire
STRIP=-strip
SRCS=mpu6050d.cpp MPU6050.cpp
OBJECTS=mpu6050d.o MPU6050.o
LIBS=../I2CWire/libi2cwire.a ../ReServant/libreservant.a
NAME=mpu6050d

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

mpu6050d.o: MPU6050.h ../ReServant/reservant.h /usr/include/jansson.h
mpu6050d.o: /usr/include/stdio.h /usr/include/features.h /usr/include/libio.h
mpu6050d.o: /usr/include/_G_config.h /usr/include/wchar.h
mpu6050d.o: /usr/include/stdlib.h /usr/include/alloca.h
mpu6050d.o: /usr/include/jansson_config.h /usr/include/hiredis/hiredis.h
mpu6050d.o: /usr/include/hiredis/async.h /usr/include/event2/http.h
mpu6050d.o: /usr/include/event2/util.h /usr/include/event2/event-config.h
mpu6050d.o: /usr/include/stdint.h /usr/include/netdb.h
mpu6050d.o: /usr/include/netinet/in.h /usr/include/endian.h
mpu6050d.o: /usr/include/rpc/netdb.h /usr/include/time.h
mpu6050d.o: /usr/include/xlocale.h ../I2CWire/I2CWire.h
MPU6050.o: MPU6050.h ../ReServant/reservant.h /usr/include/jansson.h
MPU6050.o: /usr/include/stdio.h /usr/include/features.h /usr/include/libio.h
MPU6050.o: /usr/include/_G_config.h /usr/include/wchar.h
MPU6050.o: /usr/include/stdlib.h /usr/include/alloca.h
MPU6050.o: /usr/include/jansson_config.h /usr/include/hiredis/hiredis.h
MPU6050.o: /usr/include/hiredis/async.h /usr/include/event2/http.h
MPU6050.o: /usr/include/event2/util.h /usr/include/event2/event-config.h
MPU6050.o: /usr/include/stdint.h /usr/include/netdb.h
MPU6050.o: /usr/include/netinet/in.h /usr/include/endian.h
MPU6050.o: /usr/include/rpc/netdb.h /usr/include/time.h
MPU6050.o: /usr/include/xlocale.h ../I2CWire/I2CWire.h /usr/include/syslog.h
MPU6050.o: /usr/include/math.h /usr/include/evhttp.h /usr/include/event.h
MPU6050.o: /usr/include/evutil.h /usr/include/event2/event_struct.h
MPU6050.o: /usr/include/event2/keyvalq_struct.h /usr/include/event2/event.h
MPU6050.o: /usr/include/event2/event_compat.h /usr/include/event2/buffer.h
MPU6050.o: /usr/include/event2/buffer_compat.h
MPU6050.o: /usr/include/event2/bufferevent.h
MPU6050.o: /usr/include/event2/bufferevent_struct.h
MPU6050.o: /usr/include/event2/bufferevent_compat.h /usr/include/event2/tag.h
MPU6050.o: /usr/include/event2/tag_compat.h /usr/include/event2/http_struct.h
MPU6050.o: /usr/include/event2/http_compat.h /usr/include/string.h
MPU6050.o: /usr/include/unistd.h /usr/include/getopt.h /usr/include/libgen.h
MPU6050.o: /usr/include/fcntl.h
