CC=g++
CCOPTS=-L. -I../RF24 -I ../RF24Network -I ../compatibility -I ../ReServant -I /usr/include/hiredis -g
STRIP=-strip
SRCS=pc2nrf.cpp pc2nrfd.cpp
OBJECTS=pc2nrf.o pc2nrfd.o
HEADERS=pc2nrf.h
LIBS=../RF24/libRF24.a ../RF24Network/libRF24Network.a ../compatibility/libcompatibility.a ../spi/libspi.a ../gpio/libgpio.a ../ReServant/libreservant.a -lhiredis -levent -ljansson -lpthread
NAME=pc2nrfd

.cpp.o:
		$(CC) $(CCOPTS) -c $<

all: $(NAME)

clean:
	rm -f $(OBJECTS) $(NAME)

$(NAME):	$(OBJECTS) $(LIBS)
	$(CC) -o $(NAME) $(OBJECTS) $(LIBS) $(CCOPTS)
#	$(STRIP) -s $(NAME)

# what are the source dependencies
depend: $(SRCS)
	makedepend $(CCOPTS) $(SRCS)
# DO NOT DELETE

pc2nrf.o: pc2nrf.h ../ReServant/reservant.h /usr/include/jansson.h
pc2nrf.o: /usr/include/stdio.h /usr/include/features.h
pc2nrf.o: /usr/include/stdc-predef.h /usr/include/libio.h
pc2nrf.o: /usr/include/_G_config.h /usr/include/wchar.h /usr/include/stdlib.h
pc2nrf.o: /usr/include/alloca.h /usr/include/jansson_config.h
pc2nrf.o: /usr/include/hiredis/hiredis.h /usr/include/hiredis/async.h
pc2nrf.o: /usr/include/event2/http.h /usr/include/event2/util.h
pc2nrf.o: /usr/include/event2/event-config.h /usr/include/stdint.h
pc2nrf.o: /usr/include/netdb.h /usr/include/netinet/in.h
pc2nrf.o: /usr/include/endian.h /usr/include/rpc/netdb.h /usr/include/time.h
pc2nrf.o: /usr/include/xlocale.h /usr/include/event2/bufferevent.h
pc2nrf.o: ../RF24Network/RF24Network.h ../RF24/RF24.h ../RF24/RF24_config.h
pc2nrf.o: ../spi/spi.h /usr/include/unistd.h /usr/include/getopt.h
pc2nrf.o: /usr/include/fcntl.h /usr/include/inttypes.h
pc2nrf.o: /usr/include/linux/types.h /usr/include/linux/posix_types.h
pc2nrf.o: /usr/include/linux/stddef.h /usr/include/linux/spi/spidev.h
pc2nrf.o: ../gpio/gpio.h ../compatibility/compatibility.h
pc2nrf.o: /usr/include/string.h ../../ino/include/common.h
pc2nrf.o: /usr/include/pthread.h /usr/include/sched.h /usr/include/syslog.h
pc2nrf.o: /usr/include/errno.h ../ReServant/hiredis_ext.h
pc2nrf.o: ../gpio/event_gpio.h
pc2nrfd.o: pc2nrf.h ../ReServant/reservant.h /usr/include/jansson.h
pc2nrfd.o: /usr/include/stdio.h /usr/include/features.h
pc2nrfd.o: /usr/include/stdc-predef.h /usr/include/libio.h
pc2nrfd.o: /usr/include/_G_config.h /usr/include/wchar.h
pc2nrfd.o: /usr/include/stdlib.h /usr/include/alloca.h
pc2nrfd.o: /usr/include/jansson_config.h /usr/include/hiredis/hiredis.h
pc2nrfd.o: /usr/include/hiredis/async.h /usr/include/event2/http.h
pc2nrfd.o: /usr/include/event2/util.h /usr/include/event2/event-config.h
pc2nrfd.o: /usr/include/stdint.h /usr/include/netdb.h
pc2nrfd.o: /usr/include/netinet/in.h /usr/include/endian.h
pc2nrfd.o: /usr/include/rpc/netdb.h /usr/include/time.h
pc2nrfd.o: /usr/include/xlocale.h /usr/include/event2/bufferevent.h
pc2nrfd.o: ../RF24Network/RF24Network.h ../RF24/RF24.h ../RF24/RF24_config.h
pc2nrfd.o: ../spi/spi.h /usr/include/unistd.h /usr/include/getopt.h
pc2nrfd.o: /usr/include/fcntl.h /usr/include/inttypes.h
pc2nrfd.o: /usr/include/linux/types.h /usr/include/linux/posix_types.h
pc2nrfd.o: /usr/include/linux/stddef.h /usr/include/linux/spi/spidev.h
pc2nrfd.o: ../gpio/gpio.h ../compatibility/compatibility.h
pc2nrfd.o: /usr/include/string.h ../../ino/include/common.h
pc2nrfd.o: /usr/include/pthread.h /usr/include/sched.h /usr/include/errno.h
pc2nrfd.o: /usr/include/syslog.h
