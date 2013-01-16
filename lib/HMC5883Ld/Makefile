CC=g++
CCOPTS=-L. -I/usr/include/hiredis -I../ReServant -I../I2CWire
STRIP=-strip
OBJECTS=HMC5883L.o hmc5883ld.o
HEADERS=../ReServant/reservant.h ../I2CWire/I2CWire.h HMC5883L.h
LIBS=../I2CWire/libi2cwire.a ../ReServant/libreservant.a
NAME=hmc5883ld

.cpp.o: $(HEADERS)
		$(CC) $(CCOPTS) -c $<

all: $(NAME)

clean:
	rm -f $(OBJECTS) $(NAME)

$(NAME):	$(OBJECTS) $(LIBS)
	$(CC) -o $(NAME) $(OBJECTS) $(LIBS) $(CCOPTS) -lhiredis -levent -ljansson
#	$(STRIP) -s $(NAME)
