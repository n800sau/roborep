CC=g++ -I/usr/include/hiredis
CCOPTS=-L.
STRIP=-strip
OBJECTS=I2CWire.o HMC5883L.o
HEADERS=I2CWire.h HMC5883L.h
NAME=hmc5883ld

.cpp.o: $(HEADERS)
		$(CC) $(CCOPTS) -c $<

all: $(NAME)

libi2cwire.a: $(OBJECTS) $(HEADERS)
		$(AR) r libi2cwire.a $(OBJECTS)

clean:
	rm -f $(OBJECTS) $(NAME) libi2cwire.a

$(NAME):	$(NAME).cpp libi2cwire.a
	$(CC) -o $(NAME) $(NAME).cpp $(CCOPTS) -lhiredis -levent -ljansson libi2cwire.a
#	$(STRIP) -s $(NAME)
