CC=g++ -I/usr/include/hiredis -I../I2CWire
CCOPTS=-L.
STRIP=-strip
OBJECTS=HMC5883L.o
HEADERS=HMC5883L.h
LIBS=../I2CWire/libi2cwire.a
NAME=hmc5883ld

.cpp.o: $(HEADERS)
		$(CC) $(CCOPTS) -c $<

all: $(NAME)

clean:
	rm -f $(OBJECTS) $(NAME)

$(NAME):	$(NAME).cpp $(OBJECTS) $(LIBS)
	$(CC) -o $(NAME) $(NAME).cpp $(OBJECTS) $(LIBS) $(CCOPTS) -lhiredis -levent -ljansson
#	$(STRIP) -s $(NAME)
