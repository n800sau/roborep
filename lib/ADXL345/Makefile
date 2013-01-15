CC=g++
CCOPTS=-L. -I/usr/include/hiredis -I../ReServant -I../I2CWire
STRIP=-strip
OBJECTS=adxl345d.o ADXL345.o
HEADERS=../ReServant/reservant.h ../I2CWire/I2CWire.h
LIBS=../I2CWire/libi2cwire.a ../ReServant/libreservant.a
NAME=adxl345d

.cpp.o: $(HEADERS)
		$(CC) $(CCOPTS) -c $<

all: $(NAME)

clean:
	rm -f $(OBJECTS) $(NAME)

$(NAME):	$(OBJECTS) $(LIBS)
	$(CC) -o $(NAME) $(OBJECTS) $(LIBS) $(CCOPTS) -lhiredis -levent -ljansson
#	$(STRIP) -s $(NAME)
