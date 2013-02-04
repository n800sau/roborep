CC=g++
CCOPTS=-L. -I/usr/include/hiredis -I../ReServant -I../I2CWire
STRIP=-strip
SRCS=l3g4200dd.cpp L3G4200D.cpp
OBJECTS=l3g4200dd.o L3G4200D.o
HEADERS=../ReServant/reservant.h ../I2CWire/I2CWire.h L3G4200D.h
LIBS=../I2CWire/libi2cwire.a ../ReServant/libreservant.a
NAME=l3g4200dd

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
