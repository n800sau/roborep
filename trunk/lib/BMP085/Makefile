CC=g++
CCOPTS=-I/usr/include/hiredis -I../ReServant -I../I2CWire
STRIP=-strip
SRCS=bmp085d.cpp BMP085.cpp
OBJECTS=bmp085d.o BMP085.o
LIBS=../I2CWire/libi2cwire.a ../ReServant/libreservant.a
NAME=bmp085d

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
