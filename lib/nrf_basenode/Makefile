CC=g++
CCOPTS=-I/usr/include/hiredis -I../ReServant
STRIP=-strip
SRCS=nrf_basenoded.cpp nrf_basenode.cpp
OBJECTS=nrf_basenoded.o nrf_basenode.o
LIBS=../ReServant/libreservant.a
NAME=nrf_basenoded

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

