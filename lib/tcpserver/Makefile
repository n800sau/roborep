CC=g++
CCOPTS=-L. -I/usr/include/hiredis -I../ReServant -g
STRIP=-strip
SRCS=tcpserverd.cpp tcpserver.cpp
OBJECTS=tcpserverd.o tcpserver.o
HEADERS=../ReServant/reservant.h tcpserver.h
LIBS=../ReServant/libreservant.a
NAME=tcpserverd

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
