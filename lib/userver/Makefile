CC=g++
CCOPTS=-L. -I/usr/include/hiredis -I../ReServant -g
STRIP=-strip
SRCS=userverd.cpp userver.cpp
OBJECTS=userverd.o userver.o
HEADERS=../ReServant/reservant.h userver.h
LIBS=../ReServant/libreservant.a
NAME=userverd

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
