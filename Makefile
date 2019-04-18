CC = g++
CFLAGS = -g -Wall -std=c++11
SRCS = astar.cpp
PROG = astar

OPENCV = `pkg-config opencv --cflags --libs`
LIBS = $(OPENCV)

$(PROG):$(SRCS)
	$(CC) $(CFLAGS) -o $(PROG) $(SRCS) $(LIBS)

clean:
	rm $(PROG)