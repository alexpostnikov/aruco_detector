CC = g++
CFLAGS = -std=c++11 -g -Wall
SRCS = clasterization.cpp
PROG = opencvTest 

OPENCV = `pkg-config opencv --cflags --libs`
LIBS = $(OPENCV)

$(PROG):$(SRCS)
	$(CC) $(CFLAGS) -o $(PROG) $(SRCS) $(LIBS)


