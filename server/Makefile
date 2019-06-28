CC = gcc
CXX = g++

INCS=-I.
CXXFLAGS = -Wall -std=c++11 -g $(INCS)


TAR = tar
TARFLAGS = -czvf
TARNAME = 201209806.tar.gz
TARSRCS = ${ALL} ${EXE} Makefile

SERVER =  server.cpp
EXE = server
ALL = server.cpp

default: all

all: ${EXE}

server: server.o
	$(CXX) $(CXXFLAGS) -o server server.o

server.o: shared.h server.cpp
	$(CXX) $(CXXFLAGS) -c server.cpp

.PHONY : clean
clean:
	$(RM) *.o  ${EXE} $(TARNAME) *~

tar:
	$(TAR) $(TARFLAGS) $(TARNAME) $(TARSRCS)
