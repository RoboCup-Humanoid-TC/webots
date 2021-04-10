ifeq ($(OS),Windows_NT)
TARGET = client.exe
LIBS = -lws2_32
CXX=g++
else
TARGET = main
LIBS =
endif
ifeq ($(MAKECMDGOALS),)
GOAL = release
else
GOAL = $(MAKECMDGOALS)
endif

PROG = client
SRCS = luatables.o OPKinematics.o Transform.o client.o
INCLUDE = -I/usr/include/lua5.2
LIBS += `pkg-config --cflags --libs protobuf` -llua5.2 ../../build/$(GOAL)/messages.pb.o
CFLAGS = -c -Wall -fpic

main: luatables.o OPKinematics.o Transform.o client.o
	$(CXX) $(INCLUDE) -o $(PROG) $(SRCS) $(LIBS)

luatables.o: luatables.cpp
	$(CXX) $(INCLUDE) $(CFLAGS) luatables.cpp $(LIBS)

OPKinematics.o: OPKinematics.cpp
	$(CXX) $(INCLUDE) $(CFLAGS) OPKinematics.cpp $(LIBS)

Transform.o: Transform.cpp
	$(CXX) $(INCLUDE) $(CFLAGS) Transform.cpp $(LIBS)

client.o: client.cpp
	$(CXX) $(INCLUDE) $(CFLAGS) client.cpp $(LIBS)

clean:
	rm -f $(SRCS) $(PROG)

