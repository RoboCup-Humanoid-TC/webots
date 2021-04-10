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
LIBS += `pkg-config --cflags --libs protobuf` -llua5.1 ../../build/$(GOAL)/messages.pb.o
CFLAGS = -c -Wall -fpic

main: luatables.o OPKinematics.o Transform.o client.o
	$(CXX) -o $(PROG) $(SRCS) $(LIBS)

luatables.o: luatables.cpp
	$(CXX) $(CFLAGS) luatables.cpp $(LIBS)

OPKinematics.o: OPKinematics.cpp
	$(CXX) $(CFLAGS) OPKinematics.cpp $(LIBS)

Transform.o: Transform.cpp
	$(CXX) $(CFLAGS) Transform.cpp $(LIBS)

client.o: client.cpp
	$(CXX) $(CFLAGS) client.cpp $(LIBS)

clean:
	rm -f $(SRCS) $(PROG)

