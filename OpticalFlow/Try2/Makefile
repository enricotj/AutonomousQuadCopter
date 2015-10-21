CC = g++
CFLAGS = -c -Wall
LDFLAGS =

SOURCES = Source.cpp
OBJECTS = $(SOURCES:.cpp=.o)

EXECUTABLE = optFlow

CFLAGS += `pkg-config --cflags opencv`
LDFLAGS += `pkg-config --libs opencv`

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@ $(LDFLAGS) -I/usr/local/include -lraspicam -lmmal -lmmal_core -lmmal_util

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm $(OBJECTS) $(EXECUTABLE)
