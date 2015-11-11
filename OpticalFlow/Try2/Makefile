CC = g++
CFLAGS = -c -Wall -pthread
LDFLAGS =

SOURCES = Source.cpp
OBJECTS = $(SOURCES:.cpp=.o)

EXECUTABLE = optFlow

CFLAGS += `pkg-config --cflags opencv`
LDFLAGS += `pkg-config --libs opencv`

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(OBJECTS) MotionTracker.cpp MeanShiftTracker.cpp -o $@ $(LDFLAGS) -I/usr/local/include -lraspicam -lraspicam_cv -lpigpio -lrt

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm $(OBJECTS) $(EXECUTABLE)
