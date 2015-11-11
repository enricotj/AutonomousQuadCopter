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
	$(CC) $(OBJECTS) MotionTracker.cpp MeanShiftTracker.cpp wiringPi/wiringPi/softServo.c -o $@ $(LDFLAGS) -I/usr/local/include -lraspicam -lraspicam_cv -lwiringPi -lwiringPiDev

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm $(OBJECTS) $(EXECUTABLE)
