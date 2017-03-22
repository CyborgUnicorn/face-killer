CXX = g++

LDFLAGS = -lSDL2 -lpigpio -lopencv_highgui -lopencv_core -lopencv_ml -lopencv_video -lopencv_imgproc -lopencv_calib3d -lopencv_objdetect -lraspicam_cv -lraspicam -L/usr/lib 

CPPFLAGS = -g -I/usr/local/include -I/usr/include/opencv -I/usr/include/opencv3 -I/usr/local/include/raspicam

all: 
	$(CXX) -o capture capture.cpp servo.cpp $(CPPFLAGS) $(LDFLAGS)
