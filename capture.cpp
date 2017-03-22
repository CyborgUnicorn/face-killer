#include <opencv2/core/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/video/background_segm.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "SDL2/SDL.h"
#include <raspicam_cv.h>
#include <stdio.h>

#include "servo.h"

//#define USE_SDL
#define USE_OCV

const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;

using namespace cv;
using namespace std;

CascadeClassifier face_cascade;
Ptr<BackgroundSubtractorMOG2> mog1;

Mat detect_blobs(Mat frame) {
	vector<vector<Point> > contours;
	vector<vector<Point> > contours_poly;
	vector<Rect> boundRect;
	vector<Point2f> center;
	vector<float> radius;
	
	int32_t maxIndex;
	float maxRadius = 0;
	double maxPenetration = 0;
	
	Scalar color(255,255,255);
	Scalar color2(255,0,0);
	Scalar color3(0,255,0);
	
  	vector<Vec4i> hierarchy;
	findContours(frame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
	
	contours_poly.resize(contours.size());
	boundRect.resize(contours.size());
	
	radius.resize( contours.size() );
	center.resize( contours.size() );
        
	Mat matOutput = Mat::zeros( frame.size(), CV_8UC3 );
	
	for( int i = 0; i< contours.size(); i++ ) {
		double penetration = contourArea(contours[i]);
		if (penetration > 200.0) {
			approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
			minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
			if (radius[i] > maxRadius) {
				maxRadius = radius[i];
				maxPenetration = penetration;
				maxIndex = i;
			}
		}
	}
	
	if (maxRadius > 0 && maxRadius < 200) {
		drawContours(matOutput, contours, maxIndex, color3, 2, 1, hierarchy, 0);
		rectangle( matOutput, boundRect[maxIndex].tl(), boundRect[maxIndex].br(), color, 2, 1, 0 );
		circle( matOutput, center[maxIndex], (int)maxRadius, color2, 2, 1, 0 );
		printf("Penetrated by %f (%f)\n", maxPenetration, maxRadius);
	}
	
	return matOutput;
}

void detect_faces(Mat frame) {
	std::vector<Rect> faces;
	
	face_cascade.detectMultiScale(frame, faces, 1.4, 3, 0|CASCADE_SCALE_IMAGE, Size(30,30));

	for(size_t i=0; i < faces.size(); ++i) {
		Point center(faces[i].x + faces[i].width / 2, faces[i].y + faces[i].height / 2);

		ellipse(frame, center, Size(faces[i].width/2, faces[i].height/2), 0, 0, 360, 
			Scalar( 255, 255, 255 ), 2, 2, 0);
	}
}


void bgsub_init() {
	mog1 = createBackgroundSubtractorMOG2(2, 6, 0);
    mog1->setNMixtures(3);
}

void bgsub_filter() {
	
}

uint8_t opencv_events() {
	int32_t key = waitKey(30);
	
	switch(key) {
		case 65362: //up
			servo_set_delta(SERVO_X, -15);
		break;
		case 65364: //down
			servo_set_delta(SERVO_X, +15);
		break;
		case 65361: //left
			servo_set_delta(SERVO_Y, +15);
		break;
		case 65363: //right
			servo_set_delta(SERVO_Y, -15);
		break;
		case 27: //ESC
			return 1;
		break;
		default: 
			if (key != -1)
				printf("key: %d", key);
		break;
	}
	
	return 0;
}

void opencv_flush(Mat frame) {
	imshow("video", frame);
}

int main() {
	raspicam::RaspiCam_Cv cam;
	Mat frame;
	Mat mask;
	Mat herp;
	
	printf("Starting...\n");

	cam.release();

	cam.set(CV_CAP_PROP_FORMAT, CV_8UC1);
	cam.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
	cam.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);

	servo_init();

	if (!cam.open()) {
		printf("Inti cam failed\n");
		exit(EXIT_FAILURE);
		return 1;
	}
	printf("CAM setup done\n");

	//face_cascade.load("haarcascade_frontalface_default.xml");
	face_cascade.load("haarcascade_upperbody.xml");
	//face_cascade.load("haarcascade_fullbody.xml");
	bgsub_init();

	servo_set(SERVO_X, 1500);
	servo_set(SERVO_Y, 1500);
	fflush(stdout);

	while(opencv_events() == 0) {
		cam.grab();
		cam.retrieve(frame);
		//detect_faces(mask);

		frame.convertTo(herp, -1, 2, 0);
		mog1->apply(herp, mask);
		//detect_blobs(mask);
		opencv_flush(detect_blobs(mask));
		
		if (servo_update() == 0) {
			break;
		}
		//fflush(stdout);
	}

	servo_destroy();
	cam.release();
	return 0;
}
