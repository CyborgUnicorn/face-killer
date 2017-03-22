#include <opencv2/core/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/video/background_segm.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "SDL2/SDL.h"
#include <raspicam_cv.h>
#include <stdio.h>

#include "MiniPID.h"
#include "servo.h"

//#define USE_SDL
#define USE_OCV

const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;

using namespace cv;
using namespace std;

CascadeClassifier face_cascade;
Ptr<BackgroundSubtractorMOG2> mog1;
MiniPID pidX = MiniPID(1,0,0);
MiniPID pidY = MiniPID(1,0,0);

int16_t currentServoX;
int16_t currentServoY;
	
int8_t detect_blobs(Mat frame, Point2f &foundCenter, float &foundRadius) {
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
		//drawContours(frame, contours, maxIndex, color3, 2, 1, hierarchy, 0);
		//rectangle( frame, boundRect[maxIndex].tl(), boundRect[maxIndex].br(), color, 2, 1, 0 );
		//circle( frame, center[maxIndex], (int)maxRadius, color2, 2, 1, 0 );
		//printf("Penetrated by %f (%f)\n", maxPenetration, maxRadius);
		foundCenter.x = center[maxIndex].x;
		foundCenter.y = center[maxIndex].y;
		foundRadius = maxRadius;
		return 1;
	}
	
	return -1;
}

void kill_target(Point2f targetCenter) {
	/*
	double servoX = pidX.getOutput(targetCenter.x, FRAME_WIDTH / 2);
	double servoY = pidY.getOutput(FRAME_HEIGHT / 2, targetCenter.y);
	
	double deltaX = (servoX - currentServoX) / 2;
	double deltaY = (servoY - currentServoY) / 2;
	
 	printf("%f, %f, (%f, %f)\n", servoX, servoY, targetCenter.x, targetCenter.y); 
	
	currentServoX = servo_set_delta(SERVO_Y, (int16_t)deltaX); // just because
	currentServoY = servo_set_delta(SERVO_X, (int16_t)deltaY);
	*/
		
	double offsetX = targetCenter.x - FRAME_WIDTH / 2;
	double offsetY = targetCenter.y - FRAME_HEIGHT / 2;

	double deltaX = (offsetX / FRAME_WIDTH) * -200;
	double deltaY = (offsetY / FRAME_HEIGHT) * 200;
	
	servo_set_delta(SERVO_Y, (int16_t)deltaX);
	servo_set_delta(SERVO_X, (int16_t)deltaY);
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
			currentServoX = servo_set_delta(SERVO_X, -15);
		break;
		case 65364: //down
			currentServoX = servo_set_delta(SERVO_X, +15);
		break;
		case 65361: //left
			currentServoY = servo_set_delta(SERVO_Y, +15);
		break;
		case 65363: //right
			currentServoY = servo_set_delta(SERVO_Y, -15);
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
	Point2f foundCenter;
	float foundRadius;

	printf("Starting...\n");

	pidX.setOutputLimits(1100.0, 1900.0f);
	pidY.setOutputLimits(1100.0, 1900.0f);

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

	currentServoX = 1500;
	currentServoY = 1500;

	servo_set(SERVO_X, currentServoX);
	servo_set(SERVO_Y, currentServoY);
	fflush(stdout);

	while(opencv_events() == 0) {
		cam.grab();
		cam.retrieve(frame);
		//detect_faces(mask);

		frame.convertTo(herp, -1, 2, 0);
		mog1->apply(herp, mask);
		
		if( detect_blobs(mask, foundCenter, foundRadius) == 1 ) {
			circle(frame, foundCenter, foundRadius, Scalar(255,0,0), 2, 1, 0 );
			kill_target(foundCenter);
		}
		opencv_flush(frame);
		
		if (servo_update() == 0) {
			break;
		}
		//fflush(stdout);
	}

	servo_destroy();
	cam.release();
	return 0;
}
