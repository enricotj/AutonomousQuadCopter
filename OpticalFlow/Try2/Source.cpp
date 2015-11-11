#include <opencv2/core/utility.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

//#include <thread>
#include <raspicam/raspicam.h>
#include <raspicam/raspicam_cv.h>
#include "wiringPi/wiringPi/wiringPi.h"
#include "wiringPi/wiringPi/softServo.h"
#include <iostream>
#include <ctype.h>

#include "MeanShiftTracker.h"
#include "MotionTracker.h"

using namespace cv;
using namespace std;

int frameMax = 50;

vector<Mat> frames;

const int CAM_W = 640;
const int CAM_H = 480;

const int SERVO_LEFT = -1; //clockwise
const int SERVO_RIGHT = 1; //counterclockwise
const int SERVO_UP = -1;
const int SERVO_DOWN = 1;
const int SERVO_STOP = 0;
const int SERVO_AIM_THRESH_X = 70;
const int SERVO_AIM_THRESH_Y = 70;
int prevRotX = 99999;
int prevRotY = 99999;

void moveServoX(int rot)
{
	if(prevRotX==rot) return;
	prevRotX = rot;
	cout << "turn x: " << rot << endl;
	switch (rot)
	{
		// move left
	case SERVO_LEFT:
		softServoWrite(0, 375);
		break;
		// stop
	case SERVO_STOP:
		softServoWrite(0, 450);
		break;
		// move right
	case SERVO_RIGHT:
		softServoWrite(0, 525);
		break;
		// stop
	default:
		softServoWrite(0, 450);
		break;
	}
	//softServoWrite(0, 400);
}

void moveServoY(int rot)
{
	if(prevRotY==rot) return;
	prevRotY = rot;
	cout << "turn y: " << rot << endl;
	switch (rot)
	{
		// move left

	case SERVO_UP:
		softServoWrite(1, 375);
		break;
		// stop
	case SERVO_STOP:
		softServoWrite(1, 440);
		break;
		// move right
	case SERVO_DOWN:
		softServoWrite(1, 500);
		break;
		// stop
	default:
		softServoWrite(1, 440);
		break;
	}
	//softServoWrite(0, 400);
}

void servoTest(){
	int i = 0;
	while(i<50){
		moveServoY(SERVO_DOWN);
		delay(10);
		i++;
	}
}

void aimServoTowards(Point p)
{
	int cx = CAM_W / 2;
	cout<<"x :" << p.x << endl;
	if (p.x > cx + SERVO_AIM_THRESH_X)
	{
		moveServoX(SERVO_LEFT);
	}
	else if (p.x < cx - SERVO_AIM_THRESH_X)
	{
		moveServoX(SERVO_RIGHT);
	}
	else
	{
		moveServoX(SERVO_STOP);
	}
	int cy = CAM_H / 2;
	cout<<"y :" << p.y << endl;
	if (p.y > cy + SERVO_AIM_THRESH_Y)
	{
		moveServoY(SERVO_DOWN);
	}
	else if (p.y < cy - SERVO_AIM_THRESH_Y)
	{
		moveServoY(SERVO_UP);
	}
	else
	{
		moveServoY(SERVO_STOP);
	}
	delay(10);
}

int main(int argc, const char** argv)
{
	/*
	int camNum = 0;
	VideoCapture cap;
	cap.open(camNum);
	
	if (!cap.isOpened())
	{
		cout << "***Could not initialize capturing...***\n";
		cout << "Current parameter's value: \n";
		return -1;
	}
	*/
	raspicam::RaspiCam_Cv Camera; //Cmaera object
	// Open Camera
	Camera.set(CV_CAP_PROP_FORMAT, CV_8UC3);
	Camera.set(CV_CAP_PROP_FRAME_WIDTH, CAM_W);
	Camera.set(CV_CAP_PROP_FRAME_HEIGHT, CAM_H);
	if (!Camera.open())
	{
		cout << "Cannot open the web cam" << endl;
		return -1;
	}

	Mat frame, image;

	MeanShiftTracker meanShiftTracker;
	//cap.read(frame);
	Camera.grab();
	Camera.retrieve(frame);
	MotionTracker motionTracker = MotionTracker(frame);
	
	//waitKey(10);
	
	bool start = false;
	VideoWriter video("out.avi", CV_FOURCC('M', 'J', 'P', 'G'), 24, Size(CAM_W, CAM_H), true);
	int frameCounter = 0;
	while (frameCounter < frameMax)
	{
		frameCounter++;
		Camera.grab();
		Camera.retrieve(frame);
		/*cap >> frame;
		if (frame.empty())
			break;*/

		if (!start)
		{
			image = motionTracker.process(frame);
			start = motionTracker.objectCaptured();
			if (start)
			{
				meanShiftTracker.~MeanShiftTracker();
				meanShiftTracker = MeanShiftTracker(motionTracker.getObject());
				
				// set up servos
				wiringPiSetup();
				softServoSetup(0, 1, 2, 3, 4, 5, 6, 7);
				moveServoX(SERVO_STOP);
				moveServoY(SERVO_STOP);
				delay(50);
			}
		}
		
		if (start)
		{
			image = meanShiftTracker.process(frame);
			aimServoTowards(meanShiftTracker.getObject().center);
		}

		//imshow("Track", image);
		Mat temp;
		image.copyTo(temp);
		frames.push_back(temp);

		//waitKey(10);
	}

	moveServoX(SERVO_STOP);
	moveServoY(SERVO_STOP);

	//cap.release();
	Camera.release();
	imwrite("firstFrame.jpg", frames.front());
	for (vector<Mat>::iterator it = frames.begin(); it != frames.end(); ++it)
	{
		video.write(*it);
	}

	meanShiftTracker.~MeanShiftTracker();
	motionTracker.~MotionTracker();

	return 0;
}
