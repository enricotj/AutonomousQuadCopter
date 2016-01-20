#include "Globals.h"

#ifdef ON_PI
#include <pigpio.h>
#include <raspicam/raspicam.h>
#include <raspicam/raspicam_cv.h>
#endif
#include <ctime>
#include <opencv2/core/utility.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <ctype.h>
#include "MeanShiftTracker.h"
#include "MotionTracker.h"

using namespace cv;
using namespace std;

int frameMax = 500;

vector<Mat> frames;

const int SERVO_LEFT = -1; //clockwise
const int SERVO_RIGHT = 1; //counterclockwise
const int SERVO_UP = -1;
const int SERVO_DOWN = 1;
const int SERVO_STOP = 0;
const int SERVO_AIM_THRESH_X = 50;
const int SERVO_AIM_THRESH_Y = 25;
int prevRotX = 99999;
int prevRotY = 99999;

#ifdef ON_PI
void moveServoX(int rot)
{
	if (prevRotX == rot) return;
	prevRotX = rot;
	cout << "turn x: " << rot << endl;
	switch (rot)
	{
		// move left
		case SERVO_LEFT:
			//		softServoWrite(0, 375);
			gpioServo(17, 1450);
			break;
			// stop
		case SERVO_STOP:
			gpioServo(17, 1500);
			//		digitalWrite(0,0);	
			break;
			// move right
		case SERVO_RIGHT:
			gpioServo(17, 1550);
			//		softServoWrite(0, 525);
			break;
			// stop
		default:
			//		digitalWrite(0,0);	
			break;
	}
	//softServoWrite(0, 400);
}

void moveServoY(int rot)
{
	if (prevRotY == rot) return;
	prevRotY = rot;
	cout << "turn y: " << rot << endl;
	switch (rot)
	{
		// move left

		case SERVO_UP:
			gpioServo(18, 1425);
			//		softServoWrite(1, 375);
			break;
			// stop
		case SERVO_STOP:
			gpioServo(18, 1475);
			//		digitalWrite(1,0);	
			break;
			// move right
		case SERVO_DOWN:
			gpioServo(18, 1525);
			//		softServoWrite(1, 500);
			break;
			// stop
		default:
			//		digitalWrite(1,0);	
			break;
	}
	//softServoWrite(0, 400);
}

void servoTest(int rot)
{
	int i = 0;
	while (i<50)
	{
		moveServoX(rot);
		gpioDelay(10000);
		i++;
	}
}

void aimServoTowards(Point p)
{
	int cx = CAM_W / 2;
	cout << "x :" << p.x << endl;
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
	cout << "y :" << p.y << endl;
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
	gpioDelay(9000);
}

void initializeGpioPort()
{
	gpioInitialise();
	gpioWrite(27,1);
	moveServoX(0);
	moveServoY(0);
}

void toggleGoPro(){
	gpioWrite(27,0);
	gpioDelay(3000000);
	gpioWrite(27,1);
}

void testServos()
{
	servoTest(0);
	servoTest(-1);
	servoTest(0);
	servoTest(1);
	servoTest(0);
	gpioTerminate();
}
#else
int winDelay = 50;
#endif

int main(int argc, const char** argv)
{
	Mat frame, image;
	MeanShiftTracker meanShiftTracker = MeanShiftTracker();

	float sizeThresh = CAM_W * CAM_H * 0.8;
#ifdef ON_PI
	initializeGpioPort();
	//toggleGoPro();
	//gpioDelay(5000000);
	//toggleGoPro();

	raspicam::RaspiCam_Cv Camera;
	Camera.set(CV_CAP_PROP_FORMAT, CV_8UC3);
	Camera.set(CV_CAP_PROP_FRAME_WIDTH, CAM_W);
	Camera.set(CV_CAP_PROP_FRAME_HEIGHT, CAM_H);
	if (!Camera.open())
	{
		cout << "Cannot open the web cam" << endl;
		return -1;
	}

	Camera.grab();
	Camera.retrieve(frame);

	int initServoFlag = 1;
	time_t timeV = time(0);

	std::string out = "out";
	struct tm  now = *localtime(&timeV); 
	char buf[80];
	strftime(buf, sizeof(buf), "%Y-%m-%d_%H-%M-%S", &now);
	std::string name = out + buf + ".avi";	

	VideoWriter video(name, CV_FOURCC('M', 'J', 'P', 'G'), 24, Size(CAM_W, CAM_H), true);
#else

	VideoCapture cap;
	cap.open(0);
	cap.set(CAP_PROP_FRAME_WIDTH, CAM_W);
	cap.set(CAP_PROP_FRAME_HEIGHT, CAM_H);
	cap.set(CAP_PROP_FORMAT, CV_8UC3);

	if (!cap.isOpened())
	{
		cout << "***Could not initialize capturing...***\n";
		cout << "Current parameter's value: \n";
		return -1;
	}
	cap.read(frame);
	cvWaitKey(winDelay);

#endif // ON_PI

	if (frame.empty())
		return 0;

	MotionTracker motionTracker = MotionTracker(frame);

	bool start = false;
	int frameCounter = 0;

	cout << "**********************" << endl;
	cout << "Entering main loop:" << endl;
	cout << "**********************" << endl;

	while (frameCounter < frameMax)
	{
		frameCounter++;

#ifdef ON_PI
		Camera.grab();
		Camera.retrieve(frame);
#else
		cap.read(frame);
#endif // ON_PI

		if (frame.empty())
			break;

		if (!start)
		{
			image = motionTracker.process(frame);
			start = motionTracker.objectCaptured();
			if (start)
			{
				meanShiftTracker.~MeanShiftTracker();
				meanShiftTracker = MeanShiftTracker(motionTracker.getObject());
			}
		}
		if (start)
		{
			image = meanShiftTracker.process(frame);
			if (image.rows == 1 && image.cols == 1)
			{
				start = false;
				continue;
			}
			float objSize = meanShiftTracker.getObject().size.width * meanShiftTracker.getObject().size.height;
			if (objSize > sizeThresh)
			{
				start = false;
#ifdef ON_PI
				moveServoX(0);
				moveServoY(0);
#endif
				continue;
			}

#ifdef ON_PI
			Point p = meanShiftTracker.getObject().center;
			if (p.x != 0 || p.y != 0)
			{
				if (initServoFlag == 1)
				{
					cout << "init servos:" << endl;
					//wiringPiSetup();
					//softServoSetup(0, 1, 2, 3, 4, 5, 6, 7);
					gpioSetMode(17, PI_OUTPUT);
					gpioSetMode(18, PI_OUTPUT);
					initServoFlag = 0;
				}

				aimServoTowards(p);
			}
#endif // ON_PI

		}

#ifdef ON_PI
		Mat temp;
		image.copyTo(temp);
		frames.push_back(temp);
#else
		imshow("Track", image);
		cvWaitKey(winDelay);
#endif // ON_PI

	}

#ifdef ON_PI

	moveServoX(SERVO_STOP);
	moveServoY(SERVO_STOP);
	//softServoSetup(-1,-1,-1,-1,-1,-1,-1,-1);

	Camera.release();
	imwrite("firstFrame.jpg", frames.front());
	for (vector<Mat>::iterator it = frames.begin(); it != frames.end(); ++it)
	{
		video.write(*it);
	}
	gpioTerminate();

#else

	cap.release();

#endif // ON_PI

	meanShiftTracker.~MeanShiftTracker();
	motionTracker.~MotionTracker();

	return 0;
}
