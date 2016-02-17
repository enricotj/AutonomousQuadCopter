#include "Globals.h"

#ifdef ON_PI
#include <pigpio.h>
#include <raspicam/raspicam.h>
#include <raspicam/raspicam_cv.h>
#include <curl/curl.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <termios.h>
#include <fcntl.h>
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
#include <sstream>
#include <string.h>
using namespace cv;
using namespace std;

int frameMax = 1200;

vector<Mat> frames;
const int MOVE_DELAY = 3;
const int FRAME_COUNT_THRESHOLD = 100;
const int SERVO_LEFT = -1; //clockwise
const int SERVO_RIGHT = 1; //counterclockwise
const int SERVO_UP = -1;
const int SERVO_DOWN = 1;
const int SERVO_STOP = 0;
const int SERVO_AIM_THRESH_X = CAM_W / 10;
const int SERVO_AIM_THRESH_Y = CAM_H / 8;
const int SERVO_NULL = 99999;
int prevRotX = SERVO_NULL;
int prevRotY = SERVO_NULL;
int startTime;
int trackFrameCount = 0;
int moveCount = 0;
int cx = CAM_W / 2;
int cy = CAM_H / 2;

int winDelay = 80;

int thresholdDX = 3;
int moveFlag = 0;

const double RESET_DELAY = 5.0; // in seconds (amount of time that needs to pass
time_t resetStartTime;
const int RESET_STEP_THRESHOLD = 1350;
int xsteps = 0;
int xdir = SERVO_STOP;
const double RESET_TIME_THRESHOLD = 10.0; // in seconds
time_t moveStartTime; // keeps track of the time the x servo started moving
bool resetting = false;

#ifdef ON_PI
bool recording = false;
int fd;

void startRecording() {
	CURL *curl;

	curl = curl_easy_init();
	if(curl){
		curl_easy_setopt(curl, CURLOPT_URL, "http://10.5.5.9/bacpac/SH?t=goprohero&p=%01");
		curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
		curl_easy_perform(curl);
		curl_easy_cleanup(curl);
	}
	recording = true;
}

void stopRecording() {
	CURL *curl;

	curl = curl_easy_init();
	if(curl){
		curl_easy_setopt(curl, CURLOPT_URL, "http://10.5.5.9/bacpac/SH?t=goprohero&p=%00");
		curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
		curl_easy_perform(curl);
		curl_easy_cleanup(curl);
	}
	recording = false;
}

void moveServoX(int rot, int dx)
{
	if (moveCount < MOVE_DELAY) {
		moveCount++;
		return;
	}
	std::stringstream ss;
	std::string step;
	
	int byte = 10;
	int n;
	if(dx < 20 && dx > -20) {
		cout << "Not in threshold" << endl;
		return;
	}
	
	if (prevRotX != SERVO_NULL && rot != SERVO_STOP && rot!=prevRotX) {
		//cout << "difftime :" << difftime(time(0),startTime) << endl;
		//if(double(difftime( time(0), startTime))<.05)
		return; 
	}
	startTime = time(0);
    if(rot != SERVO_STOP) {
		prevRotX = rot;    
    }
    ss << dx << "\n";
    step = ss.str();
	
	if (xsteps == 0)
	{
		xdir = rot;
		moveStartTime = time(0);
	}
	xsteps += abs(dx);
	    
    switch (rot)
    {
    	// move left
    	case SERVO_RIGHT:
			//softServoWrite(0, 375);
    		//gpioServo(17, 1465);
			cout << rot << ": move x " << dx << " byte: " << byte << endl;
			n = write(fd, step.c_str(), byte);
			break;

		// stop
		case SERVO_STOP:
			//gpioServo(17, 1500);
			//digitalWrite(0,0);
			n = write(fd,"0\n",2);
			break;

		// move right
		case SERVO_LEFT:
			cout << rot << ": move x " << dx << " byte: " << byte << endl;
			n = write(fd, step.c_str(), byte);
			//gpioServo(17, wq1525);
			//softServoWrite(0, 525);
			break;

		// stop
		default:
			//digitalWrite(0,0);	
			n = write(fd,"0\n",2);
			break;
	}
//softServoWrite(0, 400);
}

void moveServoY(int rot, int dy)
{

	int n;
	
	if (prevRotY == rot) {
	    return;
	}
	prevRotY = rot;
	

	switch (rot)
	{
		// move left

		case SERVO_UP:
			gpioServo(17, 1550);
			//		softServoWrite(1, 375);
			//n = write(fd, "T-45\n", 5);
			//n = write(fd, "T-50\n",5);
			break;
			// stop
		case SERVO_STOP:
			gpioServo(17, 1500);
			//		digitalWrite(1,0);	
			//write(fd, "T0\n",3);
			break;
			// move right
		case SERVO_DOWN:
			gpioServo(17, 1450);
			//		softServoWrite(1, 500);
			//n = write(fd, "T90\n", 4);
			//n =write(fd, "T100\n",5);
			break;
			// stop
		default:
			//		digitalWrite(1,0);	
			break;
	}

	//softServoWrite(0, 400);
}

void servoTest()
{
	int i = 0;
	//write(fd, "V90\n", 4);
	while (i<50)
	{
		moveServoX(-1,40);
		gpioDelay(100000);
		i++;
	}
	moveServoX(0, 0);
}

Point aimServoTowards(Point p, double dx)
{
	double scalearX =1;
	Point aim = Point(0, 0);
//	moveServoY(SERVO_STOP,0);
	
	/*if (abs(dx) <= thresholdDX) {
		dx = 0;
	}*/
	
//	cout << "x :" << p.x << endl;
	if (p.x > cx  && dx > 0)
	{
		cout<< "left :" << dx << endl;
		dx = abs(dx)*scalearX + ((p.x) - (cx/2.0));
		dx = -1*((dx/CAM_W)*535.0);
		if(dx < -535) dx = -535;
		moveServoX(SERVO_LEFT, int(dx));
		aim.x = SERVO_LEFT;
		//moveFlag = 1;
	}
	else if (p.x < cx && dx < 0)
	{
		cout<< "right :" << dx << endl;
		dx = abs(dx)*scalearX + ((3*cx/2) - p.x);
		dx = (dx/CAM_W)*535.0;
		if(dx > 535) dx = 535;
		moveServoX(SERVO_RIGHT,int(dx));
		aim.x = SERVO_RIGHT;
		//moveFlag = 1;
	}
	else
	{
		moveServoX(SERVO_STOP, 0);
	}
//	cout << "y :" << p.y << endl;
	if (p.y > cy + SERVO_AIM_THRESH_Y)
	{
		//cout << "MOVING UP" << endl;
//		moveServoY(SERVO_DOWN, cy-p.y);
		aim.y = SERVO_DOWN;
		//moveFlag = 1;
	}
	else if (p.y < cy - SERVO_AIM_THRESH_Y)
	{
		//cout << "MOVING DOWN" << endl;
//		moveServoY(SERVO_UP, cy-p.y);
		aim.y = SERVO_UP;
		//moveFlag = 1;
	}
	else
	{
		//cout << "STOPPING" << endl;
//		moveServoY(SERVO_STOP, 0);
	}
	
			
	
	//moveServoY(SERVO_STOP,0);
	return aim;
}

void initializeGpioPort()
{
	gpioInitialise();
	gpioSetMode(17, PI_OUTPUT);
//	gpioSetMode(18, PI_OUTPUT);
//	gpioWrite(27,1);
//	moveServoX(0);
//	moveServoY(0);
}


int openSerialPort() 
{
	// port file descriptor
	int n;
	struct termios options;
	fd = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY |O_NDELAY);
	if (fd == -1)
	{
		// could not open port
		cout << "openPort: Unable to open /dev/ttyAMA0" << endl;
		return -1;
	}
	else
	{
		fcntl(fd, F_SETFL, 0);
	}
	tcgetattr(fd,&options);
	options.c_cflag = B115200 | CS8 | CLOCAL |CREAD;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(fd,TCIFLUSH);
	tcsetattr(fd,TCSANOW, &options);
	n = write(fd, "V70\n", 4);
	cout << "first write :" << n << endl;
	if (n < 0)
	{
		cout << "write() of 4 bytes failed!" << endl;
		return -1;
	}
	return fd;
}

void toggleGoPro(){
	gpioWrite(27,0);
	gpioDelay(3000000);
	gpioWrite(27,1);
}
#endif

int main(int argc, const char** argv)
{
	Point aim = Point(0, 0);
	int i = 0;
	aim.x = -1;
	Mat frame, image;
	MeanShiftTracker meanShiftTracker = MeanShiftTracker();

	float sizeThresh = CAM_W * CAM_H * 0.8;
#ifdef ON_PI
	initializeGpioPort();
	while(i<10000000){i++;}	

	//toggleGoPro();
	//gpioDelay(5000000);
	//toggleGoPro();
	openSerialPort();
	while(i<10000000){i++;}	
//	moveServoX(-1,-2000);
//	return 0;

	raspicam::RaspiCam_Cv Camera;
	Camera.set(CV_CAP_PROP_FORMAT, CV_8UC3);
	Camera.set(CV_CAP_PROP_FRAME_WIDTH, CAM_W);
	Camera.set(CV_CAP_PROP_FRAME_HEIGHT, CAM_H);
	if (!Camera.open())
	{
		cout << "Cannot open the web cam" << endl;
		return -1;
	}
	gpioDelay(200000);
	moveServoX(SERVO_STOP, 0);
	gpioDelay(200000);
	moveServoY(SERVO_STOP, 0);
	gpioDelay(200000);

	Camera.grab();
	Camera.retrieve(frame);
	
	time_t timeV = time(0);

	std::string out = "out";
	struct tm  now = *localtime(&timeV); 
	char buf[80];
	strftime(buf, sizeof(buf), "%Y-%m-%d_%H-%M-%S", &now);
	std::string name = out + buf + ".avi";	
	//name = "out.avi";
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
	cout << "Entering Main Loop:" << endl;
	cout << "**********************" << endl;

#ifdef ON_PI
	while (frameCounter < frameMax)
#else
	while (true)
#endif
	{
		frameCounter++;
#ifdef ON_PI
		Camera.grab();
		Camera.retrieve(frame);
#else
		cap.read(frame);
#endif // ON_PI

		if (frame.empty()){
			cout << "FRAME EMPTY" << endl;
			break;
		}

		if (!resetting)
		{
			image = motionTracker.process(frame);
			start = motionTracker.objectCaptured();

			if (start)
			{
				trackFrameCount++;
				Rect r = motionTracker.getObject();
				Point p = Point(r.x + r.width / 2, r.y + r.height / 2);
				int dx = motionTracker.getDirectionX();
				cout << "DX: " << dx << endl;
#ifdef ON_PI
				aimServoTowards(p, dx);
#endif // ON_PI
			}

			double dt = abs(double(difftime(time(0), moveStartTime)));
			if (xsteps > 0 && (xsteps > RESET_STEP_THRESHOLD ||  dt > RESET_TIME_THRESHOLD))
			{
				resetting = true;
				prevRotX *= -1;
				moveServoX(prevRotX, xsteps * prevRotX);
				resetStartTime = time(0);
				xsteps = 0;
			}
		}
		else if (abs(double(difftime(time(0), resetStartTime))) > RESET_DELAY)
		{
			resetting = false;
		}
		
		
		/* // OLD CODE
		if (!start)
		{
			trackFrameCount = 0;
			image = motionTracker.process(frame);
			start = motionTracker.objectCaptured();
			if (start)
			{
				cout << "STARTING !!!!!!!" << endl;
				meanShiftTracker.~MeanShiftTracker();
				meanShiftTracker = MeanShiftTracker(motionTracker.getObject());
#ifdef ON_PI
				//startRecording();
#endif // ON_PI
			}
		}
		if (start)
			image = meanShiftTracker.process(frame);
			float objSize = meanShiftTracker.getObject().size.width * meanShiftTracker.getObject().size.height;
			if ((image.rows == 1 && image.cols == 1) || meanShiftTracker.isObjectLost()
					|| objSize > sizeThresh || trackFrameCount > FRAME_COUNT_THRESHOLD)
			{
				start = false;
#ifdef ON_PI			prevXRot = 99999;
				moveServoX(0, 0);
				moveServoY(0, 0);
				//stopRecording();
#endif
				continue;
			}
			
			Point p = meanShiftTracker.getObject().center;
			if (p.x != 0 || p.y != 0) {
				//...
			}
			// OLD CODE HERE
			if(start)
			{
				int dx = meanShiftTracker.getDirectionX();
			//	int dx = motionTracker.getDirectionX();
			//	if (!prevStart)
			//	{
			//		dx = motionTracker.getDirectionX();
			//	}
				if (dx != 0)
				{
#ifndef ON_PI
					line(image, Point(cx, cy), Point(cx + dx * 5, cy), Scalar(0, 255, 0), 3);
					circle(image, p, 4, Scalar(0, 0, 255), -1);
					Point aim = aimCheck(p, dx);
					cout << aim.x << ", " << aim.y << endl;
#endif // !ON_PI
				}

#ifdef ON_PI
			//	aimServoTowards(p, dx);
			//	meanShiftTracker.correctForServoMotion(aimServoTowards(p));
#endif // ON_PI

			//}
		}
		*/

#ifdef ON_PI
		// uncomment to view motion tracking threshold image on pi
		//image = motionTracker.getThresholdImage();
		Mat temp;
		image.copyTo(temp);
		frames.push_back(temp);
#else
		imshow("Track", image);
		cvWaitKey(winDelay);
#endif // ON_PI

	}
	cout << "**********************" << endl;
	cout << "Exiting Main Loop:" << endl;
	cout << "**********************" << endl;

#ifdef ON_PI

	if (recording) 
	{
		//stopRecording();
	}
	gpioDelay(200000);
	moveServoX(SERVO_STOP, 0);
	gpioDelay(200000);
	moveServoY(SERVO_STOP, 0);
	gpioDelay(200000);
	//softServoSetup(-1,-1,-1,-1,-1,-1,-1,-1);
	cout << "STOPPING" << endl;
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

//	meanShiftTracker.~MeanShiftTracker();
	motionTracker.~MotionTracker();

	return 0;
}
