#include "Globals.h"

#ifdef __linux__
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

bool meanShiftMode = true;

vector<Mat> frames;
const int MOVE_DELAY = 0;
const int FRAME_COUNT_THRESHOLD = 100;
const int SERVO_LEFT = 1; //clockwise
const int SERVO_RIGHT = -1; //counterclockwise
const int SERVO_UP = -1;
const int SERVO_DOWN = 1;
const int SERVO_STOP = 0;
const int SERVO_AIM_THRESH_X = CAM_W / 10;
const int SERVO_AIM_THRESH_Y = CAM_H / 8;
const int SERVO_NULL = 99999;
int prevRotX = SERVO_NULL;
int prevRotY = SERVO_NULL;
int startTime =0;
int trackFrameCount = 0;
int moveCount = 0;
int cx = CAM_W / 2;
int cy = CAM_H / 2;

int winDelay = 10;

int thresholdDX = 3;
int moveFlag = 0;

const double RESET_DELAY = 5.0; // in seconds (amount of time that needs to pass
time_t resetStartTime;
const int RESET_STEP_THRESHOLD = 2000;
int xsteps = 0;
//int xdir = SERVO_STOP;
const double RESET_TIME_THRESHOLD = 20.0; // in seconds
time_t moveStartTime; // keeps track of the time the x servo started moving
bool resetting = false;

#ifdef __linux__
bool recording = false;
int fd;

void powerOnGoPro() {
	CURL *curl;

	curl = curl_easy_init();
	if(curl){
		curl_easy_setopt(curl, CURLOPT_URL, "http://10.5.5.9/bacpac/PW?t=goprohero&p=%01");
		curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
		curl_easy_perform(curl);
		curl_easy_cleanup(curl);
	}
}
void powerOffGoPro() {
	CURL *curl;

	curl = curl_easy_init();
	if(curl){
		curl_easy_setopt(curl, CURLOPT_URL, "http://10.5.5.9/bacpac/PW?t=goprohero&p=%00");
		curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
		curl_easy_perform(curl);
		curl_easy_cleanup(curl);
	}
}

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

/***
GIMBLE MOVEMENT CODE
void moveServoX(int rot, int dx)
{
	if (moveCount < MOVE_DELAY) {
		moveCount++;
		return;
	}
	std::stringstream ss;
	std::string step;
	
	int byte = log(abs(dx))/log(10)+3;
	int n;
	if(dx < 20 && dx > -20) {
		cout << "Not in threshold: " << dx << endl;
		return;
	}
	
	//if (prevRotX != SERVO_NULL && rot != SERVO_STOP && rot!=prevRotX) {
		//cout << "difftime :" << difftime(time(0),startTime) << endl;
	//	cout << "BAD DIRECTION" << endl;
	//	return; 
	//}
	if(double(difftime( time(0), startTime))<.01){
		return;
	}
	startTime = time(0);
    if(rot != SERVO_STOP) {
		prevRotX = rot;    
    }
    ss << "P" << dx << "\n";
    step = ss.str();
	
	if (xsteps == 0)
	{
		//xdir = rot;
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
			n = write(fd, step.c_str(), byte+1);
			//gpioServo(17, wq1525);
			//softServoWrite(0, 525);
			break;

		// stop
		default:
			//digitalWrite(0,0);
			cout << "default case" << endl;
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
	while (i<5)
	{
		moveServoX(-1,50);
		gpioDelay(10000);
		i++;
	}
	i=0;
	while (i<5)
	{
		moveServoX(1,-50);
		gpioDelay(10000);
		i++;
	}
	moveServoX(0, 0);
}

Point aimServoTowards(Point p, double dx)
{
	double scalearX =1;
	Point aim = Point(0, 0);
//	moveServoY(SERVO_STOP,0);
	
	if (abs(dx) <= thresholdDX) {
		dx = 0;
	}
	
//	cout << "x :" << p.x << endl;
	if (p.x > cx  && dx > 0)
	{
		cout<< "left :" << dx << endl;
		dx = abs(dx)*scalearX + ((p.x) - (cx/2.0));
		dx = -1*((dx/CAM_W)*535.0)/2;
		if(dx < -535) dx = -535;
		moveServoX(SERVO_LEFT, int(dx));
		aim.x = SERVO_LEFT;
		//moveFlag = 1;
	}
	else if (p.x < cx && dx < 0)
	{
		cout<< "right :" << dx << endl;
		dx = abs(dx)*scalearX + ((3*cx/2) - p.x);
		dx = (dx/CAM_W)*535/2;
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
***/

//OLD GIMBLE CODE
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
#endif // ON_PI
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
#endif // __linux__

int main(int argc, const char** argv)
{
	Point aim = Point(0, 0);
	int i = 0;
	aim.x = -1;
	Mat frame, image;
	MeanShiftTracker meanShiftTracker = MeanShiftTracker();
	//powerOnGoPro();
	float sizeThresh = CAM_W * CAM_H * 0.8;
#ifdef __linux__
	initializeGpioPort();
	// delay to make sure gpioport initializes completely
	while(i<10000000){i++;}	

	//toggleGoPro();
	//gpioDelay(5000000);
	//toggleGoPro();
	openSerialPort();
	while(i<10000000){i++;}	
	
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

#endif // __linux__

	if (frame.empty())
		return 0;

	MotionTracker motionTracker = MotionTracker(frame);

	bool start = false;
	int frameCounter = 0;
	int videoCount = 0;
	cout << "**********************" << endl;
	cout << "Entering Main Loop:" << endl;
	cout << "**********************" << endl;
	

//	while (frameCounter < frameMax)

	while (videoCount < 1)
	{
		frameCounter++;
#ifdef __linux__
		Camera.grab();
		Camera.retrieve(frame);
#else
		cap.read(frame);
#endif // __linux__

		if (frame.empty()){
			cout << "FRAME EMPTY" << endl;
			break;
		}

		if (!resetting)
		{
			if (!meanShiftMode)
			{
				image = motionTracker.process(frame);
				start = motionTracker.objectCaptured();
			}
			else if (!start)
			{
				image = motionTracker.process(frame);
				if (start = motionTracker.objectCaptured())
				{
					meanShiftTracker = MeanShiftTracker(motionTracker.getObject(), motionTracker.getMask());
					image = meanShiftTracker.process(frame);
					if (!(start = !meanShiftTracker.isObjectLost()))
					{
						cout << "Object Lost" << endl;
						motionTracker = MotionTracker(frame);
					}
				}
			}
			else
			{
				image = meanShiftTracker.process(frame);
				if (!(start = !meanShiftTracker.isObjectLost()))
				{
					cout << "Object Lost" << endl;
					motionTracker = MotionTracker(frame);
				}
			}
			
			if (start)
			{
#ifdef __linux__
				if(!recording) {
					startRecording();
					recording = true;
				}
#endif // __linux__

				trackFrameCount++;
				Point p;
				int dx;
				if (meanShiftMode)
				{
					p = meanShiftTracker.getObject().center;
					dx = meanShiftTracker.getDirectionX();
				}
				else
				{
					Rect r = motionTracker.getObject();
					p = Point(r.x + r.width / 2, r.y + r.height / 2);
					dx = motionTracker.getDirectionX();
				}
				cout << "DX: " << dx << endl;

#ifdef __linux__
				//aimServoTowards(p, dx);
				aimServoTowards(p);
#endif // __linux__
			}

#ifdef __linux__
			double dt = abs(double(difftime(time(0), moveStartTime)));
			if (xsteps > 0 && (xsteps > RESET_STEP_THRESHOLD ||  dt > RESET_TIME_THRESHOLD))
			{
				cout << "RESETTING" << endl;
				resetting = true;
				prevRotX *= -1;
				recording = false;
				stopRecording();
				gpioDelay(1000000);
				moveServoX(prevRotX, -1* xsteps * prevRotX);
				resetStartTime = time(0);
				xsteps = 0;
				motionTracker.resetInitial();
			}
#endif // __linux__
		}
		else if (abs(double(difftime(time(0), resetStartTime))) > RESET_DELAY)
		{
			videoCount++;
			resetting = false;
			prevRotX = SERVO_NULL;
			cout << "RESUMING" << endl;
		}

#ifdef __linux__
		// uncomment to view motion tracking threshold image on pi
		//image = motionTracker.getThresholdImage();
		Mat temp;
		image.copyTo(temp);
		frames.push_back(temp);
#else
		imshow("Track", image);
		cvWaitKey(winDelay);
#endif // __linux__

	}
	cout << "**********************" << endl;
	cout << "Exiting Main Loop:" << endl;
	cout << "**********************" << endl;

#ifdef __linux__

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
	//imwrite("firstFrame.jpg", frames.front());
	for (vector<Mat>::iterator it = frames.begin(); it != frames.end(); ++it)
	{
		video.write(*it);
	}
	gpioTerminate();
	close(fd);
	//powerOffGoPro();
#else

	cap.release();

#endif // __linux__

	meanShiftTracker.~MeanShiftTracker();
	motionTracker.~MotionTracker();

	return 0;
}
