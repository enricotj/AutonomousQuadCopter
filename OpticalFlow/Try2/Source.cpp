#include <opencv2/core/utility.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include <raspicam/raspicam.h>
#include <raspicam/raspicam_cv.h>
#include "WiringPi-master/wiringPi/wiringPi.h"
#include "WiringPi-master/wiringPi/softServo.h"
#include <iostream>
#include <ctype.h>

using namespace cv;
using namespace std;

Mat image;
bool initialze = true;
bool backprojMode = false;
bool selectObject = false;
int trackObject = 0;
bool showHist = true;
Point origin;
Rect selection;
int vmin = 10, vmax = 256, smin = 30;
int frameMax = 100;

vector<Mat> frames;

const int CAM_W = 640;
const int CAM_H = 480;

const int SERVO_LEFT = -1;
const int SERVO_RIGHT = 1;
const int SERVO_STOP = 0;
const int SERVO_AIM_THRESH = 32;

void moveServo(int rot)
{
	switch (rot)
	{
		// move left
	case SERVO_LEFT:
		softServoWrite(0, 0);
		break;
		// stop
	case SERVO_STOP:
		softServoWrite(0, 400);
		break;
		// move right
	case SERVO_RIGHT:
		softServoWrite(0, 1000);
		break;
		// stop
	default:
		softServoWrite(0, 400);
		break;
	}
}

const int SELECTION_EVENT_A = 0;
const int SELECTION_EVENT_B = 1;

void initSelection(int event, int x, int y)
{
	if (selectObject)
	{
		selection.x = MIN(x, origin.x);
		selection.y = MIN(y, origin.y);
		selection.width = std::abs(x - origin.x);
		selection.height = std::abs(y - origin.y);

		selection &= Rect(0, 0, image.cols, image.rows);
	}

	switch (event)
	{
	case SELECTION_EVENT_A:
		origin = Point(x, y);
		selection = Rect(x, y, 0, 0);
		selectObject = true;
		break;
	case SELECTION_EVENT_B:
		selectObject = false;
		if (selection.width > 0 && selection.height > 0)
			trackObject = -1;
		break;
	}
}

void aimServoTowards(Point p)
{
	int cx = CAM_W / 2;
	int dx = p.x - cx;

	if (dx > cx + SERVO_AIM_THRESH)
	{
		moveServo(SERVO_RIGHT);
	}
	else if (dx < cx - SERVO_AIM_THRESH)
	{
		moveServo(SERVO_LEFT);
	}
	else
	{
		moveServo(SERVO_STOP);
	}
}

int main(int argc, const char** argv)
{

	wiringPiSetup();
	softServoSetup(0, 1, 2, 3, 4, 5, 6, 7);
	Rect trackWindow;
	int hsize = 16;
	float hranges[] = { 0, 180 };
	const float* phranges = hranges;
	int camNum = 0;
	int i = 0;

	/*VideoCapture cap;
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
	
	moveServo(SERVO_STOP);

	Mat frame, hsv, hue, mask, hist, histimg = Mat::zeros(200, 320, CV_8UC3), backproj;
	bool paused = false;
	VideoWriter video("out.avi", CV_FOURCC('M', 'J', 'P', 'G'), 24, Size(CAM_W, CAM_H), true);
	int frameCounter = 0;
	while (frameCounter < frameMax)
	{
		frameCounter++;
		if (!paused)
		{
			Camera.grab();
			Camera.retrieve(frame);
			/*cap >> frame;
			if (frame.empty())
				break;*/
		}

		frame.copyTo(image);

		if (!paused)
		{
			cvtColor(image, hsv, COLOR_BGR2HSV);

			if (trackObject)
			{
				int _vmin = vmin, _vmax = vmax;

				inRange(hsv, Scalar(0, smin, MIN(_vmin, _vmax)),
					Scalar(180, 256, MAX(_vmin, _vmax)), mask);
				int ch[] = { 0, 0 };
				hue.create(hsv.size(), hsv.depth());
				mixChannels(&hsv, 1, &hue, 1, ch, 1);

				if (trackObject < 0)
				{
					Mat roi(hue, selection), maskroi(mask, selection);
					calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
					normalize(hist, hist, 0, 255, NORM_MINMAX);

					trackWindow = selection;
					trackObject = 1;

					histimg = Scalar::all(0);
					int binW = histimg.cols / hsize;
					Mat buf(1, hsize, CV_8UC3);
					for (int i = 0; i < hsize; i++)
						buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180. / hsize), 255, 255);
					cvtColor(buf, buf, COLOR_HSV2BGR);

					for (int i = 0; i < hsize; i++)
					{
						int val = saturate_cast<int>(hist.at<float>(i)*histimg.rows / 255);
						rectangle(histimg, Point(i*binW, histimg.rows),
							Point((i + 1)*binW, histimg.rows - val),
							Scalar(buf.at<Vec3b>(i)), -1, 8);
					}
				}

				calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
				backproj &= mask;
				RotatedRect trackBox = CamShift(backproj, trackWindow,
					TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 10, 1));
				if (trackWindow.area() <= 1)
				{
					int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5) / 6;
					trackWindow = Rect(trackWindow.x - r, trackWindow.y - r,
						trackWindow.x + r, trackWindow.y + r) &
						Rect(0, 0, cols, rows);
				}

				if (backprojMode)
					cvtColor(backproj, image, COLOR_GRAY2BGR);
				ellipse(image, trackBox, Scalar(0, 0, 255), 3, LINE_AA);

				aimServoTowards(trackBox.center);
			}
		}
		else if (trackObject < 0)
			paused = false;

		//imshow("Track", image);
		Mat temp;
		image.copyTo(temp);
		frames.push_back(temp);
		char c = (char)waitKey(10);
		//printf("Selection x: %d, y: %d, width: %d, height: %d\n", selection.x, selection.y, selection.width, selection.height);
		//printf("TrackWind x: %d, y: %d, width: %d, height: %d\n", trackWindow.x, trackWindow.y, trackWindow.width, trackWindow.height);
		if (i < 2) {
			waitKey(200);
			if (i == 0) 
				initSelection(i, 295, 215);
			if (i==1)
				initSelection(i, 345, 265);
			i++;
		}

		if (c == 'q')
		{
			break;
		}
	}

	moveServo(SERVO_STOP);

	Camera.release();

	for (vector<Mat>::iterator it = frames.begin(); it != frames.end(); ++it)
	{
		video.write(*it);
	}

	return 0;
}