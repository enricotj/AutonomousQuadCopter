#include <iostream>
#include <raspicam/raspicam_cv.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <raspicam/raspicam.h>

#include "WiringPi-master/wiringPi/wiringPi.h"
#include "WiringPi-master/wiringPi/softServo.h"
#include <opencv2/core/utility.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/videoio.hpp"

#include <ctype.h>

using namespace cv;
using namespace std;

Mat image;

bool backprojMode = false;
bool selectObject = false;
int trackObject = -1;
Point origin;
Rect selection;
int vmin = 10, vmax = 256, smin = 30;

using namespace cv;
using namespace std;

/*void moveServo(int rot)
{
	switch (rot)
	{
		// move left
	case -1:
		softServoWrite(0, 0);
		break;
		// stop
	case 0:
		softServoWrite(0, 400);
		break;
		// move right
	case 1:
		softServoWrite(0, 1000);
		break;
		// stop
	default:
		softServoWrite(0, 400);
		break;
	}
}
*/
int lowX;
int highX;
int lowY;
int highY;
vector<Mat> frames;
int main(int argc, char** argv)
{
	//wiringPiSetup();
	//softServoSetup(0, 1, 2, 3, 4, 5, 6, 7);

	raspicam::RaspiCam_Cv Camera; //Cmaera object
	// Open Camera
	Camera.set(CV_CAP_PROP_FORMAT, CV_8UC3);
	Camera.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	Camera.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	namedWindow("Test", 0);
	if (!Camera.open())
	{
		cout << "Cannot open the web cam" << endl;
		return -1;
	}
	lowX = 319 - 32;
	highX = 319 + 32;
	lowY = 239 - 32;
	highY = 239 + 32;
	
	Rect trackWindow;
	int hsize = 16;
	float hranges[] = { 0, 180 };
	const float* phranges = hranges;
	int i = 0;
	while (true){
		i++;
		if (i > 13000000) break;
	}
	Mat frame, hsv, hue, mask, hist, histimg = Mat::zeros(640, 480, CV_8UC3), backproj;
	VideoWriter video("out.avi", CV_FOURCC('M', 'J', 'P', 'G'), 24, Size(640, 480), true);

	//namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
	//namedWindow("Rectangle Image", CV_WINDOW_KEEPRATIO);

	//Create trackbars in "Control" window
	i = 0;
	while (i < 100)
	{
		/*bool bSuccess = cap.read(imgOriginal); // read a new frame from video
		if (!bSuccess) //if not success, break loop
		{
		cout << "Cannot read a frame from video stream" << endl;
		break;
		}
		*/

		Camera.grab();
		Camera.retrieve(image);
		cvtColor(image, hsv, COLOR_BGR2HSV);
		frames.push_back(image);
		selection.x = MIN(highX, lowX);
		selection.y = MIN(highY, lowY);
		selection.width = std::abs(highX - lowX);
		selection.height = std::abs(highY - lowY);

		selection &= Rect(0, 0, image.cols, image.rows);
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
		//Mat showingImage;
		ellipse(image, trackBox, Scalar(0, 0, 255), 3, LINE_AA);
		//cvtColor(image, showingImage, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
		

		frames.push_back(image);
		i++;
		cout << "loop" << endl;
	}

	for (vector<Mat>::iterator it = frames.begin(); it != frames.end(); ++it)
	{
		video.write(*it);
	}

	Camera.release();
	return 0;

}
