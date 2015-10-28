#include <iostream>
#include <raspicam/raspicam_cv.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <raspicam/raspicam.h>

using namespace cv;
using namespace std;


int lowX;
int highX;
int lowY;
int highY;
bool track;
int num;
Mat imgOriginal;
Mat trackMatrix;
int xIndex;
int yIndex;

int px = -1;
int py = -1;

void SaveState(int state, void*) {
	trackMatrix = imgOriginal.adjustROI(highY, lowY, lowX, highX);
	track = true;
}

void findObject() {
	float min = -1;
	int xTrackIndex = 0;
	int yTrackIndex = 0;
	int range = (trackMatrix.rows + trackMatrix.cols) / 4;
	int rows = trackMatrix.rows;
	int cols = trackMatrix.cols;
	int yDiff = rows/25;
	int xDiff = cols/25;
	for (int a = yIndex - range; a < yIndex + range; a++) {
		/*if ((a + rows) >= highY){
			break;
		}*/
		for (int b = xIndex - range; b < xIndex + range; b++) {
			if (a < 0){
				a = 0;
			}if (b < 0){
				b = 0;
			}
			/*if ((b +cols) >= highX) {
				break;
			}*/
			float diff = 0;
			//printf("x: %d, y: %d, d:%f\n", b, a, min);
			bool ex = false;
			for (int y = 0; y < rows; y+=yDiff)
			{
				for (int x = 0; x < cols; x+=xDiff)
				{
					try {
						Vec3b imgColor = imgOriginal.at<Vec3b>(Point(b + x, a + y));
						Vec3b color = trackMatrix.at<Vec3b>(Point(x, y));
						diff += abs(imgColor[0] - color[0])*4 + abs(imgColor[1] - color[1]) + abs(imgColor[2] - color[2]);
					}
					catch (Exception e) {
						ex = true;
					}
				}
			}
			if ((min == -1 || diff < min) && !ex) {
				min = diff;
				xTrackIndex = b;
				yTrackIndex = a;
			}
		}
		
	}
	xIndex = xTrackIndex;
	yIndex = yTrackIndex;
}
int main(int argc, char** argv)
{
	//system("sudo modprobe bcm2835-v4l2");
	//system("sudo modprobe v4l2-common");
	//system("v4l2-ctl --overlay=1");
	raspicam::RaspiCam_Cv Camera; //Cmaera object
	// Open Camera
	Camera.set(CV_CAP_PROP_FORMAT, CV_8UC3);   
	if (!Camera.open())
	{
		cout << "Cannot open the web cam" << endl;
		return -1;
	}

	/*VideoCapture cap(0); //capture the video from web cam
	if (!cap.isOpened())  // if not success, exit program
	{
	cout << "Cannot open the web cam" << endl;
	return -1;
	}

	*/
	lowX = 319 - 32;
	highX = 319 + 32;
	lowY = 239 - 32;
	highY = 239 + 32;
	track = false;
	num = -200;
	int i = 0;
	while(true){
		i++;
		if(i>13000000) break;
	}

	VideoWriter video("out.avi",CV_FOURCC('M','J','P','G'),10,Size(1280,960),true);

	//namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
	//namedWindow("Rectangle Image", CV_WINDOW_KEEPRATIO);
	
	//Create trackbars in "Control" window
	bool first = false;
	i =0;
	while (i<20)
	{
		/*bool bSuccess = cap.read(imgOriginal); // read a new frame from video
		if (!bSuccess) //if not success, break loop
		{
			cout << "Cannot read a frame from video stream" << endl;
			break;
		}
		*/
		
		Camera.grab();
		Camera.retrieve(imgOriginal);

		/*
		if (!first) {
			cvCreateTrackbar("Low X", "Control", &lowX, imgOriginal.cols); //Hue (0 - 179)
			cvCreateTrackbar("High x", "Control", &highX, imgOriginal.cols);
			highX = imgOriginal.cols;
			cvCreateTrackbar("Low Y", "Control", &lowY, imgOriginal.rows); //Saturation (0 - 255)
			cvCreateTrackbar("High Y", "Control", &highY, imgOriginal.rows);
			cvCreateTrackbar("Hack", "Control", &num, 1);
			highY = imgOriginal.rows;
			first = true;
			//cvCreateButton("Track", SaveState, NULL, CV_PUSH_BUTTON, 1);
			//createButton("Track", SaveState, NULL, 0, false);
		}
		*/
		if (num == 1 && !track) {
			Mat imgHSV;
			imgOriginal.copyTo(imgHSV);
			trackMatrix = imgHSV(Rect(lowX, lowY, highX - lowX, highY - lowY));
			xIndex = lowX;
			yIndex = lowY;
			px = lowX;
			py = lowY;
			track =true;
		}
		else
		{
			num++;
		}
		//cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
		

		//imshow("Original", imgOriginal); //show the original image
		
		Point low = Point(lowX, lowY);
		Point high = Point(highX, highY);
		if (track) {
			findObject();
			int dx = xIndex - px;
			int dy = yIndex - py;
			if (sqrt(pow(dx, 2) + pow(dy, 2)) > 1)
			{
				xIndex += (xIndex - px) / 2;
				yIndex += (yIndex - py) / 2;
			}
			low = Point(xIndex, yIndex);
			high = Point(xIndex + (highX - lowX), yIndex + (highY - lowY));
			px = xIndex;
			py = yIndex;
		}
		Mat rectImg;
		imgOriginal.copyTo(rectImg);
		cv::rectangle(rectImg, low, high, Scalar(0, 0, 255), 1, 8, 0);
		//rectImg.convertTo(rectImg, CV_8UC3, 255.0);
		imwrite("raspicam_cv_image.jpg", rectImg);
		video.write(rectImg);
		//imshow("Rectangle Image", rectImg); //show the thresholded image
		if (waitKey(10) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
		{
			cout << "esc key is pressed by user" << endl;
			Camera.release();
			break;
		}
		i++;
	}
	Camera.release();
	return 0;

}
