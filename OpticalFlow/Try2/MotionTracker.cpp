#include "MotionTracker.h"
#include "Globals.h"

Mat frame1, frame2, grayImage1, grayImage2, thresholdImage, differenceImage;
Rect objectBoundingRectangle;
//our sensitivity value to be used in the absdiff() function
const int SENSITIVITY_VALUE = 20;
//size of blur used to smooth the intensity image output from absdiff() function
const int BLUR_SIZE = 10;

int sizeThreshLow = (int)pow(32 * CAM_W / 640, 2);
int sizeThreshHigh = (int)(0.8 * CAM_W * CAM_H);
int dthresh = 32;

int prevSize;
Point prevPos;

#ifdef ON_PI
const double MAX_SIZE_DIFF_FACTOR = 1;
const double MAX_POS_DIFF_FACTOR = 1;
#else
const double MAX_SIZE_DIFF_FACTOR = 1;
const double MAX_POS_DIFF_FACTOR = 1;
#endif
int captureThreshold = 1;
int captureCurrent = -1;

//some boolean variables for added functionality
bool objectDetected = false;

int xdir = 0;
int ydir = 0;

MotionTracker::MotionTracker(Mat& initFrame)
{
	initFrame.copyTo(frame1);
	objectBoundingRectangle = Rect(0, 0, 0, 0);
	cout << "Motion Tracker Constructed" << endl;
}

MotionTracker::~MotionTracker()
{
	frame1.release();
	frame2.release();
	grayImage1.release();
	grayImage2.release();
	differenceImage.release();
	thresholdImage.release();
	cout << "Motion Tracker Destructed" << endl;
}

void MotionTracker::searchForMovement(Mat thresholdImage)
{
	//notice how we use the '&' operator for objectDetected and cameraFeed. This is because we wish
	//to take the values passed into the function and manipulate them, rather than just working with a copy.
	//eg. we draw to the cameraFeed to be displayed in the main() function.
	Mat temp;
	thresholdImage.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	//findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );// retrieves all contours
	findContours(temp, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);// retrieves external contours

	//if contours vector is not empty, we have found some objects
	if (contours.size()>0)
	{
		//the largest contour is found at the end of the contours vector
		//we will simply assume that the biggest contour is the object we are looking for.
		vector< vector<Point> > largestContourVec;
		largestContourVec.push_back(contours.at(contours.size() - 1));
		//make a bounding rectangle around the largest contour then find its centroid
		//this will be the object's final estimated position.
		Rect tempRect = boundingRect(largestContourVec.at(0));
		objectBoundingRectangle = boundingRect(largestContourVec.at(0));
	}
}

Mat MotionTracker::process(Mat& frame)
{
	frame.copyTo(frame2);
	// convert frame1 to gray scale for frame differencing
	cv::cvtColor(frame1, grayImage1, COLOR_BGR2GRAY);
	//convert frame2 to gray scale for frame differencing
	cv::cvtColor(frame2, grayImage2, COLOR_BGR2GRAY);

	//cv::imshow("Gray Image", grayImage2);
	//cv::imshow("Threshold Image", thresholdImage);

	//perform frame differencing with the sequential images. This will output an "intensity image"
	//do not confuse this with a threshold image, we will need to perform thresholding afterwards.
	cv::absdiff(grayImage1, grayImage2, differenceImage);
	//threshold intensity image at a given sensitivity value
	cv::threshold(differenceImage, thresholdImage, SENSITIVITY_VALUE, 255, THRESH_BINARY);

	//show the difference image and threshold image
	//cv::imshow("Difference Image", differenceImage);
	//cv::imshow("Threshold Image", thresholdImage);

	//blur the image to get rid of the noise. This will output an intensity image
	cv::blur(thresholdImage, thresholdImage, cv::Size(BLUR_SIZE, BLUR_SIZE));
	//threshold again to obtain binary image from blur output
	cv::threshold(thresholdImage, thresholdImage, SENSITIVITY_VALUE, 255, THRESH_BINARY);

	//show the threshold image after it's been "blurred"
	//imshow("Final Threshold Image", thresholdImage);

	searchForMovement(thresholdImage);

	//show our captured frame
	Mat obj;
	frame2.copyTo(obj);
	objectDetected = validObjectFound();
	if (objectDetected)
	{
		double shrink = 0.65;
		double shrinkInv = 1 - shrink;

		int w = objectBoundingRectangle.width * shrinkInv;
		int h = objectBoundingRectangle.height * shrinkInv;
		
		int dw = objectBoundingRectangle.width * shrink;
		int dh = objectBoundingRectangle.height * shrink;

		int x = objectBoundingRectangle.x + xdir;
		int y = objectBoundingRectangle.y + ydir;
		
		x += dw / 2;
		y += dh / 2;
		if (x < 0)
		{
			x = 0;
		}
		if (y < 0)
		{
			y = 0;
		}
		rectangle(obj, objectBoundingRectangle, Scalar(255, 0, 0));
		objectBoundingRectangle = Rect(x, y, w, h);
		rectangle(obj, objectBoundingRectangle, Scalar(0, 0, 255));
	}
	else
	{
		Scalar c = Scalar(255, 255, 255);
		if (captureCurrent >= 1)
		{
			c = Scalar(255, 255, 0);
		}
		rectangle(obj, objectBoundingRectangle, c);
	}

#ifndef ON_PI
//	imshow("Frame", obj);
#endif

	frame2.copyTo(frame1);

	return obj;
}


Rect MotionTracker::getObject()
{
	return objectBoundingRectangle;
}

bool MotionTracker::objectCaptured()
{
	//return objectDetected;
	return false;
}

bool MotionTracker::validObjectFound()
{
	int area = objectBoundingRectangle.area();

	if (area < sizeThreshLow || area > sizeThreshHigh)
	{
		return false;
	}

	if (captureCurrent == -1)
	{
		prevSize = area;
		prevPos = Point(objectBoundingRectangle.x, objectBoundingRectangle.y);
		captureCurrent++;
		double capthresh = CAM_H * CAM_W / (objectBoundingRectangle.area() * 4);
		captureThreshold = (int)capthresh;
		return false;
	}

	int maxSizeDiff = area / MAX_SIZE_DIFF_FACTOR;
	int sizeDiff = abs(area - prevSize);

	int maxPosDiff = (objectBoundingRectangle.width + objectBoundingRectangle.height) / MAX_POS_DIFF_FACTOR;
	int x = objectBoundingRectangle.x;
	int y = objectBoundingRectangle.y;
	int px = prevPos.x;
	int py = prevPos.y;
	int posDiff = (int)sqrt(pow(x - px, 2) + pow(y - py, 2));
	
	xdir = px - x;
	ydir = py - y;
	prevPos = Point(x, y);
	prevSize = area;
	
	if (sizeDiff < maxSizeDiff
		&& posDiff < maxPosDiff
		&& posDiff > 0
		&& area >= sizeThreshLow
		&& area <= sizeThreshHigh)
	{
		captureCurrent++;
		if (captureCurrent >= captureThreshold)
		{
			return true;
		}
	}
	else
	{
		captureCurrent = -1;
	}

	return false;
}
