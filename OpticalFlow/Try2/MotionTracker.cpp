#include "MotionTracker.h"
#include "Globals.h"

bool motionOnlyMode = false;

Rect objectBoundingRectangle;
//our sensitivity value to be used in the absdiff() function
const int SENSITIVITY_VALUE = 24;
//size of blur used to smooth the intensity image output from absdiff() function
const int BLUR_SIZE = 12;

int sizeThreshLow = (int)pow(28 * CAM_W / 640, 2); // min area that a valid object can take up
int sizeThreshHigh = (int)(0.75 * CAM_W * CAM_H); // max area that a valid object can take up

int prevSize;
Point prevPos;

const double MAX_SIZE_DIFF_FACTOR = 2.0;
const double MAX_POS_DIFF_FACTOR = 0.6;
int captureThreshold = 2;
int captureCurrent = -1;

//some boolean variables for added functionality
bool objectDetected = false;
int dxMotion = 0;
int xstart = 0;
int ystart = 0;
bool dxPositive;
int minContourSize = 1*1;
bool initial = false;

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
	// convert frames to gray scale for frame differencing
	cv::cvtColor(frame1, grayImage1, COLOR_BGR2GRAY);
	cv::cvtColor(frame2, grayImage2, COLOR_BGR2GRAY);
	// get frame difference image
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

	searchForMovement(thresholdImage);

	//show the threshold image after it's been "blurred"
#ifndef ON_PI
	//imshow("Final Threshold Image", thresholdImage);
#endif

	// reset frame1
	frame2.copyTo(frame1);

	// check if a valid object was detected
	objectDetected = validObjectFound();

	// draw bounding boxes
	if (objectDetected)
	{
		// draw the bounding rectangle around the object
		rectangle(frame2, objectBoundingRectangle, Scalar(255, 0, 0), 3);
		
		// shrink the bounding rectangle
		double shrink = 0.4;
		double shrinkInv = 1 - shrink;
		int w = objectBoundingRectangle.width * shrinkInv;
		int h = objectBoundingRectangle.height * shrinkInv;
		int dw = objectBoundingRectangle.width * shrink;
		int dh = objectBoundingRectangle.height * shrink;
		
		// center the shrunken rectangle's coordinates
		int x = objectBoundingRectangle.x;
		int y = objectBoundingRectangle.y;
		x += dw / 2;
		y += dh / 2;

		// draw the smaller rectangle
		objectBoundingRectangle = Rect(x, y, w, h);
		rectangle(frame2, objectBoundingRectangle, Scalar(0, 0, 255), 3);
	}
	else
	{
		// invalid object detected
		Scalar c = Scalar(255, 255, 255);
		if (captureCurrent >= 1)
		{
			c = Scalar(255, 255, 0);
		}
		// draw bounding box of invalid object
		rectangle(frame2, objectBoundingRectangle, c);
	}

#ifndef ON_PI
	imshow("Frame", frame2);
#endif

	return frame2;
}

Mat MotionTracker::getThresholdImage()
{
	return thresholdImage;
}

Rect MotionTracker::getObject()
{
	return objectBoundingRectangle;
}

void MotionTracker::resetInitial()
{
	initial = false;
}

bool MotionTracker::objectCaptured()
{
	if (motionOnlyMode)
	{
		return false;
	}
	else
	{
		return objectDetected;
	}
	
}

int MotionTracker::getDirectionX()
{
	return dxMotion;
}

bool MotionTracker::validObjectFound()
{
	// check if the size of the object is within valid parameters
	int area = objectBoundingRectangle.area();
	if (area < sizeThreshLow || area > sizeThreshHigh)
	{
		return false;
	}

	// width/height of rectangle
	int w = objectBoundingRectangle.width;
	int h = objectBoundingRectangle.height;
	// center coordinates of the rectangle
	int x = objectBoundingRectangle.x + w / 2;
	int y = objectBoundingRectangle.y + h / 2;

	if (captureCurrent == -1)
	{
		prevSize = area;
		prevPos = Point(x, y);
		captureCurrent++;
		//double capthresh = CAM_H * CAM_W / objectBoundingRectangle.area() * 16;
		xstart = x;
		ystart = y;
		return false;
	}

	// maximum expected change in object size
	int maxSizeDiff = area / MAX_SIZE_DIFF_FACTOR;
	// measured change in object size
	int sizeDiff = abs(area - prevSize);
	// reset previous size
	prevSize = area;

	// maximum expected change in object position
	int maxPosDiff = sqrt(pow(w, 2) + pow(h, 2)) / MAX_POS_DIFF_FACTOR;
	// measured change in object position
	int px = prevPos.x;
	int py = prevPos.y;
	int posDiff = (int)sqrt(pow(x - px, 2) + pow(y - py, 2));
	// reset previous position
	prevPos = Point(x, y);
	
	// a valid object's size and position must not vary too much between frames
	if (sizeDiff < maxSizeDiff
		&& posDiff < maxPosDiff
		&& posDiff > 0)
	{
		// object motion found to be consistent
		captureCurrent++;
		if (captureCurrent >= captureThreshold)
		{
			if (!objectDetected)
			{
				dxMotion = x - xstart;
				if(initial && ((dxMotion > 0 && !dxPositive) || (dxMotion < 0 && dxPositive))) {
					dxMotion = 0;
					return false;
				}
				if(!initial) {
					dxPositive = dxMotion >= 0;
					initial = true;
				}
			}
			return true;
		}
		
	}
	else
	{
		// object motion found to be inconsistent
		captureCurrent = -1;
	}

	return false;
}

Mat MotionTracker::getFrame1()
{
	return frame1;
}