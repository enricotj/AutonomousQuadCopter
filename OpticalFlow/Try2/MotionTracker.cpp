#include "MotionTracker.h"
#include "Globals.h"

bool motionOnlyMode = false;

Mat frame1, frame2, grayImage1, grayImage2, thresholdImage, differenceImage;
Rect objectBoundingRectangle;
//our sensitivity value to be used in the absdiff() function
const int SENSITIVITY_VALUE = 24;
//size of blur used to smooth the intensity image output from absdiff() function
const int BLUR_SIZE = 12;

int sizeThreshLow = (int)pow(32 * CAM_W / 640, 2);
int sizeThreshHigh = (int)(0.75 * CAM_W * CAM_H);

int prevSize;
Point prevPos;

const double MAX_SIZE_DIFF_FACTOR = 2.0;
const double MAX_POS_DIFF_FACTOR = 0.6;
int captureThreshold = 0;
int captureCurrent = -1;

//some boolean variables for added functionality
bool objectDetected = false;
bool isMove = false;
int xdir = 0;
int ydir = 0;
int dxMotion = 0;
int xstart = 0;
int ystart = 0;
int xend = 0;
int yend = 0;
int dxPositive;
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

	/// Find the convex hull object for each contour
	/*
	vector<vector<Point> >hull(contours.size());
	for (int i = 0; i < contours.size(); i++)
	{
		if (contours[i].size() > minContourSize)
		{
			convexHull(Mat(contours[i]), hull[i], false);
			drawContours(drawing, hull, i, Scalar(0, 255, 0), 1, 8, vector<Vec4i>(), 0, Point());
			drawContours(drawing, contours, i, Scalar(255, 0, 255), 1, 8, vector<Vec4i>(), 0, Point());
		}
	}
	*/

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

		/*
		// get centroid of largest contour
		Moments m = moments(largestContourVec.at(0), false);
		int cx = m.m10 / m.m00;
		int cy = m.m01 / m.m00;
		
		// get convex hull of largest contour
		Mat drawing = Mat::zeros(temp.size(), temp.type());
		vector<vector<Point>> hull(1);
		convexHull(Mat(largestContourVec.at(0)), hull[0]);
		drawContours(drawing, hull, 0, Scalar(255), CV_FILLED, 8, vector<Vec4i>(), 0, Point());
		//drawContours(drawing, largestContourVec, 0, Scalar(255, 0, 255), 1, 8, vector<Vec4i>(), 0, Point());
		frame2.copyTo(drawing, drawing);
		circle(drawing, Point(cx, cy), 16, Scalar(0, 0, 255), 2, 8, 0);
		imshow("Object", drawing);
		*/
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

	searchForMovement(thresholdImage);

	rectangle(thresholdImage, objectBoundingRectangle, Scalar(255));

	//show the threshold image after it's been "blurred"
#ifndef ON_PI
	//imshow("Final Threshold Image", thresholdImage);
#endif

	Mat obj;
	//show our captured frame
	frame2.copyTo(obj);
	objectDetected = validObjectFound();
	//arrowedLine(obj, Point(xstart, yend), Point(xend, yend), Scalar(255, 255, 255), 1);
	if (objectDetected)
	{
//		cout << "Object detected" << endl;
		double shrink = 0.4;
		double shrinkInv = 1 - shrink;

		int w = objectBoundingRectangle.width * shrinkInv;
		int h = objectBoundingRectangle.height * shrinkInv;
		
		int dw = objectBoundingRectangle.width * shrink;
		int dh = objectBoundingRectangle.height * shrink;

		int x = objectBoundingRectangle.x;
		int y = objectBoundingRectangle.y;
		
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

		//arrowedLine(obj, Point(xstart, yend), Point(xend, yend), Scalar(0, 255, 0), 5, 8, 0, 0.5);
		rectangle(obj, objectBoundingRectangle, Scalar(255, 0, 0), 3);
		objectBoundingRectangle = Rect(x, y, w, h);
		rectangle(obj, objectBoundingRectangle, Scalar(0, 0, 255), 3);
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

Mat MotionTracker::getThresholdImage()
{
	return thresholdImage;
}

Rect MotionTracker::getObject()
{
	return objectBoundingRectangle;
}

bool MotionTracker::shouldMove()
{
	bool temp = false;
	temp = isMove;
	isMove = false;
	return temp;
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
	//return xend - xstart;
	return dxMotion;
}

bool MotionTracker::validObjectFound()
{
	int area = objectBoundingRectangle.area();

	if (area < sizeThreshLow || area > sizeThreshHigh)
	{
		return false;
	}

	int w = objectBoundingRectangle.width;
	int h = objectBoundingRectangle.height;
	int x = objectBoundingRectangle.x + w / 2;
	int y = objectBoundingRectangle.y + h / 2;

	if (captureCurrent == -1)
	{
		prevSize = area;
		prevPos = Point(x, y);
		captureCurrent++;
		//double capthresh = CAM_H * CAM_W / objectBoundingRectangle.area() * 16;
		captureThreshold = 1;
		xstart = x;
		ystart = y;
		return false;
	}

	int maxSizeDiff = area / MAX_SIZE_DIFF_FACTOR;
	int sizeDiff = abs(area - prevSize);
	
	int maxPosDiff = sqrt(pow(w, 2) + pow(h, 2)) / MAX_POS_DIFF_FACTOR;
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
			if (!objectDetected)
			{
				xend = x;
				yend = y;
				int dx = xend - xstart;
				dxMotion = dx;
				if(initial && ((dxMotion > 0 && !dxPositive) || (dxMotion < 0 && dxPositive))) {
					dxMotion = 0;
					return false;

				}
				if (dx < 0)
				{
					xend = 10;
					xstart = CAM_W - 10;
				}
				else if (dx > 0)
				{
					xstart = 10;
					xend = CAM_W - 10;
				}
				if(!initial) {
					dxPositive = dxMotion > 0;
					initial = true;
				}
				//cout << "MotionTracker:: " << dx << endl;
				isMove = true;
			}
			return true;
		}
		
	}
	else
	{
		captureCurrent = -1;
	}

	return false;
}
