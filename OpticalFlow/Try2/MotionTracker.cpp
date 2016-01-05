#include "MotionTracker.h"

Mat frame1, frame2, grayImage1, grayImage2, thresholdImage, differenceImage;
Rect objectBoundingRectangle;
//our sensitivity value to be used in the absdiff() function
const int SENSITIVITY_VALUE = 50;
//size of blur used to smooth the intensity image output from absdiff() function
const int BLUR_SIZE = 30;
//we'll have just one object to search for
//and keep track of its position.
Point theObject = Point(0, 0);

int sizeThreshLow = 64*64;
int sizeThreshHigh = 0.75 * 320 * 240;
int dthresh = 32;

//some boolean variables for added functionality
bool objectDetected = false;

MotionTracker::MotionTracker(Mat& initFrame)
{
	initFrame.copyTo(frame1);
	objectBoundingRectangle = Rect(0, 0, 0, 0);
	theObject = Point(0, 0);
}

MotionTracker::~MotionTracker()
{
	frame1.release();
	frame2.release();
	grayImage1.release();
	grayImage2.release();
	differenceImage.release();
	thresholdImage.release();
}

void MotionTracker::searchForMovement(Mat thresholdImage)
{
	//notice how we use the '&' operator for objectDetected and cameraFeed. This is because we wish
	//to take the values passed into the function and manipulate them, rather than just working with a copy.
	//eg. we draw to the cameraFeed to be displayed in the main() function.
	bool objectDetected = false;
	Mat temp;
	thresholdImage.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	//findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );// retrieves all contours
	findContours(temp, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);// retrieves external contours

	//if contours vector is not empty, we have found some objects
	if (contours.size()>0)objectDetected = true;
	else objectDetected = false;

	if (objectDetected)
	{
		for (vector< vector<Point> >::iterator it = contours.begin(); it != contours.end(); ++it)
		{
			vector< vector<Point> > largestContourVec;
			largestContourVec.push_back(*it);
			Rect objRect = boundingRect(largestContourVec.at(0));
			rectangle(frame1, objRect, Scalar(0, 0, 255));
		}
		//the largest contour is found at the end of the contours vector
		//we will simply assume that the biggest contour is the object we are looking for.
		vector< vector<Point> > largestContourVec;
		if (contours.size() < 2) {
			return;
		}
		largestContourVec.push_back(contours.at(contours.size() - 2));
		largestContourVec.push_back(contours.at(contours.size() - 1));
		//make a bounding rectangle around the largest contour then find its centroid
		//this will be the object's final estimated position.
		Rect tempRect = boundingRect(largestContourVec.at(0));
		objectBoundingRectangle = boundingRect(largestContourVec.at(0));
		if (objectCaptured())  {
				objectBoundingRectangle = boundingRect(largestContourVec.at(1));
				if (objectCaptured()) {
					int minX, maxX, minY, maxY;
					if (objectBoundingRectangle.x > tempRect.x + tempRect.width) {
						minX = tempRect.x + tempRect.width;
						maxX = objectBoundingRectangle.x;
					}
					else {
						minX = objectBoundingRectangle.x + objectBoundingRectangle.width;
						maxX = tempRect.x;
					}
					minY = max(tempRect.y, objectBoundingRectangle.y);
					maxY = minY + min(tempRect.height, objectBoundingRectangle.height);
					objectBoundingRectangle = Rect(minX, minY, maxX - minX, maxY - minY);
				}
		}
		
		//make a bounding rectangle around the largest contour then find its centroid
		//this will be the object's final estimated position.
		int xpos = objectBoundingRectangle.x + objectBoundingRectangle.width / 2;
		int ypos = objectBoundingRectangle.y + objectBoundingRectangle.height / 2;

		//update the objects positions by changing the 'theObject' array values
		theObject.x = xpos;
		theObject.y = ypos;
	}
	//make some temp x and y variables so we dont have to type out so much
	int x = theObject.x;
	int y = theObject.y;
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
	if (objectCaptured())
	{
		rectangle(obj, objectBoundingRectangle, Scalar(0, 0, 255));
	}
	else
	{
		rectangle(obj, objectBoundingRectangle, Scalar(255, 255, 0));
	}

	//imshow("Frame", obj);

	frame2.copyTo(frame1);

	return obj;
}


Rect MotionTracker::getObject()
{
	return objectBoundingRectangle;
}

bool MotionTracker::objectCaptured()
{
	int x = objectBoundingRectangle.x + objectBoundingRectangle.width / 2;
	int y = objectBoundingRectangle.y + objectBoundingRectangle.height / 2;
	int cx = frame2.cols / 2;
	int cy = frame2.rows / 2;
	int area = objectBoundingRectangle.area();

	if ( // TARGET OBJECT VALIDITY CONDITIONS
				area > sizeThreshLow
		&&		area < sizeThreshHigh
		//&&	sqrt(pow(cx - x, 2) + pow(cy - y, 2)) < dthresh
		)
	{
		return true;
	}

	return false;
}
