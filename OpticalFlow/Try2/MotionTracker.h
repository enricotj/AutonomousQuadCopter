#pragma once

//Credit to  Kyle Hounslow, December 2013

//Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software")
//, to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
//and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

//The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
//IN THE SOFTWARE.

#include <opencv2/core/utility.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <ctype.h>
#include <math.h>

using namespace std;
using namespace cv;

class MotionTracker
{

public:
	MotionTracker(Mat initFrame);
	~MotionTracker();
	Mat process(Mat frame);
	Rect getObject();
	bool objectCaptured();

private:
	Mat frame1, frame2, grayImage1, grayImage2, thresholdImage, differenceImage;
	Rect objectBoundingRectangle;
	//our sensitivity value to be used in the absdiff() function
	const int SENSITIVITY_VALUE = 35;
	//size of blur used to smooth the intensity image output from absdiff() function
	const int BLUR_SIZE = 25;
	//we'll have just one object to search for
	//and keep track of its position.
	Point theObject = Point(0, 0);

	int sizeThreshLow = 128 * 128;
	int sizeThreshHigh = 0.75 * 640 * 480;
	int dthresh = 64;

	//some boolean variables for added functionality
	bool objectDetected = false;

	void searchForMovement(Mat thresholdImage);
};

