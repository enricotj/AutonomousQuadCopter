#pragma once

#include <opencv2/core/utility.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>
#include <ctype.h>
#include "Globals.h"

using namespace cv;
using namespace std;

class MeanShiftTracker
{
public:
	MeanShiftTracker();
	MeanShiftTracker(Rect window, Mat motionMask);
	~MeanShiftTracker();

	Mat process(Mat frame);
	RotatedRect getObject();

	bool isObjectLost();

	int getDirectionX();
	int getDirectionY();
private:
	void initSelection(int event, int x, int y);

	Mat image, hsv, hue, motionMask, mask, hist, histimg, backproj;
};

