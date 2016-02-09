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
	MeanShiftTracker(Rect window);
	~MeanShiftTracker();

	Mat process(Mat frame);
	RotatedRect getObject();

	bool isObjectLost();

	void correctForServoMotion(Point aim);

	int getDirectionX();

private:
	void initSelection(int event, int x, int y);
};

