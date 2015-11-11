#pragma once

#include <opencv2/core/utility.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>
#include <ctype.h>

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

private:
	Mat image, hsv, hue, mask, hist, histimg, backproj;

	bool initialze = true;
	bool backprojMode = false;
	bool selectObject = false;
	int trackObject = 0;
	bool showHist = true;
	Point origin;
	Rect selection, trackWindow;
	RotatedRect trackBox;

	int vmin = 10, vmax = 256, smin = 30;
	int frameMax = 100;

	int hsize = 16;
	float* phranges;

	bool paused = true;

	static const int SELECTION_EVENT_A = 0;
	static const int SELECTION_EVENT_B = 1;
	int selectState = SELECTION_EVENT_A;

	void initSelection(int event, int x, int y);
};

