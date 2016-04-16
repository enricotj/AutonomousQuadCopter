#include "MeanShiftTracker.h"

bool initialze = true;
bool backprojMode = false;
bool selectObject = false;
bool usingMotionMask = true;
int trackObject = 0;
Point origin;
Rect selection, trackWindow;
RotatedRect trackBox;

int vmin = 50, vmax = 230, smin = 50, smax = 256;
int hsize = 16;

static const int SELECTION_EVENT_A = 0;
static const int SELECTION_EVENT_B = 1;

static const float BIGGEST_OBJECT_SIZE = 0.75 * CAM_H * CAM_W;

bool objLost = false;

int px = 0;
int py = 0;
int dx = 0;
int dy = 0;
// keep track of the number of frames the object's x value has not changed
int zeroDxCount = 0;
int zeroDyCount = 0;
// threshold at which no x movement triggers objLost=true event
static const int ZERO_DX_COUNT_THRESHOLD = 2;
static const int ZERO_DY_COUNT_THRESHOLD = 2;
MeanShiftTracker::MeanShiftTracker()
{
	cout << "Mean Shift Tracker Constructed" << endl;
	objLost = false;
}

MeanShiftTracker::MeanShiftTracker(Rect window, Mat initMotionMask)
{
	cout << "Initializing Mean Shift Tracker With Window..." << endl;
	histimg = Mat::zeros(200, 320, CV_8UC3);
	trackWindow = window;

	initMotionMask.copyTo(motionMask);

	initSelection(SELECTION_EVENT_A, trackWindow.x, trackWindow.y);
	initSelection(SELECTION_EVENT_B, trackWindow.x + trackWindow.width, trackWindow.y + trackWindow.height);
	px = window.x + window.width / 2;
	py = window.y + window.height /2;
	cout << "Mean Shift Tracker Constructed With Window" << endl;

	objLost = false;
}

MeanShiftTracker::~MeanShiftTracker()
{
	image.release();
	hsv.release();
	hue.release();
	mask.release();
	hist.release();
	histimg.release();
	backproj.release();
	cout << "Mean Shift Tracker Destructed" << endl;
}

void MeanShiftTracker::initSelection(int event, int x, int y)
{
	if (selectObject)
	{
		selection.x = MIN(x, origin.x);
		selection.y = MIN(y, origin.y);
		selection.width = std::abs(x - origin.x);
		selection.height = std::abs(y - origin.y);
	}

	switch (event)
	{
		case SELECTION_EVENT_A:
			origin = Point(x, y);
			selection = Rect(x, y, 0, 0);
			selectObject = true;
			break;
		case SELECTION_EVENT_B:
			selectObject = false;
			if (selection.width > 0 && selection.height > 0)
				trackObject = -1;
			break;
	}
}

Mat MeanShiftTracker::process(Mat frame)
{
	float hranges[] = { 0, 360 };
	const float* phranges = hranges;
	frame.copyTo(image);
	cvtColor(image, hsv, COLOR_BGR2HSV);

	if (trackObject)
	{
		// create hsv mask
		inRange(hsv,
			Scalar(0, smin, vmin),
			Scalar(360, smax, vmax),
			mask);
		int ch[] = { 0, 0 };
		hue.create(hsv.size(), hsv.depth());
		mixChannels(&hsv, 1, &hue, 1, ch, 1);

		if (trackObject < 0)
		{
			try
			{
				// get hue values in the region of interest
				Mat roi(hue, selection);
				// get the mask for the roi
				Mat maskroi(mask, selection);

				if (usingMotionMask)
				{
					// get the motion mask for the roi
					Mat mmask(motionMask, selection);
					// combine the motion mask and the roi mask (intersection)
					Mat maskFinal;
					bitwise_and(maskroi, mmask, maskFinal);
					//imshow("Mask Final", maskFinal);
					calcHist(&roi, 1, 0, maskFinal, hist, 1, &hsize, &phranges);
				}
				else
				{
					calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
				}
				normalize(hist, hist, 0, 255, NORM_MINMAX);
				trackWindow = selection;
				trackObject = 1;
			}
			catch (Exception e)
			{
				objLost = true;
				return image;
			}
		}

		calcBackProject(&hue, 1, 0, hist, backproj, &phranges, 0.5);
		backproj &= mask;
		trackBox = CamShift(backproj, trackWindow, TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 10, 1));
		
		if (trackWindow.area() <= 1)
		{
			int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5) / 6;
			trackWindow = Rect(trackWindow.x - r, trackWindow.y - r,
				trackWindow.x + r, trackWindow.y + r) &
				Rect(0, 0, cols, rows);
		}

		if (backprojMode)
		{
			cvtColor(backproj, image, COLOR_GRAY2BGR);
		}
		try
		{
			ellipse(image, trackBox, Scalar(0, 0, 255), 3, LINE_AA);
		}
		catch (Exception e)
		{
			objLost = true;
			return image;
		}
	}

	int size = trackWindow.width * trackWindow.height;
	if (size > BIGGEST_OBJECT_SIZE)
	{
		objLost = true;
	}

	dx = trackBox.center.x - px;
	dy = trackBox.center.x - py;
	// check if the object hasn't moved
	if (dx == 0)
	{
		zeroDxCount++;
	} else {
		zeroDxCount = 0;
	}

	if (dy == 0) {
		zeroDyCount++;
	} else {
		zeroDyCount = 0;
	}
	// if the object has not moved in awhile, then the object has been lost
	if (zeroDxCount > ZERO_DX_COUNT_THRESHOLD && zeroDyCount > ZERO_DY_COUNT_THRESHOLD)
	{
		cout << "Object has stopped moving" << endl;
		objLost = true;
	}
	

	px = trackBox.center.x;
	py = trackBox.center.y;
	return image;
}

RotatedRect MeanShiftTracker::getObject()
{
	return trackBox;
}

bool MeanShiftTracker::isObjectLost()
{
	return objLost;
}

int MeanShiftTracker::getDirectionX()
{
	return dx;
}
