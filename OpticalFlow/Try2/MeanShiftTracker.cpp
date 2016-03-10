#include "MeanShiftTracker.h"

Mat image, hsv, hue, mask, hist, histimg, backproj;

bool initialze = true;
bool backprojMode = false;
bool selectObject = false;
bool motionCorrectMode = true;
int trackObject = 0;
Point origin;
Rect selection, trackWindow;
RotatedRect trackBox;

int vmin = 50, vmax = 238, smin = 50;

int hsize = 16;

static const int SELECTION_EVENT_A = 0;
static const int SELECTION_EVENT_B = 1;

static const float BIGGEST_OBJECT_SIZE = 0.75 * CAM_H * CAM_W;

float initWidthHeightRatio = -1.0f;

bool objLost = false;

int px = 0;
int dx = 0;

MeanShiftTracker::MeanShiftTracker()
{
	cout << "Mean Shift Tracker Constructed" << endl;
	initWidthHeightRatio = -1.0f;
	objLost = false;
}

MeanShiftTracker::MeanShiftTracker(Rect window)
{
	cout << "Initializing Mean Shift Tracker With Window..." << endl;
	histimg = Mat::zeros(200, 320, CV_8UC3);
	trackWindow = window;

	initSelection(SELECTION_EVENT_A, trackWindow.x, trackWindow.y);
	initSelection(SELECTION_EVENT_B, trackWindow.x + trackWindow.width, trackWindow.y + trackWindow.height);
	px = window.x + window.width / 2;
	cout << "Mean Shift Tracker Constructed With Window" << endl;

	initWidthHeightRatio = -1.0f;

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
	px = trackBox.center.x;
	float hranges[] = { 0, 360 };
	const float* phranges = hranges;
	frame.copyTo(image);
	cvtColor(image, hsv, COLOR_BGR2HSV);

	if (trackObject)
	{
		int _vmin = vmin, _vmax = vmax;

		inRange(hsv,
			Scalar(0, smin, _vmin),
			Scalar(360, 256, _vmax),
			mask);
		int ch[] = { 0, 0 };
		hue.create(hsv.size(), hsv.depth());
		mixChannels(&hsv, 1, &hue, 1, ch, 1);

		if (trackObject < 0)
		{
			try
			{
				Mat roi(hue, selection);
				Mat maskroi(mask, selection);

				calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
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
		trackBox = CamShift(backproj, trackWindow,
			TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 3, 2));
		if (trackWindow.area() <= 1)
		{
			int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5) / 6;
			trackWindow = Rect(trackWindow.x - r, trackWindow.y - r,
				trackWindow.x + r, trackWindow.y + r) &
				Rect(0, 0, cols, rows);
		}

		if (backprojMode)
			cvtColor(backproj, image, COLOR_GRAY2BGR);
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
		return Mat::zeros(1, 1, CV_8UC1);
	}

	dx = trackBox.center.x - px;

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

void MeanShiftTracker::correctForServoMotion(Point aim)
{
	// left is clockwise (-1)
	// right is counterclockwise (+1)
	// up (-1)
	// down (+1)
	if (motionCorrectMode)
	{
		int dx = aim.x * trackWindow.width;
		int dy = aim.y * trackWindow.height * -1;
		trackWindow.x += dx;
		trackWindow.y += dy;
	}
}

int MeanShiftTracker::getDirectionX()
{
	return dx;
}
