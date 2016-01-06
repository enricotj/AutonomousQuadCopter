#include "MeanShiftTracker.h"

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

int hsize = 16;
float* phranges;

static const int SELECTION_EVENT_A = 0;
static const int SELECTION_EVENT_B = 1;

static const float BIGGEST_OBJECT_SIZE = 0.75 * 640 * 480;

MeanShiftTracker::MeanShiftTracker()
{
}

MeanShiftTracker::MeanShiftTracker(Rect window)
{
	histimg = Mat::zeros(200, 320, CV_8UC3);
	trackWindow = window;

	initSelection(SELECTION_EVENT_A, trackWindow.x, trackWindow.y);
	initSelection(SELECTION_EVENT_B, trackWindow.x + trackWindow.width, trackWindow.y + trackWindow.height);
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
	float hranges[] = { 0, 180 };
	const float* phranges = hranges;
	frame.copyTo(image);
	cvtColor(image, hsv, COLOR_BGR2HSV);

	if (trackObject)
	{
		int _vmin = vmin, _vmax = vmax;

		inRange(hsv, Scalar(0, smin, MIN(_vmin, _vmax)),
			Scalar(180, 256, MAX(_vmin, _vmax)), mask);
		int ch[] = { 0, 0 };
		hue.create(hsv.size(), hsv.depth());
		mixChannels(&hsv, 1, &hue, 1, ch, 1);

		if (trackObject < 0)
		{
			Mat roi(hue, selection);
			Mat maskroi(mask, selection);
			calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
			normalize(hist, hist, 0, 255, NORM_MINMAX);

			trackWindow = selection;
			trackObject = 1;

			histimg = Scalar::all(0);
			int binW = histimg.cols / hsize;
			Mat buf(1, hsize, CV_8UC3);
			for (int i = 0; i < hsize; i++)
				buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180. / hsize), 255, 255);
			cvtColor(buf, buf, COLOR_HSV2BGR);

			for (int i = 0; i < hsize; i++)
			{
				int val = saturate_cast<int>(hist.at<float>(i)*histimg.rows / 255);
				rectangle(histimg, Point(i*binW, histimg.rows),
					Point((i + 1)*binW, histimg.rows - val),
					Scalar(buf.at<Vec3b>(i)), -1, 8);
			}
		}

		calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
		backproj &= mask;
		trackBox = CamShift(backproj, trackWindow,
			TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 10, 1));
		if (trackWindow.area() <= 1)
		{
			int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5) / 6;
			trackWindow = Rect(trackWindow.x - r, trackWindow.y - r,
				trackWindow.x + r, trackWindow.y + r) &
				Rect(0, 0, cols, rows);
		}

		if (backprojMode)
			cvtColor(backproj, image, COLOR_GRAY2BGR);
		ellipse(image, trackBox, Scalar(0, 0, 255), 3, LINE_AA);
	}

	int size = trackWindow.width * trackWindow.height;
	if (size > BIGGEST_OBJECT_SIZE)
	{
		return Mat::zeros(1, 1, CV_8UC1);
	}

	//imshow("Track", image);

	return image;
}

RotatedRect MeanShiftTracker::getObject()
{
	return trackBox;
}
