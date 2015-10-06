#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/flann/miniflann.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <chrono>
#include <thread>
using namespace cv;
using namespace std;
#include <math.h>
static const double pi = 3.14159265358979323846;

int lowX;
int highX;
int lowY;
int highY;
bool track;
int num;
Mat imgOriginal;
Mat trackMatrix;

/* I'm hardcoding this at 400. But you should make this a #define so that you can
* change the number of features you use for an accuracy/speed tradeoff analysis.
*/
static const int number_of_features = 100;

inline static double square(int a)
{
	return a * a;
}
/* This is just an inline that allocates images. I did this to reduce clutter in the
* actual computer vision algorithmic code. Basically it allocates the requested image
* unless that image is already non-NULL. It always leaves a non-NULL image as-is even
* if that image's size, depth, and/or channels are different than the request.
*/
inline static void allocateOnDemand(IplImage **img, CvSize size, int depth, int channels
	)
{
	if (*img != NULL) return;
	*img = cvCreateImage(size, depth, channels);
	if (*img == NULL)
	{
		fprintf(stderr, "Error: Couldn't allocate image. Out of memory?\n");
		exit(-1);
	}
}
int main(void)
{
	/* Create an object that decodes the input video stream. */
	VideoCapture cap(0);

	lowX = 0;
	highX = 500;
	lowY = 0;
	highY = 500;
	track = false;
	num = 0;

	//Create trackbars in "Control" window
	bool first = false;
	if (!cap.isOpened())
	{
		/* Either the video didn't exist OR it uses a codec OpenCV
		* doesn't support.
		*/
		fprintf(stderr, "Error: Can't open video.\n");
		return -1;
	}

	/* Set the default frame size. */
	CvSize frame_size;
	frame_size.height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
	frame_size.width = cap.get(CV_CAP_PROP_FRAME_WIDTH);

	/* Make the tracking box control window */
	namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
	cvCreateTrackbar("Low X", "Control", &lowX, frame_size.width); //Hue (0 - 179)
	cvCreateTrackbar("High x", "Control", &highX, frame_size.width);
	highX = frame_size.width;
	cvCreateTrackbar("Low Y", "Control", &lowY, frame_size.height); //Saturation (0 - 255)
	cvCreateTrackbar("High Y", "Control", &highY, frame_size.height);
	cvCreateTrackbar("Hack", "Control", &num, 1);
	highY = frame_size.height;
	
	/* Determine the number of frames in the AVI. */
	long number_of_frames;
	/* Go to the end of the AVI (ie: the fraction is "1") */
	cap.set(CV_CAP_PROP_POS_AVI_RATIO, 1.);
	/* Now that we're at the end, read the AVI position in frames */
	number_of_frames = (int) cap.get(CV_CAP_PROP_POS_FRAMES);
	/* Return to the beginning */
	cap.set(CV_CAP_PROP_POS_FRAMES, 0.);
	/* Create three windows called "Frame N", "Frame N+1", and "Optical Flow"
	* for visualizing the output. Have those windows automatically change their
	* size to match the output.
	*/
	cvNamedWindow("Optical Flow", CV_WINDOW_AUTOSIZE);
	long current_frame = 0;
	while (true)
	{
		if (num == 1 && !track)
		{
			frame_size.height = highY - lowY;
			frame_size.width = highX - lowX;

			#pragma region Optical Flow
			static IplImage *frame = NULL, *frame1 = NULL, *frame1_1C = NULL, *frame2_1C =
				NULL, *eig_image = NULL, *temp_image = NULL, *pyramid1 = NULL, *pyramid2 = NULL;
			/* Go to the frame we want. Important if multiple frames are queried in
			* the loop which they of course are for optical flow. Note that the very
			* first call to this is actually not needed. (Because the correct position
			* is set outsite the for() loop.)
			*/
			cap.set(CV_CAP_PROP_POS_FRAMES, current_frame);
			/* Get the next frame of the video.
			* IMPORTANT! cvQueryFrame() always returns a pointer to the _same_
			* memory location. So successive calls:
			* frame1 = cvQueryFrame();
			* frame2 = cvQueryFrame();
			* frame3 = cvQueryFrame();
			* will result in (frame1 == frame2 && frame2 == frame3) being true.
			* The solution is to make a copy of the cvQueryFrame() output.
			*/

			allocateOnDemand(&frame, frame_size, IPL_DEPTH_8U, 1);
			Mat image;
			bool goodRead = cap.read(image);
			if (!goodRead)
			{
				/* Why did we get a NULL frame? We shouldn't be at the end. */
				fprintf(stderr, "Error: Hmm. The end came sooner than we thought.\n");
				return -1;
			}
			image = image(Rect(lowX, lowY, highX - lowX, highY - lowY));

			*frame = image;
			/* Allocate another image if not already allocated.
			* Image has ONE challenge of color (ie: monochrome) with 8-bit "color" depth.
			* This is the image format OpenCV algorithms actually operate on (mostly).
			*/
			allocateOnDemand(&frame1_1C, frame_size, IPL_DEPTH_8U, 1);
			/* Convert whatever the AVI image format is into OpenCV's preferred format.
			* AND flip the image vertically. Flip is a shameless hack. OpenCV reads
			* in AVIs upside-down by default. (No comment :-))
			*/
			cvConvertImage(frame, frame1_1C, NULL);
			/* We'll make a full color backup of this frame so that we can draw on it.
			* (It's not the best idea to draw on the static memory space of cvQueryFrame().)
			*/
			allocateOnDemand(&frame1, frame_size, IPL_DEPTH_8U, 3);
			cvConvertImage(frame, frame1, NULL);
			/* Get the second frame of video. Sample principles as the first. */
			goodRead = cap.read(image);
			if (!goodRead)
			{
				/* Why did we get a NULL frame? We shouldn't be at the end. */
				fprintf(stderr, "Error: Hmm. The end came sooner than we thought.\n");
				return -1;
			}
			image = image(Rect(lowX, lowY, highX - lowX, highY - lowY));
			*frame = image;
			allocateOnDemand(&frame2_1C, frame_size, IPL_DEPTH_8U, 1);
			cvConvertImage(frame, frame2_1C, NULL);
			/* Shi and Tomasi Feature Tracking! */
			/* Preparation: Allocate the necessary storage. */
			allocateOnDemand(&eig_image, frame_size, IPL_DEPTH_32F, 1);
			allocateOnDemand(&temp_image, frame_size, IPL_DEPTH_32F, 1);
			/* Preparation: This array will contain the features found in frame 1. */
			CvPoint2D32f frame1_features[400];
			/* Preparation: BEFORE the function call this variable is the array size
			* (or the maximum number of features to find). AFTER the function call
			* this variable is the number of features actually found.
			*/

			/* Actually run the Shi and Tomasi algorithm!!
			* "frame1_1C" is the input image.
			* "eig_image" and "temp_image" are just workspace for the algorithm.
			* The first ".01" specifies the minimum quality of the features (based on the
			eigenvalues).
			* The second ".01" specifies the minimum Euclidean distance between features.
			* "NULL" means use the entire input image. You could point to a part of the
			image.
			* WHEN THE ALGORITHM RETURNS:
			* "frame1_features" will contain the feature points.
			* "number_of_features" will be set to a value <= 400 indicating the number of
			feature points found.
			*/
			int num_features = number_of_features;
			cvGoodFeaturesToTrack(frame1_1C, eig_image, temp_image, frame1_features, 
				&num_features, .01, .01, NULL);
			/* Pyramidal Lucas Kanade Optical Flow! */
			/* This array will contain the locations of the points from frame 1 in frame 2. */
			CvPoint2D32f frame2_features[number_of_features];
			/* The i-th element of this array will be non-zero if and only if the i-th feature
			of
			* frame 1 was found in frame 2.
			*/
			char optical_flow_found_feature[number_of_features];
			/* The i-th element of this array is the error in the optical flow for the i-th
			feature
			* of frame1 as found in frame 2. If the i-th feature was not found (see the
			array above)
			* I think the i-th entry in this array is undefined.
			*/
			float optical_flow_feature_error[number_of_features];
			/* This is the window size to use to avoid the aperture problem (see slide
			"Optical Flow: Overview"). */
			CvSize optical_flow_window = cvSize(3, 3);

			/* This termination criteria tells the algorithm to stop when it has either done
			20 iterations or when
			* epsilon is better than .3. You can play with these parameters for speed vs.
			accuracy but these values
			* work pretty well in many situations.
			*/
			CvTermCriteria optical_flow_termination_criteria
				= cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3);
			/* This is some workspace for the algorithm.
			* (The algorithm actually carves the image into pyramids of different resolutions
			.)
			*/
			allocateOnDemand(&pyramid1, frame_size, IPL_DEPTH_8U, 1);
			allocateOnDemand(&pyramid2, frame_size, IPL_DEPTH_8U, 1);
			/* Actually run Pyramidal Lucas Kanade Optical Flow!!
			* "frame1_1C" is the first frame with the known features.
			* "frame2_1C" is the second frame where we want to find the first frame's
			features.
			* "pyramid1" and "pyramid2" are workspace for the algorithm.
			* "frame1_features" are the features from the first frame.
			* "frame2_features" is the (outputted) locations of those features in the second
			frame.
			* "number_of_features" is the number of features in the frame1_features array.
			* "optical_flow_window" is the size of the window to use to avoid the aperture
			problem.
			* "5" is the maximum number of pyramids to use. 0 would be just one level.
			* "optical_flow_found_feature" is as described above (non-zero iff feature found
			by the flow).
			* "optical_flow_feature_error" is as described above (error in the flow for this
			feature).
			* "optical_flow_termination_criteria" is as described above (how long the
			algorithm should look).
			* "0" means disable enhancements. (For example, the second aray isn't preinitialized
			with guesses.)
			*/
			cvCalcOpticalFlowPyrLK(frame1_1C, frame2_1C, pyramid1, pyramid2, frame1_features,
				frame2_features, number_of_features, optical_flow_window, 5,
				optical_flow_found_feature, optical_flow_feature_error,
				optical_flow_termination_criteria, 0);

			/* For fun (and debugging :)), let's draw the flow field. */
			for (int i = 0; i < number_of_features; i++)
			{
				/* If Pyramidal Lucas Kanade didn't really find the feature, skip it. */
				if (optical_flow_found_feature[i] == 0) continue;
				int line_thickness; line_thickness = 1;
				/* CV_RGB(red, green, blue) is the red, green, and blue components
				* of the color you want, each out of 255.
				*/
				CvScalar line_color; line_color = CV_RGB(255, 0, 0);
				/* Let's make the flow field look nice with arrows. */
				/* The arrows will be a bit too short for a nice visualization because of the
				high framerate
				* (ie: there's not much motion between the frames). So let's lengthen them
				by a factor of 3.
				*/
				CvPoint p, q;
				p.x = (int)frame1_features[i].x;
				p.y = (int)frame1_features[i].y;
				q.x = (int)frame2_features[i].x;
				q.y = (int)frame2_features[i].y;
				double angle; angle = atan2((double)p.y - q.y, (double)p.x - q.x);
				double hypotenuse; hypotenuse = sqrt(square(p.y - q.y) + square(p.x - q.x))
					;
				/* Here we lengthen the arrow by a factor of three. */
				q.x = (int)(p.x - 3 * hypotenuse * cos(angle));
				q.y = (int)(p.y - 3 * hypotenuse * sin(angle));
				/* Now we draw the main line of the arrow. */
				/* "frame1" is the frame to draw on.
				* "p" is the point where the line begins.
				* "q" is the point where the line stops.
				* "CV_AA" means antialiased drawing.
				* "0" means no fractional bits in the center cooridinate or radius.
				*/
				cvLine(frame1, p, q, line_color, line_thickness, CV_AA, 0);
				/* Now draw the tips of the arrow. I do some scaling so that the
				* tips look proportional to the main line of the arrow.
				*/
				p.x = (int)(q.x + 9 * cos(angle + pi / 4));
				p.y = (int)(q.y + 9 * sin(angle + pi / 4));
				cvLine(frame1, p, q, line_color, line_thickness, CV_AA, 0);
				p.x = (int)(q.x + 9 * cos(angle - pi / 4));
				p.y = (int)(q.y + 9 * sin(angle - pi / 4));
				cvLine(frame1, p, q, line_color, line_thickness, CV_AA, 0);
			}
			/* Now display the image we drew on. Recall that "Optical Flow" is the name of
			* the window we created above.
			*/
			cvShowImage("Optical Flow", frame1);
			Mat imgOriginal;
			cap.read(imgOriginal);
			imshow("Original", imgOriginal); //show the original image
			//std::this_thread::sleep_for(std::chrono::milliseconds(10));
			
			#pragma endregion
		}
		else
		{
			Mat rectImg;
			bool goodRead = cap.read(rectImg);
			if (!goodRead)
			{
				/* Why did we get a NULL frame? We shouldn't be at the end. */
				fprintf(stderr, "Error: Hmm. The end came sooner than we thought.\n");
				return -1;
			}
			cv::rectangle(rectImg, Point(lowX, lowY), Point(highX, highY), Scalar(0, 0, 255), 1, 8, 0);
			imshow("Original Image", rectImg); //show the thresholded image
		}
		/* And wait for the user to press a key (so the user has time to look at the
		image).
		* If the argument is 0 then it waits forever otherwise it waits that number of
		milliseconds.
		* The return value is the key the user pressed.
		*/
		int key_pressed;
		key_pressed = cvWaitKey(10);
		/* If the users pushes "b" or "B" go back one frame.
		* Otherwise go forward one frame.
		*/
		if (key_pressed == 'b' || key_pressed == 'B') current_frame--;
		else current_frame++;
		/* Don't run past the front/end of the AVI. */
		if (current_frame < 0) current_frame = 0;
		if (current_frame >= number_of_frames - 1) current_frame = number_of_frames - 2;
	}
}