#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
	VideoCapture cap(0); //capture the video from web cam

	if (!cap.isOpened())  // if not success, exit program
	{
		cout << "Cannot open the web cam" << endl;
		return -1;
	}

	int blur = 16;

	int minThresh = 10;
	int maxThresh = 1000;

	namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
	cvCreateTrackbar("Blur", "Control", &blur, 64); //Hue (0 - 179)
	cvCreateTrackbar("MinThresh", "Control", &minThresh, 1000); //Hue (0 - 179)
	cvCreateTrackbar("MaxThresh", "Control", &maxThresh, 1000); //Hue (0 - 179)

	// Setup SimpleBlobDetector parameters.
	SimpleBlobDetector::Params params;
	std::vector<KeyPoint> keypoints;
	Ptr<SimpleBlobDetector> detector;

	// Filter by Area.
	params.filterByArea = true;
	params.minArea = 1000;

	// Filter by Circularity
	params.filterByCircularity = false;
	params.minCircularity = 0.1;

	// Filter by Convexity
	params.filterByConvexity = false;
	params.minConvexity = 0.87;

	// Filter by Inertia
	params.filterByInertia = false;
	params.minInertiaRatio = 0.01;

	while (true)
	{
		Mat im;
		Mat imgOriginal;
		bool bSuccess = cap.read(im); // read a new frame from video

		if (!bSuccess) //if not success, break loop
		{
			cout << "Cannot read a frame from video stream" << endl;
			break;
		}

		flip(im, imgOriginal, 1);
		
		
		// Change thresholds
		params.minThreshold = minThresh;
		params.maxThreshold = maxThresh;
		
		#pragma region Blur
			Mat imgBlur;
			Size blurSize = Size(blur, blur);
			//morphological opening (remove small objects from the foreground)
			erode(imgOriginal, imgBlur, getStructuringElement(MORPH_ELLIPSE, blurSize));
			dilate(imgBlur, imgBlur, getStructuringElement(MORPH_ELLIPSE, blurSize));

			//morphological closing (fill small holes in the foreground)
			dilate(imgBlur, imgBlur, getStructuringElement(MORPH_ELLIPSE, blurSize));
			erode(imgBlur, imgBlur, getStructuringElement(MORPH_ELLIPSE, blurSize));
		#pragma endregion

		Mat samples(imgBlur.rows * imgBlur.cols, 3, CV_32F);
		for (int y = 0; y < imgBlur.rows; y++)
		{
			for (int x = 0; x < imgBlur.cols; x++)
			{
				for (int z = 0; z < 3; z++)
				{
					samples.at<float>(y + x*imgBlur.rows, z) = imgBlur.at<Vec3b>(y, x)[z];
				}
			}
		}

		int clusterCount = 3;
		Mat labels;
		int attempts = 1;
		Mat centers;
		kmeans(samples, clusterCount, labels, TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 10000, 0.0001), attempts, KMEANS_RANDOM_CENTERS, centers);
		
		Mat new_image(imgBlur.size(), imgBlur.type());
		for (int y = 0; y < imgBlur.rows; y++)
		{
			for (int x = 0; x < imgBlur.cols; x++)
			{
				int cluster_idx = labels.at<int>(y + x*imgBlur.rows, 0);
				new_image.at<Vec3b>(y, x)[0] = centers.at<float>(cluster_idx, 0);
				new_image.at<Vec3b>(y, x)[1] = centers.at<float>(cluster_idx, 1);
				new_image.at<Vec3b>(y, x)[2] = centers.at<float>(cluster_idx, 2);
			}
		}
		imshow("clustered image", new_image);
		
		Mat imgray;
		cvtColor(new_image, imgray, CV_BGR2GRAY);
		bitwise_not(imgray, imgray);
		// Set up detector with params
		detector = SimpleBlobDetector::create(params);
		// SimpleBlobDetector::create creates a smart pointer. 
		// So you need to use arrow ( ->) instead of dot ( . )
		detector->detect(imgray, keypoints);

		Mat im_with_keypoints;
		drawKeypoints(imgray, keypoints, im_with_keypoints, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

		//imshow("Blur", imgBlur);

		// Show blobs
		imshow("keypoints", im_with_keypoints);
		
		//imshow("Thresholded Image", imgBlur); //show the thresholded image
		imshow("Original", imgOriginal); //show the original image

		if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
		{
			cout << "esc key is pressed by user" << endl;
			break;
		}
	}

	detector.release();

	return 0;

}