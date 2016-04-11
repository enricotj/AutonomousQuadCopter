#pragma once

#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>

using namespace cv;
using namespace std;

class Clusters
{
public:
	Clusters();
	~Clusters();
	void update(Mat img);
	Scalar getColor(int i);
private:
	static const int MAX_CLUSTERS = 5;
	Scalar colors[MAX_CLUSTERS]; // stores mean colors
};