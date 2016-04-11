#include "Clusters.h"

Clusters::Clusters()
{
}

Clusters::~Clusters()
{
}

void Clusters::update(Mat img)
{
	RNG rng(12345);
	int k, clusterCount = rng.uniform(2, MAX_CLUSTERS + 1);
	int i, sampleCount = rng.uniform(1, 1001);
	Mat points(sampleCount, 1, CV_32FC2), labels;
	clusterCount = MIN(clusterCount, sampleCount);
	Mat centers;
	/* generate random sample from multigaussian distribution */
	for (k = 0; k < clusterCount; k++)
	{
		Point center;
		center.x = rng.uniform(0, img.cols);
		center.y = rng.uniform(0, img.rows);
		Mat pointChunk = points.rowRange(k*sampleCount / clusterCount,
			k == clusterCount - 1 ? sampleCount :
			(k + 1)*sampleCount / clusterCount);
		rng.fill(pointChunk, RNG::NORMAL, Scalar(center.x, center.y), Scalar(img.cols*0.05, img.rows*0.05));
	}
	randShuffle(points, 1, &rng);
	kmeans(points, clusterCount, labels,
		TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 10, 1.0),
		2, KMEANS_PP_CENTERS, centers);

	float bs[MAX_CLUSTERS];
	float gs[MAX_CLUSTERS];
	float rs[MAX_CLUSTERS];
	int cs[MAX_CLUSTERS];
	for (i = 0; i < MAX_CLUSTERS; i++)
	{
		bs[i] = 0;
		gs[i] = 0;
		rs[i] = 0;
		cs[i] = 0;
		colors[i] = Scalar(0, 0, 0);
	}

	for (i = 0; i < sampleCount; i++)
	{
		int clusterIdx = labels.at<int>(i);
		Point ipt = points.at<Point2f>(i);
		Point pt = Point((int)ipt.x, (int)ipt.y);
		if (pt.x <= 0 || pt.x >= img.cols || pt.y <= 0 || pt.y >= img.rows)
		{
			continue;
		}
		float b = img.at<Vec3b>(pt)[0];
		float g = img.at<Vec3b>(pt)[1];
		float r = img.at<Vec3b>(pt)[2];
		bs[clusterIdx] += b;
		gs[clusterIdx] += g;
		rs[clusterIdx] += r;
		cs[clusterIdx]++;
	}

	for (i = 0; i < MAX_CLUSTERS; i++)
	{
		int c = cs[i];
		if (c == 0)
		{
			continue;
		}
		float b = bs[i] / c;
		float g = gs[i] / c;
		float r = rs[i] / c;
		colors[i] = Scalar(b, g, r);
	}
}

Scalar Clusters::getColor(int i)
{
	return colors[i];
}