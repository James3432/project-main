#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "global_parameters.h"
#include "Point3D.h"

#define isnan(x) (x!=x)

#define MAT_TYPE CV_32FC1

using namespace cv;

#pragma once
class PointMatcher
{
public:
	PointMatcher(void);
	~PointMatcher(void);
	void setInput(Mat A, Mat B);
	void setOutput(Mat &out);
	void setNorms(Mat n);
	virtual bool run() = 0;
	float Distance(Point3D A, Point3D B);
protected:
	Mat A; Mat B; Mat out;
	Mat Pnorms;
};

class ColourMatch: public PointMatcher
{
	bool run();
};

class NormalMatch: public PointMatcher
{
	bool run();
};