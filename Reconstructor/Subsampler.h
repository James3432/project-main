#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "global_parameters.h"

#include <time.h>

#define isnan(x) (x!=x)

#define MAT_TYPE CV_32FC1

using namespace cv;

#pragma once
class Subsampler
{
public:
	Subsampler(void);
	~Subsampler(void);
	void setInput(Mat in);
	void setOutput(std::vector<int> &samp);
	void setNorms(Mat n);
	virtual Mat run(std::vector<int> &samp) = 0;
	int sign(float i);
	bool isEdge(int p);
protected:
	Mat A;
	std::vector<int> samples;
	Mat Pnorms;
};

class RandomSubsample: public Subsampler
{
	Mat run(std::vector<int> &samp);
};

class NormalSubsample: public Subsampler
{
	Mat run(std::vector<int> &samp);
};

class PoissonSubsample: public Subsampler
{
	Mat run(std::vector<int> &samp);
};