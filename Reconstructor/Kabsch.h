#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include "Point3D.h"
#include "Eigen\Eigen"
#include "opencv2\core\eigen.hpp"

#define MAT_TYPE CV_32FC1

using namespace cv;

#pragma once
/*ref*/ class Kabsch
{
public:
	Kabsch(Mat P, Mat Q);
	~Kabsch();
	Mat GetRotation();
	Mat GetTranslation();
	float GetError();	

	Mat* p0; // Centroid of P
	Mat* q0; // Centroid of Q
private:
	// Optimal rotation matrix
	Mat* U;

	// Optimal translation matrix
	Mat* Trans;

	// Root mean squared error
	float err;

	// Internal Methods
	Point3D Centroid(Mat m);
	Mat Normalise(Mat m);
	Mat Covariance(Mat P, Mat Q);
	Mat OptimalRotate(Mat A);
	Mat PointToMat(Point3D p);
	float CalcError(Mat diff);
};

