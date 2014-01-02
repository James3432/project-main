#include "Point3D.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

#pragma once
/*ref*/ class PointCloud
{
public:
	PointCloud(void);
	PointCloud(Mat m);
	PointCloud(const PointCloud &source);
	~PointCloud();
	PointCloud& operator= (const PointCloud &source);
	void AddPointCloud(PointCloud pc);
	void AddPointCloud(Mat pc);
	void AddPoint(Point3D* p);
	Point3D* GetPoint(int row);
	void DeletePoint();
	Mat GetMatrix();
	void SetMatrix(Mat m);
	int Count();
	int Dimension();
	Mat* cloud;
private:
	
	int rows;
	int cols;
};