#pragma managed(push, off)
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#pragma managed(pop)
#include "Point3D.h"

using namespace cv;

#define MAT_TYPE CV_32FC1

#pragma once
class NormalEstimator
{
public:
	NormalEstimator(Mat* m);
	~NormalEstimator(void);
	void SetPointCloud(Mat m);
	void compute();
	Mat result;
private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr NormsToPcl(pcl::PointCloud<pcl::Normal>::Ptr cloud_normals);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr NormPtsToPcl(pcl::PointCloud<pcl::PointNormal> points);
	void MatToPcl(Mat m, pcl::PointCloud<pcl::PointXYZ>::Ptr pc);
	Mat PclToMat(pcl::PointCloud<pcl::PointXYZ>::Ptr pc);
//	pcl::PointXYZ Point3DToXYZ(Point3D p);
//	Point3D XYZToPoint3D(pcl::PointXYZ p);
};

