#include "StdAfx.h"
#include "PointCloud.h"

//-------------------------
//
//   Pointclouds are represented in matrix format as:
//
//   +----+----+----+
//   | x1 | y1 | z1 |
//   +----+----+----+
//   |  . |  . |  . |
//   |  . |  . |  . | 
//   +----+----+----+
//   | xn | yn | zn |
//   +----+----+----+
//
//   Whereby each point is stored in a row
//
//-------------------------

//---------------------------------------------
// New empty pointcloud
//---------------------------------------------
PointCloud::PointCloud(void)
{
	rows = 0;
	cols = 6;
	cloud = new Mat(0,6,CV_32FC1);
}

PointCloud::PointCloud(const PointCloud &source)
{
	cloud = new Mat();
	*cloud = *(source.cloud);
	rows = cloud->rows;
	cols = cloud->cols;
}

PointCloud& PointCloud::operator= (const PointCloud &source)
{
	cloud = new Mat();
	*cloud = *(source.cloud);
	rows = cloud->rows;
	cols = cloud->cols;
    // return the existing object
    return *this;
}

PointCloud::~PointCloud()
{
	delete cloud;
}

//---------------------------------------------
// New pointcloud from matrix
//---------------------------------------------
PointCloud::PointCloud(Mat m)
{
	*cloud = m;
	rows = m.rows;
	cols = m.cols; // could cause mismatch problems if cols != 6
}

//---------------------------------------------
//
//---------------------------------------------
void PointCloud::AddPoint(Point3D* p)
{
	cloud->resize(++rows);
	cloud->at<float>(rows-1, 0) = p->X;
	cloud->at<float>(rows-1, 1) = p->Y;
	cloud->at<float>(rows-1, 2) = p->Z;
	cloud->at<float>(rows-1, 3) = p->R;
	cloud->at<float>(rows-1, 4) = p->G;
	cloud->at<float>(rows-1, 5) = p->B;
}

//---------------------------------------------
//
//---------------------------------------------
Point3D* PointCloud::GetPoint(int row)
{
	Point3D* p = new Point3D;
	p->X = cloud->at<float>(row, 0);
	p->Y = cloud->at<float>(row, 1);
	p->Z = cloud->at<float>(row, 2);
	p->R = (uchar) cloud->at<float>(row, 3);
	p->G = (uchar) cloud->at<float>(row, 4);
	p->B = (uchar) cloud->at<float>(row, 5);
	return p;
}

//---------------------------------------------
//
//---------------------------------------------
void PointCloud::DeletePoint()
{
	cloud->resize(--rows);
}

//---------------------------------------------
//
//---------------------------------------------
Mat PointCloud::GetMatrix()
{
	return *cloud;
}
void PointCloud::SetMatrix(Mat m)
{
	*cloud = m;
	cols = m.cols;
	rows = m.rows;
}

//---------------------------------------------
//
//---------------------------------------------
int PointCloud::Count()
{
	return rows;
}

//---------------------------------------------
//
//---------------------------------------------
int PointCloud::Dimension()
{
	return cols;
}

void PointCloud::AddPointCloud(PointCloud pc)
{
	Mat in = pc.GetMatrix();
	cloud->resize(rows + in.rows);
	cv::Mat tmp = (*cloud)(cv::Rect(0,rows,6,in.rows));
	in.copyTo(tmp);
	rows = cloud->rows;
}

void PointCloud::AddPointCloud(Mat pc)
{
	cloud->resize(rows + pc.rows);
	cv::Mat tmp = (*cloud)(cv::Rect(0,rows,6,pc.rows));
	pc.copyTo(tmp);
	rows = cloud->rows;
}