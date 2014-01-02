#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2\flann\flann.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <set>
#include <Eigen\Eigen>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
#include <pcl\common\eigen.h>
#include <boost\lexical_cast.hpp>
#include <flann\flann.hpp>

#include "Point3D.h"
#include "Kabsch.h"
#include "PointCloud.h"
#include "PlyWriter.h"
#include "Subsampler.h"


//#define isnan System::Double::IsNaN
#define isnan(x) (x!=x)

#define MAT_TYPE CV_32FC1
//#define MAT_TYPE64 CV_64FC1

using namespace cv;

#pragma once
/*ref*/ class IterativeClosestPoint
{
	//f ^ _E;
public:
	IterativeClosestPoint(void);
	IterativeClosestPoint(PointCloud P_in, PointCloud Q_in);
	~IterativeClosestPoint();
	void run();
	Mat getTransform();
	Mat getTranslate();
	Mat getHomogTransform();
	float getError();
	Mat getFinal();
	void SetNormals(Mat pn, Mat qn);
	void SetKdTrees(::flann::Index<::flann::L2<float> >*, ::flann::Index<::flann::L2<float> >*);
	PointCloud* getFinalPointCloud();
	int iters;
	Mat* p0;Mat* q0;
	void setErrorMat(Mat &errs, int p);
	Mat errors;
	int pair;

//	System::Windows::Forms::Form^ form;
/*
// event code for reporting progress asynchronously
event f^ IterativeClosestPoint::Event {
    void add(f ^ d) {
        _E += d;
    }
	private:
    void remove(f ^ d) {
		_E -= d;
    }

	protected:
    void raise(int i) {
        if (_E) {
			_E->Invoke(i);
        }
    }
}*/

private:
//	pcl::PointCloud<pcl::PointXYZ>::Ptr MatToPclPointcloud(Mat pc);
//	Mat PclPointcloudToMat(pcl::PointCloud<pcl::PointXYZ>::Ptr pc);
	Mat* P1;
	Mat* Q1;
	::flann::Index<::flann::L2<float> >* Pkdtree;
	::flann::Index<::flann::L2<float> >* Qkdtree;
	Mat Pnorms;
	Mat Qnorms;

	Mat* transform;
	Mat* translation;
	Mat* overall_transform;
	Mat* final;
	Mat Transform(Mat P, Mat rot, Mat trans);
	Mat SubSample(Mat A, std::vector<int> &samples);
	//Mat MatColourToPoints(Mat pc);
	bool ColourMatch(float r1, float g1, float b1, float r2, float g2, float b2);
	Point3D CalculateNormal(Mat pc, int i);
	Mat CalculateNormals(::flann::Index<::flann::L2<float> >*, Mat pc);
	bool isEdge(int p);
	bool NormalMatch(Point3D, Point3D);
	Mat computeFeature(const Mat &input, ::flann::Index<::flann::L2<float> >*, int radius);
	void computePointNormal(const Mat &cloud, const std::vector<int> &indices, Point3D &norm);
	void flipNormalTowardsViewpoint(Point3D point, Point3D vp, Point3D &norm);
	void solvePlaneParameters(const Eigen::Matrix3f &covariance_matrix, Point3D &norm /*, float &curvature*/);
	unsigned int computeMeanAndCovarianceMatrix (const Mat &cloud, const std::vector<int> &indices, Eigen::Matrix3f &covariance_matrix, Eigen::Vector4f &centroid);
	bool searchForNeighbours(::flann::Index<::flann::L2<float> >* kdtree, Mat cloud, int pt, int search_parameter, std::vector<int> &indices, std::vector<float> &dists);
	Mat ChangeOperatorSize(Mat transform);
	Mat NormToHomogOperator(Mat rot, Mat trans);
	void MatchPoints(Mat &P, Mat &P_icp, Mat&Q, Mat &Q_icp, const std::vector<int> &samples, Mat inn);
	Mat RemoveNaNs(Mat);
	Mat MatColourToPoints(Mat);
	Point3D GetPoint(Mat, int);
	float Distance(Point3D, Point3D);
	int sign(float i);
	float nan();
	int rowBelow(int);
	int rowAbove(int);
	int colBelow(int);
	int colAbove(int);
	int ptToRow(int);
	int ptToCol(int);
	int CoordsToPoint(int,int);
	float error;
	void VisualiseSubsample(Mat Q, std::vector<int> subsamples, int iters);
	void VisualisePoints(Mat P);
	void PrintDebugMat(Mat m);
	void Log(std::string s);

	void Log(int i)
	{
		Log(boost::lexical_cast<std::string>(i));
	}
	
	//---------------------------------------------------------------------------
	// Display a matrix via log window
	//---------------------------------------------------------------------------
	void PrintMat(Mat m)
	{
		std::string s;
		for(int i = m.rows-1; i>=0; i--){ // each matrix row
			s = "";
			for(int j=0; j<m.cols; j++){  // each matrix col
				s = s + " | " + boost::lexical_cast<std::string>(m.at<float>(i,j));
			}
			Log(s);
			Log("--------------------------");
		}
		Log("");
	}

};