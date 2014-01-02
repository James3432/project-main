#include "Point3D.h"
#include "PointCloud.h"
#include "IterativeClosestPoint.h"
#include "PlyWriter.h"

#include <opencv2\flann\flann.hpp>
#include <boost\lexical_cast.hpp>
#include <vector>
#include <flann\flann.hpp>
#include <pcl\pcl_base.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/features/integral_image_normal.h>
//#include <pcl/visualization/cloud_viewer.h>

//public delegate void f2(System::String^);

#pragma once
/*ref*/ class MultipleICP
{
	//f2 ^ _E;
public:
	MultipleICP(void);
	MultipleICP(PointCloud*, PointCloud*);
	MultipleICP(const MultipleICP &source);
	~MultipleICP();
	void run();
	void AddPointCloud(PointCloud*);
	void AddClouds(vector<PointCloud*>);
	Mat GetFinalTransform();
	PointCloud* GetResult();
	vector<PointCloud*> GetClouds();
	void MatToPcl(Mat pc, pcl::PointCloud<pcl::PointXYZ>::Ptr output);
	void PclToMat(Mat pc, pcl::PointCloud<pcl::PointXYZRGB>::Ptr input);
	void PclNormsToMats(Mat &output, Mat &norms, pcl::PointCloud<pcl::PointNormal> &mls_points, Mat &input);
	void SetInputParameters(__int64, double);
	/*
// event code for reporting progress asynchronously
event f2^ MultipleICP::Event {
    void add(f2 ^ d) {
        _E += d;
    }
	private:
    void remove(f2 ^ d) {
		_E -= d;
    }

	protected:
    void raise(System::String^ s) {
        if (_E) {
			_E->Invoke(s);
        }
    }
}*/

private:
public:vector<PointCloud*> clouds;
	unsigned int fixLNK2022() { return clouds.size(); } // FIX for metadata bug

	void ICP_update(int i);

	PointCloud* globalCloud;
	Mat* globalTransform;

	//cv::flann::Index* Ptree;
	//cv::flann::Index* Qtree;
	::flann::Index<::flann::L2<float> >* Pkdtree;
	::flann::Index<::flann::L2<float> >* Qkdtree;

	Mat Pnorms;
	Mat Qnorms;

	//Mat previousTransform;

	Mat ApplyTransform(Mat cloud, Mat t);

	__int64 F;
	double pixel_size;

	void GrabNextCloud(int c);
	void ClearupCloudArray(int c);

	Mat ApplyCurrentTransform(Mat pc);
	Mat ApplyHomogTransform(const Mat m);
	Mat ApplyHomogTransform(const Mat m, const Mat h);

	Mat ExtractRot(Mat);
	Mat ExtractTrans(Mat);

	Mat DownSample(Mat, int);

	void visualiseNormals(Mat norms, Mat pc);

	Mat SetColours(Mat m, int r, int g, int b);

	Mat NormToHomogPoint(Mat p);
	Mat HomogToNormPoint(Mat p);
	Mat NormToHomogPointCloud(Mat p);
	Mat HomogToNormPointCloud(Mat p);
	Mat NormToHomogOperator(Mat rot, Mat trans);
	Mat ChangeOperatorSize(Mat transform, int Dim);

	Mat MatColourToPoints(Mat m);
	Mat RemoveNaNs(Mat m);
	float nan();

	void CloudToFlannMatrix(Mat m, float* arr);
	void MatToArray(Mat m, float** arr);
	Point3D GetPoint(Mat pts, int i);

	Mat FusePoints(Mat points, Mat ref, ::flann::Index<::flann::L2<float>>* kdtree);

	Mat ComputeNormals(Mat cloud_in);

	Mat CalculateNormals(::flann::Index<::flann::L2<float> >*, Mat pc);
	Mat computeFeature(const Mat &input, ::flann::Index<::flann::L2<float> >*, int radius);
	void computePointNormal(const Mat &cloud, const std::vector<int> &indices, Point3D &norm);
	void flipNormalTowardsViewpoint(Point3D point, Point3D vp, Point3D &norm);
	void solvePlaneParameters(const Eigen::Matrix3f &covariance_matrix, Point3D &norm /*, float &curvature*/);
	unsigned int computeMeanAndCovarianceMatrix (const Mat &cloud, const std::vector<int> &indices, Eigen::Matrix3f &covariance_matrix, Eigen::Vector4f &centroid);
	bool searchForNeighbours(::flann::Index<::flann::L2<float> >* kdtree, Mat cloud, int pt, int search_parameter, std::vector<int> &indices, std::vector<float> &dists);

	void PrintDebugMat(Mat m);
	void Log(std::string s);

	
	void PrintStats(Mat m);

	Mat ImageToMat(Mat points, Mat colour);
	Mat ConvertProjToRealCoord(Mat points);
	Point3D singleConvertProjToRealCoord(Point3D p, float xtoz, float ytoz);

	inline __int64 GetCpuClocks()
	{

		// Counter
		struct { __int32 low, high; } counter;

		// Use RDTSC instruction to get clocks count
		__asm push EAX
		__asm push EDX
		__asm __emit 0fh __asm __emit 031h // RDTSC
		__asm mov counter.low, EAX
		__asm mov counter.high, EDX
		__asm pop EDX
		__asm pop EAX

		// Return result
		return *(__int64 *)(&counter);

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

