#include "StdAfx.h"
#include "NormalEstimator.h"

NormalEstimator::NormalEstimator(Mat* m)
{
	/*
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  MatToPcl(*m, cloud);


  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.03);

  // Compute the features
  ne.compute (*cloud_normals);

  // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*

  result = PclToMat(NormsToPcl(cloud_normals));
  */



	// Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  MatToPcl(*m, cloud);

  // Create a KD-Tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  // Output has the PointNormal type in order to store the normals calculated by MLS
  pcl::PointCloud<pcl::PointNormal> mls_points;

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
 
  mls.setComputeNormals (true);

  // Set parameters
  mls.setInputCloud (cloud);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.03);

  // Reconstruct
  mls.process (mls_points);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
	for(int i=0; i < mls_points.points.size(); i++){
		cloud2->push_back(pcl::PointXYZ(mls_points.points[i].normal_x, mls_points.points[i].normal_y, mls_points.points[i].normal_z));
	}

  result = PclToMat(cloud2);
}

void NormalEstimator::SetPointCloud(Mat m)
{
//	m.copyTo(*input);
}

void NormalEstimator::compute()
{

}

NormalEstimator::~NormalEstimator(void)
{
}

pcl::PointCloud<pcl::PointXYZ>::Ptr NormalEstimator::NormsToPcl(pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	for(int i=0; i < cloud_normals->points.size(); i++){
		cloud->push_back(pcl::PointXYZ(cloud_normals->points[i].normal_x, cloud_normals->points[i].normal_y, cloud_normals->points[i].normal_z));
	}
	return cloud;
}

void NormalEstimator::MatToPcl(Mat m, pcl::PointCloud<pcl::PointXYZ>::Ptr pc)
{
	for(int i=0; i < m.rows; i++){
		pc->push_back(pcl::PointXYZ(m.at<float>(i,0), m.at<float>(i,1), m.at<float>(i,2)));
	}
}

Mat NormalEstimator::PclToMat(pcl::PointCloud<pcl::PointXYZ>::Ptr pc)
{
	Mat out(pc->points.size(), 3, MAT_TYPE);
	for(int i=0; i < pc->points.size(); i++){
		out.at<float>(i,0) = pc->points[i].x;
		out.at<float>(i,1) = pc->points[i].y;
		out.at<float>(i,2) = pc->points[i].z;
	}
	return out;
}