#include "StdAfx.h"
#include "Kabsch.h"

//-----------------------------------------------------------------------------------------------------
// Constructor
//
// Takes matrices P,Q and calculates optimal rotation & translation matrices using the Kabsch algorithm
//-----------------------------------------------------------------------------------------------------
Kabsch::Kabsch(Mat P, Mat Q)
{
	// Initialise variables
	err = 0;
	U = new Mat(3,3,MAT_TYPE); 
	Trans = new Mat(3,3,MAT_TYPE);
	p0 = new Mat(3,1,MAT_TYPE);
	q0 = new Mat(3,1,MAT_TYPE);

	// Find A, the covariance matrix of normalised inputs
	Mat A = Covariance(Normalise(P), Normalise(Q));

	// Calculate the optimal rotation
	*U = OptimalRotate(A);

	// Centroids
	*p0 = PointToMat(Centroid(P));
	*q0 = PointToMat(Centroid(Q));

	// Calculation optimal translation
	*Trans = *q0 - (*U)*(*p0); // such that U*p0 + Trans = q0, since U: P->Q

	// Calculate rms error
	Mat diff = (*U)*(Normalise(P).t()) - (Normalise(Q).t()) ;
	err = CalcError(diff);

}

// Destructor
Kabsch::~Kabsch()
{
	delete U;
	delete Trans;
	delete p0;
	delete q0;
}

//---------------------------------------------
// Return optimal rotation matrix
//---------------------------------------------
Mat Kabsch::GetRotation()
{
	return *U;
}

//---------------------------------------------
// Return optimal translation matrix
//---------------------------------------------
Mat Kabsch::GetTranslation()
{
	return *Trans;
}

//---------------------------------------------
// Return root-mean-squared error value
//---------------------------------------------
float Kabsch::GetError()
{
	return err;
}

//---------------------------------------------
// Calculate the root-mean-squared-error,
// given the difference matrix between the pointclouds
//---------------------------------------------
float Kabsch::CalcError(Mat diff)
{
	float error = 0.0;
	for(int i=0; i<diff.cols; i++){
		for(int j=0; j<diff.rows; j++){
			error += (diff.at<float>(j,i) * diff.at<float>(j,i));
		}
	}
	error = error / diff.cols;
	return sqrt(error);
}

//---------------------------------------------
// Convert an OpenNI point to an OpenCV matrix
//---------------------------------------------
Mat Kabsch::PointToMat(Point3D p)
{
	Mat out = Mat(3,1,MAT_TYPE);
	out.at<float>(0,0) = p.X;
	out.at<float>(1,0) = p.Y;
	out.at<float>(2,0) = p.Z;
	return out;
}

//---------------------------------------------
// Find the centroid point of a pointcloud
//
// Just an average for each dimension:
// centroid.x = (point1.x + ... + pointn.x)/n
//---------------------------------------------
Point3D Kabsch::Centroid(Mat m)
{
	Point3D p;
	float x=0,y=0,z=0;
	for(int i=0; i<m.rows; i++){
		x += m.at<float>(i,0);
		y += m.at<float>(i,1);
		z += m.at<float>(i,2);
	}
	p.X = x/m.rows;
	p.Y = y/m.rows;
	p.Z = z/m.rows;

	return p;
}

//---------------------------------------------
// Normalise a pointcloud
// That is, subtract the centroid from all points
//---------------------------------------------
Mat Kabsch::Normalise(Mat m)
{
	Mat out;
	Point3D centre = Centroid(m);
	Mat centres(m.rows,m.cols,MAT_TYPE);
	for(int i=0; i<m.rows; i++){
		centres.at<float>(i,0) = centre.X;
		centres.at<float>(i,1) = centre.Y;
		centres.at<float>(i,2) = centre.Z;
	}
	out = m - centres;
	return out;
}

//---------------------------------------------
// Find covariance matrix of P, Q
// Cov(P,Q) = transpose(P) x Q
//---------------------------------------------
Mat Kabsch::Covariance(Mat P, Mat Q)
{
	// P.rows == Q.rows
	Mat res = P.t() * Q / 4;  //TODO:why divide by 4???
	return res;
}

//---------------------------------------------
// Calculate optimal rotatation from pointclouds P -> Q
//---------------------------------------------
Mat Kabsch::OptimalRotate(Mat A)
{
	//SVD svd(A);
	Mat u;// = svd.u;
	Mat v;// = svd.vt.t(); 

	// scratch that - use Eigen

	Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> a2;
    cv2eigen<float>(A,a2);
	Eigen::Matrix3f A_eigen = a2;

	// Compute the Singular Value Decomposition
	Eigen::JacobiSVD<Eigen::Matrix3f> svd_eigen (A_eigen, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3f u_eigen = svd_eigen.matrixU ();
	Eigen::Matrix3f v_eigen = svd_eigen.matrixV ();
    eigen2cv(u_eigen,u);
	eigen2cv(v_eigen,v);

	// end Eigen

	float d = (determinant(v * u.t()) >= 0)? 1 : -1;
	Mat dmat(3,3,CV_32FC1);
	for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			dmat.at<float>(i,j) = 0;
		}
	}
	dmat.at<float>(0,0) = 1;
	dmat.at<float>(1,1) = 1;
	dmat.at<float>(2,2) = d;
	Mat opt(3,3,MAT_TYPE);
	opt = v * dmat * u.t();

	/*
	// if value < 1/1,000,000  then assume 0
	for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			if((opt.at<float>(i,j) > -0.000001)&&(opt.at<float>(i,j) < 0.000001))  // check this
				opt.at<float>(i,j) = 0;
		}
	}*/

	return opt;
}