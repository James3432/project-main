//----------------
// Includes
//----------------
#include "StdAfx.h"
#include "IterativeClosestPoint.h"
//#include <pcl/registration/icp.h>
#include "global_parameters.h"


//--------------------------
// Constructor
//--------------------------
IterativeClosestPoint::IterativeClosestPoint(void)
{

}

//--------------------------
// Constructor
//--------------------------
IterativeClosestPoint::IterativeClosestPoint(PointCloud P_in, PointCloud Q_in)
{
//	_E = nullptr;

	// Initialise empty matrices
	P1 = new Mat(P_in.GetMatrix().rows,P_in.GetMatrix().cols,MAT_TYPE);
	Q1 = new Mat(Q_in.GetMatrix().rows,Q_in.GetMatrix().cols,MAT_TYPE);
	if(!(PASS_BY_VALUE)){
		*P1 = P_in.GetMatrix();
		*Q1 = Q_in.GetMatrix();
	}else{
		*P1 = P_in.GetMatrix().clone();
		*Q1 = Q_in.GetMatrix().clone();
	}
}

IterativeClosestPoint::~IterativeClosestPoint()
{
	delete P1;
	delete Q1;
	delete transform;
	delete translation;
	delete overall_transform;
	delete final;
	delete p0;
	delete q0;
}

void IterativeClosestPoint::setErrorMat(Mat &errs, int p)
{
	errors = errs;
	pair = p;
}

void IterativeClosestPoint::run()
{

	Mat P = *P1;
	Mat Q = *Q1;

	transform = new Mat(Mat::eye(3,3,MAT_TYPE));
	translation = new Mat(Mat::zeros(3,1,MAT_TYPE));
	overall_transform = new Mat(Mat::eye(4,4,MAT_TYPE));
	final = new Mat();
	p0 = new Mat();
	q0 = new Mat();

	error = 10000;
	float prev_error = 0;

	iters = 0;

	int sample_size = MIN(SAMPLE_SIZE, MIN(P.rows, Q.rows));

	Mat inn = RemoveNaNs(Q);

	Mat P_icp(sample_size,P.cols,MAT_TYPE);
	Mat Q_icp(sample_size,Q.cols,MAT_TYPE);
	Mat P_k(sample_size,3,MAT_TYPE);
	Mat Q_k(sample_size,3,MAT_TYPE);


	// Iterate, unless:
	//    -- the error falls below a threshold
	//    -- the difference between the current & previous errors falls below a threshold
	//    -- the number of iterations exceeds the global limit
	while((error > CONVERGENCE_CRITERION) && (fabs(prev_error - error) > STEADY_STATE_DELTA) && (iters < MAX_ITER)){

		Log("Iteration "+boost::lexical_cast<std::string>(iters));
		/*
		// ensure that the bigger pointcloud is subsampled and matched against the other, to avoid a huge number of point comparisons
		if(Q.rows > P.rows){
			Q_icp = SubSample(Q);
			P_icp = MatchPoints(P,Q_icp);
		}else{*/

		std::vector<int> subsample;
		P_icp = SubSample(P, subsample);
		

		if(VISUALISE_SAMPLING && (iters==0))
			VisualiseSubsample(P, subsample, iters);

		MatchPoints(Q, Q_icp, P, P_icp, subsample, inn);

		if(VISUALISE_MATCHING && (iters==0))
			VisualisePoints(Q_icp);

		//}
		//PrintDebugMat(P_icp);
		//PrintDebugMat(Q_icp);
		//Log("Kabsching...");
		
		//Log("P:");
		//PrintDebugMat(P_icp);
		//Log("Q:");
		//PrintDebugMat(Q_icp);
		//PrintDebugMat(Q.row(subsample[0]));

		// NOW RESIZE P,Q down to 3 cols
		P_k = MatColourToPoints(P_icp);
		Q_k = MatColourToPoints(Q_icp);

		Kabsch k(P_k,Q_k);

		*transform = k.GetRotation().t();	// this doesn't really work (need to rotate then trans at each step, can't compose them all)
		*translation = k.GetTranslation(); // see above
 		*overall_transform = (*overall_transform) * NormToHomogOperator(*transform, *translation);
		
		prev_error = error;
		error = k.GetError();
		errors.at<float>(pair, iters) = (float)(error);

		P = Transform(P, *transform, *translation);

		if(USE_NORMAL_MATCHING||USE_NORMAL_SAMPLING)
			// update the normals of P (unaffected by any translation)
			Pnorms = Pnorms * *transform;

		++iters;
//		this->Event(iters);

		/*Log("transform:");
		PrintDebugMat(*transform);
		Log("translation:");
		PrintDebugMat(*translation);
		Log("overall:");
		PrintDebugMat(*overall_transform);*/

	}

	*final = P;
	
}

// sign of a float: 0 for positive, 1 for negative
int IterativeClosestPoint::sign(float i)
{
	if(i >= 0)
		return 0;
	else
		return 1;
}

Mat IterativeClosestPoint::SubSample(Mat A, std::vector<int> &samples)
{	
	Subsampler* sampler;
	if(USE_NORMAL_SAMPLING)
		sampler = new NormalSubsample();
	else
		sampler = new RandomSubsample();

	sampler->setInput(A);
	sampler->setOutput(samples);
	if(USE_NORMAL_SAMPLING)
		sampler->setNorms(Pnorms);

	return sampler->run(samples);
}
    


void IterativeClosestPoint::MatchPoints(Mat &P, Mat &P_icp, Mat&Q, Mat &Q_icp, const std::vector<int> &samples, Mat inn)
{
	int sample_size = samples.size();
	
	// New new version - only search in a small grid around each point for a match
	Mat out(sample_size/*SAMPLE_SIZE*/,P.cols,MAT_TYPE);
	Mat in(sample_size, Q.cols,MAT_TYPE);
	//Mat P_ = P.clone(); 
	Mat P_ = P;
	Mat Q_;
	//if(USE_NORMAL_SAMPLING)
	//	Q_ = RemoveNaNs(Q);
	//else
	Q_ = Q;
	/*Mat P_points(P_.rows,3,MAT_TYPE);
	Mat temp = P_(Rect(0,0,3,P_.rows));
	temp.copyTo(P_points);*/

	//Mat Pnorms = CalculateNormals(P_);
	//Mat Qnorms;
	//if(USE_NORMAL_MATCHING)
	//	Qnorms = CalculateNormals(Q);

	float d = 1000000;
	float dist = 0;
	Point3D p;
	Point3D q;
	int minRow,minRow2;

	std::set<int> chosen_points;

	//////////////

	//notes:
	// flann returns a sqr distance
	// Rect(x,y,width height) == Rect(col, row, #cols, #rows)

	

	int output_count = 0;

	//for(int i = 0; i<Q.rows;i++){
	// Iterate over samples vector
	for(int it=0; it < sample_size; ++it){
		// select the sample point
		int point = samples[it];
		//if(it % 500 == 0) Log(it);
		//Log(boost::lexical_cast<std::string>(it));
		if(USE_FLANN_MATCHING){
			// generate query from Q
			//Mat query(1,3,MAT_TYPE);
			
			std::vector<int> indices(FLANN_SEARCH_RADIUS);
			std::vector<float> dists(FLANN_SEARCH_RADIUS);

			float query[3] = {Q_.at<float>(point,0), Q_.at<float>(point,1), Q_.at<float>(point,2)};
	
			::flann::Matrix<float> queryM(query, 1, 3);

			::flann::Matrix<int> indicesM(&indices[0], 1, indices.size());
			::flann::Matrix<float> distsM(&dists[0], 1, dists.size());

			int z=0;
			bool nomatch = false;
			Qkdtree->knnSearch(queryM,indicesM, distsM, FLANN_SEARCH_RADIUS, ::flann::SearchParams(FLANN_SEARCH_PARAMS));
			for(z =0; z<FLANN_SEARCH_RADIUS; z++){
				if(ColourMatch(Q_.at<float>(point,3),Q_.at<float>(point,4),Q_.at<float>(point,5),inn.at<float>(indices[z],3),inn.at<float>(indices[z],4),inn.at<float>(indices[z],5)))
					break;
				if(z == FLANN_SEARCH_RADIUS - 1)
					nomatch = true;
			}
			
			if(nomatch){
				minRow2 = indices[0];
				d = dists[0];
			}else{
				minRow2 = indices[z];
				d = dists[z];
			}

			//PrintMat(Q_.row(point));
			//PrintMat(inn.row(minRow2));
			//Log(retry);
			if(d > MATCH_DISTANCE_THRESHOLD)
				d = 1000000; // ie. no point found
		}else{

			d = 1000000;

			q.X = Q_.at<float>(point,0);
			q.Y = Q_.at<float>(point,1);
			q.Z = Q_.at<float>(point,2);

			for(int row = rowBelow(ptToRow(point)); row < rowAbove(ptToRow(point)); row++){
				for(int col = colBelow(ptToCol(point)); col < colAbove(ptToCol(point)); col++){

					int j = CoordsToPoint(row,col);

					if(!isnan(P_.at<float>(j,2)) && (chosen_points.find(j) == chosen_points.end())){ // don't pick NaNs or already-matched points
						p.X = P_.at<float>(j,0);
						p.Y = P_.at<float>(j,1);
						p.Z = P_.at<float>(j,2);
						dist = Distance(p,q);
		
						if (dist < d){
							if(ColourMatch(P_.at<float>(j,3),P_.at<float>(j,4),P_.at<float>(j,5),Q_.at<float>(point,3),Q_.at<float>(point,4),Q_.at<float>(point,5))){
								if(USE_NORMAL_MATCHING){

									/*if(NormalMatch(CalculateNormal(P_,j), GetPoint(Qnorms, point))){
										d = dist;
										minRow = j;
									}*/

									if(NormalMatch(GetPoint(Qnorms, j), GetPoint(Pnorms, point))){
										d = dist;
										minRow = j;
										//Log("matched");
									}//else
										//Log("missed on normal");
								}else{
									d = dist;
									minRow = j;
									//Log("matched");
								}
							}//else
								//Log("missed on colour");
						}
					}
				}
			}

		}

		if(d==1000000){
			// no match found
			// just match point to copy of itself
			//Q_.row(point).copyTo(out.row(it));
			// how often is this happening??
			//Log("no match");

		}else{
			//swap(row minRow for row i in P_)
			if(USE_FLANN_MATCHING){
				inn.row(minRow2).copyTo(out.row(output_count));			
			//	Log(d);
			}else{
				P_.row(minRow).copyTo(out.row(output_count)); // in is just P_ without NaNs (because flann uses an unorganised pointcloud)
				//P_.at<float>(minRow,2) = nan();
				chosen_points.insert(minRow);
			}
			Q.row(point).copyTo(in.row(output_count));

			output_count++;
		}

	}

	out.resize(output_count);
	in.resize(output_count);
	//Log("Points matched:");
	//Log(output_count);

	P_icp = out;
	Q_icp = in;

	//return out;
	
}

bool IterativeClosestPoint::ColourMatch(float r1, float g1, float b1, float r2, float g2, float b2)
{
	if(USE_COLOUR_MATCHING){

		int r_diff = r2 - r1;
		int g_diff = g2 - g1;
		int b_diff = b2 - b1;
		float result = /*sqrt*/((float)r_diff*r_diff + g_diff*g_diff + b_diff*b_diff);
		if(result < COLOUR_CRITERION)
			return true;
		else
			return false;

	}
	else
		return true;
}

bool IterativeClosestPoint::isEdge(int p)
{
	// return true if p is within the margins of a (640 x 480) image, as specified by IMAGE_MARGIN

	if
	(     (p < IM_WIDTH * IMAGE_MARGIN)                      // top margin
		||(p > IM_WIDTH * (IM_HEIGHT - IMAGE_MARGIN) - 1 )   // bottom margin
		||(p % IM_WIDTH < IMAGE_MARGIN)                      // left margin
		||(p % IM_WIDTH > IM_WIDTH - IMAGE_MARGIN - 1)       // right margin
	)
		return true;
	else
		return false;
}
/*
//-----------------------------------
// Strip colour information from 
// a pointcloud matrix
//-----------------------------------
Mat IterativeClosestPoint::MatColourToPoints(Mat pc)
{
	// assert: pc.cols == 6

	Mat out(pc.rows,3,MAT_TYPE);
	for(int i=0; i<pc.rows;i++){
		for(int j=0; j<3; j++){
			out.at<float>(i,j) = pc.at<float>(i,j);
		}
	}

	return out;
}*/

bool IterativeClosestPoint::NormalMatch(Point3D A, Point3D B)
{
	if(Distance(A, B) < NORMAL_CRITERION * NORMAL_CRITERION) // only gives square of distance!!
		return true;
	else
		return false;
}

float IterativeClosestPoint::Distance(Point3D A, Point3D B)
{
	Point3D diff;diff.X=A.X-B.X;diff.Y= A.Y-B.Y;diff.Z= A.Z-B.Z;
	return /*sqrt*/(diff.X * diff.X + diff.Y * diff.Y + diff.Z * diff.Z);
}

Mat IterativeClosestPoint::Transform(Mat P, Mat rot, Mat trans)
{
	// rot and trans are (3,3) and (3,1) matrices respectively
	// P is (p,6) though, so some adjustments are needed

	P = P * ChangeOperatorSize(rot);

	Mat trans2 = trans.t();

	Mat tr(P.rows,P.cols,MAT_TYPE);
	for(int i=0; i<tr.rows;i++){
		for(int j=0; j<tr.cols; j++){
			if(j < 3)
				tr.at<float>(i,j) = trans2.at<float>(0,j);
			else
				tr.at<float>(i,j) = 0;
		}
	}

	P = P + tr;
	return P;
}

// Turn a rotation matrix + translation vector into a 4D homogeneous transformation matrix
Mat IterativeClosestPoint::NormToHomogOperator(Mat rot, Mat trans){
	// rot.cols == 3, rot.rows == 3
	// out.cols == 4, out.cols == 4
	// trans is a column vector
	Mat out(4,4,MAT_TYPE);
	cv::Mat tmp = out(cv::Rect(0,0,3,3));
	rot.copyTo(tmp);
	
	// out:
	// 
	//      |0
	//  rot |0
	//      |0
	// x|y|z|1
	//

	// bottom right square
	out.at<float>(3,3) = 1;
	// bottom row
	for(int i=0; i<3; i++){
		out.at<float>(i,3) = 0;
	}
	// right-most column
	for(int i=0; i<3; i++){
		out.at<float>(3,i) = trans.at<float>(i,0);
	}

	return out;
}

//---------------------------------------------------------------------
// Switch from 3D transformation matrix to 6D (points + colours)
//---------------------------------------------------------------------
Mat IterativeClosestPoint::ChangeOperatorSize(Mat transform)
{
	//
	//  ie:   A   -->    A | 0     where each is a 3x3 submatrix of the result
	//                  -------
	//                   0 | I
	//
	Mat out = Mat::eye(transform.rows + 3, transform.cols + 3, MAT_TYPE);
	cv::Mat tmp = out(cv::Rect(0,0,transform.cols,transform.rows));
	transform.copyTo(tmp);
	return out;
}

Mat IterativeClosestPoint::getTransform()
{
	return *transform;
}

Mat IterativeClosestPoint::getTranslate()
{
	return *translation;
}

float IterativeClosestPoint::getError()
{
	return error;
}

Mat IterativeClosestPoint::getFinal()
{
	return *final;
}

Mat IterativeClosestPoint::getHomogTransform()
{
	return *overall_transform;
}

PointCloud* IterativeClosestPoint::getFinalPointCloud()
{
	return new PointCloud(*final);
}

void IterativeClosestPoint::SetNormals(Mat pn, Mat qn)
{
	Pnorms = pn;
	Qnorms = qn;
}

void IterativeClosestPoint::SetKdTrees(::flann::Index<::flann::L2<float> >* treeP, ::flann::Index<::flann::L2<float> >* treeQ)
{
	Pkdtree = treeP;
	Qkdtree = treeQ;
}

float IterativeClosestPoint::nan()
{
	unsigned long nan[2]={0xffffffff, 0x7fffffff}; 
	return *( float* )nan;
}

int IterativeClosestPoint::ptToRow(int p)
{
	return (int)(p / IM_WIDTH);
}

int IterativeClosestPoint::ptToCol(int p)
{
	return (p % IM_WIDTH);
}

int IterativeClosestPoint::rowBelow(int r)
{
	int res = r - POINT_MATCH_MARGIN;
	if(res < 0)
		return 0;
	else
		return res;
}

int IterativeClosestPoint::rowAbove(int r)
{
	int res = r + POINT_MATCH_MARGIN;
	if(res > IM_HEIGHT - 1)
		return IM_HEIGHT - 1;
	else
		return res;
}

int IterativeClosestPoint::colBelow(int c)
{
	int res = c - POINT_MATCH_MARGIN;
	if(res < 0)
		return 0;
	else
		return res;
}

int IterativeClosestPoint::colAbove(int c)
{
	int res = c + POINT_MATCH_MARGIN;
	if(res > IM_WIDTH - 1)
		return IM_WIDTH - 1;
	else
		return res;
}

int IterativeClosestPoint::CoordsToPoint(int r, int c)
{
	return (IM_WIDTH*r + c);
}

bool IterativeClosestPoint::searchForNeighbours(::flann::Index<::flann::L2<float> >* kdtree, Mat cloud, int pt, int search_parameter, std::vector<int> &indices, std::vector<float> &dists)
{

	//Mat query(1,3,MAT_TYPE);
	//cloud.row(pt).copyTo(query);
	
	/*std::vector<float> query;
	query.push_back(cloud.at<float>(pt,0));
	query.push_back(cloud.at<float>(pt,1));
	query.push_back(cloud.at<float>(pt,2));*/

	float query[3] = {cloud.at<float>(pt,0), cloud.at<float>(pt,1), cloud.at<float>(pt,2)};
	
	if(isnan(query[2]))
		return false;
	
	::flann::Matrix<float> queryM(query, 1, 3);

	::flann::Matrix<int> indicesM(&indices[0], 1, indices.size());
	::flann::Matrix<float> distsM(&dists[0], 1, dists.size());

	kdtree->knnSearch(queryM,indicesM, distsM, search_parameter, ::flann::SearchParams(FLANN_SEARCH_PARAMS));

	return true;
	
}

Mat IterativeClosestPoint::CalculateNormals(::flann::Index<::flann::L2<float> >* kdtree, Mat pc)
{
	Mat out(0,3,MAT_TYPE);

	//for(){

	//	out.resize(out.rows + 1);

	//}

	//return out;

	Mat in = MatColourToPoints(pc);

	return computeFeature(in, kdtree, NORMAL_SEARCH_RADIUS);
}

// not used?
Point3D IterativeClosestPoint::CalculateNormal(Mat pc, int pt)
{
	/// ind is the tree for pc

	int search_parameter = 5;

	std::vector<int> nn_indices (search_parameter);
	std::vector<float> nn_dists (search_parameter); 

	Point3D norm;

	if (!searchForNeighbours(Pkdtree/*is this right??*/, pc, pt, search_parameter, nn_indices, nn_dists)) //TODO: will be using FLANN
		norm.X = norm.Y = norm.Z = /*output.points[pt].curvature = */ nan();

	computePointNormal(pc, nn_indices, norm/*, output.points[pt].curvature*/);

	flipNormalTowardsViewpoint(/*pc_->points[(*indices_)[pt]],*/ GetPoint(pc, pt), Point3D(0,0,0/*vpx_, vpy_, vpz_*/), norm);

	return norm;
}

	//// start pcl

	//CHANGE TO floatS??

Mat IterativeClosestPoint::computeFeature(const Mat &input, ::flann::Index<::flann::L2<float> >* kdtree, int radius)
{
	

	Mat output(input.rows, 3, MAT_TYPE); // WILL NEED TO RESIZE DUE TO NANS? OR LEAVE NANS...

	int search_parameter = radius;

	//cv::flann::Index tree(input, cv::flann::KDTreeIndexParams(4));
	//cv::flann::Index tree(input, cv::flann::LinearIndexParams() );

	// Iterating over the entire index vector
    for (size_t pt = 0; pt < (size_t)output.rows; ++pt)
    {
		//Point3D norm(output.points[pt].normal[0], output.points[pt].normal[1], output.points[pt].normal[2]);
		Point3D norm;

		// Allocate enough space to hold the results
		// \note This resize is irrelevant for a radiusSearch ().
		std::vector<int> nn_indices (search_parameter);
		std::vector<float> nn_dists (search_parameter); 

		if (isnan(input.at<float>(pt,2)) || !searchForNeighbours(kdtree, input, pt, search_parameter, nn_indices, nn_dists)) //TODO: will be using FLANN
		{
			norm.X = norm.Y = norm.Z = /*output.points[pt].curvature = */ nan();
			continue;
		}

		//if(isnan(nn_dists[0])){
		//	Log("nan "+boost::lexical_cast<std::string>(pt));
		//	Log(boost::lexical_cast<std::string>(input.at<float>(pt,2)));
		//}

		computePointNormal(RemoveNaNs(input), nn_indices, norm/*, output.points[pt].curvature*/);

		flipNormalTowardsViewpoint(/*input_->points[(*indices_)[pt]],*/ GetPoint(input, pt), Point3D(0,0,0/*vpx_, vpy_, vpz_*/), norm);

		output.at<float>(pt,0) = norm.X;
		output.at<float>(pt,1) = norm.Y;
		output.at<float>(pt,2) = norm.Z;
    }

	

	return output;
}

void IterativeClosestPoint::computePointNormal(const Mat &cloud, const std::vector<int> &indices, Point3D &norm/*, float &curvature*/)
{
	Eigen::Matrix3f covariance_matrix;
	Eigen::Vector4f xyz_centroid;
    
	if (computeMeanAndCovarianceMatrix(cloud, indices, covariance_matrix, xyz_centroid) == 0)
    {
        norm.X = norm.Y = norm.Z = /*curvature =*/ nan();
        return;
    }

    // Get the plane normal and surface curvature
    solvePlaneParameters (covariance_matrix, norm/*, curvature*/);

}

void IterativeClosestPoint::flipNormalTowardsViewpoint(Point3D point, Point3D vp, Point3D &norm)
{
    // See if we need to flip any plane normals
    vp.X -= point.X;
    vp.Y -= point.Y;
    vp.Z -= point.Z;

    // Dot product between the (viewpoint - point) and the plane normal
    float cos_theta = (vp.X * norm.X + vp.Y * norm.Y + vp.Z * norm.Z);

    // Flip the plane normal
    if (cos_theta < 0)
    {
		norm.X *= -1;
		norm.Y *= -1;
		norm.Z *= -1;
    }

}


void IterativeClosestPoint::solvePlaneParameters(const Eigen::Matrix3f &covariance_matrix, Point3D &norm/*, float &curvature*/)
{

// Avoid getting hung on Eigen's optimizers
//  for (int i = 0; i < 9; ++i)
//    if (!pcl_isfinite (covariance_matrix.coeff (i)))
//    {
//      //PCL_WARN ("[pcl::solvePlaneParameteres] Covariance matrix has NaN/Inf values!\n");
//      nx = ny = nz = curvature = std::numeric_limits<float>::quiet_NaN ();
//      return;
//    }

	// Extract the smallest eigenvalue and its eigenvector
	EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
	EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
	pcl::eigen33 (covariance_matrix, eigen_value, eigen_vector);

	norm.X = eigen_vector [0];
	norm.Y = eigen_vector [1];
	norm.Z = eigen_vector [2];

	// Compute the curvature surface change
	float eig_sum = covariance_matrix.coeff(0) + covariance_matrix.coeff(4) + covariance_matrix.coeff(8);

	/*
	if (eig_sum != 0)
		curvature = fabsf (eigen_value / eig_sum);
	else
		curvature = 0;
		*/
}

unsigned int IterativeClosestPoint::computeMeanAndCovarianceMatrix(const Mat& cloud, const std::vector<int> &indices, Eigen::Matrix3f &covariance_matrix, Eigen::Vector4f &centroid)
{
	// create the buffer on the stack which is much faster than using cloud.points[indices[i]] and centroid as a buffer
	Eigen::Matrix<float, 1, 9, Eigen::RowMajor> accu = Eigen::Matrix<float, 1, 9, Eigen::RowMajor>::Zero ();
	size_t point_count;
	
	point_count = 0;
	for (std::vector<int>::const_iterator iter = indices.begin (); iter != indices.end (); ++iter)
	{
		if (isnan(GetPoint(cloud,*iter).Z)) // if valid point
		continue;

		++point_count;
		accu [0] += GetPoint(cloud, *iter).X * GetPoint(cloud, *iter).X;
		accu [1] += GetPoint(cloud, *iter).X * GetPoint(cloud, *iter).Y;
		accu [2] += GetPoint(cloud, *iter).X * GetPoint(cloud, *iter).Z;
		accu [3] += GetPoint(cloud, *iter).Y * GetPoint(cloud, *iter).Y; // 4
		accu [4] += GetPoint(cloud, *iter).Y * GetPoint(cloud, *iter).Z; // 5
		accu [5] += GetPoint(cloud, *iter).Z * GetPoint(cloud, *iter).Z; // 8
		accu [6] += GetPoint(cloud, *iter).X;
		accu [7] += GetPoint(cloud, *iter).Y;
		accu [8] += GetPoint(cloud, *iter).Z;
	}
	
	accu /= static_cast<float> (point_count);
	//Eigen::Vector3f vec = accu.tail<3> ();
	//centroid.head<3> () = vec;//= accu.tail<3> ();
	//centroid.head<3> () = accu.tail<3> ();    -- does not compile with Clang 3.0
	centroid[0] = accu[6]; centroid[1] = accu[7]; centroid[2] = accu[8];
	centroid[3] = 0;
	covariance_matrix.coeffRef (0) = accu [0] - accu [6] * accu [6];
	covariance_matrix.coeffRef (1) = accu [1] - accu [6] * accu [7];
	covariance_matrix.coeffRef (2) = accu [2] - accu [6] * accu [8];
	covariance_matrix.coeffRef (4) = accu [3] - accu [7] * accu [7];
	covariance_matrix.coeffRef (5) = accu [4] - accu [7] * accu [8];
	covariance_matrix.coeffRef (8) = accu [5] - accu [8] * accu [8];
	covariance_matrix.coeffRef (3) = covariance_matrix.coeff (1);
	covariance_matrix.coeffRef (6) = covariance_matrix.coeff (2);
	covariance_matrix.coeffRef (7) = covariance_matrix.coeff (5);

	return (static_cast<unsigned int> (point_count));
}

Point3D IterativeClosestPoint::GetPoint(Mat pts, int i)
{
	return Point3D(pts.at<float>(i,0),pts.at<float>(i,1),pts.at<float>(i,2));
}


Mat IterativeClosestPoint::MatColourToPoints(Mat m)
{
	Mat out(m.rows,3,MAT_TYPE);
	Mat temp = m(Rect(0,0,3,m.rows));
	temp.copyTo(out);
	return out;
}

Mat IterativeClosestPoint::RemoveNaNs(Mat m)
{
	Mat out(m.rows,m.cols,MAT_TYPE);
	int j=0;
	for(int i=0; i<m.rows; ++i){
		if (!isnan(m.at<float>(i,2))){
			m.row(i).copyTo(out.row(j));
			j++;
		}
	}
	out.resize(j);
	return out;
}

void IterativeClosestPoint::VisualisePoints(Mat P)
{
	PlyWriter* pw = new PlyWriter("c:\\users\\james\\desktop\\matches"+boost::lexical_cast<std::string>(iters)+".ply");

	Mat subs(P.rows, 6, MAT_TYPE);
	for(int i=0; i<subs.rows; ++i){
		P.row(i).copyTo(subs.row(i));
		// colour red
		subs.at<float>(i,3) = 255;
		subs.at<float>(i,4) = 0;
		subs.at<float>(i,5) = 0;
	}

	pw->AddPointCloud(subs);

	pw->WriteFinal();
	delete pw;
}

void IterativeClosestPoint::VisualiseSubsample(Mat Q, std::vector<int> subsamples, int iters)
{

	PlyWriter* pw = new PlyWriter("c:\\users\\james\\desktop\\subsample"+boost::lexical_cast<std::string>(iters)+".ply");

	//pw->AddPointCloud(Q);

	Mat subs(subsamples.size(), Q.cols, MAT_TYPE);
	for(int i=0; i<subs.rows; ++i){
		Q.row(subsamples[i]).copyTo(subs.row(i));
		// colour red
		subs.at<float>(i,3) = 255;
		subs.at<float>(i,4) = 0;
		subs.at<float>(i,5) = 0;
	}

	pw->AddPointCloud(subs);

	pw->WriteFinal();
	delete pw;

}

void IterativeClosestPoint::Log(std::string s)
{
	std::cout << s << std::endl << std::endl;
}

void IterativeClosestPoint::PrintDebugMat(Mat m)
{
	std::string s;
	for(int i = 0; i < m.rows; i++){ // each matrix row
		s = "";
		for(int j=0; j<m.cols; j++){  // each matrix col
			s = s + " | " + boost::lexical_cast<std::string>(m.at<float>(i,j));
		}
		std::cout << s << std::endl;
		std::cout << "--------------------------" << std::endl;
	}

	std::cout << std::endl;	
}