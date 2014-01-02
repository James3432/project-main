#include "StdAfx.h"
#include "MultipleICP.h"

#include "global_parameters.h"

// Idea:
// Align all pointclouds into the frame of the first pointcloud.
// Then, for each new alignment, we simply multiply the global transform by the new transform, and transform this and all subsequent clouds by this. (using homogeneous co-ordinates)
// So, for first 2, globTransform = Kabsch(pc2,pc1)  since Kabsch transforms P onto Q, ie. pc2 onto pc1

MultipleICP::MultipleICP(void)
{
//	_E = nullptr;
	//clouds = vector<PointCloud*>;	
	globalCloud = new PointCloud();
	globalTransform = new Mat(Mat::eye(4,4,MAT_TYPE));
	//previousTransform = Mat::eye(4,4,MAT_TYPE);
}

MultipleICP::MultipleICP(PointCloud* p1, PointCloud* p2)
{
//	_E = nullptr;
//	clouds = vector<PointCloud*>;	
	globalCloud = new PointCloud();
	globalTransform = new Mat(Mat::eye(4,4,MAT_TYPE));
	//previousTransform = Mat::eye(4,4,MAT_TYPE);
	clouds.push_back(p1);
	clouds.push_back(p2);
}

MultipleICP::MultipleICP(const MultipleICP &source)
{
//	_E = nullptr;
	clouds = source.clouds;
	globalCloud = new PointCloud();
	globalTransform = new Mat(Mat::eye(4,4,MAT_TYPE));
	//previousTransform = Mat::eye(4,4,MAT_TYPE);
}

MultipleICP::~MultipleICP()
{
	delete globalTransform;
	delete globalCloud;
}

void MultipleICP::run()
{
	if(DELAYED_LOAD)
	{
		GrabNextCloud(0);
	}


	std::ofstream logwriter(TRANSFORM_LOG, std::ios::out);  

	PlyWriter* pw = new PlyWriter(SAVE_LOC);		
	PlyWriter* pw_o;

	if(OUTPUT_ORIGINAL){
		pw_o = new PlyWriter(SAVE_LOC_ORIGINAL);
		pw_o->AddPointCloud(clouds[0]->GetMatrix());
	}

//	Event(clouds->size() + " clouds ready for processing.");
	Log(boost::lexical_cast<std::string>(clouds.size()) + " clouds ready for processing.");
	//globalCloud->AddPointCloud(DownSample(clouds[0]->GetMatrix(),MAX_CLOUD_SIZE));

	if(PRE_DOWNSAMPLE)
		clouds[0]->SetMatrix(DownSample(clouds[0]->GetMatrix(),PRE_DOWNSAMPLE_SIZE));

	Log("Writing first file to output...");
	if(REDUCED_OUTPUT)
		pw->AddPointCloud(DownSample(clouds[0]->GetMatrix(),MAX_CLOUD_SIZE));
	else
		pw->AddPointCloud(clouds[0]->GetMatrix());
	Log("Done");

	if(CALCULATE_KDTREES){
		Log("Building kdtree 1...");
		if(USE_NATIVE_FLANN){
			Mat cloud = RemoveNaNs(clouds[0]->GetMatrix());
	
			//int nn = 3;
			float* arr = new float[cloud.rows*3];
			CloudToFlannMatrix(cloud,arr);
			::flann::Matrix<float> dataset(arr, cloud.rows, 3); //may need stride?
			//flann::Matrix<float> query;
			//flann::Matrix<int> indices(new int[query.rows*nn], query.rows, nn);
			//flann::Matrix<float> dists(new float[query.rows*nn], query.rows, nn);
			// construct an randomized kd-tree index using 4 kd-trees
			Qkdtree = new ::flann::Index<::flann::L2<float> >(dataset, ::flann::KDTreeIndexParams(KD_TREES));
			Qkdtree->buildIndex();

		}else{
			// old method: cvflann
			Mat temp;
			cv::flann::Index Ptree;
			cv::flann::Index Qtree;	
			temp = RemoveNaNs(MatColourToPoints(clouds[0]->GetMatrix()));
			Mat temp2 = temp.clone(); // TODO: remove horrible workaround
			Qtree = cv::flann::Index(temp, cv::flann::KDTreeIndexParams(KD_TREES));
		}
	}

	//PrintDebugMat(clouds[0]->GetMatrix());

	if(USE_NORMAL_MATCHING||USE_NORMAL_SAMPLING){
		Log("Calculating normals for Q...");
		//Qnorms = CalculateNormals(Qkdtree, clouds[0]->GetMatrix());
		Qnorms = ComputeNormals(clouds[0]->GetMatrix());
		
		//Qnorms = testnorms;
		//Log("Normals computed");

		if(VISUALISE_NORMALS){
			visualiseNormals(Qnorms, clouds[0]->GetMatrix());
		}
	}

	int number_of_clouds = DELAYED_LOAD ? (END_NUM - START_NUM)/IM_STEP + 1 : clouds.size();

	Mat errors(number_of_clouds - 1, MAX_ITER, MAT_TYPE);

	for(int i=0; i < number_of_clouds - 1; i++){

		if(DELAYED_LOAD)
			GrabNextCloud(i + 1);

		// Apply the previously used transform first, in case this helps
		// TODO: does this work?
		//clouds[i+1]->SetMatrix(ApplyTransform(clouds[i+1]->GetMatrix(), previousTransform));

		Log("Pair "+boost::lexical_cast<std::string>(i));
	//	Event("Processing pair "+i);
		Log("Processing pair "+boost::lexical_cast<std::string>(i));
		//PlyWriter before("C:\\users\\james\\desktop\\meshes\\before"+i+".ply", clouds[i]->GetMatrix());
		
		if(PRE_DOWNSAMPLE)
			clouds[i+1]->SetMatrix(DownSample(clouds[i+1]->GetMatrix(),PRE_DOWNSAMPLE_SIZE));

		// Apply ICP
		IterativeClosestPoint icp(*(clouds[i+1]), *(clouds[i])); // transform the cloud at i+1 onto the cloud at i
		
		
		if(USE_NATIVE_FLANN && CALCULATE_KDTREES){
			Log("Building kdtree "+boost::lexical_cast<std::string>(i+2) + "...");	
			Mat cloud = RemoveNaNs(clouds[i+1]->GetMatrix());
			float* arr = new float[cloud.rows*3];
			CloudToFlannMatrix(cloud,arr);
			::flann::Matrix<float> dataset(arr, cloud.rows, 3); //may need stride?
			Pkdtree = new ::flann::Index<::flann::L2<float> >(dataset, ::flann::KDTreeIndexParams(KD_TREES));
			Pkdtree->buildIndex();
			Log("done");
			//delete[] arr;
			//TODO will still need to delete this somewhere...!
			icp.SetKdTrees(Pkdtree, Qkdtree);
		}

		
		if(USE_NORMAL_MATCHING||USE_NORMAL_SAMPLING){
			Log("Calculating normals for P...");
			//Pnorms = CalculateNormals(Pkdtree, clouds[i+1]->GetMatrix());
			Pnorms = ComputeNormals(clouds[i+1]->GetMatrix());
			//TODO remove
			//Pnorms = testnorms;
			//Log("Normals computed");
		}

		icp.SetNormals(Pnorms, Qnorms);

		icp.setErrorMat(errors, i);

//		icp.Event += gcnew f(this, &MultipleICP::ICP_update);
		icp.run();
	/*	
		Log("Trans:");
		PrintDebugMat(icp.getTransform());
		Log("Translate:");
		PrintDebugMat(icp.getTranslate());
		Log("Combined:");
		PrintDebugMat(NormToHomogOperator(icp.getTransform(),icp.getTranslate());*/

		/*Log("homog:");
		PrintDebugMat(icp.getHomogTransform());
		Log("actual transform:");
		PrintDebugMat(*icp.tt);
		Log("actual translate:");
		PrintDebugMat(*icp.trtr);*/

		*globalTransform = icp.getHomogTransform() * (*globalTransform);

		//previousTransform = icp.getHomogTransform();

		for(int it1=0;it1<4;it1++)
			for(int it2=0;it2<4;it2++)
				logwriter << globalTransform->at<float>(it1,it2) << "\n"; 

		//*globalTransform = NormToHomogOperator(icp.getTransform(),icp.getTranslate()) * (*globalTransform);  // this order due to the fact that transform(t) = t * transMat
//		Event("Error in ICP: "+icp.getError());
		Log("Error in ICP: "+boost::lexical_cast<std::string>(icp.getError()));

		//Old method:
		//Mat newCloudh = NormToHomogPointCloud(clouds[i+1]->GetMatrix());
		//Mat newCloud = HomogToNormPointCloud(ApplyCurrentTransform(newCloudh));
		
		Log("transforming");
		//New method: (fixed slowdown)
		Mat newCloud = ApplyHomogTransform(clouds[i+1]->GetMatrix());

		//if((i==clouds->size()-4)||(i==clouds->size()-3)||(i==clouds->size()-2))
		//globalCloud->AddPointCloud(DownSample(newCloud,MAX_CLOUD_SIZE));  // this line is a bit slow: is it the downsampling or the AddPointCloud??

		if(FUSE_POINTS)
			newCloud = FusePoints(newCloud, RemoveNaNs(clouds[i]->GetMatrix()), Qkdtree);  // Qkdtree ~~ clouds[i]
		
		//globalCloud->DownSample(MAX_CLOUD_SIZE);
		Log("reducing...");

		if(REDUCED_OUTPUT)
			newCloud = DownSample(newCloud,MAX_CLOUD_SIZE);

		Log("writing...");
		pw->AddPointCloud(newCloud);

		if(OUTPUT_ORIGINAL)
			pw_o->AddPointCloud(clouds[i+1]->GetMatrix());

		if(CALCULATE_KDTREES){
			delete Qkdtree;
			Qkdtree = Pkdtree;
		}

		if(USE_NORMAL_MATCHING||USE_NORMAL_SAMPLING)
			Qnorms = Pnorms;

		if(PAIRWISE_OUTPUT){
			PlyWriter* pw = new PlyWriter("c:\\users\\james\\desktop\\meshICPpair"+boost::lexical_cast<std::string>(i)+".ply");
			pw->AddPointCloud(ApplyHomogTransform(clouds[i+1]->GetMatrix(), icp.getHomogTransform()));
			pw->AddPointCloud(clouds[i]->GetMatrix());
			pw->WriteFinal();
			delete pw;
		}

		if(DELAYED_LOAD)
			ClearupCloudArray(i);

		//PlyWriter after("C:\\users\\james\\desktop\\meshes\\after"+i+".ply", clouds[i]->GetMatrix());

		//Event("Pair "+i+" processed");
		Log("Pair "+boost::lexical_cast<std::string>(i) +" processed");

		//Log("MultipleICP Global transform:");
		//PrintDebugMat(*globalTransform);
		Log("-----------------------");
		
	}

	Log("Error matrix: (rows are pairs, columns are iterations)");
	PrintMat(errors);
	Log("");
	Log("");
	PrintStats(errors);

	logwriter.close();

	if(CALCULATE_KDTREES)
		delete Qkdtree;  // P==Q so don't delete both

	pw->WriteFinal();
	delete pw;

	if(OUTPUT_ORIGINAL){
		pw_o->WriteFinal();
		delete pw_o;
	}
}

void MultipleICP::PrintStats(Mat m){

	Log("");

	float min = 999;
	float max = 0;
	float mean = 0;
	for(int i=0; i < m.rows; ++i){
		float val = m.at<float>(i, m.cols-1);
		mean += val;
		if(val < min)
			min = val;
		if(val > max)
			max = val;
	}
	mean = mean / m.rows;

	Log("Min: "+boost::lexical_cast<std::string>(min));
	Log("Max: "+boost::lexical_cast<std::string>(max));
	Log("Mean: "+boost::lexical_cast<std::string>(mean));

	Log("");Log("");

}

// Uniform random subsampling
Mat MultipleICP::DownSample(Mat pc, int target_points)
{
	if(PRE_DOWNSAMPLE){
		// don't shrink output, but insert NaNs
		Mat out(pc.rows, pc.cols, MAT_TYPE);
		out = pc.clone();
		srand((unsigned)time(0));
		for(int i=0; i<pc.rows; ++i){
			double r = ((double)rand() / (RAND_MAX + 1));
			// copy a row from input->output a number of times proportional to the desired sample size
			if(!(r < ((double)target_points) / (double)(pc.rows)))
				out.at<float>(i,2) = nan();
		}

		return out;

	}else{
	//if(REDUCED_OUTPUT){
		// ASSERT(A.cols == required out.cols)
		Mat out(target_points,pc.cols,MAT_TYPE);

		// METHOD 1: set
	
		//std::set<int> points;

		int available_points[640*480];
		for(int i=0; i<640*480; i++)
			available_points[i] = i;

		srand((unsigned)time(0));
		int range_max = 640*480 - 1;//pc.rows - 1;
		int r, val;
		//bool repeat;
		//PrintDebugMat(pc);
		for(int i=0; i < MIN(pc.rows, target_points); i++){
			int repeat_counter = 0;
			//Log(boost::lexical_cast<std::string>(i));
			//TODO: don't need the set anymore, check then remove it (and in the subsample method too)
			//TODO: this is inefficient, since we find an already-chosen point increasingly often. Instead, pick random number in decreasing range.(or shrinking set of unchosen points?)
			do		
			{	// Generate a random number in the range 0 - (rows-1)
				// This is an index into the array of available points
				r = (int)((double)rand() / (RAND_MAX + 1) * range_max);
				val = available_points[r];
				if(++repeat_counter > 100){  // if too many points are NaNs, give up and just output a NaN (which will be ignored later on, just end up with a smaller resulting cloud)
					r = 307199;
					val = 0; // always a NaN
					break;
				}
				//repeat = isnan(pc.at<float>(val,2));
				//if(repeat) Log(boost::lexical_cast<std::string>(pc.at<float>(val,2)));
			}
			//while((points.find(val) != points.end()) || (isnan(pc.at<float>(val,2))));   // don't pick NaNs or points already chosen (ie. in the set)	
			while(isnan(pc.at<float>(val,2)));   // don't pick NaNs or points already chosen (ie. in the set)	
			
			//if(points.find(val) != points.end())
			

			//points.insert(val);
			pc.row(val).copyTo(out.row(i));

			available_points[r] = available_points[range_max];
			range_max--;
		}

		return out;
	}
	//}
	//else
		//return pc;
}

void MultipleICP::CloudToFlannMatrix(Mat m, float* arr)
{
	for(int i=0;i<m.rows;++i){
		for(int j=0;j<3;++j)
			arr[(i*3)+j] = m.at<float>(i,j);
	}
}

void MultipleICP::MatToArray(Mat m, float** arr)
{
	for(int i=0; i<m.rows; i++){
		arr[i][0] = m.at<float>(i,0);
		arr[i][1] = m.at<float>(i,1);
		arr[i][2] = m.at<float>(i,2);
	}
}

void MultipleICP::ICP_update(int i) {
	//Event("Iteration "+i+" complete");
	Log("Iteration "+boost::lexical_cast<std::string>(i)+" complete");
}

void MultipleICP::AddPointCloud(PointCloud* p)
{
	clouds.push_back(p);
}

void MultipleICP::AddClouds(vector<PointCloud*> cs)
{
	clouds = cs;
}

Mat MultipleICP::GetFinalTransform()
{
	return *globalTransform;
}

PointCloud* MultipleICP::GetResult()
{
	return globalCloud;
}

vector<PointCloud*> MultipleICP::GetClouds()
{
	return clouds;
}

Mat MultipleICP::ApplyCurrentTransform(Mat pc)
{
	return pc * ChangeOperatorSize(*globalTransform, 4);
}

Mat MultipleICP::ExtractRot(Mat r)
{
	return r(Rect(0,0,3,3));	
}

Mat MultipleICP::ExtractTrans(Mat t)
{
	return t(Rect(0,3,3,1));
}

void MultipleICP::ClearupCloudArray(int c)
{
	if(c > 0)
		delete clouds[c-1];

	if(c == END_NUM - START_NUM - 1){
		delete clouds[c];
		delete clouds[c+1];
	}
}

Mat MultipleICP::FusePoints(Mat points, Mat ref, ::flann::Index<::flann::L2<float>>* kdtree)
{
	Mat out(points.rows, points.cols, MAT_TYPE);
	int outrow = 0;

	Mat points_ = RemoveNaNs(points);

	Log("Fusing points...");
	Log(boost::lexical_cast<std::string>(points_.rows) + " points found.");
	for(int i=0; i<points_.rows; ++i){

		std::vector<int> indices(1);
		std::vector<float> dists(1);

		float query[3] = {points_.at<float>(i,0), points_.at<float>(i,1), points_.at<float>(i,2)};
	
		::flann::Matrix<float> queryM(query, 1, 3);

		::flann::Matrix<int> indicesM(&indices[0], 1, indices.size());
		::flann::Matrix<float> distsM(&dists[0], 1, dists.size());

		kdtree->knnSearch(queryM, indicesM, distsM, 1, ::flann::SearchParams(FLANN_SEARCH_PARAMS));

		if(dists[0] > FUSION_THRESHOLD){
			points_.row(i).copyTo(out.row(outrow++));
		}
		//Log(boost::lexical_cast<std::string>(dists[0]));

	}

	Log("fused.");

	out.resize(outrow);
	return out;

}

void MultipleICP::visualiseNormals(Mat norms, Mat pc)
{
	PlyWriter* pw = new PlyWriter("c:\\users\\james\\desktop\\norms1.ply");
	int scale = 50;
	pw->AddPointCloud(pc);
	for(int i=0; i<norms.rows;++i)
	{
		if(!isnan(pc.at<float>(i,2)) && !isnan(norms.at<float>(i,0))){
			pw->AddVertex(pc.at<float>(i,0) + scale*norms.at<float>(i,0), pc.at<float>(i,1) + scale*norms.at<float>(i,1), pc.at<float>(i,2) + scale*norms.at<float>(i,2), 255, 0, 0);
		}
	}
	//pw->AddPointCloud(SetColours(MatColourToPoints(pc) + (10*norms), 255, 0, 0));

	pw->WriteFinal();
	delete pw;
}

Mat MultipleICP::SetColours(Mat m, int r, int g, int b)
{
	Mat out(m.rows, m.cols + 3, MAT_TYPE);

	Mat temp = out(Rect(0,0,3,out.rows));
	m.copyTo(temp);

	for(int i=0; i<m.rows; ++i){
		out.at<float>(i,3) = r;
		out.at<float>(i,4) = g;
		out.at<float>(i,5) = b;
	}

	return out;
}

void MultipleICP::GrabNextCloud(int c)
{
	std::string im_dir = DATA_SOURCE;
	int start_im = START_NUM;

	clouds.push_back(new PointCloud());

	std::string filename_im1 = boost::lexical_cast<std::string>(im_dir) + "\\" + FILE_NAME_COLOUR + boost::lexical_cast<std::string>(start_im + IM_STEP * c)+ ".jpg";
	std::string filename_d1 = boost::lexical_cast<std::string>(im_dir) + "\\" + FILE_NAME_DEPTH + boost::lexical_cast<std::string>(start_im + IM_STEP * c)+ ".png";

	Mat image = imread(filename_d1, -1);   // Read the file
	Mat image_col = imread(filename_im1, -1);

	clouds[c]->SetMatrix(ImageToMat(image, image_col));
	// Transform to real co-ordinates
	clouds[c]->SetMatrix(ConvertProjToRealCoord(clouds[c]->GetMatrix()));

	//MultipleICP icp;
	//icp.Event += gcnew f2(this, &FormReconst::ICP_update);
	//icp.AddPointCloud(pointcloud1);

}

Mat MultipleICP::ApplyTransform(Mat cloud, Mat t)
{
	return ApplyHomogTransform(cloud, t);
}

Mat MultipleICP::ApplyHomogTransform(const Mat m, const Mat h)
{
	Mat pc(m.rows,m.cols,MAT_TYPE);
	// pc is a Pts x 6 matrix
	Mat rot = ChangeOperatorSize(ExtractRot(h), 3); // get 6x6 rotation mat
	Mat trans = ExtractTrans(h); // get 1x6 translation mat

	pc = m * rot;

	Mat trans2 = trans/*.t()*/;

	Mat tr(pc.rows,pc.cols,MAT_TYPE);
	for(int i=0; i<tr.rows;i++){
		for(int j=0; j<tr.cols; j++){
			if(j < 3)
				tr.at<float>(i,j) = trans2.at<float>(0,j);
			else
				tr.at<float>(i,j) = 0;
		}
	}

	pc = pc + tr;
	return pc;
}

Mat MultipleICP::ApplyHomogTransform(const Mat m)
{
	return ApplyHomogTransform(m, *globalTransform);
	/*Mat pc(m.rows,m.cols,MAT_TYPE);
	// pc is a Pts x 6 matrix
	Mat rot = ChangeOperatorSize(ExtractRot(*globalTransform), 3); // get 6x6 rotation mat
	Mat trans = ExtractTrans(*globalTransform); // get 1x6 translation mat

	pc = m * rot;

	Mat trans2 = trans/*.t()*/;
/*
	Mat tr(pc.rows,pc.cols,MAT_TYPE);
	for(int i=0; i<tr.rows;i++){
		for(int j=0; j<tr.cols; j++){
			if(j < 3)
				tr.at<float>(i,j) = trans2.at<float>(0,j);
			else
				tr.at<float>(i,j) = 0;
		}
	}

	pc = pc + tr;
	return pc;*/
}

//-------------------------------------------------
// Routines dealing with conversion between
// the standard and homogeneous point co-ordinates
//
// We will use pointcloud format:
//
// X | Y | Z | H | r | g | b      (row 0)
// --------------------------
//
// where x = X/H, y = Y/H, z = Z/H
//
//-------------------------------------------------

// using column vectors here... is this right?
Mat MultipleICP::HomogToNormPoint(Mat p){
	int Dimension = p.rows - 1;
	Mat out(Dimension, 1, MAT_TYPE);
	for(int i = 0; i<Dimension; ++i){
		if(i < 3)
			out.at<float>(i,0) = p.at<float>(i,0) / p.at<float>(3,0);
		else
			out.at<float>(i,0) = p.at<float>(i+1,0);
	}
	return out;
}

Mat MultipleICP::NormToHomogPoint(Mat p){
	int Dimension = p.rows;
	Mat out(Dimension+1, 1, MAT_TYPE);
	for(int i = 0; i <= Dimension; ++i){
		if(i < 3)
			out.at<float>(i,0) = p.at<float>(i,0);
		else
			out.at<float>(i,0) = p.at<float>(i-1,0);
	}
	out.at<float>(3,0) = 1;
	return out;
}

Mat MultipleICP::HomogToNormPointCloud(Mat p){
	Mat out(p.rows,p.cols - 1,MAT_TYPE);
	Mat pointh(p.cols,1,MAT_TYPE);
	Mat point(out.cols,1,MAT_TYPE);
	for(int i=0; i<p.rows;i++){
		// form column vector of point
		Mat temp = p.row(i).t();
		temp.col(0).copyTo(pointh.col(0));
		// change co-ordinate system
		point = HomogToNormPoint(pointh);
		// copy to output matrix
		Mat temp2 = point.col(0).t();
		temp2.row(0).copyTo(out.row(i));
	}
	return out;
}

Mat MultipleICP::NormToHomogPointCloud(Mat p){
	Mat out(p.rows,p.cols + 1,MAT_TYPE);
	Mat point(p.cols, 1, MAT_TYPE);
	Mat pointh(p.cols + 1,1,MAT_TYPE);
	for(int i=0; i<p.rows;i++){
		// form column vector of point
		Mat temp = p.row(i).t();
		temp.col(0).copyTo(point.col(0));
		// change co-ordinate system
		pointh = NormToHomogPoint(point);
		// copy to output matrix
		Mat temp2 = pointh.col(0).t();
		temp2.row(0).copyTo(out.row(i));
	}
	return out;
}

// Turn a rotation matrix + translation vector into a 4D homogeneous transformation matrix
Mat MultipleICP::NormToHomogOperator(Mat rot, Mat trans){
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
Mat MultipleICP::ChangeOperatorSize(Mat transform, int Dim){
	//
	//  ie:   A   -->    A | 0     where each is a 3x3 submatrix of the result
	//                  -------
	//                   0 | I
	//
	Mat out = Mat::eye(Dim+3, Dim+3, MAT_TYPE);
	cv::Mat tmp = out(cv::Rect(0,0,Dim,Dim));
	transform.copyTo(tmp);
	return out;
}

Mat MultipleICP::MatColourToPoints(Mat m)
{
	Mat out(m.rows,3,MAT_TYPE);
	Mat temp = m(Rect(0,0,3,m.rows));
	temp.copyTo(out);
	return out;
}

Mat MultipleICP::RemoveNaNs(Mat m)
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

float MultipleICP::nan()
{
	unsigned long nan[2]={0xffffffff, 0x7fffffff}; 
	return *( float* )nan;
}




bool MultipleICP::searchForNeighbours(::flann::Index<::flann::L2<float> >* kdtree, Mat cloud, int pt, int search_parameter, std::vector<int> &indices, std::vector<float> &dists)
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

	//int64 c = GetCpuClocks();
	kdtree->knnSearch(queryM,indicesM, distsM, search_parameter, ::flann::SearchParams(FLANN_SEARCH_PARAMS));
	//std::cout << GetCpuClocks() - c << std::endl;

	return true;
	
}

Mat MultipleICP::CalculateNormals(::flann::Index<::flann::L2<float> >* kdtree, Mat pc)
{
	Mat out(0,3,MAT_TYPE);

	//for(){

	//	out.resize(out.rows + 1);

	//}

	//return out;

	Mat in = MatColourToPoints(pc);

	return computeFeature(in, kdtree, NORMAL_SEARCH_RADIUS);
}


Mat MultipleICP::computeFeature(const Mat &input, ::flann::Index<::flann::L2<float> >* kdtree, int radius)
{
	//int64 c = GetCpuClocks();

	Mat output(input.rows, 3, MAT_TYPE); // WILL NEED TO RESIZE DUE TO NANS? OR LEAVE NANS...

	Mat input_min = RemoveNaNs(input);
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

		computePointNormal(input_min, nn_indices, norm/*, output.points[pt].curvature*/);

		flipNormalTowardsViewpoint(/*input_->points[(*indices_)[pt]],*/ GetPoint(input, pt), Point3D(0,0,0/*vpx_, vpy_, vpz_*/), norm);

		output.at<float>(pt,0) = norm.X;
		output.at<float>(pt,1) = norm.Y;
		output.at<float>(pt,2) = norm.Z;
    }

	//std::cout << GetCpuClocks() - c << std::endl;

	return output;
}

void MultipleICP::computePointNormal(const Mat &cloud, const std::vector<int> &indices, Point3D &norm/*, float &curvature*/)
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

void MultipleICP::flipNormalTowardsViewpoint(Point3D point, Point3D vp, Point3D &norm)
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


void MultipleICP::solvePlaneParameters(const Eigen::Matrix3f &covariance_matrix, Point3D &norm/*, float &curvature*/)
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

unsigned int MultipleICP::computeMeanAndCovarianceMatrix(const Mat& cloud, const std::vector<int> &indices, Eigen::Matrix3f &covariance_matrix, Eigen::Vector4f &centroid)
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

// assumes pointcloud already initialised
void MultipleICP::MatToPcl(Mat pc, pcl::PointCloud<pcl::PointXYZ>::Ptr output)
{

	pcl::PointXYZ newPoint;

	for (int i=0;i<pc.rows;i++){
		newPoint.x = pc.at<float>(i,0);
		newPoint.y = pc.at<float>(i,1);
		newPoint.z = pc.at<float>(i,2);
		/*newPoint.r = pc.at<float>(i,3);
		newPoint.g = pc.at<float>(i,4);
		newPoint.b = pc.at<float>(i,5);*/
		output->at(i % 640, i / 640) = newPoint;
		//output->at(i) = newPoint;
	}
}

// assumes pc already sized
void MultipleICP::PclToMat(Mat pc, pcl::PointCloud<pcl::PointXYZRGB>::Ptr input)
{
	if(input->isOrganized()){
		for (int i=0;i<pc.rows;i++){
			pc.at<float>(i,0) = input->at(i % 640, i / 640).x;
			pc.at<float>(i,1) = input->at(i % 640, i / 640).y;
			pc.at<float>(i,2) = input->at(i % 640, i / 640).z;
			pc.at<float>(i,3) = input->at(i % 640, i / 640).r;
			pc.at<float>(i,4) = input->at(i % 640, i / 640).g;
			pc.at<float>(i,5) = input->at(i % 640, i / 640).b;
		}
	}else{
		for (int i=0;i<pc.rows;i++){
			pc.at<float>(i,0) = input->points[i].x;
			pc.at<float>(i,1) = input->points[i].y;
			pc.at<float>(i,2) = input->points[i].z;
			pc.at<float>(i,3) = input->points[i].r;
			pc.at<float>(i,4) = input->points[i].g;
			pc.at<float>(i,5) = input->points[i].b;
		}
	}
}

void MultipleICP::PclNormsToMats(Mat &output, Mat &norms, pcl::PointCloud<pcl::PointNormal> &mls_points, Mat &input)
{
	for (int i=0;i<norms.rows;i++){
		norms.at<float>(i,0) = mls_points.points[i].normal_x;
		norms.at<float>(i,1) = mls_points.points[i].normal_y;
		norms.at<float>(i,2) = mls_points.points[i].normal_z;
	}
	output = input.clone();
}

Mat MultipleICP::ComputeNormals(Mat cloud_in)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl_cloud->is_dense = false;
	pcl_cloud->width = IM_WIDTH;
	pcl_cloud->height = IM_HEIGHT;
	pcl_cloud->resize(IM_WIDTH * IM_HEIGHT);
	MatToPcl(cloud_in , pcl_cloud);

	// estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);

	//TODO: experiment with parameters
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(5.0f);
    ne.setInputCloud(pcl_cloud);
    ne.compute(*normals);
	
	Mat out(normals->points.size(), 3, MAT_TYPE);
	
	for(int i=0; i<normals->points.size();i++){
		out.at<float>(i,0) = normals->at(i).normal_x;
		out.at<float>(i,1) = normals->at(i).normal_y;
		out.at<float>(i,2) = normals->at(i).normal_z;
	}

	return out;
}


Point3D MultipleICP::GetPoint(Mat pts, int i)
{
	return Point3D(pts.at<float>(i,0),pts.at<float>(i,1),pts.at<float>(i,2));
}

void MultipleICP::PrintDebugMat(Mat m)
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

void MultipleICP::Log(std::string s)
{
	std::cout << s << std::endl << std::endl;
}

////////// IMAGE LOADING

void MultipleICP::SetInputParameters(__int64 F_, double pixel_size_)
{
	F = F_;
	pixel_size = pixel_size_;
}

//---------------------------------------------------------------------------
// Transform single point Projective->Real
//---------------------------------------------------------------------------
Point3D MultipleICP::singleConvertProjToRealCoord(Point3D p, float xtoz, float ytoz)
{
	/**
	* X_RealWorld = (X_proj / Horiz_res - 0.5) * depth_at_X * X_to_Z
	*/

	Point3D p2;

	// Normalise point.X from 0-640 to -0.5 - 0.5
	float NormX = (p.X / 640 - 0.5);  
	// Scale by depth and xtoz parameter
	p2.X = (float)(NormX * p.Z * xtoz);

	// Normalise point.Y from 0-480 to 0.5 - -0.5
	float NormY = (0.5 - p.Y / 480);
	// Scale by depth and ytoz parameter
	p2.Y = (float)(NormY * p.Z * ytoz);

	// Depth value unchanged
	p2.Z = p.Z;

	return p2;
}

//---------------------------------------------------------------------------
// Transform matrix pointcloud Projective->Real
//---------------------------------------------------------------------------
Mat MultipleICP::ConvertProjToRealCoord(Mat points)
{

	// These are the core transformation parameters
	float xtoz = 2 * 640.0 * pixel_size / F; // = 2 tan(f.o.v-h / 2)
	float ytoz = 2 * 480.0 * pixel_size / F; // = 2 tan(f.o.v-v / 2)

	Point3D p;
	Mat out(points.rows,points.cols,MAT_TYPE);
	for (int i = 0; i < points.rows; i++){
		p.X = points.at<float>(i,0);
		p.Y = points.at<float>(i,1);
		p.Z = points.at<float>(i,2);
		if(!isnan(p.Z))  // no point projecting if point is a NaN
			p = singleConvertProjToRealCoord(p, xtoz, ytoz);
		out.at<float>(i,0) = p.X;
		out.at<float>(i,1) = p.Y;
		out.at<float>(i,2) = p.Z; 
		for (int j = 3; j < points.cols; j++){
			// eg. copy colours directly across
			out.at<float>(i,j) = points.at<float>(i,j);
		}
	}

	return out;
}


//--------------------------------------------------------------------------------------
// Returns 6 column matrix with location & colour of all valid points from image data
//--------------------------------------------------------------------------------------
Mat MultipleICP::ImageToMat(Mat points, Mat colour)
{
	bool newDataType = NEW_DATA_TYPE;

	/*

	** IMAGE FORMAT **

	opencv image.at<>(y,x)  because y = row# and x = col#

	Top left is the origin

	(0,0)   +---------------------------+  (0,639)
			|                           |
			|                           |
			|                           |
	(479,0) +---------------------------+  (479,639)
	
	Note: appears transposed compared to intuition

	*/

	Mat out(points.rows * points.cols,6,MAT_TYPE);

	Mat colourArray[3];

	// Split into 1 Matrix per colour channel
	split(colour, colourArray);

	int count = 0;

	// Build pointcloud from the depth image
	for(int i=0; i<points.rows;i++){        // each image row
		for(int j=0; j<points.cols; j++){   // each image col

			// We will exclude any points of maximum/minimum depth (often representing bad surfaces such as glass, mirrors etc)
			// EDIT: No, we leave these points in, to keep the point cloud organised. They will be ignored later on.
			//if((points.at<uint16_t>(i,j) != 0) && (points.at<uint16_t>(i,j) != 65536)){

			// X, Y are simply image pixel locations
			out.at<float>(count,0) = j;
			out.at<float>(count,1) = points.rows-i-1;

			// Original data captured depth using different scaling factor
			if(newDataType)
				if((points.at<uint16_t>(i,j) == 0)||(points.at<uint16_t>(i,j) == 65536))
					out.at<float>(count,2) = nan();
				else
					out.at<float>(count,2) = (65536-((float)points.at<uint16_t>(i,j)))*(10000.0/65536.0);
			else
				if((points.at<uint16_t>(i,j) == 0)||(points.at<uint16_t>(i,j) == 65536))
					out.at<float>(count,2) = nan();
				else
					out.at<float>(count,2) = (65536-((float)points.at<uint16_t>(i,j)))*(5500.0/65536.0);

			//TODO: might want to remove this later
			// remove distant points
			if(out.at<float>(count,2) > MAX_DISTANCE)
				out.at<float>(count,2) = nan();

			// Set colours
			out.at<float>(count,3) = colourArray[2].at<uint8_t>(i,j);
			out.at<float>(count,4) = colourArray[1].at<uint8_t>(i,j);
			out.at<float>(count,5) = colourArray[0].at<uint8_t>(i,j);

			count++;
			//}
		}
	}

	out.resize(count);

	return out;
}