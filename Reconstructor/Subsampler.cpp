#include "StdAfx.h"
#include "Subsampler.h"
#include "NAN.h"


Subsampler::Subsampler(void)
{
}


Subsampler::~Subsampler(void)
{
}

// sign of a float: 0 for positive, 1 for negative
int Subsampler::sign(float i)
{
	if(i >= 0)
		return 0;
	else
		return 1;
}
bool Subsampler::isEdge(int p)
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

void Subsampler::setInput(Mat in){
	A = in;
}

void Subsampler::setOutput(std::vector<int> &samp){
	samples = samp;
}

void Subsampler::setNorms(Mat n){
	Pnorms = n;
}

Mat NormalSubsample::run(std::vector<int> &samples)
{
	
	int sample_size = MIN(SAMPLE_SIZE, A.rows);
	Mat out(sample_size,A.cols,MAT_TYPE);

	srand( (unsigned)time( NULL ) );

	Mat A_ = /*RemoveNaNs*/A;

	// are there always enough normals?? what if A is small? etc

	// conceptually, 8 buckets of points, with normals in 1 of the 8 octets of 3D space (based purely on the signs of x,y,z)

	// idea: pick randomly, increment one of 8 counters. Don't accept sample if counter == sample_size/8. Use decreasing available_points array like with standard sampling. Use timeout for fail case

	// bucket_size[ 000, 001, 010, 011, 100, 101, 110, 111 ] for [ xyz ] of normal, 0 == +ve, 1 == -ve
	int bucket_size = (int)((double)sample_size / 8.0) + 1;

	int bucket_counter[8];
	for(int i=0; i<8; ++i)
		bucket_counter[i] = bucket_size;

	std::vector<int> unused_points;

	int* available_points = new int[A_.rows];
	for(int i=0; i < A_.rows; i++)
		available_points[i] = i;

	int range_max = A_.rows - 1;
	int r, val;

	int point = 0;
	while((point < sample_size) && (range_max >= 0)){

		r = (int)((double)rand() / (RAND_MAX + 1) * range_max);
		val = available_points[r];

		if(!(isnan(A_.at<float>(val,2)) || (isnan(Pnorms.at<float>(val,0))) || ((A_.rows == 307200) && isEdge(val)))){

			// index into the array of buckets
			int bucket = sign(Pnorms.at<float>(val,2)) + 2*sign(Pnorms.at<float>(val,1)) + 4*sign(Pnorms.at<float>(val,0));

			if(bucket_counter[bucket] > 0){
				bucket_counter[bucket]--;
				A_.row(val).copyTo(out.row(point));
				samples.push_back(val);
				++point;
			}else{
				unused_points.push_back(val);
			}
		}

		available_points[r] = available_points[range_max];
		range_max--;
	}

	if(FILL_ALL_SAMPLES){
		if(unused_points.size() >= (sample_size - samples.size())) // are there enough points?
		{
			while(samples.size() < sample_size){
				// pick some random points to fill up the subsample anyway
				// TODO: is this wise?
				int rn = (int)((double)rand() / (RAND_MAX + 1) * unused_points.size());
				r = unused_points[rn];
				unused_points.erase(unused_points.begin() + rn);
				A_.row(r).copyTo(out.row(point++));
				samples.push_back(r);
			}
		}
	}else
		out.resize(point); // just in case fewer points sampled

	delete[] available_points;

	return out;
    
};

Mat RandomSubsample::run(std::vector<int> &samples)
{

	int sample_size = MIN(SAMPLE_SIZE, A.rows);
	Mat out(sample_size,A.cols,MAT_TYPE);

	srand( (unsigned)time( NULL ) );

	Mat A_ = /*RemoveNaNs*/A;
	
	int* available_points = new int[A.rows];
	for(int i=0; i < A.rows; i++)
		available_points[i] = i;

	int range_max = A.rows - 1;
	int r, val;
	int i=0;
	while((i < sample_size) && (range_max >= 0)){
		
		r = (int)((double)rand() / (RAND_MAX + 1) * range_max);
		val = available_points[r];
			
		if(!(isnan(A.at<float>(val,2)) || ((A.rows == 307200) && isEdge(val)))){

			A.row(val).copyTo(out.row(i));
			samples.push_back(val);
			++i;
		}
		available_points[r] = available_points[range_max];
		range_max--;
	}
	
	out.resize(i);
	delete[] available_points;

	return out;

};

Mat PoissonSubsample::run(std::vector<int> &samples)
{
	/*
		Implementation left as future extension
	*/
}