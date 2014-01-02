//-----------------------
// Parameter definitions    
//-----------------------

/////////////////////////
// SYSTEM
/////////////////////////
#define DEBUG_OUTPUT 1   // Whether to display console with additional debug information
#define XML_PATH "C:\\Users\\James\\Documents\\Visual Studio 2010\\Projects\\RobotController\\KinectGrabber\\config.xml"
#define IM_WIDTH 640                 // Overall image width  (used in ICP matching)
#define IM_HEIGHT 480                // Overall image height

#define PASS_BY_VALUE 1 //todo: remove

#define EXIT_ON_COMPLETION 1
#define SHUTDOWN_ON_COMPLETION 0

/////////////////////////
// INPUT
/////////////////////////
#define DATA_SOURCE "J:\\Kinect0102132"//"I:\\Kinect Data"    // directory containing source data
#define FILE_NAME_COLOUR "test"
#define FILE_NAME_DEPTH "testD"
#define START_NUM 150                 // Start of image range
#define END_NUM 155//745                  // End of image range
#define IM_STEP 1  // DEFAULT: 1

#define NEW_DATA_TYPE 1  // Whether data was captured after 01/12/2012

#define DELAYED_LOAD 1                  // Whether to delay image loading until needed during execution (required for large datasets)

#define PRE_DOWNSAMPLE 0        //TODO: fix         // THE problem with downsampling before hand is that all clouds are then UNORGANISED so, for instance, matching doesn't work (unless we used flann)
#define PRE_DOWNSAMPLE_SIZE 10000        // requires PRE_DOWNSAMPLE_SIZE >= MAX_CLOUD_SIZE

#define MAX_DISTANCE 5000  // cut off all points further from camera than this

/////////////////////////
// ICP CORE
/////////////////////////
#define CONVERGENCE_CRITERION 1      // Error allowance - "convergence criterion". RMS distance under which the icp will consider 2 clouds as matched
#define STEADY_STATE_DELTA	0	     // If error changes by less than this, stop iterating
#define MAX_ITER 10                  // Max # iterations of waiting for convergence, best: 20
#define SAMPLE_SIZE 100             // Number of points from each cloud to run icp on, best: 500
#define POINT_MATCH_MARGIN 30        // Size of area to search when matching points from 2 clouds  (NO OPTIMISATION: 640 , GIVES BEST RESULTS. FAST: 4, GIVES GOOD RESULTS)
#define IMAGE_MARGIN 50              // Width of image margin which will not be searched when performing sampling

/////////////////////////
// ICP EXTENSIONS
/////////////////////////
#define USE_COLOUR_MATCHING 0        // Bool, whether to use colour match criterion for point matching
#define COLOUR_CRITERION 100        // RMS distance between RGB colours of 2 points for them to be allowed to match

/////////////////////////
// FLANN
/////////////////////////
#define USE_FLANN_MATCHING 1                  // use flann for pointmatching  (as opposed to own custom method)
#define FLANN_SEARCH_RADIUS 100           // # nearest neighbours to search for a colour/normal match
#define USE_NORMAL_SAMPLING 1        // Whether to use normal space sampling
#define FILL_ALL_SAMPLES 1
#define USE_NORMAL_MATCHING 0        // Whether to consider normals when matching points
//TODO: add USE_NORMALS instead
#define CALCULATE_KDTREES USE_FLANN_MATCHING//0//( USE_NORMAL_SAMPLING || USE_NORMAL_MATCHING || USE_FLANN_MATCHING )  // TODO: replace by OR of above 3 conditions
#define USE_NATIVE_FLANN 1           // Whether to use flann (rather than cvflann)

#define MATCH_DISTANCE_THRESHOLD 5000 // TODO:try 100

#define NORMAL_SEARCH_RADIUS 10        // Number of neighbouring points to use when calculating a normal
#define NORMAL_CRITERION 0.5//0.175//0.765       // How close 2 norms must be to match (2 is max??).  0.765 = within 45deg
//#define NORMAL_PARAMS 123           // TODO: Parameters for normal calculation
#define FLANN_SEARCH_PARAMS 64       // Number of kdtree checks (TODO: ??)
#define KD_TREES 8                   // Number of kdtrees to parallel search (TODO: ??)

/////////////////////////
// OUTPUT
/////////////////////////
#define REDUCED_OUTPUT 1
#define MAX_CLOUD_SIZE 100000
#define FUSE_POINTS 0
#define FUSION_THRESHOLD 50 // in mm
#define PAIRWISE_OUTPUT 0
#define OUTPUT_ORIGINAL 1
#define VISUALISE_SAMPLING 0
#define VISUALISE_MATCHING 0
#define VISUALISE_NORMALS 0
#define WRITE_BUFFER_SIZE 1000
#define SAVE_LOC "c:\\users\\james\\desktop\\meshICP.ply"
#define SAVE_LOC_ORIGINAL "c:\\users\\james\\desktop\\meshICPorig.ply"
#define TRANSFORM_LOG "c:\\users\\james\\desktop\\transformations.jvk"