// Reconstructor.cpp : main project file.

#pragma once

#include "stdafx.h"

#include <cstdlib>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <sstream>
#include <stdint.h>
#include <XnCppWrapper.h>
#include <vector>

#include "MultipleICP.h"
#include "global_parameters.h"

using namespace cv;
using namespace std;
using namespace xn;


//---------------------------------------------------------------------------
// Log to window
//---------------------------------------------------------------------------
void Log(std::string txt)
{
    std::cout << txt << "\n";
}

void Log(uint16_t txt)
{
	Log(boost::lexical_cast<std::string>(txt));
}

//---------------------------------------------------------------------------
// Check if fileExists
//---------------------------------------------------------------------------
XnBool fileExists(const char *fn)
{
	XnBool exists;
	xnOSDoesFileExist(fn, &exists);
	return exists;
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

// code from MSDN
bool SystemShutdown()
{
   HANDLE hToken; 
   TOKEN_PRIVILEGES tkp; 
 
   // Get a token for this process. 
 
   if (!OpenProcessToken(GetCurrentProcess(), 
        TOKEN_ADJUST_PRIVILEGES | TOKEN_QUERY, &hToken)) 
      return( FALSE ); 
 
   // Get the LUID for the shutdown privilege. 
 
   LookupPrivilegeValue(NULL, SE_SHUTDOWN_NAME, 
        &tkp.Privileges[0].Luid); 
 
   tkp.PrivilegeCount = 1;  // one privilege to set    
   tkp.Privileges[0].Attributes = SE_PRIVILEGE_ENABLED; 
 
   // Get the shutdown privilege for this process. 
 
   AdjustTokenPrivileges(hToken, FALSE, &tkp, 0, 
        (PTOKEN_PRIVILEGES)NULL, 0); 
 
   if (GetLastError() != ERROR_SUCCESS) 
      return FALSE; 
 
   // Shut down the system and force all applications to close. 
 
   if (!ExitWindowsEx(EWX_SHUTDOWN | EWX_FORCE, 
               SHTDN_REASON_MAJOR_OPERATINGSYSTEM |
               SHTDN_REASON_MINOR_UPGRADE |
               SHTDN_REASON_FLAG_PLANNED)) 
      return FALSE; 

   //shutdown was successful
   return TRUE;
}

int main(int argc, const char* argv[] )
{

	if(SHUTDOWN_ON_COMPLETION){
		Log("** SYSTEM WILL SHUTDOWN UPON COMPLETION! **");
		Log("");
	}

	int start_im = START_NUM;
	int end_im = END_NUM;

	std::string im_dir = DATA_SOURCE;

	XnStatus nRetVal = XN_STATUS_OK;

	/* 
	Connect to Kinect if possible
	*/
	Context context;
	ScriptNode scriptNode;
	DepthGenerator depth;

	XnUInt64 F;
	XnDouble pixel_size;
	XnFieldOfView fov;

	EnumerationErrors errors;

	// Load OpenNI XML config file
	const char *fn = NULL;
	if	(! fileExists(XML_PATH))
	{
		Log("Could not find XML config files - aborting.");
	} 
	else 
	{
		fn = XML_PATH;
			
		nRetVal = context.InitFromXmlFile(fn, scriptNode, &errors);

		if (nRetVal == XN_STATUS_NO_NODE_PRESENT)
		{
			// If device not available, use known parameters
			Log("Device not connected");
			F = 120;
			pixel_size = 0.10419999808073;
			fov.fHFOV = 1.01446867075074;
			fov.fVFOV = 0.789809434496447;
		}
		else if (nRetVal != XN_STATUS_OK)
		{
			Log("Device found, connection failed");
			F = 120;
			pixel_size = 0.10419999808073;
			fov.fHFOV = 1.01446867075074;
			fov.fVFOV = 0.789809434496447;
		}
		else
		{
			// If device available, get parameters from it

			nRetVal = context.FindExistingNode(XN_NODE_TYPE_DEPTH, depth);

			depth.GetFieldOfView(fov);

			// get the focal length in mm (ZPS = zero plane distance)
			depth.GetIntProperty ("ZPD", F);

			// get the pixel size in mm ("ZPPS" = pixel size at zero plane)
			depth.GetRealProperty ("ZPPS", pixel_size);

		}

		Log("F: "+boost::lexical_cast<std::string>(F));
		Log("Pix size: "+ boost::lexical_cast<std::string>(pixel_size));
		Log("FoV H: "+boost::lexical_cast<std::string>(fov.fHFOV));
		Log("FoV V: "+boost::lexical_cast<std::string>(fov.fVFOV));

		Log("Starting...");

		MultipleICP icp;
		icp.SetInputParameters(F, pixel_size);		

		int iStartTime = GetTickCount();

		Log("Running ICP...");
		icp.run();
		Log("Finished ICP");

		int iEndTime = GetTickCount();
		int iDiff = iEndTime - iStartTime;
		cout << "Took " << ((float)iDiff/1000.0) << "s" << endl;

		Log("**************");
		Log("*  COMPLETE  *");
		Log("**************");
	}

	if(SHUTDOWN_ON_COMPLETION){
		if(SystemShutdown())
			Log("Shutting down...");
		else
			Log("Failed to shutdown");
	}

	if(EXIT_ON_COMPLETION)
		exit(0); // for some reason the program won't close otherwise...

	return 0;
}