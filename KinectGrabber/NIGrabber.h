//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------

//#include <XnOpenNI.h>
#include <XnLog.h>
#include <XnCppWrapper.h>
#include <XnFPSCalculator.h>
#include "cv.h"
#include "highgui.h"
#include <sstream>
#include <msclr/marshal_cppstd.h>
#include <stdint.h>

using namespace System;
using namespace System::ComponentModel;

namespace KinectGrabber{
	ref class FormKinect;
}

#pragma once
ref class NIGrabber
{
public:
	NIGrabber(void);	
	void run(KinectGrabber::FormKinect^);
	void stop(void);
private: 
	bool isRGB();
	String^ getLoc();
	void InitLog();
	void CloseLog();
	void WriteToLog(String^ str);
	void WriteToLog(int i);
	KinectGrabber::FormKinect^ frm;
	System::ComponentModel::BackgroundWorker^  grabber_worker;
	void Log(String^ str);
	XnBool fileExists(const char *fn);
	bool dirExists(const std::string& dirName_in);
	std::string ManagedToString(System::String^ str);
	System::Void grabber_worker_DoWork(System::Object^  sender, System::ComponentModel::DoWorkEventArgs^  e);
	System::Void grabber_worker_ProgressChanged(System::Object^  sender, System::ComponentModel::ProgressChangedEventArgs^  e);
	System::Void grabber_worker_RunWorkerCompleted(System::Object^  sender, System::ComponentModel::RunWorkerCompletedEventArgs^  e);
};

