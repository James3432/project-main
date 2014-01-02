//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------

#include "StdAfx.h"
#include "NIGrabber.h"
#include "FormKinect.h"

//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------
#define XML_PATH "C:\\Users\\James\\Documents\\Visual Studio 2010\\Projects\\RobotController\\KinectGrabber\\config.xml"

//---------------------------------------------------------------------------
// Macros
//---------------------------------------------------------------------------
/*#define CHECK_RC(rc, what)											\
	if (rc != XN_STATUS_OK)											\
	{																\
		//return what + " failed: " + xnGetStatusString(rc));		
		return "error later..."; \
		//return rc;													
	}*/

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------

using namespace xn;
using namespace msclr::interop;
using namespace System::ComponentModel;

			
//------------------------
// NIGrabber Constructor
//------------------------
NIGrabber::NIGrabber(void)
{
	this->grabber_worker = (gcnew System::ComponentModel::BackgroundWorker());
	this->grabber_worker->WorkerReportsProgress = true;
	this->grabber_worker->WorkerSupportsCancellation = true;
	this->grabber_worker->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &NIGrabber::grabber_worker_DoWork);
	this->grabber_worker->ProgressChanged += gcnew System::ComponentModel::ProgressChangedEventHandler(this, &NIGrabber::grabber_worker_ProgressChanged);
	this->grabber_worker->RunWorkerCompleted += gcnew System::ComponentModel::RunWorkerCompletedEventHandler(this, &NIGrabber::grabber_worker_RunWorkerCompleted);

}

//-----------------------
// Check if file exists
//-----------------------
XnBool NIGrabber::fileExists(const char *fn)
{
	XnBool exists;
	xnOSDoesFileExist(fn, &exists);
	return exists;
}

//--------------------
// Start grabbing
//--------------------
void NIGrabber::run(KinectGrabber::FormKinect^ form)
{
	frm = form;
	grabber_worker->RunWorkerAsync();
}

//---------------------
// Stop grabbing
//---------------------
void NIGrabber::stop(void)
{
	grabber_worker->CancelAsync();
}

//------------------------
// Write to form log box
//------------------------
void NIGrabber::Log(String^ str){
	frm->Log(str);
}

//-----------------------------
// Check if RGB capture ticked
//-----------------------------
bool NIGrabber::isRGB(){
	return frm->chk_rgb->Checked;
}


//--------------------
// Get save location
//--------------------
String^ NIGrabber::getLoc(){
	return frm->save_loc;
}



//--------------------------------------------------------------
// Wrappers for writing to Log file (handled by FormKinect.h)
//--------------------------------------------------------------
void NIGrabber::InitLog(){
	frm->InitLog(getLoc());
}

void NIGrabber::CloseLog(){
	frm->CloseLog();
}

void NIGrabber::WriteToLog(String^ str){
	frm->WriteToLog(str);
}

void NIGrabber::WriteToLog(int i){
	System::String^ str;
	str += i;
	frm->WriteToLog(str);
}

//--------------------------------------
// Check if directory exists for saving
//--------------------------------------
bool NIGrabber::dirExists(const std::string& dirName_in)
{
  DWORD ftyp = GetFileAttributesA(dirName_in.c_str());
  if (ftyp == INVALID_FILE_ATTRIBUTES)
    return false;  //something is wrong with your path!

  if (ftyp & FILE_ATTRIBUTE_DIRECTORY)
    return true;   // this is a directory!

  return false;    // this is not a directory!
}

//----------------------------------------------
// Convert managed string to unmanaged string
//----------------------------------------------
std::string NIGrabber::ManagedToString(System::String^ str){
	System::String^ tmp = str;
	std::string temp(marshal_as<std::string>(tmp));
	return temp;
}

//---------------------------------------------------------------------------
// Do grabbing
//---------------------------------------------------------------------------

	System::Void NIGrabber::grabber_worker_DoWork(System::Object^  sender, System::ComponentModel::DoWorkEventArgs^  e) {

		XnStatus nRetVal = XN_STATUS_OK;

		
		DepthGenerator depth;
		ImageGenerator imageGen; 
		Context context;
		ScriptNode scriptNode;

		StreamWriter^ pwriter;
		
		EnumerationErrors errors;

		const char *fn = NULL;
		if	(fileExists(XML_PATH)) fn = XML_PATH;
		else {
			Log("Could not find XML config files - aborting.");
		}
		//printf("Reading config from: '%s'\n", fn);
		nRetVal = context.InitFromXmlFile(fn, scriptNode, &errors);

		if (nRetVal == XN_STATUS_NO_NODE_PRESENT)
		{
			XnChar strError[1024];
			errors.ToString(strError, 1024);
			//printf("%s\n", strError);
			Log("Device not connected");
			//return (nRetVal);
		}
		else if (nRetVal != XN_STATUS_OK)
		{
			//printf("Open failed: %s\n", xnGetStatusString(nRetVal));
			Log("Device found, connection failed");
			//return (nRetVal);
		}
		else
		{

		nRetVal = context.FindExistingNode(XN_NODE_TYPE_DEPTH, depth);
		nRetVal = context.FindExistingNode(XN_NODE_TYPE_IMAGE, imageGen);
		//CHECK_RC(nRetVal, "Find depth generator");


		Log(depth.IsCapabilitySupported(XN_CAPABILITY_ALTERNATIVE_VIEW_POINT) ? "Depth viewpoint transform support.": "WARNING: no depth viewpoint transform");
		
		depth.GetAlternativeViewPointCap().SetViewPoint(imageGen);


		XnFPSData xnFPS;
		nRetVal = xnFPSInit(&xnFPS, 180);
		if (nRetVal != XN_STATUS_OK){
			Log("Fps read failed"); //, xnGetStatusString(nRetVal));
		}

		DepthMetaData depthMD;
		ImageMetaData imageMD;


		nRetVal = context.WaitOneUpdateAll(depth);
		depth.GetMetaData(depthMD);
		imageGen.GetMetaData(imageMD);


		Log("X-res: "+System::Convert::ToString(depthMD.XRes()));
		Log("Y-res: "+System::Convert::ToString(depthMD.YRes()));

		cv::Mat colorArr[3]; 
		cv::Mat colorImage; 
		cv::Mat depthImage;
		const XnRGB24Pixel* pImageRow; 
		const XnRGB24Pixel* pPixel; 
		const XnDepthPixel* pDepthRow; 
		const XnDepthPixel* pDepthPixel; 

		uchar* Bptr;
		uchar* Gptr;
		uchar* Rptr;

		uint16_t t;
		double d;

		//bool rgb = (form->chk_rgb->Checked);
		bool rgb = isRGB();
		bool mesh = false;

		std::string nativeString = ManagedToString(getLoc());
		if(!dirExists(nativeString)){
			CreateDirectory((LPCTSTR)nativeString.c_str(),NULL);
		}

		if(mesh){
			pwriter = gcnew StreamWriter(getLoc()+"\\mesh.ply");
			pwriter->WriteLine("ply");
			pwriter->WriteLine("format ascii 1.0");
			pwriter->WriteLine("comment This is a test ply file!");
			pwriter->WriteLine("element vertex 307200");
			pwriter->WriteLine("property float x");
			pwriter->WriteLine("property float y");
			pwriter->WriteLine("property float z");
			pwriter->WriteLine("property uchar red");            
			pwriter->WriteLine("property uchar green");
			pwriter->WriteLine("property uchar blue");
			pwriter->WriteLine("end_header");			
		}

		InitLog();
		WriteToLog("KinectGrabber log...");

		int i=0;

		while(!(grabber_worker->CancellationPending)){ 

			i++;

			nRetVal = context.WaitOneUpdateAll(depth);
			if (nRetVal != XN_STATUS_OK)
			{
				Log("Data grab failed"); //, xnGetStatusString(nRetVal));
			}

			xnFPSMarkFrame(&xnFPS);
		
			depth.GetMetaData(depthMD);

			if(rgb){
				imageGen.GetMetaData(imageMD);
				pImageRow = imageMD.RGB24Data(); 
				colorArr[0] = cv::Mat(imageMD.YRes(),imageMD.XRes(),CV_8U); 
				colorArr[1] = cv::Mat(imageMD.YRes(),imageMD.XRes(),CV_8U); 
				colorArr[2] = cv::Mat(imageMD.YRes(),imageMD.XRes(),CV_8U);
			}

			Log("Frame #"+System::Convert::ToString(depthMD.FrameID()));
		
			Log("Grab rate: "+System::Convert::ToString(xnFPSCalc(&xnFPS))+" fps");

	
			//imageGen.SetPixelFormat(XN_PIXEL_FORMAT_RGB24 );  /*// 
        
        
			pDepthRow = depthMD.Data(); 

			depthImage = cv::Mat(depthMD.YRes(),depthMD.XRes(),CV_16U);

			 int max=0,min=10000;

			 //assume rgb & depth image are same size
			for (int y=0; y<depthMD.YRes(); y++){ 
				pDepthPixel = pDepthRow;

				if(rgb){
					pPixel = pImageRow; 
					Bptr = colorArr[0].ptr<uchar>(y); 
					Gptr = colorArr[1].ptr<uchar>(y); 
					Rptr = colorArr[2].ptr<uchar>(y); 
				}

				uint16_t* Dptr = depthImage.ptr<uint16_t>(y);
				for(int x=0; x<depthMD.XRes(); ++x ,++pDepthPixel){ 
					if(rgb)
						++pPixel;
					//Dptr[x] = 65536 - ((65536 * (uint16_t)(*pDepthPixel)/10000.0) > 65536.0) ? 65536: (65536 * (uint16_t)(*pDepthPixel)/10000.0);
					t = (uint16_t)(*pDepthPixel);
					d = t * 65536 / 10000.0;
					if(d > 65536.0)
						Dptr[x] = 0;
					else
						Dptr[x] = 65536 - d;


					if(Dptr[x] < min)
						min = Dptr[x];
					if(Dptr[x] > max)
						max = Dptr[x];

					if(rgb){
						Bptr[x] = pPixel->nBlue; 
						Gptr[x] = pPixel->nGreen; 
						Rptr[x] = pPixel->nRed; 
						//Dptr[x] = (uint16_t)(*pDepthPixel);
						if(mesh && i==20){
							XnPoint3D p[1] = {x, y, (float)(*pDepthPixel)/100};
							XnPoint3D p2[1];
							depth.ConvertProjectiveToRealWorld(1,p,p2);
							pwriter->WriteLine(p2[0].X+" "+p2[0].Y+" "+-p2[0].Z+" "+Rptr[x]+" "+Gptr[x]+" "+Bptr[x]);    
							//pwriter->WriteLine(x+" "+(imageMD.YRes()-y)+" "+Dptr[x]/100+" "+Rptr[x]+" "+Gptr[x]+" "+Bptr[x]);    
						}
					}
				} 

				pDepthRow += depthMD.XRes();
				if(rgb)
					pImageRow += imageMD.XRes(); 
			} 

			Log("min: " +min);
			Log("max: "+max);

			if(rgb)
				cv::merge(colorArr,3,colorImage); 

			std::stringstream st;
			//st << depthMD.FrameID();
			st << i;
			std::string id_str = st.str();

			//CapturedFrames folder must exist!!! 
		
			std::string str_aux = nativeString+ "\\test" + id_str +".jpg"; 

			if(rgb){
				IplImage bgrIpl = colorImage;                      
				cvSaveImage(str_aux.c_str(),&bgrIpl);     
			}

			str_aux = nativeString+ "\\testD" + id_str +".png"; 
	   
			IplImage bgrIpl2 = depthImage;                      
			cvSaveImage(str_aux.c_str(),&bgrIpl2);   

			if(mesh && i==20){
				pwriter->Close();
			}

			WriteToLog(i);

		}

		//pwriter->Close();

		CloseLog();

		depth.Release();
		imageGen.Release();
		scriptNode.Release();
		context.Release();

		Log("Kinect data grabbed");

		}
	}


//---------------------------------------------------------------------------
// Grabbing report
//---------------------------------------------------------------------------

	System::Void NIGrabber::grabber_worker_ProgressChanged(System::Object^  sender, System::ComponentModel::ProgressChangedEventArgs^  e) {
		 }


//---------------------------------------------------------------------------
// Stop grabbing
//---------------------------------------------------------------------------

	System::Void NIGrabber::grabber_worker_RunWorkerCompleted(System::Object^  sender, System::ComponentModel::RunWorkerCompletedEventArgs^  e) {
		//Log("Kinect data grabbed");	
		//CloseLog();
	}
