#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <sstream>
#include <stdint.h>
#include <msclr/marshal.h>
#include <msclr/marshal_cppstd.h>
#include <XnCppWrapper.h>
#include <vector>
//#include <math.h>  NEEDED??
//#include <float.h>
//#include <limits>

#include "PlyWriter.h"
#include "Point3D.h"
#include "PointCloud.h"
#include "Kabsch.h"
#include "IterativeClosestPoint.h"
#include "MultipleICP.h"
#include "NAN.h"
#include "global_parameters.h"

//#define NAN System::Double::NaN       // or: numeric_limits<float>::quiet_NaN()
//#define isnan System::Double::IsNaN
//#define NAN (unsigned long nan[2]={0xffffffff, 0x7fffffff}; *( double* )nan)
//#define isnan(x) (x!=x)

using namespace msclr::interop;

using namespace cv;
using namespace std;
using namespace xn;


namespace Reconstructor {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;
	using namespace System::IO;

	/// <summary>
	/// Summary for FormReconst
	/// </summary>
	public ref class FormReconst : public System::Windows::Forms::Form
	{
	public:
		FormReconst(void)
		{
			InitializeComponent();
		}

	protected:
		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		~FormReconst()
		{
			if (components)
			{
				delete components;
			}
		}
	private: System::Windows::Forms::Button^  btn_run;
	private: System::Windows::Forms::OpenFileDialog^  open_depth1;

	private: System::Windows::Forms::Button^  btn_depth1;

	private: System::Windows::Forms::Label^  lbl_stat;
	private: System::Windows::Forms::ListBox^  log_win;
	private: System::Windows::Forms::Button^  btn_col2;
	private: System::Windows::Forms::Button^  btn_col1;
	private: System::Windows::Forms::Button^  btn_depth2;
	private: System::Windows::Forms::OpenFileDialog^  open_col1;
	private: System::Windows::Forms::OpenFileDialog^  open_depth2;
	private: System::Windows::Forms::OpenFileDialog^  open_col2;
	private: System::ComponentModel::BackgroundWorker^  worker;
	private: System::Windows::Forms::ProgressBar^  prog_bar;
	private: System::Windows::Forms::GroupBox^  groupBox1;
	private: System::Windows::Forms::GroupBox^  groupBox2;

	protected: 

	private:
		/// <summary>
		/// Required designer variable.
		/// </summary>
		System::ComponentModel::Container ^components;

#pragma region Windows Form Designer generated code
		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		void InitializeComponent(void)
		{
			System::ComponentModel::ComponentResourceManager^  resources = (gcnew System::ComponentModel::ComponentResourceManager(FormReconst::typeid));
			this->btn_run = (gcnew System::Windows::Forms::Button());
			this->open_depth1 = (gcnew System::Windows::Forms::OpenFileDialog());
			this->btn_depth1 = (gcnew System::Windows::Forms::Button());
			this->lbl_stat = (gcnew System::Windows::Forms::Label());
			this->log_win = (gcnew System::Windows::Forms::ListBox());
			this->btn_col2 = (gcnew System::Windows::Forms::Button());
			this->btn_col1 = (gcnew System::Windows::Forms::Button());
			this->btn_depth2 = (gcnew System::Windows::Forms::Button());
			this->open_col1 = (gcnew System::Windows::Forms::OpenFileDialog());
			this->open_depth2 = (gcnew System::Windows::Forms::OpenFileDialog());
			this->open_col2 = (gcnew System::Windows::Forms::OpenFileDialog());
			this->worker = (gcnew System::ComponentModel::BackgroundWorker());
			this->prog_bar = (gcnew System::Windows::Forms::ProgressBar());
			this->groupBox1 = (gcnew System::Windows::Forms::GroupBox());
			this->groupBox2 = (gcnew System::Windows::Forms::GroupBox());
			this->groupBox1->SuspendLayout();
			this->groupBox2->SuspendLayout();
			this->SuspendLayout();
			// 
			// btn_run
			// 
			this->btn_run->Location = System::Drawing::Point(25, 118);
			this->btn_run->Name = L"btn_run";
			this->btn_run->Size = System::Drawing::Size(130, 43);
			this->btn_run->TabIndex = 0;
			this->btn_run->Text = L"Run";
			this->btn_run->UseVisualStyleBackColor = true;
			this->btn_run->Click += gcnew System::EventHandler(this, &FormReconst::btn_run_Click);
			// 
			// open_depth1
			// 
			this->open_depth1->Filter = L"Images (*.PNG;*.JPG)|*.PNG;*.JPG";
			this->open_depth1->Title = L"Open image";
			this->open_depth1->FileOk += gcnew System::ComponentModel::CancelEventHandler(this, &FormReconst::open_depth1_FileOk);
			// 
			// btn_depth1
			// 
			this->btn_depth1->Location = System::Drawing::Point(24, 28);
			this->btn_depth1->Name = L"btn_depth1";
			this->btn_depth1->Size = System::Drawing::Size(105, 45);
			this->btn_depth1->TabIndex = 1;
			this->btn_depth1->Text = L"Depth";
			this->btn_depth1->UseVisualStyleBackColor = true;
			this->btn_depth1->Click += gcnew System::EventHandler(this, &FormReconst::btn_depth1_Click);
			// 
			// lbl_stat
			// 
			this->lbl_stat->AutoSize = true;
			this->lbl_stat->Location = System::Drawing::Point(197, 133);
			this->lbl_stat->MinimumSize = System::Drawing::Size(196, 13);
			this->lbl_stat->Name = L"lbl_stat";
			this->lbl_stat->Size = System::Drawing::Size(196, 13);
			this->lbl_stat->TabIndex = 2;
			this->lbl_stat->Text = L"No image set, run to use default images.";
			// 
			// log_win
			// 
			this->log_win->FormattingEnabled = true;
			this->log_win->HorizontalScrollbar = true;
			this->log_win->Location = System::Drawing::Point(25, 167);
			this->log_win->Name = L"log_win";
			this->log_win->ScrollAlwaysVisible = true;
			this->log_win->SelectionMode = System::Windows::Forms::SelectionMode::None;
			this->log_win->Size = System::Drawing::Size(604, 290);
			this->log_win->TabIndex = 3;
			// 
			// btn_col2
			// 
			this->btn_col2->Location = System::Drawing::Point(162, 29);
			this->btn_col2->Name = L"btn_col2";
			this->btn_col2->Size = System::Drawing::Size(95, 44);
			this->btn_col2->TabIndex = 4;
			this->btn_col2->Text = L"Colour";
			this->btn_col2->UseVisualStyleBackColor = true;
			this->btn_col2->Click += gcnew System::EventHandler(this, &FormReconst::btn_col2_Click);
			// 
			// btn_col1
			// 
			this->btn_col1->Location = System::Drawing::Point(162, 28);
			this->btn_col1->Name = L"btn_col1";
			this->btn_col1->Size = System::Drawing::Size(105, 44);
			this->btn_col1->TabIndex = 5;
			this->btn_col1->Text = L"Colour";
			this->btn_col1->UseVisualStyleBackColor = true;
			this->btn_col1->Click += gcnew System::EventHandler(this, &FormReconst::btn_col1_Click);
			// 
			// btn_depth2
			// 
			this->btn_depth2->Location = System::Drawing::Point(29, 28);
			this->btn_depth2->Name = L"btn_depth2";
			this->btn_depth2->Size = System::Drawing::Size(95, 45);
			this->btn_depth2->TabIndex = 6;
			this->btn_depth2->Text = L"Depth";
			this->btn_depth2->UseVisualStyleBackColor = true;
			this->btn_depth2->Click += gcnew System::EventHandler(this, &FormReconst::btn_depth2_Click);
			// 
			// open_col1
			// 
			this->open_col1->FileName = L"openFileDialog1";
			this->open_col1->FileOk += gcnew System::ComponentModel::CancelEventHandler(this, &FormReconst::open_col1_FileOk);
			// 
			// open_depth2
			// 
			this->open_depth2->FileName = L"openFileDialog2";
			this->open_depth2->FileOk += gcnew System::ComponentModel::CancelEventHandler(this, &FormReconst::open_depth2_FileOk);
			// 
			// open_col2
			// 
			this->open_col2->FileName = L"openFileDialog3";
			this->open_col2->FileOk += gcnew System::ComponentModel::CancelEventHandler(this, &FormReconst::open_col2_FileOk);
			// 
			// worker
			// 
			this->worker->WorkerReportsProgress = true;
			this->worker->WorkerSupportsCancellation = true;
			this->worker->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &FormReconst::worker_DoWork);
			// 
			// prog_bar
			// 
			this->prog_bar->Location = System::Drawing::Point(25, 472);
			this->prog_bar->Name = L"prog_bar";
			this->prog_bar->Size = System::Drawing::Size(604, 23);
			this->prog_bar->Style = System::Windows::Forms::ProgressBarStyle::Continuous;
			this->prog_bar->TabIndex = 7;
			// 
			// groupBox1
			// 
			this->groupBox1->Controls->Add(this->btn_depth1);
			this->groupBox1->Controls->Add(this->btn_col1);
			this->groupBox1->Location = System::Drawing::Point(25, 12);
			this->groupBox1->Name = L"groupBox1";
			this->groupBox1->Size = System::Drawing::Size(292, 100);
			this->groupBox1->TabIndex = 8;
			this->groupBox1->TabStop = false;
			this->groupBox1->Text = L"Pointcloud 1";
			// 
			// groupBox2
			// 
			this->groupBox2->Controls->Add(this->btn_depth2);
			this->groupBox2->Controls->Add(this->btn_col2);
			this->groupBox2->Location = System::Drawing::Point(341, 12);
			this->groupBox2->Name = L"groupBox2";
			this->groupBox2->Size = System::Drawing::Size(288, 100);
			this->groupBox2->TabIndex = 9;
			this->groupBox2->TabStop = false;
			this->groupBox2->Text = L"Pointcloud 2";
			// 
			// FormReconst
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(641, 507);
			this->Controls->Add(this->groupBox2);
			this->Controls->Add(this->groupBox1);
			this->Controls->Add(this->prog_bar);
			this->Controls->Add(this->log_win);
			this->Controls->Add(this->lbl_stat);
			this->Controls->Add(this->btn_run);
			this->Icon = (cli::safe_cast<System::Drawing::Icon^  >(resources->GetObject(L"$this.Icon")));
			this->Name = L"FormReconst";
			this->StartPosition = System::Windows::Forms::FormStartPosition::CenterScreen;
			this->Text = L"Reconstructor";
			this->groupBox1->ResumeLayout(false);
			this->groupBox2->ResumeLayout(false);
			this->ResumeLayout(false);
			this->PerformLayout();

		}
#pragma endregion

	//---------------------------------------------------------------------------
	// Variables
	//---------------------------------------------------------------------------
	System::String^ filename_d1;
	System::String^ filename_im1;
	System::String^ filename_d2;
	System::String^ filename_im2;
	PlyWriter* pw;

	//---------------------------------------------------------------------------
	// Log to window
	//---------------------------------------------------------------------------
	void Log(System::String^ txt)
	{
        log_win->Items->Insert(0,"---------------------");
        log_win->Items->Insert(0,txt);

		ForceRefresh();
    }

	void Log(uint16_t txt)
	{
        Log(System::Convert::ToString(txt));
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

	//----------------------------------------------
	// Convert managed string to unmanaged string
	//----------------------------------------------
	private: std::string ManagedToString(System::String^ str)
	{
		System::String^ tmp = str;
		std::string temp(marshal_as<std::string>(tmp));
		return temp;
	}

//---------------------------------------------------------------------------
// Reconstruct Btn-Click
//---------------------------------------------------------------------------
private: System::Void btn_run_Click(System::Object^  sender, System::EventArgs^  e) 		
{
	ShowConsole();

	//
	//for(int itx=START_NUM; itx < END_NUM;++itx){

	//
	int start_im = START_NUM;
	int end_im = END_NUM;
		//int start_im = itx;
		//int end_im = itx+1;

	System::String^ im_dir = DATA_SOURCE;
	/*
	System::String^ default_save_loc = "C:\\users\\james\\desktop\\testD244.png";

	if(!filename_d1 || filename_d1 == default_save_loc){
		filename_d1 = default_save_loc;
		filename_im1="C:\\users\\james\\desktop\\test244.jpg";
		filename_d2 = "C:\\users\\james\\desktop\\testD245.png";
		filename_im2="C:\\users\\james\\desktop\\test245.jpg";
		Log("No filename set, using default");
	}
	else{
		filename_im1 = "C:\\users\\james\\desktop\\test\\test20.jpg";
		Log("Filename set");
	}
	*/
	Mat image;
	Mat image_col;
		
	Mat image2;
	Mat image_col2;
/*
	image = imread(ManagedToString(filename_d1), -1);   // Read the file
	image_col = imread(ManagedToString(filename_im1), -1);
	image2 = imread(ManagedToString(filename_d2), -1);
	image_col2 = imread(ManagedToString(filename_im2), -1);
	*/
/*	if(!image.data || !image_col.data) // Check for invalid input
	{
		lbl_stat->Text =  "Could not open or find the image(s)";
	}
	else
	{*/
		Log("rows: "+System::Convert::ToString(image_col.rows));
		Log("cols: "+System::Convert::ToString(image_col.cols));
		Log("element size: "+System::Convert::ToString(image_col.elemSize())+" bytes");
		Log("type: "+System::Convert::ToString(image_col.type()));

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

			lbl_stat->Text = "Running";

			//Log("Max depth: "+System::Convert::ToString(depth.GetDeviceMaxDepth()));
			Log("F: "+System::Convert::ToString(F));
			Log("Pix size: "+System::Convert::ToString(pixel_size));
			Log("FoV H: "+System::Convert::ToString(fov.fHFOV));
			Log("FoV V: "+System::Convert::ToString(fov.fVFOV));

			Log("Starting...");
				
			// current problem is that transforms must use homogonous coords to support translation - need to integrate this with the 6D matrix representation
			// EDIT: no need for homogeneous co-ords, but could use as future extension

			// THIS SECTION IS FOR RUNNING EVERYTHING IN THE WORKER THREAD
			//Object args[] = {image, image_col, image2, image_col2, depth};
			//im1 = new Mat();
			//im2 = new Mat();
			//im3 = new Mat();
			//im4 = new Mat();
			//d = new DepthGenerator();
			//*im1 = image;*im2 = image_col; *im3 = image2; *im4 = image_col2; *d = depth;
			//worker->RunWorkerAsync();
			// END SECTION

				

			// TESTING MATRIX INPUT
			//Mat* pointcloud1 = new Mat();
			//Mat* pointcloud2 = new Mat();
			/*
			PointCloud p2;		
			p2.AddPoint(gcnew Point3D(1000,0,0,0,0,0));
			p2.AddPoint(gcnew Point3D(0,0,0,0,0,0));
			p2.AddPoint(gcnew Point3D(0,1000,0,0,0,0));
			p2.AddPoint(gcnew Point3D(0,0,-1000,0,0,0));
			p2.AddPoint(gcnew Point3D(1000,1000,0,0,0,0));
			PointCloud p1;
			p1.AddPoint(gcnew Point3D(1000,1000,4000,255,0,0));
			p1.AddPoint(gcnew Point3D(2000,0,3000,255,0,0));
			p1.AddPoint(gcnew Point3D(1000,0,4000,255,0,0));
			p1.AddPoint(gcnew Point3D(1000,1000,3000,255,0,0));
			p1.AddPoint(gcnew Point3D(1000,0,3000,255,0,0));*/

			/*PointCloud p2;		
			p2.AddPoint(gcnew Point3D(1000,0,0,0,0,0));
			p2.AddPoint(gcnew Point3D(0,0,0,0,0,0));
			p2.AddPoint(gcnew Point3D(0,1000,0,0,0,0));
			p2.AddPoint(gcnew Point3D(1000,1000,0,0,0,0));
			PointCloud p1;
			p1.AddPoint(gcnew Point3D(3000,0,0,0,0,0));
			p1.AddPoint(gcnew Point3D(2000,0,0,0,0,0));
			p1.AddPoint(gcnew Point3D(2000,1000,0,0,0,0));
			p1.AddPoint(gcnew Point3D(3000,1000,0,0,0,0));*/
				
			//PointCloud p1;
		//	PointCloud p2;		
			/*
			p1.AddPoint(gcnew Point3D(500,0,500,255,0,0));
			p1.AddPoint(gcnew Point3D(500,0,-500,255,0,0));
			p1.AddPoint(gcnew Point3D(500,1000,500,255,0,0));
			p1.AddPoint(gcnew Point3D(500,1000,-500,255,0,0));
			*/
			/*p1.AddPoint(gcnew Point3D(853.5534,0,-353.5534,255,0,0));
			p1.AddPoint(gcnew Point3D(146.4466,0,353.5534,255,0,0));
			
			p1.AddPoint(gcnew Point3D(146.4466,1000,353.5534,255,0,0));	
			p1.AddPoint(gcnew Point3D(853.5534,1000,-353.5534,255,0,0));

			p2.AddPoint(gcnew Point3D(1000,0,0,0,0,0));
			p2.AddPoint(gcnew Point3D(0,0,0,0,0,0));
			p2.AddPoint(gcnew Point3D(0,1000,0,0,0,0));
			p2.AddPoint(gcnew Point3D(1000,1000,0,0,0,0));*/
				
			/*
			1207.107 0 0
			-207.1068 0 0
			-207.1068 1000 0
			1207.107 1000 0
			*/
			// negations messing something up - shouldn't be 1400 apart, only 1000
	
				
			//*pointcloud1 = p1.GetMatrix();*pointcloud2 = p2.GetMatrix();

			// END INPUT

			// DIRECT KABSCH TESTING
		/*	Kabsch k(MatColourToPoints(*pointcloud1), MatColourToPoints(*pointcloud2));
			PrintMat(k.GetRotation());
			PrintMat(k.GetTranslation());
			//PrintMat(* k.Final);	

			PrintMat(* k.a);
			PrintMat(* k.u1);
			PrintMat(* k.v1);
			PrintMat(* k.dmat1);
			PrintMat(* k.s);
			PrintMat(* k.tt);
			PrintMat(* k.cc);*/

			// END TESTING

			// NORMAL IMAGE INPUT
			// Load pointclouds
			//PointCloud^ pointcloud1 = gcnew PointCloud();
			//PointCloud^ pointcloud2 = gcnew PointCloud();

			MultipleICP icp;

			if(DELAYED_LOAD){
				
				icp.SetInputParameters(F, pixel_size);

			}
			else
			{

				vector<PointCloud*> clouds;// = vector<PointCloud*>;

				clouds.push_back(new PointCloud());

				filename_im1 = im_dir + "\\test" +start_im+ ".jpg";
				filename_d1 = im_dir + "\\testD" +start_im+ ".png";

				image = imread(ManagedToString(filename_d1), -1);   // Read the file
				image_col = imread(ManagedToString(filename_im1), -1);

				clouds[0]->SetMatrix(ImageToMat(image, image_col));
				// Transform to real co-ordinates
				clouds[0]->SetMatrix(ConvertProjToRealCoord(depth, clouds[0]->GetMatrix()));

				//MultipleICP icp;
				//icp.Event += gcnew f2(this, &FormReconst::ICP_update);
				//icp.AddPointCloud(pointcloud1);
				
				Log(end_im - start_im + 1 + " files ready to load.");
				Log("Loading image data");

				int i=1;
				for(int im = start_im + 1; im <= end_im; im++, i++){

					this->prog_bar->Value = 0;

					filename_im1 = im_dir + "\\test" +im+ ".jpg";
					filename_d1 = im_dir + "\\testD" +im+ ".png";

					image = imread(ManagedToString(filename_d1), -1);   // Read the file
					image_col = imread(ManagedToString(filename_im1), -1);

					clouds.push_back(new PointCloud());
					clouds[i]->SetMatrix(ImageToMat(image, image_col));
					// Transform to real co-ordinates
					clouds[i]->SetMatrix(ConvertProjToRealCoord(depth, clouds[i]->GetMatrix()));
					// END INPUT
				
					this->prog_bar->Step = 10;
					
					/*
					// calculate initial error
					
					//Mat* P = new Mat(); *P = *pointcloud1; Mat* Q = new Mat(); *Q = *pointcloud2;
					//EqualisePointClouds(P,Q);
					
					Mat diff = *P - *Q;

					double error = 0;
					for(int i=0; i<diff.cols; i++){
						for(int j=0; j<diff.rows; j++){
							error += (diff.at<float>(j,i) * diff.at<float>(j,i));
						}
					}
					error = error / diff.cols;
					error = sqrt(error);
					*/

					/*
					// Write out initial pointclouds
					pw = gcnew PlyWriter("c:\\users\\james\\desktop\\mesh.ply");
					pw->AddPointCloud(*pointcloud1);
					pw->AddPointCloud(*pointcloud2);
				
					pw->WriteFinal();*/

					// DO ICP HERE
					//icp.AddPointCloud(%pointcloud2);

					this->prog_bar->Value = 100;

					//Mat result = ChangeOperatorSize(icp.getTransform());

					//*pointcloud1 = ApplyTransform(result, *pointcloud1);
					//*pointcloud1 = ApplyTranslation(icp.getTranslate(), *pointcloud1);
					// This would only work if translations could be combined with transforms (ie. homog. co-ords)

					/*
					// Get the resulting, transformed "P" pointcloud
					*pointcloud1 = icp.getFinal(); // need to preserve colours somehow

					// Write out final pointclouds (Q unchanged)
					pw = gcnew PlyWriter("c:\\users\\james\\desktop\\meshICP.ply");
					
					// TODO: change pointclouds to actual pointcloud objects
					//pointcloud1->AddPointCloud(*pointcloud2);
					pointcloud1->resize(pointcloud1->rows + pointcloud2->rows);
					// Should really do this by copying whole submatrix:
					for (int i=0; i<pointcloud2->rows; i++){
						pointcloud2->row(i).copyTo(pointcloud1->row(pointcloud1->rows - pointcloud2->rows + i));
					}
					///////////

					pw->AddPointCloud(*pointcloud1);
					//pw->AddPointCloud(*pointcloud2);

					pw->WriteFinal();
					*/


					// Testing output
					/*PrintMat(*(icp.pbefore));
					Log("pbefore:");
					PrintMat(*(icp.aftermatch));
					Log("paftermatch:");*/
					/*
					PrintMat(icp.getTransform());
					Log("Last rotation:");
					PrintMat(icp.getTranslate());
					Log("Last translation:");
					//Log("Initial diff: " + Convert::ToString(error));
					Log("Error: " + Convert::ToString(icp.getError()));

					Log("Iterations: "+Convert::ToString(icp.iters));

					Log("");
					Log("Image " + im + " finished processing.");
					*/
					
				}

				icp.AddClouds(clouds);

				Log("Image data loaded");

			}

			Log("Running ICP...");
			icp.run();
			Log("Finished ICP");

			//PointCloud^ pc = gcnew PointCloud();
			//pc = icp.GetResult();

			//TODO: pc is too big, need to write straight to ply instead (ie. make PlyWriter accept pointers)

			// Write out final pointclouds
			//pw = gcnew PlyWriter("c:\\users\\james\\desktop\\meshICP.ply");	
			//pw->AddPointCloud(pc->GetMatrix());
			//pw->WriteFinal();

			/*
			Mat* m = new Mat(8,3,MAT_TYPE);
			m->at<float>(0,0) = 1; m->at<float>(0,1) = 0; m->at<float>(0,2) = 0;
			m->at<float>(1,0) = 3; m->at<float>(1,1) = 0; m->at<float>(1,2) = 1;
			m->at<float>(2,0) = 2; m->at<float>(2,1) = 1; m->at<float>(2,2) = 0;
			m->at<float>(3,0) = 0; m->at<float>(3,1) = 1; m->at<float>(3,2) = 1;
			m->at<float>(4,0) = 7; m->at<float>(4,1) = 10; m->at<float>(4,2) = 0;
			m->at<float>(5,0) = 63; m->at<float>(5,1) = 10; m->at<float>(5,2) = 1;
			m->at<float>(6,0) = 24; m->at<float>(6,1) = 1; m->at<float>(6,2) = 10;
			m->at<float>(7,0) = 10; m->at<float>(7,1) = 1; m->at<float>(7,2) = 1;
			*/

			Log("**************");
			Log("*  COMPLETE  *");
			Log("**************");

			//PrintMat(icp.GetFinalTransform());

			//PrintMat(icp.getFinal());
			//Log("Resulting P:");

		//	PrintMat(* icp.p0);
		//	Log("p0:");

		//	PrintMat(* icp.q0);
		//	Log("q0:");
		//}
		//
		//}
		//
	}

	// TEST POINTCLOUDS
	/*
	PointCloud^ pc  = gcnew PointCloud();

	pc->AddPoint(gcnew Point3D(1,2,3));
	pc->AddPoint(gcnew Point3D(4,5,6));

	Log(Convert::ToString(pc->rows));

	Mat ma = pc->GetMatrix();
	PrintMat(ma);
	*/

	// TEST KABSCH
	/*
	Mat m = Mat(3,3,MAT_TYPE);
	m.at<float>(0,0)=1;m.at<float>(0,1)=7;m.at<float>(0,2)=56;
	m.at<float>(1,0)=7;m.at<float>(1,1)=2;m.at<float>(1,2)=34;
	m.at<float>(2,0)=3;m.at<float>(2,1)=4;m.at<float>(2,2)=6;

	Mat m2 = Mat(3,3,MAT_TYPE);
	m2.at<float>(0,0)=8;m2.at<float>(0,1)=34;m2.at<float>(0,2)=7;
	m2.at<float>(1,0)=2;m2.at<float>(1,1)=6;m2.at<float>(1,2)=2;
	m2.at<float>(2,0)=7;m2.at<float>(2,1)=4;m2.at<float>(2,2)=0;

	Log("");
	Kabsch k(m,m2);
	PrintMat(k.GetRotation());
	Log("Rot:");
	Log(Convert::ToString(k.GetError()));
	Log("Err:");
	PrintMat(k.GetTranslation());
	Log("Trans:");
	*/

}


private: System::Void ICP_update(System::String^ s) 
{
	//prog_bar->PerformStep();
	Log(s);

	ForceRefresh();
}

void ForceRefresh()
{
	HWND hWnd;
	hWnd = static_cast<HWND>(Handle.ToPointer());
	InvalidateRect (hWnd, NULL, TRUE);
	UpdateWindow (hWnd);
}

	// Variables for worker thread
	Mat* im1;
	Mat* im2;
	Mat* im3;
	Mat* im4;
	DepthGenerator* d;

private: System::Void worker_DoWork(System::Object^  sender, System::ComponentModel::DoWorkEventArgs^  e) 
{
	/*
	Invoke(gcnew Action<System::String^>(this, &FormReconst::Log), "starting...");
	Mat image = *im1;
	Mat image_col = *im2;
	Mat image2 = *im3;
	Mat image_col2 = *im4;
	DepthGenerator depth = *d;
	Invoke(gcnew Action<System::String^>(this, &FormReconst::Log), "ready");

	// Load pointclouds
	Mat pointcloud1 = ImageToMat(image, image_col);
	Mat pointcloud2 = ImageToMat(image2, image_col2);
	Invoke(gcnew Action<System::String^>(this, &FormReconst::Log), "update 1...");

	// Transform to real co-ordinates
	pointcloud1 = ConvertProjToRealCoord(depth, pointcloud1);
	pointcloud2 = ConvertProjToRealCoord(depth, pointcloud2);
	Invoke(gcnew Action<System::String^>(this, &FormReconst::Log), "update 2...");

	// Initial pointclouds
	pw = gcnew PlyWriter("c:\\users\\james\\desktop\\mesh.ply");
	pw->AddPointCloud(pointcloud1);
	pw->AddPointCloud(pointcloud2);
	pw->WriteFinal();
	Invoke(gcnew Action<System::String^>(this, &FormReconst::Log), "update 3...");

	// Apply ICP
	IterativeClosestPoint icp(MatColourToPoints(pointcloud1), MatColourToPoints(pointcloud2));
	Mat result = ChangeOperatorSize(icp.getTransform());
	pointcloud1 = ApplyTransform(result, pointcloud1);
	pw = gcnew PlyWriter("c:\\users\\james\\desktop\\meshICP.ply");
	pw->AddPointCloud(pointcloud1);
	pw->AddPointCloud(pointcloud2);
	pw->WriteFinal();
	*/
}



//---------------------------------------------------------------------------
// Display a matrix via log window
//---------------------------------------------------------------------------
void PrintMat(Mat m)
{
	System::String^ s;
	for(int i = m.rows-1; i>=0; i--){ // each matrix row
		s = "";
		for(int j=0; j<m.cols; j++){  // each matrix col
			s = s + " | " + System::Convert::ToString(m.at<float>(i,j));
		}
		log_win->Items->Insert(0,s);
		log_win->Items->Insert(0,"--------------------------");
	}
	log_win->Items->Insert(0,"");
}

//---------------------------------------------------------------------------
// Load image dialog button
//---------------------------------------------------------------------------
private: System::Void open_depth1_FileOk(System::Object^  sender, System::ComponentModel::CancelEventArgs^  e) 
{
	filename_d1 = open_depth1->FileName;
	lbl_stat->Text = filename_d1;
}
private: System::Void open_col1_FileOk(System::Object^  sender, System::ComponentModel::CancelEventArgs^  e) 
{
	filename_im1 = open_depth1->FileName;
	lbl_stat->Text = filename_im1;
}
private: System::Void open_depth2_FileOk(System::Object^  sender, System::ComponentModel::CancelEventArgs^  e) 
{
	filename_d2 = open_depth1->FileName;
	lbl_stat->Text = filename_d2;
}
private: System::Void open_col2_FileOk(System::Object^  sender, System::ComponentModel::CancelEventArgs^  e) 
{
	filename_im2 = open_depth1->FileName;
	lbl_stat->Text = filename_im2;
}
private: System::Void btn_depth1_Click(System::Object^  sender, System::EventArgs^  e) 
{
	open_depth1->ShowDialog();
}
private: System::Void btn_col1_Click(System::Object^  sender, System::EventArgs^  e) 
{
	open_col1->ShowDialog();
}
private: System::Void btn_depth2_Click(System::Object^  sender, System::EventArgs^  e) 
{
	open_depth2->ShowDialog();
}
private: System::Void btn_col2_Click(System::Object^  sender, System::EventArgs^  e) 
{
	open_col2->ShowDialog();
}

//------------------------------------
// Match the size of 2 pointclouds
//------------------------------------
private: void EqualisePointClouds(Mat* P, Mat* Q)
{
	int size = P->rows;
	if(Q->rows < size){
		size = Q->rows;
		P->resize(size);
	}
	else
	{
		Q->resize(size);
	}
}

//---------------------------------------------------------------------------
// Transform single point Projective->Real
//---------------------------------------------------------------------------
private: Point3D singleConvertProjToRealCoord(Point3D p, float xtoz, float ytoz)
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
// Transform collection of points Projective->Real
//---------------------------------------------------------------------------
private: System::Void ConvertProjToRealCoord(DepthGenerator depth, int n, array<Point3D*>^ p, array<Point3D*>^ p2)
{
	XnUInt64 F;
	XnDouble pixel_size;

	/* 
		If a Kinect is available, get intrinsic parameters from device.
		Otherwise, use previously measured ones (result should be identical)
	*/
	if(!depth)
	{
		F = 120;                         // Focal length, in mm
		pixel_size = 0.10419999808073;   // Size/width in mm of each recorded pixel
	}
	else
	{
		// get the focal length in mm (ZPS = zero plane distance)
		depth.GetIntProperty ("ZPD", F);

		// get the pixel size in mm ("ZPPS" = pixel size at zero plane)
		depth.GetRealProperty ("ZPPS", pixel_size);
	}

	// These are the core transformation parameters
	float xtoz = 2 * 640.0 * pixel_size / F; // = 2 tan(f.o.v-h / 2)
	float ytoz = 2 * 480.0 * pixel_size / F; // = 2 tan(f.o.v-v / 2)

	// Convert each point in input array individually
	for(int i = 0; i < n; i++){
		*p2[i] = singleConvertProjToRealCoord(*p[i], xtoz, ytoz);
		//p2[i] = p[i];  // no transformation: for testing
	}
}

//---------------------------------------------------------------------------
// Transform matrix pointcloud Projective->Real
//---------------------------------------------------------------------------
private: Mat ConvertProjToRealCoord(DepthGenerator depth, Mat points)
{
	XnUInt64 F;
	XnDouble pixel_size;

	/* 
		If a Kinect is available, get intrinsic parameters from device.
		Otherwise, use previously measured ones (result should be identical)
	*/
	if(!depth)
	{
		F = 120;                         // Focal length, in mm
		pixel_size = 0.10419999808073;   // Size/width in mm of each recorded pixel
	}
	else
	{
		// get the focal length in mm (ZPS = zero plane distance)
		depth.GetIntProperty ("ZPD", F);

		// get the pixel size in mm ("ZPPS" = pixel size at zero plane)
		depth.GetRealProperty ("ZPPS", pixel_size);
	}

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
Mat ImageToMat(Mat points, Mat colour)
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
			//if(out.at<float>(count,2) > 3000)
				//out.at<float>(count,2) = nan();

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

array<Point3D*>^ MatToArray(Mat m)
{
	array<Point3D*>^ ps = gcnew array<Point3D*>(640*480);
	return ps;
}

Mat ArrayToMat(array<Point3D*>^ ps)
{
	Mat out;
	return out;
}

//----------------------------------------------
// Apply matrix transform to pointcloud matrix
//----------------------------------------------
Mat ApplyTransform(Mat transform, Mat pc)
{
	return pc*(transform);   // result = points * operator  because of the shape of pointcloud matrix representation
								 // transform matrix is transposed because we are left-multiplying
}

PointCloud ApplyTransform(Mat transform, PointCloud pc)
{
	pc.SetMatrix(pc.GetMatrix() * transform);
	return pc;
}

Mat ApplyTranslation(Mat translate, Mat pc)
{
	Mat out(1,6,MAT_TYPE);
	for(int i=0; i<6;i++){
		if(i<3){
			out.at<float>(0,i) = translate.at<float>(0,i);
		}else{
			out.at<float>(0,i) = 0;
		}
	}

	out.resize(pc.rows);
	for(int i=0; i<out.rows;i++){
		for(int j=0; j<out.cols;j++){
			if(j<3){
				out.at<float>(i,j) = translate.at<float>(0,j);
			}else{
				out.at<float>(i,j) = 0;
			}
		}
	}

	return out + pc;
}

//---------------------------------------------------------------------
// Switch from 3D transformation matrix to 6D (points + colours)
//---------------------------------------------------------------------
Mat ChangeOperatorSize(Mat transform)
{
	//
	//  ie:   A   -->    A | 0     where each is a 3x3 submatrix of the result
	//                  -------
	//                   0 | I
	//
	Mat out = Mat::eye(6, 6, MAT_TYPE);
	cv::Mat tmp = out(cv::Rect(0,0,3,3));
	transform.copyTo(tmp);
	return out;
}

//-----------------------------------
// Strip colour information from 
// a pointcloud matrix
//-----------------------------------
Mat MatColourToPoints(Mat pc)
{
	// assert: pc.cols == 6

	Mat out(pc.rows,3,MAT_TYPE);
	for(int i=0; i<pc.rows;i++){
		for(int j=0; j<3; j++){
			out.at<float>(i,j) = pc.at<float>(i,j);
		}
	}

	return out;
}

void ShowConsole()
{
	if(DEBUG_OUTPUT && AllocConsole()) {
		freopen("CONOUT$", "wt", stdout);
		SetConsoleTitle(L"Debug Console");
		SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_GREEN | FOREGROUND_BLUE | FOREGROUND_RED);
	}
}

}; // END OF CLASS

} // END OF NAMESPACE

