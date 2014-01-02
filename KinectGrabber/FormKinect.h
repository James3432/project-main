#using <System.dll>

#include "NIGrabber.h"
#include <sstream>

using namespace System;
using namespace System::IO;
using namespace System::Net;
using namespace System::Net::Sockets;
using namespace System::Text;
using namespace System::Threading;

#pragma once

namespace KinectGrabber {

	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;

	/// <summary>
	/// Form for Kinect Grabber interface
	/// </summary>
	public ref class FormKinect : public System::Windows::Forms::Form
	{
	public:
		FormKinect(void)
		{
			InitializeComponent();
			//
			//TODO: Add the constructor code here
			//
			save_loc = "C:\\users\\james\\desktop\\Kinect Data";
		}

	protected:
		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		~FormKinect()
		{
			if (components)
			{
				delete components;
			}
			try{
				client->Close();
			}catch(Exception^ e){}
		}
	public: System::Windows::Forms::Button^  btn_start;
	public: System::Windows::Forms::ListBox^  log_win;
	protected: 

	protected: 


	private: System::ComponentModel::BackgroundWorker^  server_worker;
	private: System::Windows::Forms::Button^  btn_stop;
	private: System::Windows::Forms::Button^  btn_save;
	public: System::Windows::Forms::CheckBox^  chk_rgb;
	private: 




	private: System::Windows::Forms::FolderBrowserDialog^  folder_sel;


	public: 

	public: 
	private: 
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
			System::ComponentModel::ComponentResourceManager^  resources = (gcnew System::ComponentModel::ComponentResourceManager(FormKinect::typeid));
			this->btn_start = (gcnew System::Windows::Forms::Button());
			this->log_win = (gcnew System::Windows::Forms::ListBox());
			this->server_worker = (gcnew System::ComponentModel::BackgroundWorker());
			this->btn_stop = (gcnew System::Windows::Forms::Button());
			this->btn_save = (gcnew System::Windows::Forms::Button());
			this->chk_rgb = (gcnew System::Windows::Forms::CheckBox());
			this->folder_sel = (gcnew System::Windows::Forms::FolderBrowserDialog());
			this->SuspendLayout();
			// 
			// btn_start
			// 
			this->btn_start->Location = System::Drawing::Point(12, 12);
			this->btn_start->Name = L"btn_start";
			this->btn_start->Size = System::Drawing::Size(191, 47);
			this->btn_start->TabIndex = 0;
			this->btn_start->Text = L"Start server";
			this->btn_start->UseVisualStyleBackColor = true;
			this->btn_start->Click += gcnew System::EventHandler(this, &FormKinect::btn_start_Click);
			// 
			// log_win
			// 
			this->log_win->FormattingEnabled = true;
			this->log_win->HorizontalScrollbar = true;
			this->log_win->Location = System::Drawing::Point(12, 129);
			this->log_win->Name = L"log_win";
			this->log_win->ScrollAlwaysVisible = true;
			this->log_win->SelectionMode = System::Windows::Forms::SelectionMode::None;
			this->log_win->Size = System::Drawing::Size(191, 121);
			this->log_win->TabIndex = 1;
			// 
			// server_worker
			// 
			this->server_worker->WorkerReportsProgress = true;
			this->server_worker->WorkerSupportsCancellation = true;
			this->server_worker->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &FormKinect::server_worker_DoWork);
			this->server_worker->ProgressChanged += gcnew System::ComponentModel::ProgressChangedEventHandler(this, &FormKinect::server_worker_ProgressChanged);
			this->server_worker->RunWorkerCompleted += gcnew System::ComponentModel::RunWorkerCompletedEventHandler(this, &FormKinect::server_worker_RunWorkerCompleted);
			// 
			// btn_stop
			// 
			this->btn_stop->Enabled = false;
			this->btn_stop->Location = System::Drawing::Point(12, 65);
			this->btn_stop->Name = L"btn_stop";
			this->btn_stop->Size = System::Drawing::Size(191, 47);
			this->btn_stop->TabIndex = 2;
			this->btn_stop->Text = L"Stop server";
			this->btn_stop->UseVisualStyleBackColor = true;
			this->btn_stop->Click += gcnew System::EventHandler(this, &FormKinect::btn_stop_Click);
			// 
			// btn_save
			// 
			this->btn_save->Location = System::Drawing::Point(13, 257);
			this->btn_save->Name = L"btn_save";
			this->btn_save->Size = System::Drawing::Size(93, 25);
			this->btn_save->TabIndex = 3;
			this->btn_save->Text = L"Save location...";
			this->btn_save->UseVisualStyleBackColor = true;
			this->btn_save->Click += gcnew System::EventHandler(this, &FormKinect::btn_save_Click);
			// 
			// chk_rgb
			// 
			this->chk_rgb->AutoSize = true;
			this->chk_rgb->Checked = true;
			this->chk_rgb->CheckState = System::Windows::Forms::CheckState::Checked;
			this->chk_rgb->Location = System::Drawing::Point(114, 262);
			this->chk_rgb->Name = L"chk_rgb";
			this->chk_rgb->Size = System::Drawing::Size(89, 17);
			this->chk_rgb->TabIndex = 4;
			this->chk_rgb->Text = L"Capture RGB";
			this->chk_rgb->UseVisualStyleBackColor = true;
			// 
			// folder_sel
			// 
			this->folder_sel->SelectedPath = L"C:\\Users\\James\\Desktop\\Kinect Data";
			// 
			// FormKinect
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(215, 294);
			this->Controls->Add(this->chk_rgb);
			this->Controls->Add(this->btn_save);
			this->Controls->Add(this->btn_stop);
			this->Controls->Add(this->log_win);
			this->Controls->Add(this->btn_start);
			this->FormBorderStyle = System::Windows::Forms::FormBorderStyle::FixedSingle;
			this->Icon = (cli::safe_cast<System::Drawing::Icon^  >(resources->GetObject(L"$this.Icon")));
			this->MaximizeBox = false;
			this->Name = L"FormKinect";
			this->Text = L"Kinect Grabber";
			this->Load += gcnew System::EventHandler(this, &FormKinect::FormKinect_Load);
			this->ResumeLayout(false);
			this->PerformLayout();

		}
#pragma endregion

//************************************************************
// Variables
//************************************************************

public: TcpClient^ client;

// Current file save location for Kinect grabber
public: String^ save_loc;

public: NIGrabber^ ni;

private: StreamWriter^ pwriter;

private: bool isGrabbing;


//************************************************************
// Methods
//************************************************************


//---------------------------------------------------------------------------
// Start server OnClick
//---------------------------------------------------------------------------
private: 
	System::Void btn_start_Click(System::Object^  sender, System::EventArgs^  e) {
		btn_start->Enabled = false;
		btn_stop->Enabled = true;
		server_worker->RunWorkerAsync();
	}


//---------------------------------------------------------------------------
// Init help info
//---------------------------------------------------------------------------
private: 
	void LogHelp(){
        log_win->Items->Insert(0,"KinectGrabber v1.0");
    }


//---------------------------------------------------------------------------
// Logger
//---------------------------------------------------------------------------
public: 
	void Log(String^ txt){
        log_win->Items->Insert(0,"---------------------");
        log_win->Items->Insert(0,txt);
    }




//---------------------------------------------------------------------------
// Server worker
//---------------------------------------------------------------------------
private: 
	System::Void server_worker_DoWork(System::Object^  sender, System::ComponentModel::DoWorkEventArgs^  e) {
		try{
			// Log("Starting server...");
			server_worker->ReportProgress(1);
	   
			// Set the TcpListener on port 13000.
			Int32 port = 13000;
			IPAddress^ localAddr = IPAddress::Parse( "127.0.0.1" );

			TcpListener^ server = gcnew TcpListener( localAddr,port );

			// Start listening for client requests.
			server->Start();

			// Buffer for reading data 
			array<Byte>^bytes = gcnew array<Byte>(256);
			String^ data = nullptr;

			server_worker->ReportProgress(2);

			// Enter the listening loop. 
			while ( true ){

				// Perform a blocking call to accept requests. 
				// You could also user server.AcceptSocket() here.
				client = server->AcceptTcpClient();
				//Console::WriteLine( "Connected!" );
				server_worker->ReportProgress(3);

				data = nullptr;

				// Get a stream Object* for reading and writing
				NetworkStream^ stream = client->GetStream();
				Int32 i;

				// Loop to receive all the data sent by the client. 
				while ( i = stream->Read( bytes, 0, bytes->Length ) ){  // while connection is open...

					// Translate data bytes to a ASCII String*.
					data = System::Text::Encoding::ASCII->GetString( bytes, 0, i );
					//Console::WriteLine( "Received: {0}", data );
					server_worker->ReportProgress(4,data);

					// Process the data sent by the client.
					data = data->ToUpper();
					array<Byte>^msg = System::Text::Encoding::ASCII->GetBytes( data );

					// Send back a response.
					stream->Write( msg, 0, msg->Length );
					Console::WriteLine( "Sent: {0}", data );
					if(server_worker->CancellationPending)
						break;

				}

				// Shutdown and end connection
				client->Close();

				server_worker->ReportProgress(5);

				if(server_worker->CancellationPending)
					break;

			}
		}
		catch ( SocketException^ e ) 
		{
			//Console::WriteLine( "SocketException: {0}", e );
			server_worker->ReportProgress(99);
		}
	}


//---------------------------------------------------------------------------
// TCP server report
//---------------------------------------------------------------------------
private: System::Void server_worker_ProgressChanged(System::Object^  sender, System::ComponentModel::ProgressChangedEventArgs^  e) {
				String^ token = (String^)(e->UserState);
				switch(e->ProgressPercentage){
					 case 1:Log("Starting server...");
						break;
					 case 2:Log("Server connected");
						break;
					 case 3:Log("Client found");
						break;
					 case 4:Log("Data received: "+e->UserState);
						 if(token == "startGrab"){
							 Log("Starting grab");
					 
							 ni = gcnew NIGrabber();
							 // need to start in new worker thread
					 
							 ni->run(this);

							 isGrabbing = true;

						 }
						 if(token == "stopGrab"){
							 Log("Stopping grab");
							 // need to stop worker thread
							 ni->stop();

							 isGrabbing = false;

						 }
						 if(isGrabbing){
							 if(token == "forward"){
								 WriteToLog("fwd");
							 }
							 if(token == "back"){
								 WriteToLog("back");
							 }
							 if(token == "left"){
								 WriteToLog("left");
							 }
							 if(token == "right"){
								 WriteToLog("right");
							 }
						 }
						break;
					 case 5:Log("Client disconnected");
						break;

					 case 99:Log("Connection error");
						break;
				}
		 }


//---------------------------------------------------------------------------
// Server finished running
//---------------------------------------------------------------------------
private: System::Void server_worker_RunWorkerCompleted(System::Object^  sender, System::ComponentModel::RunWorkerCompletedEventArgs^  e) {
			 client->Close();
			 btn_start->Enabled = true;
			 btn_stop->Enabled = false;
			 //server_worker->CancellationPending = false;
			 Log("Server stopped");
		 }


//---------------------------------------------------------------------------
// Stop TCP server button
//---------------------------------------------------------------------------
private: System::Void btn_stop_Click(System::Object^  sender, System::EventArgs^  e) {
			 server_worker->CancelAsync();
		 }


//---------------------------------------------------------------------------
// Folder dialog
//---------------------------------------------------------------------------
private: System::Void btn_save_Click(System::Object^  sender, System::EventArgs^  e) {
			// Show the FolderBrowserDialog.
			System::Windows::Forms::DialogResult result = folder_sel->ShowDialog();
			if ( result == System::Windows::Forms::DialogResult::OK )
			{
				save_loc = folder_sel->SelectedPath;
				Log("Save location: "+save_loc);
			}
		 }


//---------------------------------------------------------------------------
// Form OnLoad
//---------------------------------------------------------------------------
private: System::Void FormKinect_Load(System::Object^  sender, System::EventArgs^  e) {			 
			 LogHelp();
			 btn_start->PerformClick();
			 isGrabbing = false;
			 Log(getTime());
		 }


//----------------------------------
// Get time
//--------------------------------
private: String^ getTime(){
			SYSTEMTIME st;
			GetLocalTime(&st);
			return String::Format("{0}:{1}:{2}:{3}:{4}:{5}:{6}",st.wYear,st.wMonth,st.wDay,st.wHour,st.wMinute,st.wSecond,st.wMilliseconds);
		 }


//-------------------------
// Log file writer
//-------------------------
public: void InitLog(String^ loc){
			pwriter = gcnew StreamWriter(loc+"\\log.txt");
			Log("write to "+loc);
}

public: void WriteToLog(String^ str){
			pwriter->WriteLine(getTime() + " " + str);
			Log("write "+getTime()+str);
}

public: void CloseLog(){
			pwriter->Close();
			Log("Log file written");
}



	};
}



