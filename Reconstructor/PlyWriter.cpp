#include "StdAfx.h"
#include "PlyWriter.h"
#include "global_parameters.h"

PlyWriter::PlyWriter(std::string loc)
{
	SetLoc(loc);
	vertices = 0;

	// loc must be ***.ply
	//pwriter = gcnew StreamWriter(filename + ".temp");
	pwriter.open(filename + ".temp", std::ios::out);

	// try increasing write buffer size
	//const std::size_t buf_size = 64000;
	//char my_buffer[buf_size];
	//pwriter.rdbuf()->pubsetbuf(my_buffer, buf_size);
	
	// line will be replaced by real header upon closure (since vertex count currently unknown)
	//pwriter->WriteLine("**ply**");
	//pwriter << "**ply**" << std::endl;
}

PlyWriter::PlyWriter(std::string loc, Mat pc)
{
	PlyWriter temp(loc);
	temp.AddPointCloud(pc);
	temp.WriteFinal();
}

void PlyWriter::SetLoc(std::string loc)
{
	filename = loc;
}

//TODO: speed up with custom buffer
void PlyWriter::WriteFinal()
{
	Log("Writing PLY file, this may take a minute...");

	//FlushBuffer();

	pwriter.close();

	const unsigned int length = 8192;
	char buffer[length];

	//StreamReader^ preader = gcnew StreamReader(filename + ".temp");
	std::ifstream preader;
	preader.open(filename + ".temp");

	//pwriter_out = gcnew StreamWriter(filename);
	pwriter_out.open(filename, std::ios::out);

	std::string LineRead;

	//bool header_written = false;

	pwriter_out << "ply\n";
	pwriter_out << "format ascii 1.0\n";
	pwriter_out << "comment this is a ply file holding a mesh created by KinectGrabber (James King 2012)\n";
	pwriter_out << "element vertex " + boost::lexical_cast<std::string>(vertices) + "\n";
	pwriter_out << "property float x\n";
	pwriter_out << "property float y\n";
	pwriter_out << "property float z\n";
	pwriter_out << "property uchar red\n";            
	pwriter_out << "property uchar green\n";
	pwriter_out << "property uchar blue\n";
	pwriter_out << "end_header\n";

	while(! preader.eof()){

		//LineRead = preader->ReadLine();
		//getline(preader, LineRead);
		preader.read(buffer, length);

		//std::string line = LineRead + "\n";
		//const char* str = line.c_str();

		pwriter_out.write(buffer, preader.gcount());

		//pwriter_out << LineRead + "\n"; //<< std::endl;

	}

    pwriter_out.close();
	preader.close();

	//marshal_context ^ context = gcnew marshal_context();
	//const char* temp = context->marshal_as<const char*>(filename + ".temp");
	std::string removal = filename + ".temp";
	remove(removal.c_str());
	//delete context;

	Log("Finished.");
}

void PlyWriter::FlushBuffer()
{
	if(write_buffer.size() > 0)
		while(! write_buffer.empty() ){
			std::string temp = write_buffer.front();
			write_buffer.pop();
			pwriter.write(temp.c_str(), temp.length() );
		}
}

void PlyWriter::AddVertex(Point3D p, uint8_t c[])  // c == [r,g,b]
{
	if(!isnan(p.Z)){ // don't write NaNs
		// y and z co-ordinates flipped for convenient viewing
		//pwriter->WriteLine((float)(p.X) + " " + (float)(-p.Y) + " " + (float)(-p.Z) + " " + c[0] + " " + c[1] + " " + c[2]);
		pwriter << boost::lexical_cast<std::string>((float)(p.X)) + " " + boost::lexical_cast<std::string>((float)(-p.Y)) + " " + boost::lexical_cast<std::string>((float)(-p.Z)) + " " + boost::lexical_cast<std::string>(c[0]) + " " + boost::lexical_cast<std::string>(c[1]) + " " + boost::lexical_cast<std::string>(c[2]) + "\n";//<< std::endl;
		vertices++;
	}
}
void PlyWriter::AddVertex(float x, float y, float z, int r, int g, int b)
{
	if(!isnan(z)){ // don't write NaNs
		//pwriter->WriteLine((float)x + " " + (float)(-y) + " " + (float)(-z) + " " + r + " " + g + " " + b);
		
		//pwriter << boost::lexical_cast<std::string>((float)x) + " " + boost::lexical_cast<std::string>((float)(-y)) + " " + boost::lexical_cast<std::string>((float)(-z)) + " " + boost::lexical_cast<std::string>(r) + " " + boost::lexical_cast<std::string>(g) + " " + boost::lexical_cast<std::string>(b) + "\n";//<< std::endl;
		std::stringstream ss;
		ss << (x) << " " << (-y) << " " << (-z) << " " << (r) << " " << (g) << " " << (b) << "\n";
		pwriter.write(ss.str().c_str(), ss.str().length());
		//write_buffer.push(ss.str());

		//if(write_buffer.size() > WRITE_BUFFER_SIZE)
		//	FlushBuffer();

		vertices++;
	}
}

void PlyWriter::AddPointCloud(Mat pc)
{
	if(pc.cols == 6){
		for(int i=0; i<pc.rows; i++){
			AddVertex(pc.at<float>(i,0),pc.at<float>(i,1),pc.at<float>(i,2),(int)pc.at<float>(i,3),(int)pc.at<float>(i,4),(int)pc.at<float>(i,5));
		}
	}
	else
	{
		for(int i=0; i<pc.rows; i++){
			AddVertex(pc.at<float>(i,0),pc.at<float>(i,1),pc.at<float>(i,2),0,0,0);
		}
	}
}

void PlyWriter::Close(void)
{
	pwriter.close();
	pwriter.close();
}

void PlyWriter::Log(std::string s)
{
	std::cout << s << std::endl << std::endl;
}