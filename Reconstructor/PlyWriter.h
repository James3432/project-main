#include <iostream>
#include <fstream>
#include <stdint.h>
#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Point3D.h"
#include <boost\lexical_cast.hpp>
#include <queue>


//#define isnan System::Double::IsNaN
#define isnan(x) (x!=x)

//using namespace System;
//using namespace System::IO;
//using namespace msclr::interop;

using namespace cv;

#pragma once
/*ref */class PlyWriter
{
public:
	PlyWriter(std::string loc);
	PlyWriter(std::string loc, Mat pc);
	void SetLoc(std::string loc);
	void WriteFinal();
	void AddVertex(Point3D p, uint8_t c[]);
	void AddVertex(float x, float y, float z, int r, int g, int b);
	void AddPointCloud(Mat pc);
	void Close(void);
	void FlushBuffer();
private:
	void Log(std::string s);
	std::string filename;
	std::ofstream pwriter;
	std::ofstream pwriter_out;
	int vertices;

	std::queue<std::string> write_buffer;

	template<typename T>
	std::string numtostr(T val)
	{
		std::stringstream s;
		s << val;
		return s.str();
	}

};

