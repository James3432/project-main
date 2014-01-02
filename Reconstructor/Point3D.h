//typedef unsigned char uchar;

#pragma once
/*ref */class Point3D
{
public:
	Point3D(void);
	Point3D(float,float,float);
	Point3D(const Point3D &source);
	Point3D& operator= (const Point3D &source);
	Point3D(float,float,float,int,int,int);
	float X;
	float Y;
	float Z;
	int R;
	int G;
	int B;
};

