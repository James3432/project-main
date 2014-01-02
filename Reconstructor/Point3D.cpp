#include "StdAfx.h"
#include "Point3D.h"


Point3D::Point3D(void)
{
	X=0; Y=0; Z=0;
	R=0; G=0; B=0;
}

Point3D::Point3D(const Point3D &source): X(source.X),Y(source.Y),Z(source.Z),R(source.R),G(source.G),B(source.B)
{
}

Point3D& Point3D::operator= (const Point3D &source)
{
	X=source.X;Y=source.Y;Z=source.Z;R=source.R;G=source.G;B=source.B;
    // return the existing object
    return *this;
}

Point3D::Point3D(float f1,float f2,float f3)
{
	X=f1; Y=f2; Z=f3;
	R=0; G=0; B=0;
}

Point3D::Point3D(float f1,float f2,float f3,int c1,int c2,int c3)
{
	X=f1; Y=f2; Z=f3;
	R=c1; G=c2; B=c3;
}