#include "StdAfx.h"
#include "PointMatcher.h"


PointMatcher::PointMatcher(void)
{
}


PointMatcher::~PointMatcher(void)
{
}

bool ColourMatch::run()
{
	int r_diff = r2 - r1;
	int g_diff = g2 - g1;
	int b_diff = b2 - b1;
	float result = /*sqrt*/((float)r_diff*r_diff + g_diff*g_diff + b_diff*b_diff);
	if(result < COLOUR_CRITERION)
		return true;
	else
		return false;
}

bool NormalMatch::run()
{
	if(Distance(A, B) < NORMAL_CRITERION * NORMAL_CRITERION) // only gives square of distance!!
		return true;
	else
		return false;
}

float PointMatcher::Distance(Point3D A, Point3D B)
{
	Point3D diff;diff.X=A.X-B.X;diff.Y= A.Y-B.Y;diff.Z= A.Z-B.Z;
	return /*sqrt*/(diff.X * diff.X + diff.Y * diff.Y + diff.Z * diff.Z);
}