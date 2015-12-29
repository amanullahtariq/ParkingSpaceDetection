#include "stdafx.h"
#include "LineRelationsAngle.h"

LineRelationsAngle::LineRelationsAngle()
{
}


LineRelationsAngle::~LineRelationsAngle()
{
}

LineRelationsAngle::LineRelationsAngle(MyLines lin)
{
	angle = lin.getAngle();
	Add(lin);
};

float LineRelationsAngle::getDistanceBetweenTwoPoints(int x1, int  y1, int x2, int y2)
{
	float result = (x2 - x1) ^ 2 + (y2 - y1) ^ 2;
	return sqrt(result);
}

float LineRelationsAngle::getAverage()
{
	float length = 0;
	for (int i = 0; i < liness.size(); i++)
	{
		length = length + getDistanceBetweenTwoPoints(liness[i].getX1(), liness[i].getX2(), liness[i].getY1(), liness[i].getY2());
	}

	return length / liness.size();
}

void LineRelationsAngle::Add(MyLines l)
{
	liness.push_back(l);
}

float LineRelationsAngle::getAngle()	{ return angle; }
std::vector<MyLines> LineRelationsAngle::getLines()	{ return liness; }
