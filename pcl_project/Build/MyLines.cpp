#include "stdafx.h"
#include "MyLines.h"

MyLines::MyLines()
{
}


MyLines::~MyLines()
{
}

MyLines::MyLines(float x11, float y11, float  x22, float  y22, float anglee){
	x1 = x11;
	x2 = x22;
	y1 = y11;
	y2 = y22;


	slope = (y22 - y11) / (x22 - x11);

	angle = anglee;
	midpointX = (x2 + x1) / 2;
	midpointY = (y2 + y1) / 2;
	length = getDistanceBetweenTwoPoints(x1, y1, x2, y2);
	ceilLength = ceil(length);
}


float MyLines::getDistanceBetweenTwoPoints(float x1, float  y1, float x2, float y2)
{
	return sqrt(powf((x2 - x1), 2) + powf((y2 - y1), 2));
}

float MyLines::getX1()		{ return x1; }
float MyLines::getY1()		{ return y1; }
float MyLines::getX2()		{ return x2; }
float MyLines::getY2()		{ return y2; }
float MyLines::getlength()	{ return length; }
float MyLines::getCeilLength() { return ceilLength; }

float MyLines::getMidPointX() { return midpointX; }
float MyLines::getMidPointY() { return midpointY; }
float MyLines::getSlope(){ return slope; }
float MyLines::getAngle()	{ return angle; }