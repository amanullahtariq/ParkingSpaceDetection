
#pragma once
#include <cmath>

class MyLines
{
	float x1, y1;
	float x2, y2;

	float angle;
	float slope;

	float midpointX;
	float midpointY;

	float length;
	int ceilLength;
public:

	MyLines();
	~MyLines();
	MyLines(float x11, float y11, float  x22, float  y22, float anglee);
	float getDistanceBetweenTwoPoints(float x1, float  y1, float x2, float y2);
	float getX1();
	float getX2();
	float getY1();
	float getY2();
	float getlength();
	float getCeilLength();

	float getMidPointX();
	float getMidPointY();

	float getSlope();


	float getAngle();


};

