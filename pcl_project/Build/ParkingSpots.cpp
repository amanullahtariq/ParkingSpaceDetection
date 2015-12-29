#include "stdafx.h"
#include "ParkingSpots.h"

ParkingSpots::ParkingSpots()
{
}

ParkingSpots::~ParkingSpots()
{
}

ParkingSpots::ParkingSpots(float point1x, float point1y,
	float point2x, float point2y,
	float point3x, float point3y,
	float point4x, float point4y)
{
	point1X = point1x;
	point2X = point2x;
	point3X = point3x;
	point4X = point4x;

	point1Y = point1y;
	point2Y = point2y;
	point3Y = point3y;
	point4Y = point4y;
}

ParkingSpots::ParkingSpots(float point1x, float point1y,
	float point2x, float point2y,
	float point3x, float point3y,
	float point4x, float point4y, int width, int height,float aspectratio,
	float umpx, float umpy,
	float lmpx, float lmpy)
{

	if (height == 0)
		height = 1;

	point1X = point1x;
	point2X = point2x;
	point3X = point3x;
	point4X = point4x;

	point1Y = point1y;
	point2Y = point2y;
	point3Y = point3y;
	point4Y = point4y;

	upper_midpointx = umpx;
	upper_midpointy = umpy;


	lower_midpointx = lmpx;
	lower_midpointy = lmpy;

	rectWidth = width;
	rectHeight = height;


	aspectRatio = aspectratio;
}

float ParkingSpots::getPoint1X() { return point1X; }
float ParkingSpots::getPoint1Y() { return point1Y; }
float ParkingSpots::getPoint2X() { return point2X; }
float ParkingSpots::getPoint2Y() { return point2Y; }
float ParkingSpots::getPoint3X() { return point3X; }
float ParkingSpots::getPoint3Y() { return point3Y; }
float ParkingSpots::getPoint4X() { return point4X; }
float ParkingSpots::getPoint4Y() { return point4Y; }
float ParkingSpots::getAspectRatio() { return aspectRatio; }
float ParkingSpots::getRectWidth() { return rectWidth; }
float ParkingSpots::getRectHeight() { return rectHeight; }