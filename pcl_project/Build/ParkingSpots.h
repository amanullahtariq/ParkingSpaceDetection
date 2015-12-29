#pragma once
class ParkingSpots
{
	float point1X, point1Y;
	float point2X, point2Y;
	float point3X, point3Y;
	float point4X, point4Y;
	float upper_midpointx, upper_midpointy;
	float lower_midpointx, lower_midpointy;
	float aspectRatio;
	float rectWidth;
	float rectHeight;
public:
	ParkingSpots();
	~ParkingSpots();

	ParkingSpots(float point1x, float point1y,
		float point2x, float point2y,
		float point3x, float point3y,
		float point4x, float point4y);

	ParkingSpots(float point1x, float point1y,
		float point2x, float point2y,
		float point3x, float point3y,
		float point4x, float point4y, int width, int height,float aspectratio,
		float umpx,float umpy,
		float lmpx, float lmpy
		);

	float getPoint1X();
	float getPoint1Y();
	float getPoint2X();
	float getPoint2Y();
	float getPoint3X();
	float getPoint3Y();
	float getPoint4X();
	float getPoint4Y();
	float getMidPointX();
	float getMidPointY();
	float getAspectRatio();
	float getRectWidth();
	float getRectHeight();

		
};

