#pragma once

#include "MyLines.h"
#include <cmath>
#include <vector>

class LineRelationsAngle
{
	std::vector<MyLines> liness;
	float angle;

public:
	LineRelationsAngle();
	~LineRelationsAngle();
	LineRelationsAngle(MyLines lin);

	float getDistanceBetweenTwoPoints(int x1, int  y1, int x2, int y2);
	float getAverage();
	void Add(MyLines l);
	std::vector<MyLines> getLines();
	float getAngle();
};

