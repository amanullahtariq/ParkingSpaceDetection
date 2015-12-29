#pragma once

#include "MyLines.h"
#include <vector>
class LinesByLength
{
	float linelength;
	float x1, x2;
	float y1, y2;
	std::vector<MyLines> myliness;

public:
	LinesByLength();
	~LinesByLength();
	LinesByLength(MyLines line);
	void Add(MyLines li);
	float getlength();
	std::vector<MyLines> getLines();
};

