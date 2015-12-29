#pragma once
#include "MyLines.h"
#include <vector>

class MidPointRelations
{
	std::vector<MyLines> mylinesss;
	float aspectratio;
	float midpointDistance;

public:
	MidPointRelations(MyLines line, float midpointdis, float aspect);
	MidPointRelations();
	~MidPointRelations();
	float getAspectRatio();
	void Add(MyLines line);
	std::vector<MyLines> getLines();
	float getMidPointDistance();
};

