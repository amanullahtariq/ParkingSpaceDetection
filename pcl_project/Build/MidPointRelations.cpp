
#include "stdafx.h"
#include "MidPointRelations.h"

MidPointRelations::MidPointRelations()
{

}


MidPointRelations::~MidPointRelations()
{

}



MidPointRelations::MidPointRelations(MyLines line, float midpointdis, float aspect)
{
	mylinesss.push_back(line);
	midpointDistance = midpointdis;
	aspectratio = aspect;
}

float MidPointRelations::getAspectRatio(){ return aspectratio; }
void MidPointRelations::Add(MyLines line)
{
	mylinesss.push_back(line);
};

std::vector<MyLines> MidPointRelations::getLines()
{
	return mylinesss; 
}

float MidPointRelations::getMidPointDistance()
{
	return midpointDistance;
}
